//! Talos IVC board with DAC transmitter: Rust port of `talos/IVCDAC`.
//!
//! IVCDAC is the IVCBoard2 firmware cut down to a data gathering tool for characterising the acoustic channel: it
//! plays single tones on command and timestamps their start and end, and the receive side build (`rx-side` feature,
//! `RX_SIDE` in the C `dac_config.h`, off by default as there) streams the comms band magnitudes of every capture.
//! Despite the name, transmission uses the same PWM drive as IVCBoard2. ROS topics (served by
//! `titan_bridge --board ivc_dac`, node `ivc_dac`):
//! * `ivc_dac/send_freq` (`std_msgs/Int8`, subscribed, best effort): play one tone for 500 ms (0 low, 2 sync, else high)
//! * `ivc_dac/tx_start`, `ivc_dac/tx_end` (`builtin_interfaces/Time`, published): when that tone started and stopped
//! * `ivc_dac/rx_state` (`riptide_msgs2/IvcRx`, published by `rx-side` builds): low, high and sync band magnitudes
//! * `ivc/pinger/enable`, `ivc/pinger/set_freq_khz` (subscribed): switch the `rx-side` filter to a pinger band
//! * leftover IVCBoard2 topics that have no effect on this firmware, created for parity: `ivc/enqueue_data`,
//!   `ivc/storage/configure`, `ivc/set_min_snr` and the silent publishers `ivc/rx_new`, `ivc/adc_dump`,
//!   `ivc/new_transducer`, `ivc/pinger/selected_freq_amp_stream`, `ivc/pinger/selected_freq_khz`,
//!   `ivc/rx_sample_stream`, `ivc/rx_sample_snr_stream`
//! * plus the standard `heartbeat`, `state/firmware` and `state/kill`
//!
//! CANmore remote commands, those of `canmore.c`: `write_lock`, `tx_queue_empty`, `receiving_packet`,
//! `set_comms_mode` and `set_pinger_mode`; see [`COMMANDS`]. The C IVCDAC had the file but never registered them
//! (`register_canmore_commands` is commented out in `ivc_init`, and it used the USB transport).
//!
//! Pins as on IVCBoard2: transducer drive on GPIO 7, filter output select on GPIO 3, board ID on GPIO 6, hydrophone
//! on ADC 1 (GPIO 27). Like IVCBoard2 this talks to the bridge over the internal CAN bus instead of USB micro-ROS.

#![no_std]
#![no_main]

// Same hardware as IVCBoard2
#[allow(dead_code)]
#[path = "../../ivc_board2/src/hw.rs"]
mod hw;

use core::cell::Cell;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2};
use embassy_rp::{bind_interrupts, dma};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;

use canmore::client_ids::CLIENT_ID_IVC_BOARD;
use ivc2_core::pinger::PingerFreq;
use ivc2_core::{DEBUG_TONE_MS, Symbol};
use titan_link::{DeviceToHost, HostToDevice, ivc_dac};
use titan_rp2040::board::{self, BoardConfig, Bus};
use titan_rp2040::control::{Args, Command, Response};
use titan_rp2040::link;

#[cfg(not(feature = "rx-side"))]
bind_interrupts!(struct Irqs {
    // DMA_CH1/2 belong to the CAN block's SPI
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

#[cfg(feature = "rx-side")]
bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
    // DMA_CH0 feeds the ADC; DMA_CH1/2 belong to the CAN block's SPI
    DMA_IRQ_0 => dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>, dma::InterruptHandler<DMA_CH1>,
        dma::InterruptHandler<DMA_CH2>;
});

/// Pinger filter selection; only the receive side uses it.
static PINGER: Mutex<CriticalSectionRawMutex, Cell<(bool, PingerFreq)>> =
    Mutex::new(Cell::new((false, PingerFreq::DEFAULT)));
static TONE: Signal<CriticalSectionRawMutex, Symbol> = Signal::new();
/// Set while [`tone_task`] plays a tone, for `write_lock`.
static PLAYING: AtomicBool = AtomicBool::new(false);

fn handle_board_message(msg: HostToDevice) {
    match msg {
        HostToDevice::IvcDac(ivc_dac::HostToDevice::SendFreq(value)) => TONE.signal(Symbol::from_debug_value(value)),
        HostToDevice::IvcDac(ivc_dac::HostToDevice::PingerEnable(enabled)) => {
            PINGER.lock(|p| p.set((enabled, p.get().1)));
        }
        HostToDevice::IvcDac(ivc_dac::HostToDevice::PingerSetFreqKhz(khz)) => match PingerFreq::from_khz(khz) {
            Some(freq) => PINGER.lock(|p| p.set((p.get().0, freq))),
            None => defmt::warn!("Ignoring pinger frequency of {} kHz", khz),
        },
        other => defmt::warn!("Unexpected message for IVCDAC: {}", other),
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    board::start(
        spawner,
        titan_rp2040::can_block!(p, Irqs),
        BoardConfig {
            name: "talos/ivc_board",
            client_id: CLIENT_ID_IVC_BOARD,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            on_board_message: handle_board_message,
            remote_commands: &COMMANDS,
            ..BoardConfig::DEFAULT
        },
    );

    // Selects the low pass filtered transducer output
    let output_select = Output::new(p.PIN_3, Level::High);
    let is_talos = Input::new(p.PIN_6, Pull::Down).is_low();
    defmt::info!("This board {} talos", if is_talos { "is" } else { "is not" });

    spawner.spawn(tone_task(hw::Transmitter::new(p.PWM_SLICE3, p.PIN_7), output_select).unwrap());

    #[cfg(feature = "rx-side")]
    {
        use embassy_rp::adc::{self, Adc};
        let adc = Adc::new(p.ADC, Irqs, adc::Config::default());
        let input = adc::Channel::new_pin(p.PIN_27, Pull::None);
        let adc_dma = dma::Channel::new(p.DMA_CH0, Irqs);
        spawner.spawn(rx_side::gather_task(hw::Hydrophone::new(adc, input, adc_dma)).unwrap());
    }
}

/// Plays tones from `ivc_dac/send_freq` and reports when each starts and stops.
///
/// The C callback published `tx_start`, started the tone and published `tx_end` straight away, while the tone kept
/// playing for another 500 ms; `tx_end` is now sent when the tone actually stops. A new command during a tone
/// retunes it (with a new `tx_start`) and restarts the 500 ms, where in C the first command's alarm cut it short.
#[embassy_executor::task]
async fn tone_task(mut tx: hw::Transmitter, _output_select: Output<'static>) -> ! {
    loop {
        let mut symbol = TONE.wait().await;
        PLAYING.store(true, Ordering::Relaxed);
        loop {
            send(ivc_dac::DeviceToHost::TxStart);
            tx.play(symbol);
            match select(Timer::after_millis(DEBUG_TONE_MS), TONE.wait()).await {
                Either::First(()) => break,
                Either::Second(next) => symbol = next,
            }
        }
        tx.enable(false);
        PLAYING.store(false, Ordering::Relaxed);
        send(ivc_dac::DeviceToHost::TxEnd);
    }
}

fn send(msg: ivc_dac::DeviceToHost) {
    if link::is_connected() && !link::send(&DeviceToHost::IvcDac(msg)) {
        defmt::warn!("CAN queue full, dropped IVCDAC report");
    }
}

// ========================================
// CANmore remote commands (`canmore.c`)
// ========================================

/// The remote commands of `canmore.c`, in `register_canmore_commands` order. This firmware has no packet modem, so
/// the status queries report the closest state it has (see each handler). The mode commands change [`PINGER`], the
/// same path as the bridge's `ivc/pinger/*` messages; like those, they only matter to `rx-side` builds.
///
/// The C callbacks were `void` functions cast to the `int` callback type, so their return code was whatever was left
/// in r0; these return 0 on success and 1 on an error.
static COMMANDS: [Command; 5] = [
    Command::new("write_lock", "", "get the status of the write lock bool", write_lock),
    Command::new("tx_queue_empty", "", "see if the tx queue is empty", tx_queue_empty),
    Command::new("receiving_packet", "", "check if the receiving packet flag is set", receiving_packet),
    Command::new("set_comms_mode", "", "set IVC to listen for comms signals", set_comms_mode),
    Command::new(
        "set_pinger_mode",
        "[pinger freq in khz (20, 25, 30, 35, 40)]",
        "will set ivc to listen for that frequency\n",
        set_pinger_mode,
    ),
];

fn bool_str(value: bool) -> &'static str {
    if value { "true" } else { "false" }
}

/// `tx.flags.is_writing`. Nothing set it in the C IVCDAC (only packet transmission did, which this firmware never
/// runs), so it always read false; the closest state here is whether a `send_freq` tone is playing.
fn write_lock(_args: &Args<'_>, out: &mut Response<'_>) -> i32 {
    let _ = writeln!(out, "is_writing: [{}]", bool_str(PLAYING.load(Ordering::Relaxed)));
    0
}

/// `tx_queue_empty()`. There is no transmit queue here: `ivc/enqueue_data` is not forwarded to this board (in C it
/// filled a queue nothing ever drained), so it is always empty.
fn tx_queue_empty(_args: &Args<'_>, out: &mut Response<'_>) -> i32 {
    let _ = writeln!(out, "empty: [true]");
    0
}

/// `rx.flags.receiving_packet`. There is no packet receiver here (the C IVCDAC never set the flag either), so it is
/// always false.
fn receiving_packet(_args: &Args<'_>, out: &mut Response<'_>) -> i32 {
    let _ = writeln!(out, "receiving_packet: [false]");
    0
}

/// Clears `rx.flags.is_pinger_mode`, like `ivc/pinger/enable` false.
fn set_comms_mode(_args: &Args<'_>, _out: &mut Response<'_>) -> i32 {
    PINGER.lock(|p| p.set((false, p.get().1)));
    0
}

/// Selects the pinger frequency (`rx.pinger_mode`) without turning the pinger mode on, like C.
///
/// C checked `argc != 1`, but its `argc` counts the command name, so the command always failed; this takes exactly
/// one argument as the usage says. Only the five listed frequencies are accepted, by exact text like C's `strcmp`
/// (37 kHz stays reachable through `ivc/pinger/set_freq_khz`).
fn set_pinger_mode(args: &Args<'_>, out: &mut Response<'_>) -> i32 {
    let (2, Some(arg)) = (args.len(), args.get(1)) else {
        let _ = writeln!(out, "ERROR: arg must be exactly one of [ 20 | 25 | 30 | 35 | 40 ]");
        return 1;
    };
    let freq = match arg {
        "20" => PingerFreq::K20,
        "25" => PingerFreq::K25,
        "30" => PingerFreq::K30,
        "35" => PingerFreq::K35,
        "40" => PingerFreq::K40,
        _ => {
            let _ = writeln!(out, "ERROR: <{arg}> is unrecognized");
            return 1;
        }
    };
    PINGER.lock(|p| p.set((p.get().0, freq)));
    0
}

#[cfg(feature = "rx-side")]
mod rx_side {
    use embassy_futures::yield_now;
    use embassy_time::Instant;

    use ivc2_core::NUM_SAMPLES;
    use ivc2_core::detect::comms;
    use ivc2_core::filter::FilterSelect;
    use ivc2_core::spectrum::{Analyzer, BandSet};

    use super::*;

    /// `rx_gather`: the comms band magnitudes of every capture. Like `process_capture_buffer`, the pinger switch
    /// selects the pinger filter even though the bands stay the comms ones.
    #[embassy_executor::task]
    pub async fn gather_task(mut hydrophone: hw::Hydrophone) -> ! {
        let mut samples = [0u8; NUM_SAMPLES];
        let mut analyzer = Analyzer::new();
        let bands = BandSet::new(comms::BANDS);
        loop {
            let captured_at = match hydrophone.capture(&mut samples).await {
                Ok(at) => at,
                Err(err) => {
                    defmt::warn!("ADC capture failed: {}", err);
                    continue;
                }
            };
            let filter = match PINGER.lock(Cell::get) {
                (true, freq) => FilterSelect::Pinger(freq),
                (false, _) => FilterSelect::Comms,
            };
            analyzer.prepare(&samples, filter);
            yield_now().await;
            let m = analyzer.measure(&bands);
            let age_us = (Instant::now() - captured_at).as_micros() as u32;
            send(ivc_dac::DeviceToHost::RxState {
                low: m[comms::LOW],
                high: m[comms::HIGH],
                sync: m[comms::SYNC],
                age_us,
            });
        }
    }
}
