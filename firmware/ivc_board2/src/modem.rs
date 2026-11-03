//! The IVCBoard2 modem, ported from `ivc.c`, `rx.c`, `tx.c`, `consensus.c` and `fft/`.
//!
//! The C firmware ran everything from `ivc_tick` in the main loop, with a repeating timer clocking out transmitted
//! symbols. Here [`rx_task`] is `ivc_tick`: each pass captures and analyses one window and runs whichever mode is
//! active (pinger, flash recorder or comms), and [`tx_task`] owns the transducer and plays packets and debug tones.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::select::{Either, select};
use embassy_futures::yield_now;
use embassy_rp::gpio::Output;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};

use ivc2_core::consensus::Consensus;
use ivc2_core::detect::{self, comms};
use ivc2_core::filter::FilterSelect;
use ivc2_core::pinger::{self, PingerFreq};
use ivc2_core::receiver::Receiver;
use ivc2_core::spectrum::{Analyzer, BandSet};
use ivc2_core::storage::StorageMode;
use ivc2_core::tx::packet_slots;
use ivc2_core::{DEBUG_TONE_MS, DEFAULT_MIN_SNR, NUM_SAMPLES, SYMBOL_PERIOD_MS, Symbol, TX_QUEUE_DEPTH};
use titan_link::{DeviceToHost, HostToDevice, ivc2};
use titan_rp2040::link;

use crate::hw::{Hydrophone, Transmitter};
use crate::recorder::Recorder;

/// Settings changed from ROS.
#[derive(Clone, Copy)]
struct Settings {
    pinger_enabled: bool,
    pinger_freq: PingerFreq,
    storage: StorageMode,
    min_snr: f32,
}

static SETTINGS: Mutex<CriticalSectionRawMutex, Cell<Settings>> = Mutex::new(Cell::new(Settings {
    pinger_enabled: false,
    pinger_freq: PingerFreq::DEFAULT,
    storage: StorageMode::Off,
    min_snr: DEFAULT_MIN_SNR,
}));

fn update_settings(f: impl FnOnce(&mut Settings)) {
    SETTINGS.lock(|cell| {
        let mut settings = cell.get();
        f(&mut settings);
        cell.set(settings);
    });
}

/// Bytes waiting to be transmitted (`tx_msg_queue`).
static TX_QUEUE: Channel<CriticalSectionRawMutex, u8, TX_QUEUE_DEPTH> = Channel::new();
/// The byte [`rx_task`] handed to [`tx_task`].
static TX_PACKET: Signal<CriticalSectionRawMutex, u8> = Signal::new();
/// Set while a packet is being transmitted (`tx.flags.is_writing`).
static WRITING: AtomicBool = AtomicBool::new(false);
static DEBUG_TONE: Signal<CriticalSectionRawMutex, Symbol> = Signal::new();

const SYMBOL_PERIOD: Duration = Duration::from_millis(SYMBOL_PERIOD_MS);
const PINGER_REPORT_PERIOD: Duration = Duration::from_millis(pinger::FREQ_REPORT_PERIOD_MS);

/// Link hook for IVCBoard2 messages from the bridge (the C executor callbacks).
pub fn handle_board_message(msg: HostToDevice) {
    let HostToDevice::Ivc2(msg) = msg else {
        defmt::warn!("Unexpected message for IVCBoard2: {}", msg);
        return;
    };
    match msg {
        ivc2::HostToDevice::EnqueueData(data) => match TX_QUEUE.try_send(data) {
            Ok(()) => defmt::info!("enqueueing {=u8}", data),
            Err(_) => defmt::warn!("IVC transmit queue full, dropping {=u8:#x}", data),
        },
        ivc2::HostToDevice::PingerEnable(enabled) => update_settings(|s| s.pinger_enabled = enabled),
        ivc2::HostToDevice::PingerSetFreqKhz(khz) => match PingerFreq::from_khz(khz) {
            Some(freq) => update_settings(|s| s.pinger_freq = freq),
            None => defmt::warn!("Ignoring pinger frequency of {} kHz", khz),
        },
        ivc2::HostToDevice::StorageConfigure(config) => match StorageMode::from_config(config) {
            Some(mode) => update_settings(|s| s.storage = mode),
            None => defmt::warn!("Ignoring storage configuration {}", config),
        },
        ivc2::HostToDevice::SetMinSnr(snr) => update_settings(|s| s.min_snr = snr),
        ivc2::HostToDevice::DebugSendFreq(value) => DEBUG_TONE.signal(Symbol::from_debug_value(value)),
    }
}

/// Plays packets handed over by [`rx_task`], and debug tones.
///
/// In C a debug tone (`debug_tx`) reprogrammed the PWM even in the middle of a packet, and its end alarm cleared
/// the transmitter's busy flag. Here a debug tone waits for the packet to finish, like the IVCBoard port.
#[embassy_executor::task]
pub async fn tx_task(mut tx: Transmitter, _output_select: Output<'static>) -> ! {
    loop {
        match select(TX_PACKET.wait(), DEBUG_TONE.wait()).await {
            Either::First(data) => {
                defmt::info!("Transmitting {=u8:#x}", data);
                let slots = packet_slots(data);
                tx.play(slots[0]);
                let mut ticker = Ticker::every(SYMBOL_PERIOD);
                for &symbol in &slots[1..] {
                    ticker.next().await;
                    tx.play(symbol);
                }
                ticker.next().await;
                tx.enable(false);
                WRITING.store(false, Ordering::Relaxed);
            }
            Either::Second(symbol) => {
                tx.play(symbol);
                Timer::after_millis(DEBUG_TONE_MS).await;
                tx.enable(false);
            }
        }
    }
}

/// `ivc_tick`: one capture per pass, in the mode selected from ROS.
#[embassy_executor::task]
pub async fn rx_task(mut hydrophone: Hydrophone, mut recorder: Recorder) -> ! {
    let mut samples = [0u8; NUM_SAMPLES];
    let mut analyzer = Analyzer::new();
    let comms_bands = BandSet::new(comms::BANDS);
    let pinger_bands = PingerFreq::ALL.map(|freq| BandSet::new(freq.bands()));
    let mut consensus = Consensus::new();
    let mut receiver = Receiver::new();
    let mut next_pinger_report: Option<Instant> = None;

    loop {
        let settings = SETTINGS.lock(Cell::get);

        if settings.pinger_enabled {
            // rx_handle_pinger, plus the 1 s timer that publishes the selected frequency
            let next = next_pinger_report.get_or_insert_with(|| Instant::now() + PINGER_REPORT_PERIOD);
            if Instant::now() >= *next {
                *next += PINGER_REPORT_PERIOD;
                if link::is_connected() {
                    link::send(&DeviceToHost::Ivc2(ivc2::DeviceToHost::PingerFreqKhz(settings.pinger_freq.khz())));
                }
            }
            let freq = settings.pinger_freq;
            if !capture(&mut hydrophone, &mut samples).await {
                continue;
            }
            analyzer.prepare(&samples, FilterSelect::Pinger(freq));
            yield_now().await;
            let magnitudes = analyzer.measure(&pinger_bands[freq.index()]);
            // PINGER_HISTORY_SIZE is 1, so the "maximum of the history" is just this capture's amplitude
            if link::is_connected() {
                link::send(&DeviceToHost::Ivc2(ivc2::DeviceToHost::PingerAmplitude(magnitudes[pinger::SIGNAL_INDEX])));
            }
            continue;
        }
        next_pinger_report = None;

        if settings.storage != StorageMode::Off {
            recorder.step(settings.storage, &mut samples).await;
            continue;
        }
        recorder.stop();

        // Comms: observe, then attempt_reading and attempt_writing
        if !capture(&mut hydrophone, &mut samples).await {
            continue;
        }
        analyzer.prepare(&samples, FilterSelect::Comms);
        yield_now().await;
        let magnitudes = analyzer.measure(&comms_bands);
        consensus.update(detect::observe(&magnitudes, analyzer.noise_floor(), settings.min_snr));

        if let Some(data) = receiver.poll(&mut consensus, WRITING.load(Ordering::Relaxed)) {
            defmt::info!("packet read complete with: {=u8:#x}", data);
            if !link::send(&DeviceToHost::Ivc2(ivc2::DeviceToHost::RxNew(data))) {
                defmt::warn!("Dropped received IVC byte {=u8:#x}, CAN queue full", data);
            }
        }

        // In C attempt_writing dequeued every pass even while a packet was going out, overwriting the bits still to
        // be sent. Only start a packet once the previous one is done.
        if !receiver.is_receiving()
            && !WRITING.load(Ordering::Relaxed)
            && let Ok(data) = TX_QUEUE.try_receive()
        {
            WRITING.store(true, Ordering::Relaxed);
            TX_PACKET.signal(data);
        }
    }
}

/// Captures one window, returning false (after logging) if the ADC failed.
async fn capture(hydrophone: &mut Hydrophone, samples: &mut [u8; NUM_SAMPLES]) -> bool {
    match hydrophone.capture(samples).await {
        Ok(_) => true,
        Err(err) => {
            defmt::warn!("ADC capture failed: {}", err);
            false
        }
    }
}
