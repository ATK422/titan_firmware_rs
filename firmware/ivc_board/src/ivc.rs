//! The IVC acoustic modem, ported from `ivc.c` and `fft.c`.
//!
//! The C version spread the protocol over repeating timers and alarm callbacks that ran the FFT in interrupt
//! context. Here [`sampler_task`] keeps capturing 1 ms windows from the ADC, and [`ivc_task`] runs the protocol as
//! straight-line async code: idle detection, transmit with ACK and retransmission, and receive with ACK/NACK.
//! Transmit timing is the C schedule; receive and ACK timing follow [`ivc_core::timing`] so Rust and C boards still
//! line up despite the much faster detector, and the sender listens for the ACK over a window so that it also
//! arrives from a distant receiver.

use embassy_futures::select::{Either4, select, select4};
use embassy_futures::yield_now;
use embassy_rp::Peri;
use embassy_rp::adc::{self, Adc};
use embassy_rp::dma;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::{PIN_7, PWM_SLICE3};
use embassy_rp::pwm::{self, Pwm};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_sync::watch::{self, Watch};
use embassy_time::{Duration, Instant, Ticker, Timer, with_deadline};
use fixed::FixedU16;
use fixed::types::extra::U4;

use ivc_core::*;
use titan_link::{DeviceToHost, HostToDevice, ivc};
use titan_rp2040::link;

/// A packet waiting to be transmitted.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct TxRequest {
    pub data: u8,
    pub is_heartbeat: bool,
}

static TX_QUEUE: Channel<CriticalSectionRawMutex, TxRequest, TX_QUEUE_DEPTH> = Channel::new();
static DEBUG_TONE: Signal<CriticalSectionRawMutex, f32> = Signal::new();
/// The most recent completed capture.
static CAPTURES: Watch<CriticalSectionRawMutex, Capture, 1> = Watch::new();

/// One 1 ms window of hydrophone samples.
#[derive(Clone)]
pub struct Capture {
    /// When the last sample was taken.
    end: Instant,
    samples: [u8; NUM_SAMPLES],
}

const SYMBOL_PERIOD: Duration = Duration::from_millis(SYMBOL_PERIOD_MS);
const DEBUG_TONE_DURATION: Duration = Duration::from_millis(1000);

/// Queues a packet, dropping it if the queue is full (like `ivc_enqueue_packet`).
pub fn enqueue(data: u8, is_heartbeat: bool) {
    if TX_QUEUE.try_send(TxRequest { data, is_heartbeat }).is_err() {
        defmt::warn!("IVC transmit queue full, dropping {=u8:#x}", data);
    }
}

/// Link hook for IVC messages from the bridge.
pub fn handle_board_message(msg: HostToDevice) {
    match msg {
        HostToDevice::Ivc(ivc::HostToDevice::Tx(data)) => enqueue(data, false),
        HostToDevice::Ivc(ivc::HostToDevice::DebugTxFrequency(hz)) => DEBUG_TONE.signal(hz),
        other => defmt::warn!("Unexpected message for the IVC board: {}", other),
    }
}

/// Continuously captures [`NUM_SAMPLES`] 8-bit samples at 500 kS/s (one window per millisecond).
#[embassy_executor::task]
pub async fn sampler_task(
    mut adc: Adc<'static, adc::Async>,
    mut input: adc::Channel<'static>,
    mut dma: dma::Channel<'static>,
) -> ! {
    let sender = CAPTURES.sender();
    let mut samples = [0u8; NUM_SAMPLES];
    loop {
        // A divider of 0 runs conversions back to back: 48 MHz / 96 cycles = 500 kS/s
        match adc.read_many(&mut input, &mut samples, 0, &mut dma).await {
            Ok(()) => sender.send(Capture { end: Instant::now(), samples }),
            Err(err) => defmt::warn!("ADC capture failed: {}", err),
        }
    }
}

/// The transducer drive: a 50 % square wave on GPIO 7 (PWM 3B) whose frequency selects the symbol.
pub struct Transmitter {
    pwm: Pwm<'static>,
    config: pwm::Config,
    wrap: u16,
}

impl Transmitter {
    pub fn new(slice: Peri<'static, PWM_SLICE3>, pin: Peri<'static, PIN_7>) -> Self {
        // Pick the wrap so the clock divider stays below 256 down to 5 kHz (99 at 125 MHz)
        let sysclk = embassy_rp::clocks::clk_sys_freq() as f32;
        let wrap = (sysclk / 5000.0 / 255.0) as u16 + 1;
        let mut config = pwm::Config::default();
        config.top = wrap - 1;
        config.compare_b = wrap / 2;
        config.enable = false;
        Transmitter { pwm: Pwm::new_output_b(slice, pin, config.clone()), config, wrap }
    }

    /// Sets the output frequency. Returns false (leaving it unchanged) if the divider cannot reach it.
    fn set_frequency(&mut self, hz: f32) -> bool {
        let divider = embassy_rp::clocks::clk_sys_freq() as f32 / (hz * self.wrap as f32);
        if !(1.0..256.0).contains(&divider) {
            return false;
        }
        // 8.4 fixed point, fraction truncated like pwm_set_clkdiv
        self.config.divider = FixedU16::<U4>::from_bits((divider * 16.0) as u16);
        self.pwm.set_config(&self.config);
        true
    }

    fn set_symbol(&mut self, bit: bool) {
        self.set_frequency(if bit { FREQ_HIGH_HZ } else { FREQ_LOW_HZ });
    }

    fn enable(&mut self, on: bool) {
        self.config.enable = on;
        self.pwm.set_config(&self.config);
    }
}

pub struct Modem {
    tx: Transmitter,
    /// Selects the low pass filtered transducer output; held high for the life of the modem.
    _output_select: Output<'static>,
    captures: watch::Receiver<'static, CriticalSectionRawMutex, Capture, 1>,
    detector: BandDetector<2>,
    /// The fixed detection thresholds for this vehicle.
    thresholds: Thresholds,
    /// Noise in the detection bands, measured while idle (only used with the `adaptive-threshold` feature).
    noise: NoiseFloor,
    is_talos: bool,
    /// Remaining retransmissions before the link counts as broken.
    link_established: u8,
    /// SSFB may only transmit until this time.
    ssfb_tx_until: Instant,
}

impl Modem {
    pub fn new(tx: Transmitter, output_select: Output<'static>, is_talos: bool) -> Self {
        Modem {
            tx,
            _output_select: output_select,
            captures: CAPTURES.receiver().expect("only one modem"),
            detector: BandDetector::new(BANDS, SAMPLE_RATE_HZ, NUM_SAMPLES),
            thresholds: if is_talos { TALOS_THRESHOLDS } else { SSFB_THRESHOLDS },
            noise: NoiseFloor::new(),
            is_talos,
            link_established: 0,
            ssfb_tx_until: Instant::MIN,
        }
    }

    /// The thresholds to classify with: the fixed ones, raised above the noise floor with `adaptive-threshold`.
    fn thresholds(&self) -> Thresholds {
        if cfg!(feature = "adaptive-threshold") { self.noise.thresholds(self.thresholds) } else { self.thresholds }
    }

    /// Classifies the most recent capture.
    async fn sample(&mut self) -> Option<bool> {
        let capture = self.captures.get().await;
        classify(self.detector.process(&capture.samples), self.thresholds())
    }

    async fn run(mut self) -> ! {
        let mut next_heartbeat = Instant::now();
        loop {
            let now = Instant::now();

            // Only Talos sends heartbeats and waits for SSFB to answer
            if self.is_talos && now >= next_heartbeat {
                enqueue(HEARTBEAT_DATA, true);
                next_heartbeat = now + Duration::from_millis(HEARTBEAT_PERIOD_MS);
            }

            // SSFB may only transmit for a short window after hearing from Talos
            let may_transmit = self.is_talos || now < self.ssfb_tx_until;
            if may_transmit && let Ok(request) = TX_QUEUE.try_receive() {
                self.transmit(request).await;
                continue;
            }

            let heartbeat_at = if self.is_talos { next_heartbeat } else { Instant::MAX };
            let (is_talos, ssfb_tx_until) = (self.is_talos, self.ssfb_tx_until);
            let transmit_ready = async move {
                if !may_transmit {
                    core::future::pending::<()>().await
                } else if is_talos {
                    TX_QUEUE.ready_to_receive().await
                } else {
                    // Wake when the window closes too, so a later request waits for the next window
                    select(TX_QUEUE.ready_to_receive(), Timer::at(ssfb_tx_until)).await;
                }
            };

            match select4(self.captures.changed(), Timer::at(heartbeat_at), transmit_ready, DEBUG_TONE.wait()).await {
                Either4::First(capture) => {
                    let magnitudes = self.detector.process(&capture.samples);
                    if cfg!(feature = "calibration") {
                        link::send(&DeviceToHost::Ivc(ivc::DeviceToHost::RxDebugMagnitude(magnitudes[0])));
                    } else if classify(magnitudes, self.thresholds()) == Some(true) {
                        self.receive(capture.end).await;
                    } else if cfg!(feature = "adaptive-threshold") {
                        // Only idle captures count towards the noise floor: never our own packets or answers
                        self.noise.update(magnitudes);
                    }
                    // Captures arrive faster than they are processed; let the other tasks run between them
                    yield_now().await;
                }
                Either4::Second(()) | Either4::Third(()) => {}
                Either4::Fourth(hz) => self.debug_tone(hz).await,
            }
        }
    }

    /// Sends one packet, waits for the ACK and retransmits while the link is established.
    async fn transmit(&mut self, request: TxRequest) {
        let word = encode_packet(request.data, request.is_heartbeat);
        loop {
            let onset = Instant::now();
            let mut symbols = packet_symbols(word);
            self.tx.set_symbol(symbols.next().unwrap());
            self.tx.enable(true);
            let mut ticker = Ticker::every(SYMBOL_PERIOD);
            for bit in symbols {
                ticker.next().await;
                self.tx.set_symbol(bit);
            }
            ticker.next().await;
            self.tx.enable(false);

            let response = self.listen_for_ack(onset).await;
            let ack = response.is_ack();
            if response == Response::Silence {
                defmt::info!("No response heard from receiver");
            }
            defmt::info!("Got response of {} from receiver", ack);

            Timer::at(onset + Duration::from_millis(timing::NEXT_TX_MS)).await;
            if ack {
                self.link_established = MAX_MISSED_PACKETS;
                return;
            } else if self.link_established > 0 {
                self.link_established -= 1;
            } else {
                defmt::warn!("IVC link broken!");
                return;
            }
        }
    }

    /// Classifies every capture in the ACK window of the packet whose start symbol began at `onset` until the
    /// receiver's answer is clear (see [`AckListener`]). Listening over a window instead of sampling once lets the
    /// answer arrive late, as it does from a distant receiver.
    async fn listen_for_ack(&mut self, onset: Instant) -> Response {
        let at = |ms: u64| onset + Duration::from_millis(ms);
        Timer::at(at(timing::ACK_LISTEN_START_MS)).await;
        let mut listener = AckListener::new();
        // Also stop at the end of the window if the captures stop coming
        let deadline = at(timing::ACK_LISTEN_END_MS + 1);
        loop {
            let Ok(capture) = with_deadline(deadline, self.captures.changed()).await else {
                return Response::Silence;
            };
            let since_onset_us = capture.end.saturating_duration_since(onset).as_micros();
            if since_onset_us < timing::ACK_LISTEN_START_MS * 1000 {
                // Left over from before the window: skip it rather than spend a detector run on it
                continue;
            }
            let symbol = classify(self.detector.process(&capture.samples), self.thresholds());
            if let Some(response) = listener.push(since_onset_us, symbol) {
                let after_packet_ms = since_onset_us / 1000 - timing::TX_END_MS;
                defmt::debug!("{} heard {} ms after the packet ended", response, after_packet_ms);
                return response;
            }
            // As in `run`: captures come faster than they are processed, so `changed` is always ready at once
            yield_now().await;
        }
    }

    /// Receives the packet whose start symbol filled the capture ending at `detected_at`, then answers with ACK or
    /// NACK. Every step is scheduled from the estimated start symbol onset, so processing time does not shift it.
    async fn receive(&mut self, detected_at: Instant) {
        let onset = Instant::from_micros(timing::start_onset_us(detected_at.as_micros()));
        let at = |ms: u64| onset + Duration::from_millis(ms);

        let mut assembler = PacketAssembler::new();
        let mut symbol_index = 0;
        let packet = loop {
            Timer::at(at(timing::rx_symbol_sample_ms(symbol_index))).await;
            symbol_index += 1;
            let symbol = self.sample().await;
            if let Some(packet) = assembler.push(symbol) {
                break packet;
            }
        };

        Timer::at(at(timing::RX_ACK_START_MS)).await;
        self.tx.set_symbol(packet.valid);
        self.tx.enable(true);

        if !packet.valid {
            defmt::info!("Got CRC_BAD: got {} and expected {}", packet.crc, crc4(packet.data));
        } else {
            self.link_established = MAX_MISSED_PACKETS;
            defmt::info!("Got CRC_GOOD");
            if packet.is_heartbeat {
                defmt::info!("Got heartbeat");
            } else if !link::send(&DeviceToHost::Ivc(ivc::DeviceToHost::Rx(packet.data))) {
                defmt::warn!("Dropped received IVC byte {=u8:#x}, CAN queue full", packet.data);
            }
        }

        Timer::at(at(timing::RX_ACK_END_MS)).await;
        self.tx.enable(false);
        self.ssfb_tx_until = Instant::now() + Duration::from_millis(SSFB_TX_WINDOW_MS);
    }

    /// Plays a tone for one second (`ivc/debug_tx_frequency`).
    async fn debug_tone(&mut self, hz: f32) {
        if !self.tx.set_frequency(hz) {
            defmt::warn!("Debug tone of {} Hz is out of range", hz);
            return;
        }
        self.tx.enable(true);
        Timer::after(DEBUG_TONE_DURATION).await;
        self.tx.enable(false);
    }
}

/// Runs the modem forever.
#[embassy_executor::task]
pub async fn ivc_task(modem: Modem) -> ! {
    modem.run().await
}
