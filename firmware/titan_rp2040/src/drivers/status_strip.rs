//! WS2812B status strip along the top of the Mk2 electronics cages, ported from `driver_status_strip`.
//!
//! The strip has [`NUM_LEDS`] pixels and shows one colour in one of four [`Mode`]s. A status flash (two quick
//! pulses of another colour) can be laid over it to signal an event, such as a kill switch change, without losing
//! the colour command. After power up the strip plays an orange startup animation: the rear half fills in from
//! the middle outwards, the whole strip lights, then the front half empties out towards the middle.
//!
//! [`run`] refreshes the strip every [`UPDATE_PERIOD`], as the C repeating timer did. Other tasks control it
//! through a [`StatusStripControl`]. The pixels are driven by embassy-rp's WS2812 PIO program and DMA rather than
//! the C driver's chained DMA control blocks, so each refresh sends a full frame.
//!
//! [`run`] is generic over the PIO block and state machine, so the board wraps it in its own task:
//!
//! ```ignore
//! static STRIP: StatusStripControl = StatusStripControl::new();
//!
//! #[embassy_executor::task]
//! async fn strip_task(mut leds: PioWs2812<'static, PIO0, 0, { status_strip::NUM_LEDS }, Grb>) -> ! {
//!     status_strip::run(&mut leds, &STRIP, FIRST_PIXEL_IS_REAR).await
//! }
//!
//! // in main
//! let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
//! let program = PioWs2812Program::new(&mut common);
//! let leds = PioWs2812::new(&mut common, sm0, p.DMA_CH2, Irqs, p.PIN_x, &program);
//! spawner.spawn(strip_task(leds).unwrap());
//! ```

use core::cell::Cell;

use embassy_rp::pio::Instance;
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Ticker};
pub use smart_leds::RGB8;

/// Number of pixels on the strip. Must be a multiple of 4.
pub const NUM_LEDS: usize = 12;
const _: () = assert!(NUM_LEDS.is_multiple_of(4));
/// Time between refreshes (`LED_UPDATE_INTERVAL_MS`).
pub const UPDATE_PERIOD: Duration = Duration::from_millis(50);

/// Refreshes per breath cycle, and the period of the animation timer (`LED_TIMER_PERIOD_TICKS`).
const TIMER_PERIOD: u32 = 120;
/// Refreshes per fast flash cycle.
const FAST_FLASH_PERIOD: u32 = 6;
/// Refreshes per slow flash cycle.
const SLOW_FLASH_PERIOD: u32 = 40;
/// Refreshes per status flash pulse.
const FLASH_PULSE_PERIOD: u32 = 6;
/// Pulses per status flash.
const FLASH_PULSE_COUNT: u32 = 2;
/// Refreshes per step of the startup animation.
const STARTUP_TICK_DURATION: u32 = 2;
/// Colour of the startup animation.
const STARTUP_COLOR: RGB8 = RGB8 { r: 255, g: 10, b: 0 };
const OFF: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
/// The strip in quarters: a quarter of end colour, a half of middle colour, a quarter of end colour.
const DEFAULT_SEGMENTS: [usize; 3] = [NUM_LEDS / 4, NUM_LEDS / 2, NUM_LEDS / 4];

/// How the strip shows its colour.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum Mode {
    /// Steady colour.
    Solid,
    /// On and off, 1 s each.
    SlowFlash,
    /// On and off, 150 ms each.
    FastFlash,
    /// Fades in and out over 6 s.
    Breath,
}

#[derive(Clone, Copy)]
struct Commands {
    enabled: bool,
    set: Option<(Mode, RGB8)>,
    flash: Option<RGB8>,
}

/// Controls the strip from any task or interrupt. Declare it as a `static` and pass it to [`run`].
pub struct StatusStripControl {
    commands: Mutex<CriticalSectionRawMutex, Cell<Commands>>,
}

impl StatusStripControl {
    pub const fn new() -> Self {
        StatusStripControl { commands: Mutex::new(Cell::new(Commands { enabled: true, set: None, flash: None })) }
    }

    fn update(&self, f: impl FnOnce(&mut Commands)) {
        self.commands.lock(|cell| {
            let mut commands = cell.get();
            f(&mut commands);
            cell.set(commands);
        });
    }

    /// Shows `color` in `mode` from the next refresh, restarting the animation (`status_strip_set`).
    pub fn set(&self, mode: Mode, color: RGB8) {
        self.update(|c| c.set = Some((mode, color)));
    }

    /// Turns the strip off (`status_strip_clear`).
    pub fn clear(&self) {
        self.set(Mode::Solid, OFF);
    }

    /// Flashes `color` twice over the current colour command (`status_strip_status_flash`).
    pub fn status_flash(&self, color: RGB8) {
        self.update(|c| c.flash = Some(color));
    }

    /// Re-enables output after [`StatusStripControl::disable`] (`status_strip_enable`).
    pub fn enable(&self) {
        self.update(|c| c.enabled = true);
    }

    /// Blanks the strip, including flashes and the startup animation, while keeping the state so
    /// [`StatusStripControl::enable`] picks up where it would have been (`status_strip_disable`).
    pub fn disable(&self) {
        self.update(|c| c.enabled = false);
    }

    fn take(&self) -> Commands {
        self.commands.lock(|cell| {
            let commands = cell.get();
            cell.set(Commands { set: None, flash: None, ..commands });
            commands
        })
    }
}

impl Default for StatusStripControl {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum StartupPhase {
    Grow,
    Full,
    Shrink,
    Blank,
    Done,
}

#[derive(Clone, Copy)]
struct Flash {
    color: RGB8,
    timer: u32,
    count: u32,
}

/// Scales each channel of `color` by `num / den`.
fn scale(color: RGB8, num: u32, den: u32) -> RGB8 {
    let s = |c: u8| (c as u32 * num / den) as u8;
    RGB8 { r: s(color.r), g: s(color.g), b: s(color.b) }
}

/// Triangle fade over `period` ticks: up from 0 to `color` in the first half, back down in the second.
fn fade(color: RGB8, tick: u32, period: u32) -> RGB8 {
    let half = period / 2;
    if tick % period < half {
        scale(color, tick, half)
    } else {
        let down = scale(color, tick - half, half);
        RGB8 { r: color.r - down.r, g: color.g - down.g, b: color.b - down.b }
    }
}

/// The strip's animation state, advanced once per refresh. This is the body of the C `status_strip_refresh`.
pub struct Animator {
    first_pixel_is_rear: bool,
    mode: Mode,
    target: RGB8,
    timer: u32,
    flash: Option<Flash>,
    startup_phase: StartupPhase,
    startup_index: usize,
    /// Pixel counts of the end, middle and end segments.
    segments: [usize; 3],
    middle: RGB8,
    end: RGB8,
}

impl Animator {
    /// Starts with the startup animation. `first_pixel_is_rear` is true if the strip's data input is at the rear.
    pub const fn new(first_pixel_is_rear: bool) -> Self {
        Animator {
            first_pixel_is_rear,
            mode: Mode::Solid,
            target: OFF,
            timer: 0,
            flash: None,
            startup_phase: StartupPhase::Grow,
            startup_index: 0,
            segments: DEFAULT_SEGMENTS,
            middle: OFF,
            end: OFF,
        }
    }

    pub fn set(&mut self, mode: Mode, color: RGB8) {
        self.timer = 0;
        self.mode = mode;
        self.target = color;
    }

    pub fn status_flash(&mut self, color: RGB8) {
        self.flash = Some(Flash { color, timer: 0, count: 0 });
    }

    /// Advances one refresh and returns the frame to send.
    pub fn tick(&mut self, enabled: bool) -> [RGB8; NUM_LEDS] {
        if self.startup_phase != StartupPhase::Done {
            self.tick_startup();
        } else {
            let color = self.mode_color();
            self.timer = (self.timer + 1) % TIMER_PERIOD;
            let color = match &mut self.flash {
                Some(flash) => {
                    let color = fade(flash.color, flash.timer, FLASH_PULSE_PERIOD);
                    flash.timer += 1;
                    if flash.timer >= FLASH_PULSE_PERIOD {
                        flash.timer = 0;
                        flash.count += 1;
                        if flash.count >= FLASH_PULSE_COUNT {
                            self.flash = None;
                        }
                    }
                    color
                }
                None => color,
            };
            self.middle = color;
            self.end = color;
        }

        if !enabled {
            self.middle = OFF;
            self.end = OFF;
        }
        self.frame()
    }

    fn tick_startup(&mut self) {
        self.timer += 1;
        if self.timer < STARTUP_TICK_DURATION {
            return;
        }
        self.timer = 0;
        match self.startup_phase {
            StartupPhase::Grow | StartupPhase::Shrink => {
                self.startup_index += 1;
                let i = self.startup_index;
                // Light the middle segment when growing with the first pixel at the front (the rear half is the
                // middle of the chain), or when shrinking with the first pixel at the rear
                if (self.startup_phase == StartupPhase::Grow) ^ self.first_pixel_is_rear {
                    (self.middle, self.end) = (STARTUP_COLOR, OFF);
                } else {
                    (self.middle, self.end) = (OFF, STARTUP_COLOR);
                }
                // `i` is the number of pixels lit at each end of the rear half (or dark, when shrinking)
                self.segments = if self.first_pixel_is_rear {
                    [i, NUM_LEDS - 2 * i, i]
                } else {
                    [NUM_LEDS / 2 - i, 2 * i, NUM_LEDS / 2 - i]
                };
                if i == NUM_LEDS / 2 - 1 {
                    self.startup_index = 0;
                    self.startup_phase =
                        if self.startup_phase == StartupPhase::Grow { StartupPhase::Full } else { StartupPhase::Blank };
                }
            }
            StartupPhase::Full => {
                self.segments = DEFAULT_SEGMENTS;
                (self.middle, self.end) = (STARTUP_COLOR, STARTUP_COLOR);
                self.startup_phase = StartupPhase::Shrink;
            }
            StartupPhase::Blank => {
                self.segments = DEFAULT_SEGMENTS;
                (self.middle, self.end) = (OFF, OFF);
                self.startup_phase = StartupPhase::Done;
            }
            StartupPhase::Done => {}
        }
    }

    /// The colour command for the current timer value, before any status flash.
    fn mode_color(&self) -> RGB8 {
        match self.mode {
            Mode::Solid => self.target,
            Mode::FastFlash | Mode::SlowFlash => {
                let period = if self.mode == Mode::FastFlash { FAST_FLASH_PERIOD } else { SLOW_FLASH_PERIOD };
                if self.timer % period < period / 2 { self.target } else { OFF }
            }
            Mode::Breath => {
                // Square the triangle to make the fade look smoother
                let c = fade(self.target, self.timer, TIMER_PERIOD);
                let sq = |v: u8| ((v as u32 * v as u32) >> 8) as u8;
                RGB8 { r: sq(c.r), g: sq(c.g), b: sq(c.b) }
            }
        }
    }

    fn frame(&self) -> [RGB8; NUM_LEDS] {
        let [first, middle, _] = self.segments;
        core::array::from_fn(|i| if i >= first && i < first + middle { self.middle } else { self.end })
    }
}

/// Plays the startup animation, then shows whatever `control` commands, refreshing every [`UPDATE_PERIOD`].
/// `first_pixel_is_rear` is true if the strip's first pixel is at the rear of the cage.
pub async fn run<P: Instance, const S: usize>(
    leds: &mut PioWs2812<'_, P, S, NUM_LEDS, Grb>,
    control: &StatusStripControl,
    first_pixel_is_rear: bool,
) -> ! {
    let mut animator = Animator::new(first_pixel_is_rear);
    let mut ticker = Ticker::every(UPDATE_PERIOD);
    loop {
        ticker.next().await;
        let commands = control.take();
        if let Some((mode, color)) = commands.set {
            animator.set(mode, color);
        }
        if let Some(color) = commands.flash {
            animator.status_flash(color);
        }
        let frame = animator.tick(commands.enabled);
        leds.write(&frame).await;
    }
}
