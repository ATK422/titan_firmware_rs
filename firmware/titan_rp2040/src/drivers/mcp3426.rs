//! Microchip MCP3426 2-channel 16-bit delta-sigma ADC, ported from `driver_mcp3426`.
//!
//! [`Mcp3426::run`] does what the C callback chain did:
//! 1. writes `0x06` to the ADC (see [`Mcp3426::reset`])
//! 2. selects a channel in continuous mode, 16 bits, gain 1
//! 3. waits [`READ_DELAY`] and reads the result, repeating every [`READ_DELAY`] until the ADC reports it ready
//! 4. stores the voltage for the channel the result belongs to and selects the next channel, back to 2
//!
//! The C driver cycles through four channel selections although the MCP3426 only has channels 1 and 2 (3 and 4
//! exist on the MCP3428). This keeps that, so each channel still updates about every 800 ms as before.
//!
//! # Errors
//!
//! A failed transfer counts towards [`ERROR_THRESHOLD`] consecutive failures. Once there, `on_error` is called for
//! each further failure and every stored voltage is cleared, until a read succeeds. As in C the count starts at
//! `ERROR_THRESHOLD - 1`, so a failure before the first successful read reports straight away (the ADC is
//! probably missing).
//!
//! # Differences from C
//!
//! * After a failed transfer the C driver's callback chain simply stopped, so the ADC was never read again. This
//!   retries the failed step after [`READ_DELAY`].
//! * Output codes are read as signed, as the datasheet specifies. The C driver read them as unsigned, so noise
//!   just below 0 V came out as about 4.096 V.
//! * Voltages that haven't been read are `None` instead of NaN.

use core::cell::Cell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use titan_sensors::mcp3426::{self, CHANNEL_COUNT, Gain, Mode, SampleRate};

use super::{I2cError, i2c_read, i2c_write};

/// Default 7-bit address (address pins floating or tied low), as used by the Backplane.
pub const DEFAULT_ADDRESS: u8 = 0x68;
/// Delay between selecting a channel and reading it, and between polls of a result that isn't ready. A 16-bit
/// conversion takes 67 ms.
pub const READ_DELAY: Duration = Duration::from_millis(100);
/// Consecutive failures before `on_error` is called and the voltages are cleared.
pub const ERROR_THRESHOLD: u8 = 3;

/// The byte the C driver sent as a "general call reset". It went to the ADC's own address rather than the general
/// call address, so the ADC takes it as a configuration write (one-shot, channel 1, 14 bits, gain 4) that the
/// following channel selection overrides. Kept as is: a real general call would reset every device on the bus.
const RESET_CMD: u8 = 0x06;
const MODE: Mode = Mode::Continuous;
const RATE: SampleRate = SampleRate::Bits16;
const GAIN: Gain = Gain::X1;

/// ADC channel selection.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum Channel {
    Ch1 = 0,
    Ch2 = 1,
    Ch3 = 2,
    Ch4 = 3,
}

impl Channel {
    pub const ALL: [Channel; CHANNEL_COUNT] = [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4];

    fn next(self) -> Channel {
        Self::ALL[(self as usize + 1) % CHANNEL_COUNT]
    }
}

/// The latest voltage on each channel, shared between [`Mcp3426::run`] and its readers. Declare it as a `static`.
pub struct Voltages {
    values: Mutex<CriticalSectionRawMutex, Cell<[Option<f32>; CHANNEL_COUNT]>>,
}

impl Voltages {
    pub const fn new() -> Self {
        Voltages { values: Mutex::new(Cell::new([None; CHANNEL_COUNT])) }
    }

    /// The last voltage read on `channel` (`mcp3426_read_voltage`), or `None` if there is none.
    ///
    /// Like the C driver, values don't expire; they're only cleared after repeated failures.
    pub fn get(&self, channel: Channel) -> Option<f32> {
        self.values.lock(|v| v.get()[channel as usize])
    }

    fn set(&self, channel: Channel, voltage: f32) {
        self.values.lock(|v| {
            let mut values = v.get();
            values[channel as usize] = Some(voltage);
            v.set(values);
        });
    }

    fn clear(&self) {
        self.values.lock(|v| v.set([None; CHANNEL_COUNT]));
    }
}

impl Default for Voltages {
    fn default() -> Self {
        Self::new()
    }
}

/// Next step of the polling loop.
#[derive(Clone, Copy)]
enum Step {
    Reset,
    Select(Channel),
    Read,
}

/// An MCP3426 on an I2C bus.
pub struct Mcp3426<I> {
    i2c: I,
    address: u8,
}

impl<I: I2c> Mcp3426<I> {
    pub fn new(i2c: I, address: u8) -> Self {
        Mcp3426 { i2c, address }
    }

    /// Sends the C driver's initialization byte, `0x06`, to the ADC's own address. That's a configuration write,
    /// not a general call reset; [`Mcp3426::select`] overrides it.
    pub async fn reset(&mut self) -> Result<(), I2cError> {
        i2c_write(&mut self.i2c, self.address, &[RESET_CMD]).await
    }

    /// Starts continuous 16-bit, gain 1 conversions on `channel`.
    pub async fn select(&mut self, channel: Channel) -> Result<(), I2cError> {
        let config = mcp3426::config_byte(channel as u8, MODE, RATE, GAIN);
        i2c_write(&mut self.i2c, self.address, &[config]).await
    }

    /// Reads the latest conversion. Returns `None` if it isn't ready yet, otherwise the channel it belongs to and
    /// the voltage on that channel.
    pub async fn read(&mut self) -> Result<Option<(Channel, f32)>, I2cError> {
        let mut buf = [0u8; 3];
        i2c_read(&mut self.i2c, self.address, &mut buf).await?;
        Ok(mcp3426::parse_read(&buf)
            .map(|sample| (Channel::ALL[sample.channel as usize], mcp3426::code_to_voltage(sample.code))))
    }

    /// Resets the ADC and cycles through the channels forever, storing each voltage in `voltages`. See the module
    /// docs for when `on_error` is called.
    pub async fn run(&mut self, voltages: &Voltages, mut on_error: impl FnMut(I2cError)) -> ! {
        let mut consecutive_errors = ERROR_THRESHOLD - 1;
        let mut step = Step::Reset;
        loop {
            let result = match step {
                Step::Reset => self.reset().await.map(|()| Step::Select(Channel::Ch1)),
                Step::Select(channel) => self.select(channel).await.map(|()| Step::Read),
                Step::Read => match self.read().await {
                    Ok(Some((channel, voltage))) => {
                        consecutive_errors = 0;
                        voltages.set(channel, voltage);
                        Ok(Step::Select(channel.next()))
                    }
                    Ok(None) => {
                        consecutive_errors = 0;
                        Ok(Step::Read)
                    }
                    Err(err) => Err(err),
                },
            };

            match result {
                Ok(next) => step = next,
                Err(err) => {
                    defmt::warn!("MCP3426 transfer failed: {}", err);
                    consecutive_errors = consecutive_errors.saturating_add(1);
                    if consecutive_errors >= ERROR_THRESHOLD {
                        on_error(err);
                        voltages.clear();
                    }
                }
            }

            // The C driver waited before every read, and the chain stopped after an error; retry after the same
            // delay. Writes that succeeded go straight on to the next step.
            if matches!(step, Step::Read) || result.is_err() {
                Timer::after(READ_DELAY).await;
            }
        }
    }
}
