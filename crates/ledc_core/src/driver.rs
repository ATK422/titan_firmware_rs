//! Register access to the two L99LD20 controllers over a shared SPI bus, ported from `ledc_commands.c`
//! (`LEDBoard`) and `ledc_io.c` (`LEDBoard2`).
//!
//! Each controller has its own active low chip select. A transfer drops the chip select, waits 1 µs, exchanges one
//! frame, waits 1 µs and raises it again, as in C. Multi-step sequences that need millisecond pauses between frames
//! (unlocking and enabling) are left to the caller so it can sleep without blocking.

use core::convert::Infallible;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

use crate::l99ld20::*;

/// The two controllers of an LED board.
pub struct Ledc<SPI, CS, D> {
    spi: SPI,
    /// Chip selects of [`Chip::Ledc1`] and [`Chip::Ledc2`].
    cs: [CS; 2],
    delay: D,
}

impl<SPI: SpiBus<u8>, CS: OutputPin<Error = Infallible>, D: DelayNs> Ledc<SPI, CS, D> {
    /// `spi` must run in mode 0 (CPOL 0, CPHA 0), MSB first, 8 bit words; the C firmware used 4 MHz.
    pub fn new(spi: SPI, mut cs1: CS, mut cs2: CS, delay: D) -> Self {
        let Ok(()) = cs1.set_high();
        let Ok(()) = cs2.set_high();
        Ledc { spi, cs: [cs1, cs2], delay }
    }

    /// Exchanges one frame with `chip`.
    pub fn transfer(&mut self, chip: Chip, opcode: Opcode, address: u8, data: u32) -> Result<Response, SPI::Error> {
        let tx = encode_frame(opcode, address, data);
        let mut rx = [0u8; 4];
        let cs = &mut self.cs[chip as usize];
        let Ok(()) = cs.set_low();
        self.delay.delay_us(1);
        let result = self.spi.transfer(&mut rx, &tx).and_then(|()| self.spi.flush());
        self.delay.delay_us(1);
        let Ok(()) = cs.set_high();
        result.map(|()| Response::from_bytes(rx))
    }

    pub fn read(&mut self, chip: Chip, address: u8) -> Result<Response, SPI::Error> {
        self.transfer(chip, Opcode::Read, address, 0)
    }

    /// Reads a status register and clears its latched flags.
    pub fn read_clear(&mut self, chip: Chip, address: u8) -> Result<Response, SPI::Error> {
        self.transfer(chip, Opcode::ReadClear, address, 0)
    }

    /// Reads the device information ROM.
    pub fn read_rom(&mut self, chip: Chip, address: u8) -> Result<Response, SPI::Error> {
        self.transfer(chip, Opcode::ReadRom, address, 0)
    }

    /// Writes a register. Bit 0 of `data` is replaced by the frame's parity bit.
    pub fn write(&mut self, chip: Chip, address: u8, data: u32) -> Result<Response, SPI::Error> {
        self.transfer(chip, Opcode::Write, address, data)
    }

    /// Reads a register, lets `f` change it and writes it back.
    pub fn modify(&mut self, chip: Chip, address: u8, f: impl FnOnce(u32) -> u32) -> Result<Response, SPI::Error> {
        let current = self.read(chip, address)?.data;
        self.write(chip, address, f(current))
    }

    /// Sets the PWM duties of both bucks of `chip` (`buck_set_brightness`, one read-modify-write per chip).
    pub fn set_duties(&mut self, chip: Chip, duties: [u16; 2]) -> Result<(), SPI::Error> {
        self.modify(chip, CR1, |cr1| Cr1(cr1).with_duty(Buck::Buck1, duties[0]).with_duty(Buck::Buck2, duties[1]).0)?;
        Ok(())
    }

    /// Chooses what controls a buck (`buck_set_control_mode`).
    pub fn set_din_map(&mut self, chip: Chip, buck: Buck, map: DinMap) -> Result<(), SPI::Error> {
        self.modify(chip, CR3, |cr3| Cr3(cr3).with_din_map(buck, map).0)?;
        Ok(())
    }

    /// Sets a buck's peak inductor current select (`buck_set_peak_current`).
    pub fn set_peak_current(&mut self, chip: Chip, buck: Buck, current: u8) -> Result<(), SPI::Error> {
        self.modify(chip, CR2, |cr2| Cr2(cr2).with_peak_current(buck, current).0)?;
        Ok(())
    }

    /// Toggles the watchdog trigger, preserving the rest of CR4 (`pet_watchdog` in `LEDBoard2`).
    pub fn toggle_watchdog(&mut self, chip: Chip) -> Result<(), SPI::Error> {
        self.modify(chip, CR4, |cr4| cr4 ^ CR4_WATCHDOG_TRIGGER)?;
        Ok(())
    }

    /// Reads back the state reported by the `leds` debug command.
    pub fn status(&mut self, chip: Chip) -> Result<(GlobalStatus, ControllerStatus), SPI::Error> {
        let cr1 = self.read(chip, CR1)?;
        let cr2 = self.read(chip, CR2)?;
        let sr2 = self.read(chip, SR2)?;
        Ok((sr2.global_status, ControllerStatus::new(Cr1(cr1.data), Cr2(cr2.data), Sr2(sr2.data))))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    //! Runs the driver against a simulated pair of controllers.

    extern crate std;

    use super::*;
    use core::cell::RefCell;
    use embedded_hal::digital::ErrorType as PinErrorType;
    use embedded_hal::spi::ErrorType as SpiErrorType;
    use std::vec::Vec;

    /// Register file and chip select state of both controllers, plus a log of every frame.
    pub struct Bus {
        pub regs: [[u32; 64]; 2],
        pub selected: [bool; 2],
        pub frames: Vec<(usize, [u8; 4])>,
        pub global_status: u8,
    }

    impl Default for Bus {
        fn default() -> Self {
            Bus { regs: [[0; 64]; 2], selected: [false; 2], frames: Vec::new(), global_status: 0 }
        }
    }

    pub struct Spi<'a>(pub &'a RefCell<Bus>);
    pub struct Cs<'a>(pub &'a RefCell<Bus>, pub usize);
    pub struct NoDelay;

    impl SpiErrorType for Spi<'_> {
        type Error = Infallible;
    }

    impl SpiBus<u8> for Spi<'_> {
        fn read(&mut self, _words: &mut [u8]) -> Result<(), Infallible> {
            unimplemented!()
        }

        fn write(&mut self, _words: &[u8]) -> Result<(), Infallible> {
            unimplemented!()
        }

        fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Infallible> {
            let mut bus = self.0.borrow_mut();
            let chip = match bus.selected {
                [true, false] => 0,
                [false, true] => 1,
                other => panic!("chip selects {other:?} during transfer"),
            };
            let frame: [u8; 4] = write.try_into().expect("4 byte frames");
            assert!(has_odd_parity(frame), "bad parity in {frame:02x?}");
            bus.frames.push((chip, frame));
            let word = u32::from_be_bytes(frame);
            let address = (word >> 24 & 0x3F) as usize;
            let data = word & DATA_MASK;
            let value = bus.regs[chip][address];
            match word >> 30 {
                0 => bus.regs[chip][address] = data,
                2 => bus.regs[chip][address] = 0,
                _ => {}
            }
            let answer = (bus.global_status as u32) << 24 | value;
            read.copy_from_slice(&answer.to_be_bytes());
            Ok(())
        }

        fn transfer_in_place(&mut self, _words: &mut [u8]) -> Result<(), Infallible> {
            unimplemented!()
        }

        fn flush(&mut self) -> Result<(), Infallible> {
            Ok(())
        }
    }

    impl PinErrorType for Cs<'_> {
        type Error = Infallible;
    }

    impl OutputPin for Cs<'_> {
        fn set_low(&mut self) -> Result<(), Infallible> {
            self.0.borrow_mut().selected[self.1] = true;
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Infallible> {
            self.0.borrow_mut().selected[self.1] = false;
            Ok(())
        }
    }

    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    pub fn ledc(bus: &RefCell<Bus>) -> Ledc<Spi<'_>, Cs<'_>, NoDelay> {
        Ledc::new(Spi(bus), Cs(bus, 0), Cs(bus, 1), NoDelay)
    }

    #[test]
    fn transfers_select_one_chip_and_decode_answers() {
        let bus = RefCell::new(Bus { global_status: 0x02, ..Default::default() });
        bus.borrow_mut().regs[1][CR3 as usize] = 0xAB_CDEF;
        let mut ledc = ledc(&bus);
        let r = ledc.read(Chip::Ledc2, CR3).unwrap();
        assert_eq!(r.data, 0xAB_CDEF);
        assert!(r.global_status.global_warning());
        assert_eq!(bus.borrow().selected, [false, false]);
        // 0x43 already has three ones, so no parity bit
        assert_eq!(bus.borrow().frames, [(1, [0x43, 0, 0, 0])]);
        // A read of CR1 needs one (the C LEDBoard sent its reads without it)
        ledc.read(Chip::Ledc1, CR1).unwrap();
        assert_eq!(bus.borrow().frames[1], (0, [0x41, 0, 0, 1]));
    }

    #[test]
    fn read_modify_write_helpers() {
        let bus = RefCell::new(Bus::default());
        bus.borrow_mut().regs[0][CR1 as usize] = 0b1110;
        bus.borrow_mut().regs[0][CR2 as usize] = 0x00_0F04;
        let mut ledc = ledc(&bus);
        ledc.set_duties(Chip::Ledc1, [0x3FF, 0x100]).unwrap();
        ledc.set_peak_current(Chip::Ledc1, Buck::Buck1, 45).unwrap();
        ledc.set_din_map(Chip::Ledc1, Buck::Buck2, DinMap::PwmDimming).unwrap();
        ledc.toggle_watchdog(Chip::Ledc1).unwrap();
        let regs = bus.borrow().regs[0];
        // The parity bit is replaced; the other CR1 bits are kept
        assert_eq!(regs[CR1 as usize] & !1, 0x3FF << 14 | 0x100 << 4 | 0b1110);
        assert_eq!(regs[CR2 as usize] & !1, 45 << 18 | 0x0F04);
        assert_eq!(regs[CR3 as usize] & !1, 0x1000);
        assert_eq!(regs[CR4 as usize] & CR4_WATCHDOG_TRIGGER, CR4_WATCHDOG_TRIGGER);
        ledc.toggle_watchdog(Chip::Ledc1).unwrap();
        assert_eq!(bus.borrow().regs[0][CR4 as usize] & CR4_WATCHDOG_TRIGGER, 0);
        // Controller 2 was never touched
        assert!(bus.borrow().frames.iter().all(|&(chip, _)| chip == 0));
    }

    #[test]
    fn status_and_read_clear() {
        let bus = RefCell::new(Bus { global_status: 0x01, ..Default::default() });
        bus.borrow_mut().regs[1][CR1 as usize] = 0b10;
        bus.borrow_mut().regs[1][CR2 as usize] = 0b1000;
        bus.borrow_mut().regs[1][SR2 as usize] = 1 << 15;
        let mut ledc = ledc(&bus);
        let (gs, status) = ledc.status(Chip::Ledc2).unwrap();
        assert!(gs.fail_safe());
        assert_eq!(status, ControllerStatus { unlocked: true, standby: true, enabled: false, watchdog_quarters: 1 });
        assert_eq!(ledc.read_clear(Chip::Ledc2, SR2).unwrap().data, 1 << 15);
        assert_eq!(bus.borrow().regs[1][SR2 as usize], 0);
    }
}
