//! The bus scheduler and request/reply layer, ported from `dynamixel_schedule.c`, `dynamixel_comms.c` and the
//! public API in `dynamixel.c`.
//!
//! The C scheduler chained callbacks under a transfer lock: queued commands first, then the periodic refresh, which
//! paused whenever a command was queued. [`Driver::run`] is the same loop written as straight-line async code.

use core::cell::{Cell, RefCell};

use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Instant, Timer};
use heapless::Vec;

use dynamixel_protocol::control_table::{self, addr};
use dynamixel_protocol::{Eeprom, Packet, Ram, packet, parse_exact};

use crate::uart::{HalfDuplexUart, PioRegs};
use crate::{
    Error, ErrorKind, Event, Id, MAX_COMMANDS, MAX_MISSED_PINGS, MAX_WRITE_LEN, PACKET_BUFFER_SIZE, REFRESH_INTERVAL,
    REPLY_TIMEOUT, Source,
};

/// Bytes of a reply up to and including the instruction byte (`DYNAMIXEL_REPLY_HEADER_SIZE`).
const REPLY_HEADER_LEN: usize = 8;
/// Transmit buffer, and the most a reply may have after its header (`uart_buf`: parameters plus error and CRC).
const UART_BUF_LEN: usize = PACKET_BUFFER_SIZE + 3;

/// Tracked state of one servo (`struct dynamixel_state`).
#[derive(Clone, Copy)]
struct ServoState {
    id: Id,
    eeprom: Eeprom,
    ram: Ram,
    missed_pings: u8,
    connected: bool,
    alert_notified: bool,
    first_connect_attempted: bool,
}

impl ServoState {
    const fn new(id: Id) -> Self {
        ServoState {
            id,
            eeprom: EEPROM_ZERO,
            ram: RAM_ZERO,
            missed_pings: 0,
            connected: false,
            alert_notified: false,
            first_connect_attempted: false,
        }
    }
}

// `Default` isn't const
const EEPROM_ZERO: Eeprom = Eeprom {
    model_num: 0,
    model_info: 0,
    firmware_version: 0,
    id: 0,
    baud_rate: 0,
    return_delay_time: 0,
    drive_mode: 0,
    operating_mode: 0,
    secondary_id: 0,
    protocol_type: 0,
    homing_offset: 0,
    moving_threshold: 0,
    temperature_limit: 0,
    max_voltage_limit: 0,
    min_voltage_limit: 0,
    pwm_limit: 0,
    velocity_limit: 0,
    max_position_limit: 0,
    min_position_limit: 0,
    startup_config: 0,
    shutdown: 0,
};

const RAM_ZERO: Ram = Ram {
    torque_enable: false,
    led: false,
    status_return_level: 0,
    registered_instruction: false,
    hardware_error_status: 0,
    velocity_i_gain: 0,
    velocity_p_gain: 0,
    position_d_gain: 0,
    position_i_gain: 0,
    position_p_gain: 0,
    feedforward_2nd_gain: 0,
    feedforward_1st_gain: 0,
    bus_watchdog: 0,
    goal_pwm: 0,
    goal_velocity: 0,
    profile_accel: 0,
    profile_vel: 0,
    goal_position: 0,
    realtime_tick: 0,
    moving: false,
    moving_status: 0,
    present_pwm: 0,
    present_load: 0,
    present_velocity: 0,
    present_position: 0,
    velocity_trajectory: 0,
    position_trajectory: 0,
    present_input_voltage: 0,
    present_temperature: 0,
    backup_ready: false,
};

/// A queued request.
enum Command {
    Write { id: Id, address: u16, data: Vec<u8, MAX_WRITE_LEN> },
    ReadEeprom { id: Id },
}

/// The servos on the bus and the command queue, shared between the [`Driver`] and the rest of the firmware.
pub struct Servos<const N: usize> {
    states: Mutex<CriticalSectionRawMutex, RefCell<[ServoState; N]>>,
    queue: Channel<CriticalSectionRawMutex, Command, MAX_COMMANDS>,
    /// An error from a command method, reported by the driver task on its next pass.
    caller_error: Mutex<CriticalSectionRawMutex, Cell<Option<Error>>>,
}

impl<const N: usize> Servos<N> {
    /// Tracks the servos with these IDs.
    pub const fn new(ids: [Id; N]) -> Self {
        let mut states = [ServoState::new(0); N];
        let mut i = 0;
        while i < N {
            states[i] = ServoState::new(ids[i]);
            i += 1;
        }
        Servos {
            states: Mutex::new(RefCell::new(states)),
            queue: Channel::new(),
            caller_error: Mutex::new(Cell::new(None)),
        }
    }

    fn with<R>(&self, id: Id, f: impl FnOnce(&mut ServoState) -> R) -> Option<R> {
        self.states.lock(|states| states.borrow_mut().iter_mut().find(|s| s.id == id).map(f))
    }

    fn with_index<R>(&self, index: usize, f: impl FnOnce(&mut ServoState) -> R) -> R {
        self.states.lock(|states| f(&mut states.borrow_mut()[index]))
    }

    /// True if the servo is connected, so its RAM is current (`dynamixel_check_connected`).
    pub fn is_connected(&self, id: Id) -> bool {
        self.with(id, |s| s.connected).unwrap_or(false)
    }

    /// The servo's EEPROM as last read, or `None` if it isn't tracked or isn't connected (`dynamixel_get_eeprom`).
    /// It's read on connect and by [`Servos::request_eeprom_rescan`].
    pub fn eeprom(&self, id: Id) -> Option<Eeprom> {
        self.with(id, |s| s.connected.then_some(s.eeprom)).flatten()
    }

    /// The servo's RAM from the latest refresh, or `None` if it isn't tracked or isn't connected
    /// (`dynamixel_get_ram`).
    pub fn ram(&self, id: Id) -> Option<Ram> {
        self.with(id, |s| s.connected.then_some(s.ram)).flatten()
    }

    /// The servo's present position (`dynamixel_get_position`).
    pub fn position(&self, id: Id) -> Option<i32> {
        self.ram(id).map(|ram| ram.present_position)
    }

    /// The tracked IDs, in polling order.
    pub fn ids(&self) -> [Id; N] {
        self.states.lock(|states| core::array::from_fn(|i| states.borrow()[i].id))
    }

    fn report(&self, error: Error) {
        self.caller_error.lock(|e| e.set(Some(error)));
    }

    fn enqueue(&self, command: Command) -> bool {
        if self.queue.try_send(command).is_err() {
            self.report(Error::new(ErrorKind::CommandQueueFull, Source::Schedule, 0));
            return false;
        }
        true
    }

    /// Queues a write of `data` to the control table at `address` (`dynamixel_schedule_write_packet`). Returns
    /// false if the queue is full (the error callback also hears about it).
    pub fn write(&self, id: Id, address: u16, data: &[u8]) -> bool {
        let Ok(data) = Vec::from_slice(data) else {
            self.report(Error::new(ErrorKind::Request, Source::Schedule, 0));
            return false;
        };
        self.enqueue(Command::Write { id, address, data })
    }

    /// Changes a servo's ID (`dynamixel_set_id`).
    pub fn set_id(&self, old: Id, new: Id) -> bool {
        self.write(old, addr::ID, &[new])
    }

    /// Enables or disables torque (`dynamixel_enable_torque`). While torque is on the servo holds its goal and its
    /// EEPROM is locked.
    pub fn enable_torque(&self, id: Id, enabled: bool) -> bool {
        self.write(id, addr::TORQUE_ENABLE, &[enabled as u8])
    }

    /// Sets the goal position (`dynamixel_set_target_position`).
    pub fn set_target_position(&self, id: Id, position: i32) -> bool {
        self.write(id, addr::GOAL_POSITION, &position.to_le_bytes())
    }

    /// Sets the homing offset (`dynamixel_set_homing_offset`). The tracked EEPROM isn't updated until
    /// [`Servos::request_eeprom_rescan`].
    pub fn set_homing_offset(&self, id: Id, offset: i32) -> bool {
        self.write(id, addr::HOMING_OFFSET, &offset.to_le_bytes())
    }

    /// Queues an EEPROM read; [`Event::EepromRead`] follows when it succeeds (`dynamixel_request_eeprom_rescan`).
    pub fn request_eeprom_rescan(&self, id: Id) -> bool {
        self.enqueue(Command::ReadEeprom { id })
    }
}

/// A request to send.
#[derive(Clone, Copy)]
enum Request<'a> {
    Ping,
    Read { address: u16, length: u16 },
    Write { address: u16, data: &'a [u8] },
}

const READ_EEPROM: Request<'static> =
    Request::Read { address: control_table::EEPROM_START, length: control_table::EEPROM_LENGTH };
const READ_RAM: Request<'static> =
    Request::Read { address: control_table::RAM_START, length: control_table::RAM_LENGTH };

/// What to do after one refresh step.
enum Step {
    /// Move on to the next servo.
    Next,
    /// A command was queued while the reply was in flight; run it, then refresh the same servo again.
    Retry,
}

/// Owns the bus. Run it in a task with [`Driver::run`].
pub struct Driver<'d, PIO: PioRegs, const SM: usize, const TX_IRQ: usize, const ERR_IRQ: usize, const N: usize> {
    uart: HalfDuplexUart<'d, PIO, SM, TX_IRQ, ERR_IRQ>,
    servos: &'static Servos<N>,
    tx_buf: [u8; UART_BUF_LEN],
    rx_buf: [u8; REPLY_HEADER_LEN + UART_BUF_LEN],
    params: [u8; PACKET_BUFFER_SIZE],
}

impl<'d, PIO: PioRegs, const SM: usize, const TX_IRQ: usize, const ERR_IRQ: usize, const N: usize>
    Driver<'d, PIO, SM, TX_IRQ, ERR_IRQ, N>
{
    pub fn new(uart: HalfDuplexUart<'d, PIO, SM, TX_IRQ, ERR_IRQ>, servos: &'static Servos<N>) -> Self {
        Driver {
            uart,
            servos,
            tx_buf: [0; UART_BUF_LEN],
            rx_buf: [0; REPLY_HEADER_LEN + UART_BUF_LEN],
            params: [0; PACKET_BUFFER_SIZE],
        }
    }

    /// Runs the bus forever. `on_event` and `on_error` play the part of the C `event_cb` and `error_cb`; they run in
    /// this task, so keep them short.
    pub async fn run(mut self, mut on_event: impl FnMut(Event, Id), mut on_error: impl FnMut(Error)) -> ! {
        // Give the line time to settle (dynamixel_init)
        Timer::after_millis(1).await;

        // The first refresh starts right away; later ones every REFRESH_INTERVAL, start to start
        let mut next_refresh = Instant::now() + REFRESH_INTERVAL;
        let mut refreshing: Option<usize> = Some(0);

        loop {
            if let Some(error) = self.servos.caller_error.lock(|e| e.take()) {
                defmt::debug!("Dynamixel command rejected: {}", error);
                on_error(error);
            }

            // Pending commands come first: they may change the state a refresh would read
            if let Ok(command) = self.servos.queue.try_receive() {
                self.execute(command, &mut on_event, &mut on_error).await;
                continue;
            }

            if let Some(index) = refreshing {
                if Instant::now() >= next_refresh {
                    // refresh_timer_cb fired before the previous refresh finished
                    defmt::debug!("Dynamixel refresh overran its interval");
                    on_error(Error::new(ErrorKind::Driver, Source::Schedule, 0));
                    next_refresh += REFRESH_INTERVAL;
                }
                if index >= N {
                    refreshing = None;
                    continue;
                }
                match self.refresh(index, &mut on_event, &mut on_error).await {
                    Step::Next => {
                        self.refresh_done(index, &mut on_event);
                        refreshing = Some(index + 1);
                    }
                    Step::Retry => {}
                }
                continue;
            }

            match select(self.servos.queue.receive(), Timer::at(next_refresh)).await {
                Either::First(command) => self.execute(command, &mut on_event, &mut on_error).await,
                Either::Second(()) => {
                    next_refresh += REFRESH_INTERVAL;
                    // Don't try to catch up on missed rounds
                    if next_refresh <= Instant::now() {
                        next_refresh = Instant::now() + REFRESH_INTERVAL;
                    }
                    refreshing = Some(0);
                }
            }
        }
    }

    /// Sends one request and waits for the reply (`dynamixel_send_packet` and its receive callbacks).
    async fn transfer(&mut self, len: usize, id: Id) -> Result<Packet<'_>, Error> {
        let comms = |kind, wrapped| Error::new(kind, Source::Comms, wrapped);

        self.uart.write(&self.tx_buf[..len]).await.map_err(|e| comms(ErrorKind::Ttl, e as u8))?;
        let (header, body) = self.rx_buf.split_at_mut(REPLY_HEADER_LEN);
        self.uart.read(header, REPLY_TIMEOUT).await.map_err(|e| comms(ErrorKind::Ttl, e as u8))?;

        // The length counts the instruction byte, which is already in the header
        let body_len = u16::from_le_bytes([header[5], header[6]]) as usize;
        if !(2..=UART_BUF_LEN + 1).contains(&body_len) {
            return Err(comms(ErrorKind::Packet, 0));
        }
        self.uart.read(&mut body[..body_len - 1], REPLY_TIMEOUT).await.map_err(|e| comms(ErrorKind::Ttl, e as u8))?;

        let reply = &self.rx_buf[..REPLY_HEADER_LEN + body_len - 1];
        let packet = parse_exact(reply, &mut self.params).map_err(|e| comms(ErrorKind::Packet, e.lib_code()))?;
        if packet.id != id {
            return Err(comms(ErrorKind::InvalidId, packet.id));
        }
        if packet.instruction_error() != 0 {
            return Err(comms(ErrorKind::Response, packet.instruction_error()));
        }
        Ok(packet)
    }

    /// Builds a request into the transmit buffer, then runs it. Build failures are driver errors when the scheduler
    /// made the request and request errors when a command did, as in C.
    async fn request(&mut self, id: Id, request: Request<'_>, source: Source) -> Result<Packet<'_>, Error> {
        let buf = &mut self.tx_buf;
        let built = match request {
            Request::Ping => packet::ping(buf, id),
            Request::Read { address, length } => packet::read(buf, id, address, length),
            Request::Write { address, data } => packet::write(buf, id, address, data),
        };
        let len = match built {
            Ok(packet) => packet.len(),
            Err(_) => {
                let kind = if source == Source::Schedule { ErrorKind::Driver } else { ErrorKind::Request };
                return Err(Error::new(kind, source, 0));
            }
        };
        self.transfer(len, id).await
    }

    async fn execute(
        &mut self,
        command: Command,
        on_event: &mut impl FnMut(Event, Id),
        on_error: &mut impl FnMut(Error),
    ) {
        match command {
            Command::Write { id, address, data } => {
                // Writes have no feedback beyond errors; the periodic refresh shows their effect (write_cmd_cb)
                let result = self.request(id, Request::Write { address, data: &data }, Source::Comms).await;
                if let Err(error) = result {
                    defmt::debug!("Error executing queued write to {}: {}", id, error);
                    on_error(error);
                }
            }
            Command::ReadEeprom { id } => {
                let result =
                    self.request(id, READ_EEPROM, Source::Comms).await.map(|packet| Eeprom::decode(packet.params));
                match result {
                    Ok(eeprom) => {
                        if self.servos.with(id, |s| s.eeprom = eeprom).is_some() {
                            on_event(Event::EepromRead, id);
                        } else {
                            defmt::debug!("EEPROM read for untracked servo {}", id);
                            on_error(Error::new(ErrorKind::Request, Source::Schedule, 0));
                        }
                    }
                    Err(error) => {
                        defmt::debug!("Error executing queued EEPROM read of {}: {}", id, error);
                        on_error(error);
                    }
                }
            }
        }
    }

    /// One step of the refresh for the servo at `index` (`trigger_next_refresh` and the callbacks it chains).
    async fn refresh(
        &mut self,
        index: usize,
        on_event: &mut impl FnMut(Event, Id),
        on_error: &mut impl FnMut(Error),
    ) -> Step {
        let (id, connected) = self.servos.with_index(index, |s| (s.id, s.connected));

        if connected {
            // periodic_ram_read_cb
            let result = self.request(id, READ_RAM, Source::Schedule).await.map(|p| (Ram::decode(p.params), p.alert()));
            let (ram, alert) = match result {
                Ok(read) => read,
                Err(error) if error.kind == ErrorKind::Driver => {
                    on_error(error);
                    return Step::Next;
                }
                Err(error) => {
                    defmt::debug!("[ID {}] Error during refresh: {}", id, error);
                    let disconnected = self.servos.with_index(index, |s| {
                        s.missed_pings = s.missed_pings.saturating_add(1);
                        if s.missed_pings >= MAX_MISSED_PINGS {
                            s.connected = false;
                        }
                        !s.connected
                    });
                    if disconnected {
                        defmt::debug!("Dynamixel disconnected - ID {}", id);
                        on_event(Event::Disconnected, id);
                    }
                    return Step::Next;
                }
            };

            // A command queued while the read was in flight may change what was read: drop the data, run the
            // command, then read again
            if !self.servos.queue.is_empty() {
                return Step::Retry;
            }

            let notify_alert = self.servos.with_index(index, |s| {
                s.missed_pings = 0;
                s.ram = ram;
                let notify = alert && !s.alert_notified;
                s.alert_notified = alert;
                notify
            });
            on_event(Event::RamRead, id);
            if notify_alert {
                on_event(Event::Alert, id);
            }
            return Step::Next;
        }

        // Not connected: ping, check the model, read EEPROM and RAM, then disable torque (connect_ping_cb and the
        // initial_* callbacks)
        match self.connect(id).await {
            Ok((eeprom, ram)) => {
                self.servos.with_index(index, |s| {
                    s.eeprom = eeprom;
                    s.ram = ram;
                    s.missed_pings = 0;
                    s.connected = true;
                });
                defmt::debug!("New Dynamixel connected - ID {}", id);
                on_event(Event::Connected, id);
            }
            Err(Some(error)) if error.kind == ErrorKind::Driver => on_error(error),
            Err(Some(error)) => defmt::debug!("[ID {}] Error connecting: {}", id, error),
            Err(None) => {}
        }
        Step::Next
    }

    /// Tries to bring up a servo. `Err(None)` means it answered with an unexpected model number.
    async fn connect(&mut self, id: Id) -> Result<(Eeprom, Ram), Option<Error>> {
        let ping = self.request(id, Request::Ping, Source::Schedule).await?;
        let model = match ping.params {
            [lo, hi, ..] => u16::from_le_bytes([*lo, *hi]),
            _ => return Err(Some(Error::new(ErrorKind::Packet, Source::Schedule, 0))),
        };
        if model != control_table::XL430_MODEL_NUMBER {
            defmt::debug!("Invalid model {}, expected {}", model, control_table::XL430_MODEL_NUMBER);
            return Err(None);
        }

        let eeprom = Eeprom::decode(self.request(id, READ_EEPROM, Source::Schedule).await?.params);
        let ram = Ram::decode(self.request(id, READ_RAM, Source::Schedule).await?.params);

        // Disable torque in case the servo was disconnected but not power cycled
        let torque_off = Request::Write { address: addr::TORQUE_ENABLE, data: &[0] };
        self.request(id, torque_off, Source::Schedule).await?;
        Ok((eeprom, ram))
    }

    /// Finishes a servo's refresh (`handle_refresh_transfer_done`): a servo that didn't answer the very first attempt
    /// is reported disconnected once, so the board knows it's missing.
    fn refresh_done(&self, index: usize, on_event: &mut impl FnMut(Event, Id)) {
        let (id, report) = self.servos.with_index(index, |s| {
            let first = !s.first_connect_attempted;
            s.first_connect_attempted = true;
            (s.id, first && !s.connected)
        });
        if report {
            on_event(Event::Disconnected, id);
        }
    }
}
