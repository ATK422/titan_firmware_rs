//! Actuator board (`talos/Actuator`): node `actuator_mk2`.
//!
//! Topics and services of the C `ros.c`, `ros_actuators.c`, `actuator_v2_dynamixel.c` and
//! `actuator_v1_parameters.c`:
//!
//! | Name | Type | |
//! |---|---|---|
//! | `state/actuator/status` | `riptide_msgs2/ActuatorStatus` | published, best effort |
//! | `state/actuator/busy` | `std_msgs/Bool` | published, best effort |
//! | `state/actuator/dynamixel` | `riptide_msgs2/DynamixelStatus` | published, best effort (V2) |
//! | `command/actuator/{torpedo,dropper,notify_reload}` | `std_srvs/Trigger` | service |
//! | `command/actuator/{claw,arm}` | `std_srvs/SetBool` | service |
//! | `command/actuator/torpedo_marker/{go_home,set_home}`, `command/actuator/claw/set_closed_pos` | `std_srvs/Trigger` | service (V2) |
//! | `claw_timing_ms`, `dropper_active_timing_ms`, `torpedo_coil*_timing_us` | integer | parameters (V1) |
//!
//! # Services
//!
//! rclrs 0.8 has dynamic publishers and subscriptions but no dynamic services, so [`std_srvs`] binds the two
//! service types by hand: `#[repr(C)]` copies of the C message structs, implementing the `rosidl_runtime_rs`
//! traits, with the type support taken from `libstd_srvs__rosidl_typesupport_c` (in every ROS install, found
//! through the link paths rclrs already sets up). Each call is forwarded to the board as a numbered
//! [`Command`](actuator::Command) and answered when the board's result with the same number comes back, or with a
//! failure after [`COMMAND_TIMEOUT`].
//!
//! # Variants
//!
//! The board reports whether it runs the V1 or V2 actuators in its status. The variant specific services, topic and
//! parameters are created the first time the board says which it is, so the node shows the same interface as the C
//! build for that variant.

use std::collections::HashMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::task::{Context, Poll, Waker};
use std::thread;
use std::time::Duration;

use anyhow::{Result, anyhow, bail};
use canmore::client_ids;
use rclrs::{
    DynamicMessage, IntoPrimitiveOptions, MandatoryParameter, Node, ParameterRange, Service, SimpleValueMut, ValueMut,
};
use titan_link::actuator::{
    self, ActuatorError, ClawState, Command, DropperState, TorpedoState, TorpedoTiming, Variant,
};
use titan_link::{DeviceToHost, HostToDevice};

use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros::{self, Publisher};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "actuator",
    node_name: "actuator_mk2",
    namespace: "talos",
    client_id: client_ids::CLIENT_ID_ACTUATOR_BOARD,
    transport: Transport::Can { fd: true },
    create: |node, link, logger| Ok(Box::new(ActuatorTopics::new(node, link, logger)?)),
};

/// How long a service call waits for the board's answer.
pub const COMMAND_TIMEOUT: Duration = Duration::from_millis(500);

/// Hand written bindings for the `std_srvs` services the board offers.
pub mod std_srvs {
    use std::borrow::Cow;
    use std::ffi::c_void;

    use rclrs::rosidl_runtime_rs::{self as rosidl, Message, RmwMessage, Service};

    #[link(name = "std_srvs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_service_type_support_handle__std_srvs__srv__Trigger() -> *const c_void;
        fn rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__Trigger_Request() -> *const c_void;
        fn rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__Trigger_Response() -> *const c_void;
        fn rosidl_typesupport_c__get_service_type_support_handle__std_srvs__srv__SetBool() -> *const c_void;
        fn rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__SetBool_Request() -> *const c_void;
        fn rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__SetBool_Response() -> *const c_void;
    }

    /// Implements the message traits for a `#[repr(C)]` struct that is its own RMW representation.
    macro_rules! rmw_message {
        ($ty:ty, $name:literal, $support:ident) => {
            impl Message for $ty {
                type RmwMsg = Self;
                fn into_rmw_message(msg: Cow<'_, Self>) -> Cow<'_, Self> {
                    msg
                }
                fn from_rmw_message(msg: Self) -> Self {
                    msg
                }
            }

            impl RmwMessage for $ty {
                const TYPE_NAME: &'static str = $name;
                fn get_type_support() -> *const c_void {
                    // SAFETY: the type support getters have no preconditions
                    unsafe { $support() }
                }
            }
        };
    }

    /// `std_srvs/srv/Trigger_Request` (`std_srvs__srv__Trigger_Request`).
    #[repr(C)]
    #[derive(Clone, Debug, Default)]
    pub struct TriggerRequest {
        pub structure_needs_at_least_one_member: u8,
    }

    /// `std_srvs/srv/Trigger_Response` (`std_srvs__srv__Trigger_Response`).
    #[repr(C)]
    #[derive(Clone, Debug, Default)]
    pub struct TriggerResponse {
        pub success: bool,
        pub message: rosidl::String,
    }

    /// `std_srvs/srv/SetBool_Request` (`std_srvs__srv__SetBool_Request`).
    #[repr(C)]
    #[derive(Clone, Debug, Default)]
    pub struct SetBoolRequest {
        pub data: bool,
    }

    /// `std_srvs/srv/SetBool_Response` (`std_srvs__srv__SetBool_Response`).
    #[repr(C)]
    #[derive(Clone, Debug, Default)]
    pub struct SetBoolResponse {
        pub success: bool,
        pub message: rosidl::String,
    }

    rmw_message!(
        TriggerRequest,
        "std_srvs/srv/Trigger_Request",
        rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__Trigger_Request
    );
    rmw_message!(
        TriggerResponse,
        "std_srvs/srv/Trigger_Response",
        rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__Trigger_Response
    );
    rmw_message!(
        SetBoolRequest,
        "std_srvs/srv/SetBool_Request",
        rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__SetBool_Request
    );
    rmw_message!(
        SetBoolResponse,
        "std_srvs/srv/SetBool_Response",
        rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__SetBool_Response
    );

    /// `std_srvs/srv/Trigger`.
    pub struct Trigger;

    impl Service for Trigger {
        type Request = TriggerRequest;
        type Response = TriggerResponse;
        fn get_type_support() -> *const c_void {
            // SAFETY: no preconditions
            unsafe { rosidl_typesupport_c__get_service_type_support_handle__std_srvs__srv__Trigger() }
        }
    }

    /// `std_srvs/srv/SetBool`.
    pub struct SetBool;

    impl Service for SetBool {
        type Request = SetBoolRequest;
        type Response = SetBoolResponse;
        fn get_type_support() -> *const c_void {
            // SAFETY: no preconditions
            unsafe { rosidl_typesupport_c__get_service_type_support_handle__std_srvs__srv__SetBool() }
        }
    }
}

use std_srvs::{SetBool, SetBoolRequest, SetBoolResponse, Trigger, TriggerRequest, TriggerResponse};

// ========================================
// Command round trips
// ========================================

/// How a command ended, from the bridge's point of view.
type Outcome = Result<(), String>;

/// Where the answer to one command lands.
#[derive(Default)]
struct Slot {
    state: Mutex<(Option<Outcome>, Option<Waker>)>,
    done: Condvar,
}

impl Slot {
    /// Stores the outcome unless one is already there, and wakes whoever waits.
    fn complete(&self, outcome: Outcome) {
        let mut state = self.state.lock().unwrap();
        if state.0.is_some() {
            return;
        }
        state.0 = Some(outcome);
        if let Some(waker) = state.1.take() {
            waker.wake();
        }
        self.done.notify_all();
    }

    /// Blocks until the outcome arrives.
    fn wait(&self) -> Outcome {
        let mut state = self.state.lock().unwrap();
        loop {
            if let Some(outcome) = state.0.take() {
                return outcome;
            }
            state = self.done.wait(state).unwrap();
        }
    }
}

/// Resolves to a command's outcome.
struct Reply(Arc<Slot>);

impl Future for Reply {
    type Output = Outcome;
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Outcome> {
        let mut state = self.0.state.lock().unwrap();
        match state.0.take() {
            Some(outcome) => Poll::Ready(outcome),
            None => {
                state.1 = Some(cx.waker().clone());
                Poll::Pending
            }
        }
    }
}

/// Sends commands to the board and matches up the answers.
struct Commander {
    link: Arc<dyn Link>,
    pending: Mutex<(u8, HashMap<u8, Arc<Slot>>)>,
}

impl Commander {
    /// Sends `command`; the returned slot completes with the board's answer, an error or a timeout.
    fn send(self: &Arc<Self>, command: Command) -> Arc<Slot> {
        let slot = Arc::new(Slot::default());
        let seq = {
            let mut pending = self.pending.lock().unwrap();
            let (next, map) = &mut *pending;
            if map.len() >= 256 {
                slot.complete(Err("too many commands in flight".into()));
                return slot;
            }
            while map.contains_key(next) {
                *next = next.wrapping_add(1);
            }
            let seq = *next;
            *next = next.wrapping_add(1);
            map.insert(seq, slot.clone());
            seq
        };

        if let Err(err) = self.link.send(&HostToDevice::Actuator(actuator::HostToDevice::Command { seq, command })) {
            self.pending.lock().unwrap().1.remove(&seq);
            slot.complete(Err(format!("Sending to the actuator board failed: {err:#}")));
            return slot;
        }

        let this = self.clone();
        let timeout_slot = slot.clone();
        thread::spawn(move || {
            thread::sleep(COMMAND_TIMEOUT);
            if this.pending.lock().unwrap().1.remove(&seq).is_some() {
                timeout_slot.complete(Err("Timed out waiting for the actuator board".into()));
            }
        });
        slot
    }

    /// Delivers the board's answer to command `seq`. Returns false if nobody is waiting (it timed out).
    fn answer(&self, seq: u8, result: Result<(), ActuatorError>) -> bool {
        let Some(slot) = self.pending.lock().unwrap().1.remove(&seq) else { return false };
        slot.complete(result.map_err(|err| err.message().to_string()));
        true
    }

    /// The service response fields for a command: success, and the C firmware's message.
    fn call(self: &Arc<Self>, command: Command) -> impl Future<Output = (bool, String)> + Send + use<> {
        let reply = Reply(self.send(command));
        async move {
            match reply.await {
                Ok(()) => (true, String::new()),
                Err(message) => (false, message),
            }
        }
    }
}

// ========================================
// Topics
// ========================================

/// riptide_msgs2/ActuatorStatus constants. The C firmware was built against an older message with
/// `CLAW_CLOSED = 4, CLAW_OPENING = 5, CLAW_CLOSING = 6`; the current message splits closed into bottle and spoon
/// (4 and 5), so states are mapped by name and closed reports as `CLAW_BOTTLE_CLOSED` (still 4).
fn claw_state(state: ClawState) -> u8 {
    match state {
        ClawState::Error => 0,
        ClawState::Disarmed => 1,
        ClawState::Unknown => 2,
        ClawState::Opened => 3,
        ClawState::Closed => 4,
        ClawState::Opening => 6,
        ClawState::Closing => 7,
    }
}

fn torpedo_state(state: TorpedoState) -> u8 {
    match state {
        TorpedoState::Error => 0,
        TorpedoState::Disarmed => 1,
        TorpedoState::Charging => 2,
        TorpedoState::Charged => 3,
        TorpedoState::Firing => 4,
        TorpedoState::Fired => 5,
    }
}

fn dropper_state(state: DropperState) -> u8 {
    match state {
        DropperState::Error => 0,
        DropperState::Disarmed => 1,
        DropperState::Ready => 2,
        DropperState::Dropping => 3,
        DropperState::Dropped => 4,
    }
}

// Accessors `ros.rs` doesn't have yet
macro_rules! setter {
    ($name:ident, $variant:ident, $ty:ty) => {
        fn $name(msg: &mut DynamicMessage, field: &str, value: $ty) -> Result<()> {
            match msg.get_mut(field) {
                Some(ValueMut::Simple(SimpleValueMut::$variant(slot))) => {
                    *slot = value;
                    Ok(())
                }
                Some(other) => bail!("field {field} is {other:?}, expected {}", stringify!($variant)),
                None => bail!("message has no field {field}"),
            }
        }
    };
}

setter!(set_u16, Uint16, u16);
setter!(set_i16, Int16, i16);
setter!(set_i32, Int32, i32);

fn fill_status(msg: &mut DynamicMessage, s: &actuator::Status) -> Result<()> {
    ros::set_bool(msg, "actuators_armed", s.armed)?;
    ros::set_u8(msg, "claw_state", claw_state(s.claw))?;
    // This board has no rack; rack_state stays at its default, RACK_ERROR (the C message had no such field)
    ros::set_u8(msg, "torpedo_state", torpedo_state(s.torpedo))?;
    ros::set_u8(msg, "torpedo_available_count", s.torpedo_available)?;
    ros::set_u8(msg, "dropper_state", dropper_state(s.dropper))?;
    ros::set_u8(msg, "dropper_available_count", s.dropper_available)
}

fn fill_dynamixel(msg: &mut DynamicMessage, d: &actuator::DynamixelStatus) -> Result<()> {
    set_u16(msg, "model", d.model)?;
    ros::set_u8(msg, "fw_version", d.fw_version)?;
    ros::set_u8(msg, "id", d.id)?;
    ros::set_u8(msg, "hw_err_status", d.hw_err_status)?;
    ros::set_bool(msg, "torque_enable", d.torque_enable)?;
    ros::set_bool(msg, "moving", d.moving)?;
    set_u16(msg, "voltage", d.voltage)?;
    ros::set_u8(msg, "temperature", d.temperature)?;
    set_i32(msg, "position", d.position)?;
    set_i16(msg, "load", d.load)
}

/// The V1 timing parameters (`actuator_v1_parameters.c`).
struct TimingParameters {
    claw: MandatoryParameter<i64>,
    dropper: MandatoryParameter<i64>,
    torpedo: Vec<(TorpedoTiming, MandatoryParameter<i64>)>,
}

/// Whatever exists only for one actuator variant.
enum VariantTopics {
    V1(TimingParameters),
    V2 { dynamixel: Publisher, _services: Vec<Service<Trigger>> },
}

pub struct ActuatorTopics {
    node: Node,
    logger: String,
    commander: Arc<Commander>,
    status: Publisher,
    busy: Publisher,
    /// Last reported arm state, for refusing timing changes while armed as the C firmware did.
    armed: Arc<AtomicBool>,
    variant: Mutex<Option<(Variant, VariantTopics)>>,
    _trigger_services: Vec<Service<Trigger>>,
    _set_bool_services: Vec<Service<SetBool>>,
}

fn trigger_service(node: &Node, commander: &Arc<Commander>, name: &str, command: Command) -> Result<Service<Trigger>> {
    let commander = commander.clone();
    Ok(node.create_async_service::<Trigger, _>(name, move |_request: TriggerRequest| {
        let reply = commander.call(command);
        async move {
            let (success, message) = reply.await;
            TriggerResponse { success, message: message.as_str().into() }
        }
    })?)
}

fn set_bool_service(
    node: &Node,
    commander: &Arc<Commander>,
    name: &str,
    command: fn(bool) -> Command,
) -> Result<Service<SetBool>> {
    let commander = commander.clone();
    Ok(node.create_async_service::<SetBool, _>(name, move |request: SetBoolRequest| {
        let reply = commander.call(command(request.data));
        async move {
            let (success, message) = reply.await;
            SetBoolResponse { success, message: message.as_str().into() }
        }
    })?)
}

impl ActuatorTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let commander = Arc::new(Commander { link, pending: Mutex::new((0, HashMap::new())) });
        let trigger_services = vec![
            trigger_service(node, &commander, "command/actuator/torpedo", Command::FireTorpedo)?,
            trigger_service(node, &commander, "command/actuator/dropper", Command::DropMarker)?,
            trigger_service(node, &commander, "command/actuator/notify_reload", Command::NotifyReload)?,
        ];
        let set_bool_services = vec![
            set_bool_service(node, &commander, "command/actuator/claw", |open| Command::Claw { open })?,
            set_bool_service(node, &commander, "command/actuator/arm", |arm| {
                if arm { Command::Arm } else { Command::Disarm }
            })?,
        ];
        Ok(ActuatorTopics {
            node: node.clone(),
            logger,
            commander,
            status: Publisher::new(node, "riptide_msgs2/msg/ActuatorStatus", "state/actuator/status".best_effort())?,
            busy: Publisher::new(node, "std_msgs/msg/Bool", "state/actuator/busy".best_effort())?,
            armed: Arc::new(AtomicBool::new(false)),
            variant: Mutex::new(None),
            _trigger_services: trigger_services,
            _set_bool_services: set_bool_services,
        })
    }

    /// Creates the variant's own interface the first time the board reports it.
    fn ensure_variant(&self, variant: Variant) -> Result<()> {
        let mut current = self.variant.lock().unwrap();
        match &*current {
            Some((known, _)) if *known == variant => return Ok(()),
            Some((known, _)) => bail!("board switched from {known:?} to {variant:?} actuators; restart the bridge"),
            None => {}
        }
        rclrs::log_info!(&self.logger, "Board runs the {variant:?} actuators");
        let topics = match variant {
            Variant::V1 => VariantTopics::V1(self.declare_timing_parameters()?),
            Variant::V2 => VariantTopics::V2 {
                dynamixel: Publisher::new(
                    &self.node,
                    "riptide_msgs2/msg/DynamixelStatus",
                    "state/actuator/dynamixel".best_effort(),
                )?,
                _services: vec![
                    trigger_service(
                        &self.node,
                        &self.commander,
                        "command/actuator/torpedo_marker/go_home",
                        Command::TorpedoMarkerMoveHome,
                    )?,
                    trigger_service(
                        &self.node,
                        &self.commander,
                        "command/actuator/torpedo_marker/set_home",
                        Command::TorpedoMarkerSetHome,
                    )?,
                    trigger_service(
                        &self.node,
                        &self.commander,
                        "command/actuator/claw/set_closed_pos",
                        Command::ClawSetClosedPosition,
                    )?,
                ],
            },
        };
        *current = Some((variant, topics));
        Ok(())
    }

    /// Declares one timing parameter. Values outside 1..=65535 are refused (`IS_VALID_TIMING`), and so are changes
    /// while armed if `armed_locked`, as the firmware would refuse them; accepted changes go to the board.
    fn timing_parameter(
        &self,
        name: &'static str,
        default: u16,
        armed_locked: bool,
        command: impl Fn(u16) -> Command + Send + Sync + 'static,
    ) -> Result<MandatoryParameter<i64>> {
        let armed = self.armed.clone();
        let param = self
            .node
            .declare_parameter(name)
            .default(default as i64)
            .range(ParameterRange { lower: Some(1), upper: Some(u16::MAX as i64), step: None })
            .validate(move |_value| {
                if armed_locked && armed.load(Ordering::Relaxed) {
                    Err("can't change while the actuators are armed".into())
                } else {
                    Ok(())
                }
            })
            .mandatory()
            .map_err(|err| anyhow!("declaring {name}: {err:?}"))?;
        let commander = self.commander.clone();
        let logger = self.logger.clone();
        param.on_change(move |value| {
            // The range guarantees the value fits
            log_outcome(&logger, name, &commander, command(*value as u16));
        });
        Ok(param)
    }

    fn declare_timing_parameters(&self) -> Result<TimingParameters> {
        let claw = self.timing_parameter("claw_timing_ms", actuator::DEFAULT_CLAW_TIMING_MS, false, |ms| {
            Command::SetClawTiming { ms }
        })?;
        let dropper =
            self.timing_parameter("dropper_active_timing_ms", actuator::DEFAULT_DROPPER_TIMING_MS, true, |ms| {
                Command::SetDropperTiming { ms }
            })?;
        let mut torpedo = Vec::new();
        for timing in TorpedoTiming::ALL {
            let param = self.timing_parameter(timing.parameter_name(), timing.default_us(), true, move |us| {
                Command::SetTorpedoTiming { timing, us }
            })?;
            torpedo.push((timing, param));
        }
        Ok(TimingParameters { claw, dropper, torpedo })
    }

    /// Sends every timing to the board, as the C parameter server did when ROS connected.
    fn push_timings(&self) {
        let variant = self.variant.lock().unwrap();
        let Some((_, VariantTopics::V1(params))) = &*variant else { return };
        let as_u16 = |p: &MandatoryParameter<i64>| p.get().clamp(1, u16::MAX as i64) as u16;
        log_outcome(
            &self.logger,
            "claw_timing_ms",
            &self.commander,
            Command::SetClawTiming { ms: as_u16(&params.claw) },
        );
        log_outcome(
            &self.logger,
            "dropper_active_timing_ms",
            &self.commander,
            Command::SetDropperTiming { ms: as_u16(&params.dropper) },
        );
        for (timing, param) in &params.torpedo {
            log_outcome(
                &self.logger,
                timing.parameter_name(),
                &self.commander,
                Command::SetTorpedoTiming { timing: *timing, us: as_u16(param) },
            );
        }
    }
}

/// Sends a parameter command and logs a failure when the answer comes in.
fn log_outcome(logger: &str, name: &'static str, commander: &Arc<Commander>, command: Command) {
    let slot = commander.send(command);
    let logger = logger.to_string();
    thread::spawn(move || {
        if let Err(message) = slot.wait() {
            rclrs::log_warn!(&logger, "Setting {name} failed: {message}");
        }
    });
}

impl BoardTopics for ActuatorTopics {
    fn handle(&self, msg: &DeviceToHost) -> Result<bool> {
        let DeviceToHost::Actuator(msg) = msg else { return Ok(false) };
        match msg {
            actuator::DeviceToHost::CommandResult { seq, result } => {
                if !self.commander.answer(*seq, *result) {
                    rclrs::log_warn!(&self.logger, "Late answer to actuator command {seq}: {result:?}");
                }
            }
            actuator::DeviceToHost::Status(status) => {
                self.armed.store(status.armed, Ordering::Relaxed);
                self.ensure_variant(status.variant)?;
                self.busy.publish(|m| ros::set_bool(m, "data", status.busy))?;
                self.status.publish(|m| fill_status(m, status))?;
            }
            actuator::DeviceToHost::Dynamixel(status) => {
                self.ensure_variant(Variant::V2)?;
                if let Some((_, VariantTopics::V2 { dynamixel, .. })) = &*self.variant.lock().unwrap() {
                    dynamixel.publish(|m| fill_dynamixel(m, status))?;
                }
            }
            actuator::DeviceToHost::RequestTimings => {
                self.ensure_variant(Variant::V1)?;
                self.push_timings();
            }
        }
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    //! Run inside `riptide-shell`: these go through real ROS type support and DDS, in process.

    use super::*;
    use std::sync::mpsc;

    use rclrs::{Context, CreateBasicExecutor, IntoNodeOptions, SpinOptions};

    use crate::link::Event;

    /// A board that answers every command with `answer`.
    struct FakeBoard {
        answer: Box<dyn Fn(Command) -> Result<(), ActuatorError> + Send + Sync>,
        topics: Mutex<Option<Arc<ActuatorTopics>>>,
    }

    impl Link for FakeBoard {
        fn send(&self, msg: &HostToDevice) -> Result<()> {
            let HostToDevice::Actuator(actuator::HostToDevice::Command { seq, command }) = msg else {
                return Ok(());
            };
            let result = (self.answer)(*command);
            let topics = self.topics.lock().unwrap().clone();
            let seq = *seq;
            // Answer from another thread, like the bridge's receive loop
            thread::spawn(move || {
                if let Some(topics) = topics {
                    topics
                        .handle(&DeviceToHost::Actuator(actuator::DeviceToHost::CommandResult { seq, result }))
                        .unwrap();
                }
            });
            Ok(())
        }

        fn receive(&self) -> Result<Event> {
            Ok(Event::Idle)
        }
    }

    #[test]
    fn services_round_trip_through_the_board() -> Result<()> {
        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let node = executor.create_node("actuator_mk2".namespace("titan_bridge_actuator_test"))?;
        let board = Arc::new(FakeBoard {
            answer: Box::new(|command| match command {
                Command::FireTorpedo => Err(ActuatorError::NotArmed),
                _ => Ok(()),
            }),
            topics: Mutex::new(None),
        });
        let topics = Arc::new(ActuatorTopics::new(&node, board.clone(), "test".into())?);
        *board.topics.lock().unwrap() = Some(topics.clone());

        let torpedo = node.create_client::<Trigger>("command/actuator/torpedo")?;
        let arm = node.create_client::<SetBool>("command/actuator/arm")?;
        let (tx, rx) = mpsc::channel();

        for _ in 0..100 {
            if torpedo.service_is_ready()? && arm.service_is_ready()? {
                break;
            }
            executor.spin(SpinOptions::default().timeout(Duration::from_millis(50)));
        }

        let tx_torpedo = tx.clone();
        let _torpedo_call = torpedo.call_then(&TriggerRequest::default(), move |response: TriggerResponse| {
            tx_torpedo.send(("torpedo", response.success, response.message.to_string())).unwrap();
        })?;
        let _arm_call = arm.call_then(&SetBoolRequest { data: true }, move |response: SetBoolResponse| {
            tx.send(("arm", response.success, response.message.to_string())).unwrap();
        })?;

        let mut got = Vec::new();
        for _ in 0..100 {
            executor.spin(SpinOptions::default().timeout(Duration::from_millis(50)));
            while let Ok(r) = rx.try_recv() {
                got.push(r);
            }
            if got.len() == 2 {
                break;
            }
        }
        got.sort();
        assert_eq!(got, [("arm", true, String::new()), ("torpedo", false, "Not Armed".to_string())]);
        Ok(())
    }

    #[test]
    fn variant_topics_are_created_once() -> Result<()> {
        let context = Context::default_from_env()?;
        let executor = context.create_basic_executor();
        let node = executor.create_node("actuator_mk2".namespace("titan_bridge_actuator_variant_test"))?;
        let board = Arc::new(FakeBoard { answer: Box::new(|_| Ok(())), topics: Mutex::new(None) });
        let topics = ActuatorTopics::new(&node, board, "test".into())?;
        let status = actuator::Status {
            variant: Variant::V2,
            armed: false,
            busy: false,
            claw: ClawState::Closed,
            torpedo: TorpedoState::Charged,
            torpedo_available: 2,
            dropper: DropperState::Ready,
            dropper_available: 2,
        };
        assert!(topics.handle(&DeviceToHost::Actuator(actuator::DeviceToHost::Status(status)))?);
        assert!(topics.handle(&DeviceToHost::Actuator(actuator::DeviceToHost::Dynamixel(Default::default())))?);
        assert!(topics.handle(&DeviceToHost::Actuator(actuator::DeviceToHost::RequestTimings)).is_err());
        assert!(!topics.handle(&DeviceToHost::Heartbeat)?);
        Ok(())
    }

    #[test]
    fn timing_parameters_push_on_request() -> Result<()> {
        let context = Context::default_from_env()?;
        let executor = context.create_basic_executor();
        let node = executor.create_node("actuator_mk2".namespace("titan_bridge_actuator_v1_test"))?;
        let (tx, rx) = mpsc::channel();
        let tx = Mutex::new(tx);
        let board = Arc::new(FakeBoard {
            answer: Box::new(move |command| {
                tx.lock().unwrap().send(command).unwrap();
                Ok(())
            }),
            topics: Mutex::new(None),
        });
        let topics = Arc::new(ActuatorTopics::new(&node, board.clone(), "test".into())?);
        *board.topics.lock().unwrap() = Some(topics.clone());
        topics.handle(&DeviceToHost::Actuator(actuator::DeviceToHost::RequestTimings))?;
        let sent: Vec<Command> = rx.try_iter().collect();
        assert_eq!(sent.len(), 7);
        assert_eq!(sent[0], Command::SetClawTiming { ms: 4500 });
        assert_eq!(sent[1], Command::SetDropperTiming { ms: 250 });
        assert_eq!(sent[2], Command::SetTorpedoTiming { timing: TorpedoTiming::Coil1On, us: 23000 });
        assert_eq!(sent[6], Command::SetTorpedoTiming { timing: TorpedoTiming::Coil3On, us: 13000 });
        Ok(())
    }
}
