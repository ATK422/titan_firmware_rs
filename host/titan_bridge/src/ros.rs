//! Thin helpers over rclrs dynamic messages.
//!
//! The bridge uses dynamic publishers and subscriptions so it runs against the stock `riptide-shell` install
//! (`std_msgs`, `riptide_msgs2`) without generating Rust message crates with `rosidl_generator_rs`.

use anyhow::{Context, Result, anyhow, bail};
use rclrs::{
    DynamicMessage, DynamicMessageMetadata, DynamicMessageViewMut, DynamicPublisher, Node, PublisherOptions,
    SimpleValue, SimpleValueMut, Value, ValueMut,
};

/// A dynamic publisher together with the metadata to create its messages.
pub struct Publisher {
    publisher: DynamicPublisher,
    metadata: DynamicMessageMetadata,
}

impl Publisher {
    pub fn new<'a>(node: &Node, type_name: &str, options: impl Into<PublisherOptions<'a>>) -> Result<Self> {
        let message_type = type_name.try_into().map_err(|e| anyhow!("{type_name}: {e:?}"))?;
        let metadata = DynamicMessageMetadata::new(type_name.try_into().map_err(|e| anyhow!("{type_name}: {e:?}"))?)
            .map_err(|e| anyhow!("loading type support for {type_name}: {e:?}"))?;
        let publisher = node
            .create_dynamic_publisher(message_type, options)
            .with_context(|| format!("creating {type_name} publisher"))?;
        Ok(Publisher { publisher, metadata })
    }

    /// Creates a default message, lets `fill` set its fields, and publishes it.
    pub fn publish(&self, fill: impl FnOnce(&mut DynamicMessage) -> Result<()>) -> Result<()> {
        let mut msg = self.metadata.create().map_err(|e| anyhow!("creating message: {e:?}"))?;
        fill(&mut msg)?;
        self.publisher.publish(msg).context("publishing")
    }
}

fn simple_mut<'a>(msg: &'a mut DynamicMessage, field: &str) -> Result<SimpleValueMut<'a>> {
    match msg.get_mut(field) {
        Some(ValueMut::Simple(value)) => Ok(value),
        Some(_) => bail!("field {field} is not a simple value"),
        None => bail!("message has no field {field}"),
    }
}

fn simple<'a>(msg: &'a DynamicMessage, field: &str) -> Result<SimpleValue<'a>> {
    match msg.get(field) {
        Some(Value::Simple(value)) => Ok(value),
        Some(_) => bail!("field {field} is not a simple value"),
        None => bail!("message has no field {field}"),
    }
}

macro_rules! accessors {
    ($($set:ident, $get:ident, $variant:ident, $ty:ty;)*) => {$(
        #[allow(dead_code)]
        pub fn $set(msg: &mut DynamicMessage, field: &str, value: $ty) -> Result<()> {
            match simple_mut(msg, field)? {
                SimpleValueMut::$variant(slot) => {
                    *slot = value;
                    Ok(())
                }
                other => bail!("field {field} is {other:?}, expected {}", stringify!($variant)),
            }
        }

        #[allow(dead_code)]
        pub fn $get(msg: &DynamicMessage, field: &str) -> Result<$ty> {
            match simple(msg, field)? {
                SimpleValue::$variant(value) => Ok(*value),
                other => bail!("field {field} is {other:?}, expected {}", stringify!($variant)),
            }
        }
    )*};
}

accessors! {
    set_bool, get_bool, Boolean, bool;
    set_i8, get_i8, Int8, i8;
    set_u8, get_u8, Uint8, u8;
    set_i16, get_i16, Int16, i16;
    set_u16, get_u16, Uint16, u16;
    set_i32, get_i32, Int32, i32;
    set_u32, get_u32, Uint32, u32;
    set_f32, get_f32, Float, f32;
}

pub fn set_string(msg: &mut DynamicMessage, field: &str, value: &str) -> Result<()> {
    match simple_mut(msg, field)? {
        SimpleValueMut::String(slot) => {
            *slot = value.into();
            Ok(())
        }
        other => bail!("field {field} is {other:?}, expected String"),
    }
}

#[allow(dead_code)]
pub fn get_string(msg: &DynamicMessage, field: &str) -> Result<String> {
    match simple(msg, field)? {
        SimpleValue::String(value) => Ok(value.to_string()),
        other => bail!("field {field} is {other:?}, expected String"),
    }
}

/// Splits nanoseconds since the epoch into the `sec`/`nanosec` pair of `builtin_interfaces/Time`.
pub fn split_time(nanoseconds: i64) -> (i32, u32) {
    (nanoseconds.div_euclid(1_000_000_000) as i32, nanoseconds.rem_euclid(1_000_000_000) as u32)
}

/// The current wall clock time in nanoseconds since the epoch, for message stamps.
#[allow(dead_code)]
pub fn now_nanoseconds() -> i64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_nanos() as i64)
}

macro_rules! set_time_fields {
    ($msg:expr, $nanoseconds:expr) => {{
        let (sec, nanosec) = split_time($nanoseconds);
        match $msg.get_mut("sec") {
            Some(ValueMut::Simple(SimpleValueMut::Int32(slot))) => *slot = sec,
            other => bail!("time.sec is {other:?}, expected Int32"),
        }
        match $msg.get_mut("nanosec") {
            Some(ValueMut::Simple(SimpleValueMut::Uint32(slot))) => *slot = nanosec,
            other => bail!("time.nanosec is {other:?}, expected Uint32"),
        }
        Ok::<(), anyhow::Error>(())
    }};
}

fn submessage<'a>(msg: &'a mut DynamicMessage, field: &str) -> Result<DynamicMessageViewMut<'a>> {
    match msg.get_mut(field) {
        Some(ValueMut::Simple(SimpleValueMut::Message(view))) => Ok(view),
        _ => bail!("message has no nested {field} message"),
    }
}

/// Fills a `builtin_interfaces/Time` message.
#[allow(dead_code)]
pub fn set_time(msg: &mut DynamicMessage, nanoseconds: i64) -> Result<()> {
    set_time_fields!(msg, nanoseconds)
}

/// Fills the nested `builtin_interfaces/Time` field `field`.
#[allow(dead_code)]
pub fn set_stamp(msg: &mut DynamicMessage, field: &str, nanoseconds: i64) -> Result<()> {
    let mut stamp = submessage(msg, field)?;
    set_time_fields!(stamp, nanoseconds)
}

/// Fills the `std_msgs/Header` field `header`: its stamp, and its frame ID unless `frame_id` is `None`.
#[allow(dead_code)]
pub fn set_header(msg: &mut DynamicMessage, nanoseconds: i64, frame_id: Option<&str>) -> Result<()> {
    let mut header = submessage(msg, "header")?;
    match header.get_mut("stamp") {
        Some(ValueMut::Simple(SimpleValueMut::Message(mut stamp))) => set_time_fields!(stamp, nanoseconds)?,
        _ => bail!("header.stamp is not a message"),
    }
    if let Some(frame_id) = frame_id {
        match header.get_mut("frame_id") {
            Some(ValueMut::Simple(SimpleValueMut::String(slot))) => *slot = frame_id.into(),
            _ => bail!("header.frame_id is not a string"),
        }
    }
    Ok(())
}
