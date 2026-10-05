#![no_std]

#[cfg(feature = "std")]
extern crate std;

pub mod config_digest;
pub mod device_wire6;
pub mod frame6;
mod board;
mod input_capture;
mod scheduled_core;
mod step_generator;
mod device_events;
#[cfg(feature = "std")]
mod virtual_board;

pub use board::*;
pub use input_capture::*;
pub use scheduled_core::*;
pub use step_generator::*;
pub use device_events::*;
#[cfg(feature = "std")]
pub use virtual_board::*;

#[path = "../boards/welder_retrofit.rs"]
pub mod welder_retrofit;

// Sensor layout generated from ProcessKit's canonical welding channel contract.
#[path = "../../../processkit/schema/weld_contract.rs"]
pub mod weld_contract;
