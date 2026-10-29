//! The jog pendant's hardware-independent core: what it shows (`font`, `screen`, `view`) and what it does (`core`): debounce the
//! controls, send the `PENDANT_STATE` heartbeat, take the controller's labels and outputs, drive the LEDs and the display. The
//! firmware calls it with what the hardware reads and sends what it returns; a simulation does the same with scripted input,
//! so both run the same code. `#![no_std]`, no allocation.
#![no_std]

#[cfg(test)]
extern crate std;

pub mod font;
pub mod screen;
pub mod view;
pub mod core;

pub use crate::core::{DisplayPower, Inputs, PendantCore};
pub use screen::{Framebuffer, Screen, Style};
pub use view::View;
