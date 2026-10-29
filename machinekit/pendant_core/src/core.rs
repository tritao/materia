//! The pendant's behaviour without its hardware. The firmware reads the controls into `Inputs`, calls `tick` every millisecond or so and
//! sends the `PENDANT_STATE` frame it returns every 10 ms; it passes the controller's `PENDANT_CONFIG` and `PENDANT_OUTPUT` frames
//! to `on_frame`, drives the LEDs from `leds()` and the display from `screen()`. Time is a millisecond counter that may wrap.

use crate::screen::Screen;
use crate::view::{layout, View, AXES, KEYS, LABEL};
use operator_link::operator_wire::{ConfigKind, LeaseState, PendantState};
use operator_link::{decode_config, decode_output, Invalid, CONFIG_ID, FLAG_CHAIN_CLOSED, FLAG_WHEEL_ENABLE, OUTPUT_ID};

/// The raw reading of the controls at one instant.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Inputs {
    /// The wheel's free-running quadrature count; the pendant sends the change since its last frame.
    pub wheel_counts: u16,
    /// 0 for OFF, 1 to 11.
    pub position: u8,
    /// 0, 1, 2 for x1, x10, x100.
    pub step: u8,
    /// Bits 0 to 5: Start, Hold, Stop, F1, F2, F3.
    pub keys: u8,
    pub wheel_enable: bool,
    pub chain_closed: bool,
}

/// What the display should be doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DisplayPower {
    On,
    /// Idle for a while: low contrast, to limit burn-in.
    Dim,
    /// Idle for long: blanked.
    Off,
}

/// A debounced reading of everything except the wheel's count.
#[derive(Clone, Copy, PartialEq, Eq)]
struct Snapshot {
    position: u8,
    step: u8,
    keys: u8,
    flags: u8,
}

impl Snapshot {
    fn of(inputs: &Inputs) -> Self {
        Snapshot {
            position: inputs.position,
            step: inputs.step,
            keys: inputs.keys,
            flags: (if inputs.wheel_enable { FLAG_WHEEL_ENABLE } else { 0 }) | (if inputs.chain_closed { FLAG_CHAIN_CLOSED } else { 0 }),
        }
    }
}

pub struct PendantCore {
    sequence: u16,
    last_state_ms: u32,
    started: bool,
    last_counts: u16,
    candidate: Snapshot,
    candidate_since: u32,
    stable: Snapshot,
    // From the controller.
    leds: u8,
    lease: LeaseState,
    status: [u8; 21],
    axis_names: [[u8; LABEL]; AXES],
    key_names: [[u8; LABEL]; KEYS],
    last_output_ms: Option<u32>,
    last_activity_ms: u32,
}

impl PendantCore {
    /// Frames go out this often; the controller's lease counts on them.
    pub const STATE_PERIOD_MS: u32 = 10;
    /// A control must read the same this long before it counts.
    pub const DEBOUNCE_MS: u32 = 5;
    /// No output frame from the controller for this long and the link is down.
    pub const LINK_TIMEOUT_MS: u32 = 500;
    /// Idle for this long and the display dims, then blanks.
    pub const DIM_AFTER_MS: u32 = 10_000;
    pub const BLANK_AFTER_MS: u32 = 60_000;

    pub fn new(now_ms: u32) -> Self {
        let idle = Snapshot { position: 0, step: 0, keys: 0, flags: FLAG_CHAIN_CLOSED };
        PendantCore {
            sequence: 0,
            last_state_ms: now_ms,
            started: false,
            last_counts: 0,
            candidate: idle,
            candidate_since: now_ms,
            stable: idle,
            leds: 0,
            lease: LeaseState::NoLease,
            status: [0; 21],
            axis_names: [[0; LABEL]; AXES],
            key_names: [[0; LABEL]; KEYS],
            last_output_ms: None,
            last_activity_ms: now_ms,
        }
    }

    /// Reads the controls; returns the `PENDANT_STATE` payload when one is due.
    pub fn tick(&mut self, now_ms: u32, inputs: &Inputs) -> Option<[u8; PendantState::SIZE]> {
        let reading = Snapshot::of(inputs);
        if reading != self.candidate {
            self.candidate = reading;
            self.candidate_since = now_ms;
        } else if reading != self.stable && now_ms.wrapping_sub(self.candidate_since) >= Self::DEBOUNCE_MS {
            self.stable = reading;
            self.last_activity_ms = now_ms;
        }
        if !self.started {
            // The first call has no earlier count to take a change from.
            self.started = true;
            self.last_counts = inputs.wheel_counts;
            self.last_state_ms = now_ms.wrapping_sub(Self::STATE_PERIOD_MS);
        }
        if now_ms.wrapping_sub(self.last_state_ms) < Self::STATE_PERIOD_MS { return None; }
        self.last_state_ms = now_ms;
        let delta = inputs.wheel_counts.wrapping_sub(self.last_counts) as i16;
        self.last_counts = inputs.wheel_counts;
        if delta != 0 { self.last_activity_ms = now_ms; }
        self.sequence = self.sequence.wrapping_add(1);
        let state = PendantState {
            sequence: self.sequence,
            wheel_delta: delta,
            position: self.stable.position,
            step: self.stable.step,
            buttons: self.stable.keys,
            flags: self.stable.flags,
        };
        let mut payload = [0u8; PendantState::SIZE];
        state.encode(&mut payload).ok()?;
        Some(payload)
    }

    /// A frame from the controller: `PENDANT_CONFIG` (a label) or `PENDANT_OUTPUT` (LEDs, lease, status line). Other ids are not ours.
    pub fn on_frame(&mut self, now_ms: u32, id: u32, payload: &[u8]) -> Result<(), Invalid> {
        match id {
            CONFIG_ID => {
                let config = decode_config(payload)?;
                if config.kind == ConfigKind::Axis as u8 {
                    self.axis_names[config.index as usize - 1] = config.text;
                } else {
                    self.key_names[config.index as usize] = config.text;
                }
            }
            OUTPUT_ID => {
                let output = decode_output(payload)?;
                let status_changed = output.text != self.status;
                self.leds = output.leds;
                self.lease = match output.lease {
                    1 => LeaseState::Granted,
                    2 => LeaseState::Lost,
                    _ => LeaseState::NoLease,
                };
                self.status = output.text;
                self.last_output_ms = Some(now_ms);
                if status_changed { self.last_activity_ms = now_ms; }
            }
            _ => {}
        }
        Ok(())
    }

    pub fn link_up(&self, now_ms: u32) -> bool {
        self.last_output_ms.map_or(false, |at| now_ms.wrapping_sub(at) <= Self::LINK_TIMEOUT_MS)
    }

    /// The key LEDs, in the button order of `PENDANT_STATE`; all lit with no link, to say the pendant is awake but cut off.
    pub fn leds(&self, now_ms: u32) -> u8 {
        if self.link_up(now_ms) { self.leds } else { 0 }
    }

    pub fn view(&self, now_ms: u32) -> View {
        View {
            position: self.stable.position,
            step: self.stable.step,
            wheel_enable: self.stable.flags & FLAG_WHEEL_ENABLE != 0,
            chain_closed: self.stable.flags & FLAG_CHAIN_CLOSED != 0,
            lease: self.lease,
            link_up: self.link_up(now_ms),
            status: self.status,
            axis_names: self.axis_names,
            key_names: self.key_names,
        }
    }

    pub fn screen(&self, now_ms: u32) -> Screen { layout(&self.view(now_ms)) }

    /// Dim after a while idle and blank after a longer one; any control, wheel turn or new status line wakes it.
    pub fn display_power(&self, now_ms: u32) -> DisplayPower {
        let idle = now_ms.wrapping_sub(self.last_activity_ms);
        if idle >= Self::BLANK_AFTER_MS { DisplayPower::Off } else if idle >= Self::DIM_AFTER_MS { DisplayPower::Dim } else { DisplayPower::On }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use operator_link::operator_wire::{PendantConfig, PendantOutput};
    use operator_link::decode_state;

    fn name<const N: usize>(s: &str) -> [u8; N] {
        let mut out = [0u8; N];
        out[..s.len()].copy_from_slice(s.as_bytes());
        out
    }

    fn idle() -> Inputs { Inputs { chain_closed: true, ..Inputs::default() } }

    #[test]
    fn the_heartbeat_is_every_ten_milliseconds() {
        let mut core = PendantCore::new(0);
        let mut sent = std::vec::Vec::new();
        for ms in 0..=100 {
            if let Some(frame) = core.tick(ms, &idle()) { sent.push((ms, decode_state(&frame).unwrap().sequence)); }
        }
        let times: std::vec::Vec<u32> = sent.iter().map(|s| s.0).collect();
        assert_eq!(times, [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]);
        assert!(sent.windows(2).all(|w| w[1].1 == w[0].1.wrapping_add(1)));
    }

    #[test]
    fn a_control_counts_only_after_it_settles() {
        let mut core = PendantCore::new(0);
        let mut inputs = idle();
        core.tick(0, &inputs);
        inputs.position = 3;
        // Contact bounce: it reads 3, then 4, then 3, and only the last settles.
        core.tick(10, &inputs);
        inputs.position = 4;
        core.tick(12, &inputs);
        inputs.position = 3;
        core.tick(13, &inputs);
        assert_eq!(core.view(13).position, 0);
        core.tick(17, &inputs);
        assert_eq!(core.view(17).position, 0);
        core.tick(18, &inputs);
        assert_eq!(core.view(18).position, 3);
        let frame = core.tick(20, &inputs).unwrap();
        assert_eq!(decode_state(&frame).unwrap().position, 3);
    }

    #[test]
    fn the_wheel_sends_the_change_since_the_last_frame_across_the_wrap() {
        let mut core = PendantCore::new(0);
        let mut inputs = idle();
        inputs.wheel_counts = 65_530;
        core.tick(0, &inputs);
        inputs.wheel_counts = 4;
        let frame = core.tick(10, &inputs).unwrap();
        assert_eq!(decode_state(&frame).unwrap().wheel_delta, 10);
        inputs.wheel_counts = 1;
        let frame = core.tick(20, &inputs).unwrap();
        assert_eq!(decode_state(&frame).unwrap().wheel_delta, -3);
    }

    #[test]
    fn keys_and_flags_travel_in_the_frame() {
        let mut core = PendantCore::new(0);
        let inputs = Inputs { keys: 0b101, wheel_enable: true, chain_closed: false, step: 2, position: 11, ..Inputs::default() };
        core.tick(0, &inputs);
        core.tick(6, &inputs);
        let frame = core.tick(10, &inputs).unwrap();
        let state = decode_state(&frame).unwrap();
        assert_eq!((state.buttons, state.flags, state.step, state.position), (0b101, FLAG_WHEEL_ENABLE, 2, 11));
    }

    fn config(kind: ConfigKind, index: u8, text: &str) -> [u8; 16] {
        let mut out = [0u8; 16];
        PendantConfig { kind: kind as u8, index, text: name(text), reserved: [0; 2] }.encode(&mut out).unwrap();
        out
    }

    fn output(leds: u8, lease: u8, text: &str) -> [u8; 24] {
        let mut out = [0u8; 24];
        PendantOutput { leds, lease, text: name(text), reserved: 0 }.encode(&mut out).unwrap();
        out
    }

    #[test]
    fn labels_and_outputs_fill_the_screen() {
        let mut core = PendantCore::new(0);
        assert!(!core.link_up(0));
        core.on_frame(5, CONFIG_ID, &config(ConfigKind::Axis, 3, "Z")).unwrap();
        core.on_frame(5, CONFIG_ID, &config(ConfigKind::Key, 1, "SPINDLE")).unwrap();
        core.on_frame(6, OUTPUT_ID, &output(0b100001, 1, "Gearbox engaged")).unwrap();
        let mut inputs = idle();
        inputs.position = 3;
        core.tick(7, &inputs);
        core.tick(13, &inputs);
        assert!(core.link_up(13));
        assert_eq!(core.leds(13), 0b100001);
        let screen = core.screen(13);
        assert_eq!(std::str::from_utf8(&screen.rows[0]).unwrap().trim(), "Z");
        assert_eq!(std::str::from_utf8(&screen.rows[5]).unwrap().trim_end(), "Gearbox engaged");
        assert_eq!(&screen.rows[7][7..13], b"SPINDL");
    }

    #[test]
    fn a_silent_controller_is_a_dead_link_and_dark_leds() {
        let mut core = PendantCore::new(0);
        core.on_frame(0, OUTPUT_ID, &output(0b111, 1, "")).unwrap();
        assert!(core.link_up(500) && core.leds(500) == 0b111);
        assert!(!core.link_up(501) && core.leds(501) == 0);
        let screen = core.screen(501);
        assert_eq!(std::str::from_utf8(&screen.rows[0]).unwrap().trim(), "NO LINK");
        // A frame from a later time brings it back.
        core.on_frame(600, OUTPUT_ID, &output(0b1, 1, "")).unwrap();
        assert!(core.link_up(601));
    }

    #[test]
    fn bad_frames_are_refused_and_other_ids_ignored() {
        let mut core = PendantCore::new(0);
        assert!(core.on_frame(0, OUTPUT_ID, &[0u8; 23]).is_err());
        assert!(core.on_frame(0, CONFIG_ID, &config(ConfigKind::Axis, 3, "Z")[..15]).is_err());
        assert!(core.on_frame(0, 0x7FF, &[1, 2, 3]).is_ok());
        assert!(!core.link_up(0));
    }

    #[test]
    fn the_display_dims_then_blanks_when_idle_and_wakes_on_use() {
        let mut core = PendantCore::new(0);
        let mut inputs = idle();
        core.tick(0, &inputs);
        assert_eq!(core.display_power(9_999), DisplayPower::On);
        assert_eq!(core.display_power(10_000), DisplayPower::Dim);
        assert_eq!(core.display_power(60_000), DisplayPower::Off);
        inputs.position = 2;
        core.tick(60_010, &inputs);
        core.tick(60_020, &inputs);
        assert_eq!(core.display_power(60_020), DisplayPower::On);
        // A new status line from the controller wakes it too.
        core.on_frame(80_000, OUTPUT_ID, &output(0, 1, "Door open")).unwrap();
        assert_eq!(core.display_power(80_001), DisplayPower::On);
    }

    #[test]
    fn the_sequence_wraps() {
        let mut core = PendantCore::new(0);
        core.sequence = u16::MAX - 1;
        let inputs = idle();
        let a = decode_state(&core.tick(0, &inputs).unwrap()).unwrap().sequence;
        let b = decode_state(&core.tick(10, &inputs).unwrap()).unwrap().sequence;
        let c = decode_state(&core.tick(20, &inputs).unwrap()).unwrap().sequence;
        assert_eq!((a, b, c), (u16::MAX, 0, 1));
    }
}
