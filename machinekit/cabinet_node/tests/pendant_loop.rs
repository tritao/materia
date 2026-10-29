//! The pendant and the cabinet node, end to end and without hardware: the pendant core runs against the real operator service over a
//! fake CAN link, driven by scripted input at a millisecond clock. What the operator would see (the screen, the LEDs, the display power)
//! and what the host would get (the events) are asserted. The host plays its part too: the profile's labels once at the start and an
//! allowing `OperatorState` every 100 ms.

use cabinet_node::OperatorService;
use operator_link::operator_frame;
use operator_link::operator_wire::{OperatorMessage, OperatorState, PendantConfig, PendantEvent, PendantOutput};
use operator_link::profile::CompiledProfile;
use operator_link::{CONFIG_ID, OUTPUT_ID, STATE_ID};
use pendant_core::{DisplayPower, Inputs, PendantCore};

const VECTORS: &str = include_str!("../../operator_link/vectors/profiles.txt");

fn unhex(s: &str) -> Vec<u8> {
    (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
}

/// A compiled profile with its reason texts, and the label records the host would send, from the Haxe compiler's vectors.
fn profile(name: &str) -> (CompiledProfile, Vec<Vec<u8>>) {
    let line = VECTORS.lines().find(|l| l.starts_with(name)).unwrap();
    let f: Vec<&str> = line.split(' ').collect();
    let mut compiled = CompiledProfile::from_record(&unhex(f[2])).unwrap();
    if f[3] != "-" {
        for reason in f[3].split(';') { compiled.set_reason(&unhex(reason)).unwrap(); }
    }
    let labels = if f[4] == "-" { vec![] } else { f[4].split(';').map(unhex).collect() };
    (compiled, labels)
}

fn operator_frame_of(kind: OperatorMessage, payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![0u8; operator_frame::MAX_FRAME_SIZE];
    let n = operator_frame::encode(kind as u8, payload, &mut frame).unwrap();
    frame.truncate(n);
    frame
}

struct Loop<'a> {
    core: PendantCore,
    service: OperatorService<'a>,
    ms: u32,
    host_sequence: u16,
    /// A condition mask the controller builds from its own facts: bit 0 the gearbox, bit 3 a program running.
    active: u32,
    /// Whether the cable is connected, in both directions.
    cable: bool,
    events: Vec<PendantEvent>,
    inputs: Inputs,
}

impl<'a> Loop<'a> {
    fn new(profile: &'a CompiledProfile, labels: &[Vec<u8>]) -> Self {
        let mut service = OperatorService::new(profile);
        for label in labels { service.on_operator_frame(&operator_frame_of(OperatorMessage::Config, label), 0).unwrap(); }
        Loop {
            core: PendantCore::new(0),
            service,
            ms: 0,
            host_sequence: 0,
            active: 0,
            cable: true,
            events: vec![],
            inputs: Inputs { chain_closed: true, ..Inputs::default() },
        }
    }

    /// Runs for `ms` milliseconds with the controls as they are in `self.inputs`.
    fn run(&mut self, ms: u32) {
        for _ in 0..ms {
            let now_us = self.ms as u64 * 1000;
            // The host: an allowing state every 100 ms.
            if self.ms % 100 == 0 {
                self.host_sequence += 1;
                let mut payload = [0u8; OperatorState::SIZE];
                OperatorState { sequence: self.host_sequence, jog_allowed: 1, reserved: 0 }.encode(&mut payload).unwrap();
                self.service.on_operator_frame(&operator_frame_of(OperatorMessage::State, &payload), now_us).unwrap();
            }
            // The pendant: its heartbeat reaches the controller if the cable is there.
            if let Some(frame) = self.core.tick(self.ms, &self.inputs) {
                if self.cable {
                    assert!(STATE_ID != 0);
                    if let Some(event) = self.service.on_pendant_frame(&frame, now_us, self.active).unwrap() { self.events.push(event); }
                }
            }
            // The controller: LEDs, lease and status every 100 ms, and one label a moment later.
            if self.cable && self.ms % 100 == 50 {
                let mut output = [0u8; PendantOutput::SIZE];
                self.service.pendant_output(now_us, self.active).encode(&mut output).unwrap();
                self.core.on_frame(self.ms, OUTPUT_ID, &output).unwrap();
                if let Some(config) = self.service.next_config() {
                    let mut payload = [0u8; PendantConfig::SIZE];
                    config.encode(&mut payload).unwrap();
                    self.core.on_frame(self.ms, CONFIG_ID, &payload).unwrap();
                }
            }
            self.service.poll(now_us);
            self.ms += 1;
        }
    }

    fn row(&self, row: usize) -> String {
        let screen = self.core.screen(self.ms);
        screen.rows[row].iter().map(|&c| c as char).collect::<String>().trim_end().to_string()
    }

    fn screen(&self) -> Vec<String> { (0..8).map(|r| self.row(r)).collect() }

    fn select(&mut self, position: u8) { self.inputs.position = position; }

    fn distance(&self) -> i32 { self.events.iter().map(|e| e.distance).sum() }
}

#[test]
fn a_pendant_that_has_not_heard_from_the_controller_says_so() {
    let (compiled, labels) = profile("hobber");
    let mut sim = Loop::new(&compiled, &labels);
    sim.cable = false;
    sim.run(300);
    assert_eq!(sim.screen()[0], " NO LINK");
    assert_eq!(sim.row(3), "    NO CONTROLLER");
    assert_eq!(sim.core.leds(sim.ms), 0);
}

#[test]
fn the_labels_arrive_one_per_frame_and_fill_the_screen() {
    let (compiled, labels) = profile("hobber");
    let mut sim = Loop::new(&compiled, &labels);
    sim.run(2_000);
    // Fourteen labels at one per 100 ms are all there after 1.4 s.
    assert_eq!(sim.row(0).trim(), "OFF");
    assert_eq!(sim.row(7), "Door   Hob    Light");
    sim.select(5);
    sim.run(50);
    assert_eq!(sim.row(0).trim(), "B hob");
    assert_eq!(sim.row(2), "POS 5  STEP X1    ---");
}

#[test]
fn turning_the_wheel_reaches_the_host_and_shows_the_step_and_the_enable() {
    let (compiled, labels) = profile("hobber");
    let mut sim = Loop::new(&compiled, &labels);
    sim.run(1_500);
    sim.select(3);
    sim.inputs.step = 1;
    sim.inputs.wheel_enable = true;
    sim.run(100);
    let before = sim.distance();
    // Five counts every 10 ms for 100 ms at x10 (10 units a count) is 500 units.
    for _ in 0..10 {
        sim.inputs.wheel_counts = sim.inputs.wheel_counts.wrapping_add(5);
        sim.run(10);
    }
    sim.run(10);
    assert_eq!(sim.distance() - before, 500);
    assert!(sim.events.iter().any(|e| e.axis == 4 && e.flags & 1 == 1), "the pendant holds authority on position 3, host axis 4");
    assert_eq!(sim.row(0).trim(), "Z");
    assert_eq!(sim.row(2), "POS 3  STEP X10   WHL");
}

#[test]
fn a_refused_axis_shows_why_and_does_not_move() {
    let (compiled, labels) = profile("hobber");
    let mut sim = Loop::new(&compiled, &labels);
    sim.run(1_500);
    sim.select(5);
    sim.inputs.wheel_enable = true;
    sim.active = 1; // the gearbox is engaged: B and C are refused
    sim.run(100);
    let before = sim.distance();
    for _ in 0..10 {
        sim.inputs.wheel_counts = sim.inputs.wheel_counts.wrapping_add(5);
        sim.run(10);
    }
    sim.run(100);
    assert_eq!(sim.distance(), before, "no move");
    assert_eq!(sim.row(5), "Gearbox engaged");
    // The gearbox lets go: the line clears and the axis moves.
    sim.active = 0;
    sim.run(200);
    assert_eq!(sim.row(5), "");
    for _ in 0..4 {
        sim.inputs.wheel_counts = sim.inputs.wheel_counts.wrapping_add(5);
        sim.run(10);
    }
    sim.run(20);
    assert!(sim.distance() > before);
}

#[test]
fn an_open_estop_chain_is_an_alert_and_lights_stop() {
    let (compiled, labels) = profile("hobber");
    let mut sim = Loop::new(&compiled, &labels);
    sim.run(1_500);
    sim.select(2);
    sim.run(50);
    assert_eq!(sim.row(3), "");
    sim.inputs.chain_closed = false;
    sim.run(200);
    assert_eq!(sim.row(3), "  E-STOP CHAIN OPEN");
    assert_eq!(sim.row(5), "E-stop chain open");
    assert_eq!(sim.core.leds(sim.ms) & 4, 4, "the Stop LED");
    sim.inputs.chain_closed = true;
    sim.run(200);
    assert_eq!(sim.row(3), "");
    assert_eq!(sim.core.leds(sim.ms) & 4, 0);
}

#[test]
fn a_running_program_lights_start_and_blocks_the_pendant() {
    let (compiled, labels) = profile("hobber");
    let mut sim = Loop::new(&compiled, &labels);
    sim.run(1_500);
    sim.select(1);
    sim.active = 1 << 3; // a program is running
    sim.run(200);
    assert_eq!(sim.core.leds(sim.ms) & 1, 1, "the Start LED");
    assert_eq!(sim.row(5), "Program running");
}

#[test]
fn a_cut_cable_ends_the_lease_and_the_pendant_must_be_released_before_it_jogs_again() {
    let (compiled, labels) = profile("hobber");
    let mut sim = Loop::new(&compiled, &labels);
    sim.run(1_500);
    sim.select(3);
    sim.inputs.wheel_enable = true;
    sim.run(100);
    // The cable is cut for a second.
    sim.cable = false;
    sim.run(1_000);
    assert_eq!(sim.row(0).trim(), "NO LINK");
    // It comes back with the enable still held: the lease is lost and the keys must be released.
    sim.cable = true;
    sim.run(300);
    assert_eq!(sim.row(3), "     LEASE LOST");
    assert_eq!(sim.row(5), "Release the keys");
    sim.inputs.wheel_enable = false;
    sim.run(300);
    assert_eq!(sim.row(3), "");
    assert_eq!(sim.row(5), "");
}

#[test]
fn the_display_dims_and_blanks_when_idle_and_wakes_on_a_turn() {
    let (compiled, labels) = profile("hobber");
    let mut sim = Loop::new(&compiled, &labels);
    sim.run(1_500);
    assert_eq!(sim.core.display_power(sim.ms), DisplayPower::On);
    sim.run(10_000);
    assert_eq!(sim.core.display_power(sim.ms), DisplayPower::Dim);
    sim.run(50_000);
    assert_eq!(sim.core.display_power(sim.ms), DisplayPower::Off);
    sim.inputs.wheel_counts = 3;
    sim.run(20);
    assert_eq!(sim.core.display_power(sim.ms), DisplayPower::On);
}

#[test]
fn the_drill_tap_profile_has_its_own_keys_and_axes() {
    let (compiled, labels) = profile("drill-tap");
    let mut sim = Loop::new(&compiled, &labels);
    sim.run(1_500);
    sim.select(5);
    sim.run(50);
    // Position 5 is the spindle on this machine.
    assert_eq!(sim.row(0).trim(), "Spindle");
    assert_eq!(sim.row(7), "Door   Spin   MQL");
}

/// One whole frame, pinned: the exact pixels the OLED gets for the hobber's Z axis at x10 with the wheel enabled.
#[test]
fn the_oled_frame_for_the_hobber_on_z_is_pinned() {
    let (compiled, labels) = profile("hobber");
    let mut sim = Loop::new(&compiled, &labels);
    sim.run(1_500);
    sim.select(3);
    sim.inputs.step = 1;
    sim.inputs.wheel_enable = true;
    sim.run(120);
    let frame = sim.core.screen(sim.ms).render();
    if std::env::var("PENDANT_PRINT_FRAME").is_ok() {
        for y in 0..64 {
            println!("{}", (0..128).map(|x| if frame.pixel(x, y) { '#' } else { '.' }).collect::<String>());
        }
    }
    assert_eq!(frame.checksum(), 2924317326);
}
