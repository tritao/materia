//! Desktop device adapter for the RobotClient -> robotd -> RKD5 PTY test.
use robotkit_device_protocol::{Device, DeviceProtocol, JointState, JointTarget, StateMeta, MAX_FRAME_SIZE, MAX_JOINTS};
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::FromRawFd;
use std::time::{Duration, Instant};

const JOINTS: usize = 3;
const WATCHDOG_NS: u64 = 1_000_000_000;

struct FakeDevice {
    state: [JointState; JOINTS],
    stops: usize,
    resets: usize,
    targets: usize,
    timestamp: u64,
}

impl Device for FakeDevice {
    fn stop_all(&mut self) {
        self.stops += 1;
        for joint in &mut self.state {
            joint.velocity = 0.0;
            joint.effort = 0.0;
        }
    }

    fn apply_targets(&mut self, targets: &[JointTarget]) -> bool {
        for target in targets {
            let joint = &mut self.state[target.joint as usize];
            match target.mode {
                1 => joint.position = target.value,
                2 => joint.velocity = target.value,
                3 => joint.effort = target.value,
                _ => return false,
            }
        }
        self.targets += 1;
        true
    }

    fn reset_safety(&mut self) -> bool {
        self.resets += 1;
        true
    }

    fn read_state(&mut self, joints: &mut [JointState; MAX_JOINTS as usize]) -> StateMeta {
        joints[..JOINTS].copy_from_slice(&self.state);
        self.timestamp += 1;
        StateMeta { timestamp_ns: self.timestamp, safety: 0, fault: 0, joint_count: JOINTS as u8 }
    }
}

fn write_all(port: &mut File, bytes: &[u8]) {
    let mut remaining = bytes;
    while !remaining.is_empty() {
        match port.write(remaining) {
            Ok(0) => panic!("PTY write returned zero"),
            Ok(size) => remaining = &remaining[size..],
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock =>
                std::thread::sleep(Duration::from_millis(1)),
            Err(error) => panic!("PTY write failed: {error}"),
        }
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    let port_fd: i32 = args.next().expect("PTY descriptor").parse().unwrap();
    let control_fd: i32 = args.next().expect("completion descriptor").parse().unwrap();
    let mut port = unsafe { File::from_raw_fd(port_fd) };
    let mut control = unsafe { File::from_raw_fd(control_fd) };
    let fingerprint = std::array::from_fn(|index| index as u8);
    let mut protocol = DeviceProtocol::new(JOINTS as u8, fingerprint).unwrap();
    let zero = JointState { position: 0.0, velocity: 0.0, effort: 0.0 };
    let mut device = FakeDevice { state: [zero; JOINTS], stops: 0, resets: 0, targets: 0, timestamp: 0 };
    let start = Instant::now();
    let mut last_state = Instant::now();
    let mut read_buffer = [0u8; 32];
    let mut frame = [0u8; MAX_FRAME_SIZE];
    let mut outbound = [[0u8; MAX_FRAME_SIZE]; 2];
    let mut sizes = [0usize; 2];
    loop {
        let mut signal = [0u8; 1];
        match control.read(&mut signal) {
            Ok(0) => break,
            Ok(_) => panic!("unexpected control signal"),
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(error) => panic!("control pipe failed: {error}"),
        }
        let now = start.elapsed().as_nanos() as u64;
        if protocol.poll_watchdog(now, WATCHDOG_NS, &mut device) {
            let size = protocol.encode_state(&mut device, &mut frame).unwrap();
            write_all(&mut port, &frame[..size]);
        }
        if protocol.session() != 0 && last_state.elapsed() >= Duration::from_millis(25) {
            let size = protocol.encode_state(&mut device, &mut frame).unwrap();
            write_all(&mut port, &frame[..size]);
            last_state = Instant::now();
        }
        match port.read(&mut read_buffer) {
            Ok(size) if size > 0 => {
                let accepted = protocol.feed(&read_buffer[..size], now, &mut device, |bytes| {
                    let slot = sizes.iter().position(|size| *size == 0).unwrap();
                    outbound[slot][..bytes.len()].copy_from_slice(bytes);
                    sizes[slot] = bytes.len();
                });
                for index in 0..sizes.len() {
                    if sizes[index] != 0 {
                        write_all(&mut port, &outbound[index][..sizes[index]]);
                        sizes[index] = 0;
                    }
                }
                if accepted != 0 {
                    let size = protocol.encode_state(&mut device, &mut frame).unwrap();
                    write_all(&mut port, &frame[..size]);
                }
            }
            Ok(_) => {}
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock || error.raw_os_error() == Some(5) => {}
            Err(error) => panic!("PTY read failed: {error}"),
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    assert!(device.stops >= 2, "session takeover and disconnect must stop the device");
    assert!(device.resets >= 2, "each controller must explicitly reset safety");
    assert!(device.targets > 0, "no target reached the device");
}
