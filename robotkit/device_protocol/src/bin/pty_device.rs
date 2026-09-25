//! Desktop PTY adapter used only by the cross-language serial test.
use robotkit_device_protocol::{Device, DeviceProtocol, JointState, JointTarget, StateMeta, MAX_FRAME_SIZE, MAX_JOINTS};
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::FromRawFd;
use std::time::{Duration, Instant};

const WATCHDOG_NS: u64 = 5_000_000_000;

struct FakeDevice {
    stops: usize,
    resets: usize,
    targets: usize,
    timestamp: u64,
}

impl Device for FakeDevice {
    fn stop_all(&mut self) { self.stops += 1; }
    fn apply_targets(&mut self, targets: &[JointTarget]) -> bool {
        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].value, 1.1f32);
        self.targets += 1;
        true
    }
    fn reset_safety(&mut self) -> bool { self.resets += 1; true }
    fn read_state(&mut self, joints: &mut [JointState; MAX_JOINTS as usize]) -> StateMeta {
        self.timestamp += 1;
        joints[0] = JointState { position: 1.0, velocity: -2.0, effort: 0.5 };
        StateMeta { timestamp_ns: self.timestamp, safety: 0, fault: 0, joint_count: 1 }
    }
}

fn write_chunks(port: &mut File, bytes: &[u8]) {
    for chunk in bytes.chunks(3) {
        let mut remaining = chunk;
        while !remaining.is_empty() {
            match port.write(remaining) {
                Ok(0) => panic!("PTY write returned zero"),
                Ok(n) => remaining = &remaining[n..],
                Err(error) if error.kind() == std::io::ErrorKind::WouldBlock =>
                    std::thread::sleep(Duration::from_millis(1)),
                Err(error) => panic!("PTY write failed: {error}"),
            }
        }
    }
}

fn main() {
    // The C++ parent passes inherited PTY and completion-pipe descriptors.
    let mut args = std::env::args().skip(1);
    let port_fd: i32 = args.next().unwrap().parse().unwrap();
    let completion_fd: i32 = args.next().unwrap().parse().unwrap();
    let mut port = unsafe { File::from_raw_fd(port_fd) };
    let mut completion = unsafe { File::from_raw_fd(completion_fd) };
    let fingerprint = std::array::from_fn(|index| index as u8);
    let mut protocol = DeviceProtocol::new(1, fingerprint).unwrap();
    let mut device = FakeDevice { stops: 0, resets: 0, targets: 0, timestamp: 0 };
    let start = Instant::now();
    let mut read_buffer = [0u8; 17];
    let mut frame = [0u8; MAX_FRAME_SIZE];
    let mut ack = [0u8; MAX_FRAME_SIZE];
    let mut ack_size = 0;
    loop {
        let mut control = [0u8; 1];
        match completion.read(&mut control) {
            Ok(0) => break,
            Ok(1) if control[0] == b'W' => {
                let simulated_now = start.elapsed().as_nanos() as u64 + WATCHDOG_NS;
                assert!(protocol.poll_watchdog(simulated_now, WATCHDOG_NS, &mut device));
                let size = protocol.encode_state(&mut device, &mut frame).unwrap();
                write_chunks(&mut port, &frame[..size]);
            }
            Ok(_) => panic!("unexpected completion data"),
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(error) => panic!("completion pipe failed: {error}"),
        }
        let now_ns = start.elapsed().as_nanos() as u64;
        match port.read(&mut read_buffer) {
            Ok(n) if n > 0 => {
                let accepted = protocol.feed(&read_buffer[..n], now_ns, &mut device, |bytes| {
                    ack[..bytes.len()].copy_from_slice(bytes);
                    ack_size = bytes.len();
                });
                if ack_size != 0 {
                    write_chunks(&mut port, &ack[..ack_size]);
                    ack_size = 0;
                }
                if accepted != 0 {
                    let size = protocol.encode_state(&mut device, &mut frame).unwrap();
                    write_chunks(&mut port, &frame[..size]);
                }
            }
            Ok(_) => {}
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock ||
                error.raw_os_error() == Some(5) => {} // EIO while no slave is open.
            Err(error) => panic!("PTY read failed: {error}"),
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    assert_eq!(device.stops, 4); // Three sessions and one watchdog expiry.
    assert_eq!(device.resets, 2);
    assert_eq!(device.targets, 1);
    assert_eq!(protocol.last_sequence(), 0); // Mismatched final session is safe.
    assert!(!protocol.model_matches());
}
