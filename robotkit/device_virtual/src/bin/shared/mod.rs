use robotkit_device_virtual::VirtualDevice;
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::FromRawFd;
use std::time::{Duration, Instant};

fn write_all(port: &mut File, mut bytes: &[u8]) {
    while !bytes.is_empty() {
        match port.write(bytes) {
            Ok(0) => panic!("PTY write returned zero"),
            Ok(n) => bytes = &bytes[n..],
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock =>
                std::thread::sleep(Duration::from_millis(1)),
            Err(error) => panic!("PTY write failed: {error}"),
        }
    }
}

pub fn run(joints: usize, fingerprint: [u8; 16]) {
    let mut args = std::env::args().skip(1);
    let port_fd: i32 = args.next().expect("PTY descriptor").parse().unwrap();
    let control_fd: i32 = args.next().expect("completion descriptor").parse().unwrap();
    let mut port = unsafe { File::from_raw_fd(port_fd) };
    let mut control = unsafe { File::from_raw_fd(control_fd) };
    let scale = [1_000.0; 64];
    let mut device = VirtualDevice::new(1_000_000, 40_000, 50_000, 0,
        joints, scale, fingerprint, 2).expect("minimal RKD6 device");
    let start = Instant::now();
    let mut last_advance = Instant::now();
    let mut input = Vec::new();
    let mut read_buffer = [0u8; 4096];
    loop {
        let mut signal = [0u8; 1];
        match control.read(&mut signal) {
            Ok(0) => break,
            Ok(_) => panic!("unexpected control signal"),
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {},
            Err(error) => panic!("control pipe failed: {error}"),
        }
        if last_advance.elapsed() >= Duration::from_millis(1) {
            assert!(device.advance(start.elapsed().as_nanos() as u64));
            last_advance = Instant::now();
        }
        match port.read(&mut read_buffer) {
            Ok(n) if n > 0 => input.extend_from_slice(&read_buffer[..n]),
            Ok(_) => {},
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {},
            Err(error) if error.raw_os_error() == Some(5) => {}, // PTY slave not opened yet
            Err(error) => panic!("PTY read failed: {error}"),
        }
        while input.len() >= 8 {
            if &input[..4] != b"RKD6" { input.remove(0); continue; }
            let payload = u16::from_le_bytes([input[6], input[7]]) as usize;
            if payload > 5000 { input.remove(0); continue; }
            let size = 8 + payload + 4;
            if input.len() < size { break; }
            let frame: Vec<u8> = input.drain(..size).collect();
            if !device.feed(&frame) && !matches!(frame[4], 6 | 7) {
                panic!("RKD6 device rejected frame kind {}", frame[4]);
            }
        }
        while let Some(frame) = device.take_frame() { write_all(&mut port, &frame); }
        std::thread::sleep(Duration::from_millis(1));
    }
}
