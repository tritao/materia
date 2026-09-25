#![no_std]
#![no_main]

use cortex_m::peripheral::DWT;
use cortex_m_rt::entry;
use embedded_hal_old::serial::{Read, Write};
use nb::Error::WouldBlock;
use panic_halt as _;
use robotkit_device_protocol::{
    Device, DeviceProtocol, JointState, JointTarget, StateMeta, MAX_FRAME_SIZE, MAX_JOINTS,
};
use stm32g4xx_hal::{prelude::*, pwr::PwrExt, rcc, serial::FullConfig, stm32};

mod fingerprint;

const JOINTS: usize = 2;
const STATE_PERIOD_NS: u64 = 25_000_000;
const WATCHDOG_NS: u64 = 500_000_000;
const VIRTUAL_LIMIT: f32 = 100.0;
// HSI runs at 16 MHz. Confirm oscillator accuracy and timing on the bench.
const HSI_CYCLES_PER_MICROSECOND: u64 = 16;

struct FakeDevice {
    joints: [JointState; JOINTS],
    timestamp_ns: u64,
    last_update_ns: u64,
}

impl FakeDevice {
    fn new() -> Self {
        let zero = JointState { position: 0.0, velocity: 0.0, effort: 0.0 };
        Self { joints: [zero; JOINTS], timestamp_ns: 0, last_update_ns: 0 }
    }

    fn tick(&mut self, now_ns: u64) {
        let dt_ns = now_ns.saturating_sub(self.last_update_ns);
        let dt_s = dt_ns as f32 / 1_000_000_000.0;
        for joint in &mut self.joints {
            joint.position = (joint.position + joint.velocity * dt_s)
                .clamp(-VIRTUAL_LIMIT, VIRTUAL_LIMIT);
        }
        self.timestamp_ns = now_ns;
        self.last_update_ns = now_ns;
    }
}

impl Device for FakeDevice {
    fn stop_all(&mut self) {
        for joint in &mut self.joints {
            joint.velocity = 0.0;
            joint.effort = 0.0;
        }
    }

    fn apply_targets(&mut self, targets: &[JointTarget]) -> bool {
        if targets.iter().any(|target| target.value.abs() > VIRTUAL_LIMIT) {
            return false;
        }
        for target in targets {
            let joint = &mut self.joints[target.joint as usize];
            match target.mode {
                1 => joint.position = target.value,
                2 => joint.velocity = target.value,
                3 => joint.effort = target.value,
                _ => return false,
            }
        }
        true
    }

    fn reset_safety(&mut self) -> bool { true }

    fn read_state(&mut self, joints: &mut [JointState; MAX_JOINTS as usize]) -> StateMeta {
        joints[..JOINTS].copy_from_slice(&self.joints);
        StateMeta {
            timestamp_ns: self.timestamp_ns,
            safety: 0,
            fault: 0,
            joint_count: JOINTS as u8,
        }
    }
}

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi(), pwr);
    let gpioc = dp.GPIOC.split(&mut rcc);
    // NUCLEO-G474RE D1/D0: PC4 USART1_TX, PC5 USART1_RX.
    let tx_pin = gpioc.pc4.into_alternate();
    let rx_pin = gpioc.pc5.into_alternate();
    let serial = dp.USART1.usart(
        tx_pin, rx_pin, FullConfig::default().baudrate(460_800.bps()), &mut rcc,
    ).unwrap();
    let (mut tx, mut rx) = serial.split();

    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
    let mut previous_cycles = DWT::cycle_count();
    let mut elapsed_cycles = 0u64;
    let mut protocol = DeviceProtocol::new(JOINTS as u8, fingerprint::MODEL_FINGERPRINT).unwrap();
    let mut device = FakeDevice::new();
    let mut last_state_ns = 0u64;
    let mut state_frame = [0u8; MAX_FRAME_SIZE];

    loop {
        let current_cycles = DWT::cycle_count();
        elapsed_cycles += current_cycles.wrapping_sub(previous_cycles) as u64;
        previous_cycles = current_cycles;
        let now_ns = elapsed_cycles / HSI_CYCLES_PER_MICROSECOND * 1_000;
        device.tick(now_ns);

        if protocol.poll_watchdog(now_ns, WATCHDOG_NS, &mut device) {
            send_state(&protocol, &mut device, &mut state_frame, &mut tx);
        }
        if protocol.session() != 0 && now_ns - last_state_ns >= STATE_PERIOD_NS {
            send_state(&protocol, &mut device, &mut state_frame, &mut tx);
            last_state_ns = now_ns;
        }

        match rx.read() {
            Ok(byte) => {
                protocol.feed(&[byte], now_ns, &mut device, |frame| {
                    for &value in frame {
                        nb::block!(tx.write(value)).unwrap();
                    }
                });
            }
            Err(WouldBlock) => {}
            Err(nb::Error::Other(_)) => {
                // A UART error consumes no watchdog credit; valid traffic must resume.
            }
        }
    }
}

fn send_state<T>(protocol: &DeviceProtocol, device: &mut FakeDevice,
                 frame: &mut [u8; MAX_FRAME_SIZE], tx: &mut T)
where T: Write<u8> {
    let size = protocol.encode_state(device, frame).unwrap();
    for &byte in &frame[..size] {
        nb::block!(tx.write(byte)).ok();
    }
}
