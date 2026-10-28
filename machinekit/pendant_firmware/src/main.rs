//! Pendant firmware skeleton: sends `PENDANT_STATE` as a CAN-FD frame every 10 ms (the heartbeat the cabinet
//! node's lease counts), using the generated `operator-link` codec. The controls (wheel timer, switches,
//! buttons, OLED) are not read yet, so the state is fixed: axis position 3, step 1, wheel enabled, chain closed.
//! Compile-checked for both chips; not yet run on hardware.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::peripherals::FDCAN1;
use embassy_stm32::{bind_interrupts, can, Config};
use operator_link::operator_wire::PendantState;
use operator_link::{FLAG_CHAIN_CLOSED, FLAG_WHEEL_ENABLE, STATE_ID};
use panic_halt as _;

#[cfg(feature = "nucleo-g474")]
bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

// On the G0B1 the FDCAN interrupt lines are shared with TIM16 and TIM17.
#[cfg(feature = "g0b1")]
bind_interrupts!(struct Irqs {
    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Config::default());
    let mut can = can::CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    can.set_bitrate(500_000);
    can.set_fd_data_bitrate(2_000_000, true);
    let mut can = can.start(can::OperatingMode::NormalOperationMode);

    let mut sequence: u16 = 0;
    loop {
        let state = PendantState { sequence, wheel_delta: 0, position: 3, step: 1, buttons: 0, flags: FLAG_WHEEL_ENABLE | FLAG_CHAIN_CLOSED };
        let mut payload = [0u8; PendantState::SIZE];
        state.encode(&mut payload).unwrap();
        let frame = can::frame::FdFrame::new_standard(STATE_ID as u16, &payload).unwrap();
        let _ = can.write_fd(&frame).await;
        sequence = sequence.wrapping_add(1);
        embassy_time::Timer::after_millis(10).await;
    }
}
