//! Pendant firmware: thin glue between the hardware and `pendant-core`, which holds everything the pendant does (debounce, the
//! `PENDANT_STATE` heartbeat, the controller's labels and outputs, the screen). This file reads what the hardware offers into
//! `pendant_core::Inputs`, sends the frames the core returns and passes the controller's frames to it. `board` reads the controls (wheel
//! on TIM2's encoder mode, keys, the selector chain on SPI1, the E-stop read-back) and drives the key LEDs and the OLED on I2C1; the
//! bench Nucleo has none of those parts and reports fixed inputs. Compile-checked for both chips; not yet run on hardware. The
//! pendant's pins come from the board's pin table (`build.rs` generates `pins`).
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::peripherals::FDCAN1;
use embassy_stm32::{bind_interrupts, can, Config};
use embassy_futures::select::{select, Either};
use embedded_can::Id;
use operator_link::STATE_ID;
use panic_halt as _;
use pendant_core::PendantCore;

mod board;

#[cfg(feature = "h503")]
mod pins {
    include!(concat!(env!("OUT_DIR"), "/pins.rs"));
}

#[cfg(feature = "nucleo-g474")]
bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

#[cfg(feature = "h503")]
bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Config::default());
    // The bench Nucleo's transceiver is on PA11/PA12; on the pendant those are USB, so FDCAN1 uses PB12/PB13.
    #[cfg(feature = "nucleo-g474")]
    let (mut board, mut can) = (board::Board::new(), can::CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs));
    #[cfg(feature = "h503")]
    let (mut board, mut can) = {
        let pins = pins::take_pins!(p);
        let (board, (can_rx, can_tx)) = board::Board::new(pins, p.TIM2, p.SPI1, p.I2C1);
        (board, can::CanConfigurator::new(p.FDCAN1, can_rx, can_tx, Irqs))
    };
    can.set_bitrate(500_000);
    can.set_fd_data_bitrate(2_000_000, true);
    let can = can.start(can::OperatingMode::NormalOperationMode);
    let (mut tx, mut rx, _) = can.split();

    let mut core = PendantCore::new(0);
    let started = embassy_time::Instant::now();
    loop {
        let now = started.elapsed().as_millis() as u32;
        let inputs = board.read(now);
        if let Some(payload) = core.tick(now, &inputs) {
            let frame = can::frame::FdFrame::new_standard(STATE_ID as u16, &payload).unwrap();
            let _ = tx.write_fd(&frame).await;
        }
        // Wait for a controller frame for up to a millisecond, then tick again.
        if let Either::First(Ok(envelope)) = select(rx.read_fd(), embassy_time::Timer::after_millis(1)).await {
            if let Id::Standard(id) = envelope.frame.id() {
                let _ = core.on_frame(now, id.as_raw() as u32, envelope.frame.data());
            }
        }
        board.show(now, &core);
    }
}
