//! The board's controls and indicators, read and driven for the core: the wheel on TIM2's encoder mode, the keys and the E-stop
//! read-back as inputs, the two selector switches through the 74HC165 chain on SPI1, the key LEDs, and the SSD1309 OLED on I2C1.
//! Everything that is a decision lives in `pendant_core::hardware` and the core; this file only moves bits to and from pins.
//! The bench Nucleo has none of these parts, so its `Board` reports the fixed inputs the firmware used before.

#[cfg(feature = "nucleo-g474")]
use pendant_core::{Inputs, PendantCore};

#[cfg(feature = "nucleo-g474")]
pub struct Board;

#[cfg(feature = "nucleo-g474")]
impl Board {
    pub fn new() -> Self { Board }

    pub fn read(&mut self, _now_ms: u32) -> Inputs {
        Inputs { position: 3, step: 1, wheel_enable: true, chain_closed: true, ..Inputs::default() }
    }

    pub fn show(&mut self, _now_ms: u32, _core: &PendantCore) {}
}

#[cfg(feature = "h503")]
pub use h503::Board;

#[cfg(feature = "h503")]
mod h503 {
    use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
    use embassy_stm32::i2c::{self, I2c};
    use embassy_stm32::mode::Blocking;
    use embassy_stm32::peripherals::{I2C1, PB12, PB13, SPI1, TIM2};
    use embassy_stm32::Peri;
    use embassy_stm32::spi::{self, Spi};
    use embassy_stm32::time::Hertz;
    use embassy_stm32::timer::qei::{self, Qei};
    use pendant_core::hardware::{self, Selectors};
    use pendant_core::screen::{Framebuffer, WIDTH};
    use pendant_core::{DisplayPower, Inputs, PendantCore};

    /// The selector chain is read this often; the switches are slow and the 74HC165 needs no more.
    const SELECTOR_PERIOD_MS: u32 = 5;
    /// The screen is composed this often; a page is sent when it differs from what the display holds.
    const SCREEN_PERIOD_MS: u32 = 20;

    /// A key or sense input; the pull-ups are on the board.
    fn input(pin: Peri<'static, impl embassy_stm32::gpio::Pin>) -> Input<'static> { Input::new(pin, Pull::None) }

    fn led(pin: Peri<'static, impl embassy_stm32::gpio::Pin>) -> Output<'static> { Output::new(pin, Level::Low, Speed::Low) }

    pub struct Board {
        wheel: Qei<'static, TIM2>,
        sr_load: Output<'static>,
        sr: Spi<'static, Blocking, spi::mode::Master>,
        keys: [Input<'static>; 6],
        wheel_enable: Input<'static>,
        estop_sense: Input<'static>,
        leds: [Output<'static>; 6],
        oled: I2c<'static, Blocking, i2c::Master>,
        /// The last complete reading of each switch, kept while it is between two positions.
        position: u8,
        step: u8,
        next_selector_ms: u32,
        next_screen_ms: u32,
        /// What the display holds, and which of its pages differ from the screen the core wants.
        shown: Framebuffer,
        stale: u8,
        power: Option<DisplayPower>,
    }

    impl Board {
        /// Takes the board's pins and the peripherals that serve them; the CAN pins are not this board's and are handed back.
        pub fn new(
            pins: crate::pins::Pins,
            tim2: Peri<'static, TIM2>,
            spi1: Peri<'static, SPI1>,
            i2c1: Peri<'static, I2C1>,
        ) -> (Self, (Peri<'static, PB12>, Peri<'static, PB13>)) {
            let can = (pins.can_rx, pins.can_tx);
            let wheel = Qei::new(tim2, pins.enc_a, pins.enc_b, qei::Config::default());
            let mut spi_config = spi::Config::default();
            spi_config.frequency = Hertz::mhz(1);
            // The 74HC165 shifts on the rising edge and presents the next bit after it: mode 0.
            let sr = Spi::new_blocking_rxonly(spi1, pins.sr_sck, pins.sr_data, spi_config);
            let mut i2c_config = i2c::Config::default();
            i2c_config.frequency = Hertz::khz(400);
            let oled = I2c::new_blocking(i2c1, pins.i2c1_scl, pins.i2c1_sda, i2c_config);
            let mut reset = Output::new(pins.oled_res, Level::Low, Speed::Low);
            // The display needs its reset held for a moment, then time to start before the first command.
            cortex_m::asm::delay(64_000);
            reset.set_high();
            core::mem::forget(reset);
            cortex_m::asm::delay(640_000);
            let mut board = Board {
                wheel,
                sr_load: Output::new(pins.sr_load, Level::High, Speed::Low),
                sr,
                keys: [
                    input(pins.key_start),
                    input(pins.key_hold),
                    input(pins.key_stop),
                    input(pins.key_f1),
                    input(pins.key_f2),
                    input(pins.key_f3),
                ],
                wheel_enable: input(pins.key_wheel_en),
                estop_sense: input(pins.estop_sense),
                leds: [
                    led(pins.led_start),
                    led(pins.led_hold),
                    led(pins.led_stop),
                    led(pins.led_f1),
                    led(pins.led_f2),
                    led(pins.led_f3),
                ],
                oled,
                position: 0,
                step: 0,
                next_selector_ms: 0,
                next_screen_ms: 0,
                shown: Framebuffer::new(),
                stale: 0xFF,
                power: None,
            };
            let mut init = [0u8; 1 + hardware::OLED_INIT.len()];
            init[0] = hardware::OLED_COMMAND;
            init[1..].copy_from_slice(&hardware::OLED_INIT);
            let _ = board.oled.blocking_write(hardware::OLED_ADDRESS, &init);
            (board, can)
        }

        /// The raw reading of the controls. A key reads low when pressed, the E-stop sense high while the loop is energised.
        pub fn read(&mut self, now_ms: u32) -> Inputs {
            if now_ms.wrapping_sub(self.next_selector_ms) as i32 >= 0 {
                self.next_selector_ms = now_ms.wrapping_add(SELECTOR_PERIOD_MS);
                self.read_selectors();
            }
            let mut keys = 0u8;
            for (bit, key) in self.keys.iter().enumerate() {
                if key.is_low() { keys |= 1 << bit }
            }
            Inputs {
                wheel_counts: self.wheel.count(),
                position: self.position,
                step: self.step,
                keys,
                wheel_enable: self.wheel_enable.is_low(),
                chain_closed: hardware::chain_closed(self.estop_sense.is_high()),
            }
        }

        fn read_selectors(&mut self) {
            // Load the contacts into the registers, then shift all 16 bits out.
            self.sr_load.set_low();
            cortex_m::asm::delay(64);
            self.sr_load.set_high();
            let mut word = [0u8; 2];
            if self.sr.blocking_read(&mut word).is_ok() {
                let Selectors { position, step } = hardware::selectors(u16::from_be_bytes(word));
                if let Some(position) = position { self.position = position }
                if let Some(step) = step { self.step = step }
            }
        }

        /// Drives the key LEDs and keeps the display showing what the core wants: one page per call, so the heartbeat is never held up
        /// by more than a page's transfer.
        pub fn show(&mut self, now_ms: u32, core: &PendantCore) {
            let leds = core.leds(now_ms);
            for (bit, led) in self.leds.iter_mut().enumerate() {
                led.set_level(if leds & (1 << bit) != 0 { Level::High } else { Level::Low });
            }
            if now_ms.wrapping_sub(self.next_screen_ms) as i32 >= 0 {
                self.next_screen_ms = now_ms.wrapping_add(SCREEN_PERIOD_MS);
                let want = core.screen(now_ms).render();
                for page in 0..want.pages.len() {
                    if want.pages[page] != self.shown.pages[page] {
                        self.shown.pages[page] = want.pages[page];
                        self.stale |= 1 << page;
                    }
                }
                let power = core.display_power(now_ms);
                if self.power != Some(power) {
                    self.power = Some(power);
                    let mut command = [0u8; 5];
                    command[0] = hardware::OLED_COMMAND;
                    command[1..].copy_from_slice(&hardware::oled_power(power));
                    let _ = self.oled.blocking_write(hardware::OLED_ADDRESS, &command);
                }
            }
            if self.stale != 0 {
                let page = self.stale.trailing_zeros() as usize;
                let mut command = [0u8; 4];
                command[0] = hardware::OLED_COMMAND;
                command[1..].copy_from_slice(&hardware::oled_page(page as u8));
                let mut data = [0u8; 1 + WIDTH];
                data[0] = hardware::OLED_DATA;
                data[1..].copy_from_slice(&self.shown.pages[page]);
                let sent = self.oled.blocking_write(hardware::OLED_ADDRESS, &command).is_ok()
                    && self.oled.blocking_write(hardware::OLED_ADDRESS, &data).is_ok();
                // A failed write leaves the page stale, so it is sent again on the next call.
                if sent { self.stale &= !(1 << page) }
            }
        }
    }
}
