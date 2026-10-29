# Pendant firmware (skeleton)

Rust (`embassy-stm32`) firmware for the jog pendant. It sends the `PENDANT_STATE` heartbeat as a CAN-FD frame
every 10 ms using the generated `operator-link` codec. The controls are not read yet.

| Feature | Chip | Target | Use |
|---|---|---|---|
| `nucleo-g474` | STM32G474RE | `thumbv7em-none-eabihf` | Bench, on a Nucleo with a CAN-FD transceiver (FDCAN1 on PA11/PA12) |
| `h503` | STM32H503RB | `thumbv8m.main-none-eabihf` | The pendant's own MCU (FDCAN1 on PB12/PB13) |

```text
cargo build --release --target thumbv7em-none-eabihf     --features nucleo-g474
cargo build --release --target thumbv8m.main-none-eabihf --features h503
```

Both build to a linked image (about 39 KB; `.cargo/config.toml` passes cortex-m-rt's `link.x` for each target, and a
target without it links an empty image); neither has run on hardware. embassy-stm32 0.6 provides `CanConfigurator`,
`set_fd_data_bitrate` and `write_fd`. The FDCAN kernel clock is still the default: the board clocks it from an HSE crystal,
so set the clock tree from that crystal before relying on the bit timing. Next: the wheel on TIM2's encoder mode, the
keys with debounce, the selectors through the 74HC165 chain on SPI1, the OLED on I2C1, and receiving `PENDANT_CONFIG`
and `PENDANT_OUTPUT`.

## Pendant pin assignment (STM32H503RBT6, LQFP-64)

Alternate functions are from stm32-metapac 21 (ST's data). The board (pcbkit, EN4) wires these nets; the firmware uses
these pins. The axis (12 positions) and step (3) switches are read through two 74HC165 on SPI1, so the keys keep their own
pins and 11 pins stay free. F1 to F3 (keys and LEDs) and two spare pins form the matrix group a later membrane keypad reuses
(`PENDANT_PLAN.md`, "Board v1 pin budget").

| Pin | Net | Function |
|---|---|---|
| PA0 | ENC_A | TIM2_CH1 (AF1), the wheel's phase A through a 5 V-tolerant buffer |
| PA1 | ENC_B | TIM2_CH2 (AF1), phase B |
| PA2 | ESTOP_SENSE | E-stop loop read-back (isolated input) |
| PA4 | SR_LOAD | 74HC165 parallel load |
| PA5 | SR_SCK | SPI1_SCK (AF5) |
| PA6 | SR_DATA | SPI1_MISO (AF5) |
| PA11 / PA12 | USB_DM / USB_DP | USB (AF10) |
| PA13 / PA14 | SWDIO / SWCLK | SWD |
| PB0 / PB1 / PB2 / PB3 | KEY_START / KEY_HOLD / KEY_STOP / KEY_WHEEL_EN | key contacts (pull-up, interrupt) |
| PB4 / PB5 / PB8 / PB10 | LED_START / LED_HOLD / LED_STOP / LED_WHEEL_EN | key LED drivers |
| PB6 / PB7 | I2C1_SCL / I2C1_SDA | I2C1 (AF4): the OLED, and a keypad ID memory later |
| PB12 / PB13 | CAN_RX / CAN_TX | FDCAN1 (AF9) to the isolated transceiver |
| PB14 | OLED_RES | the OLED's reset |
| PC0 | KEYPAD_ID | ADC1_INP10, a keypad ID resistor later |
| PC1 to PC4 | ENABLE_1 to ENABLE_4 | reserved: the enabling switch's channels |
| PC6 / PC7 / PC8 | KEY_F1 / KEY_F2 / KEY_F3 | key contacts; matrix group |
| PC9 / PC10 / PC11 | LED_F1 / LED_F2 / LED_F3 | key LED drivers; matrix group |
| PC12, PD2 | MATRIX_5, MATRIX_6 | spare; matrix group |
| PH0 / PH1 | OSC_IN / OSC_OUT | HSE crystal |
| PA3, PA7, PA8, PA9, PA10, PA15, PB15, PC5, PC13, PC14, PC15 | — | free (PA9/PA10: USART1 for a debug console; PC13 to PC15 are low-drive) |

To check on the datasheet before the board is drawn: which of these pins are 5 V tolerant (the wheel's phases are buffered
either way), and the drive limits of the LED pins (the 16 mm buttons' LEDs are driven through transistors, not directly).
