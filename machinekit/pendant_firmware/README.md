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

The pin table is data: `machinekit/examples/pendant/electronics/pendant/pendant.pins.json`, one entry per port pin (net,
function, note; null for the 11 free pins). The board's circuit wires those nets on those pins and pcbkit's pin check proves
the netlist follows the table; `build.rs` generates this firmware's `pins` module from the same file (a field per assigned
net, `pins::take_pins!(p)`), so a pin moved in the table moves in both. A pin that cannot carry the peripheral it is given
fails to compile where the field is used. Alternate functions are from stm32-metapac 21 (ST's data). The axis (12
positions) and step (3) switches are read through two 74HC165 on SPI1; F1 to F3 (keys and LEDs) and two spare pins form the
matrix group a later membrane keypad reuses (`PENDANT_PLAN.md`, "Board v1 pin budget"). PA8, PA9, PA15 and PB3 to PB8 are
in the VDDIO2 domain, which the LQFP-64 package bonds to VDD internally (DS14053: a separate VDDIO2 pin exists only on
WLCSP25).
