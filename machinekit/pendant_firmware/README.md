# Pendant firmware (skeleton)

Rust (`embassy-stm32`) firmware for the jog pendant. It sends the `PENDANT_STATE` heartbeat as a CAN-FD frame
every 10 ms using the generated `operator-link` codec. The controls are not read yet.

| Feature | Chip | Target | Use |
|---|---|---|---|
| `nucleo-g474` | STM32G474RE | `thumbv7em-none-eabihf` | Bench, on a Nucleo with a CAN-FD transceiver |
| `g0b1` | STM32G0B1RE | `thumbv6m-none-eabi` | The pendant's own MCU |

```text
cargo build --release --target thumbv7em-none-eabihf --features nucleo-g474
cargo build --release --target thumbv6m-none-eabi   --features g0b1
```

FDCAN1 uses PA11 (RX) and PA12 (TX) on both. Both builds pass (embassy-stm32 0.6: `CanConfigurator`,
`set_fd_data_bitrate`, `write_fd`); neither has run on hardware. The G474 example in the embassy repository
also sets the FDCAN kernel clock from a PLL, which this skeleton leaves at the default: set it before relying
on the bit timing. Next: the wheel on an encoder timer, the switches and buttons with debounce, the OLED,
and receiving `PENDANT_CONFIG` and `PENDANT_OUTPUT`.
