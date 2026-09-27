# NUCLEO-G474RE RKD6 minimal bench adapter

This is the external MCU reference target for Linux-to-device UART bring-up.
It runs the no_std RKD6 scheduled core with two virtual wheel joints. No pins drive motors. The adapter provides USART1 RX/TX and DWT-derived monotonic time.

The board uses its HSI 16 MHz clock and USART1 at 921600 baud, 8N1. On the
NUCLEO-G474RE, Arduino D1 is PC4/USART1_TX and D0 is PC5/USART1_RX. Connect
D1 to the STM32MP1 UART RX, D0 to its TX, and connect grounds. Both ends must
use 3.3 V logic. Verify the exact MP1 header pins and UART device node for the
chosen Olimex board before wiring.

From this directory:

```sh
rustup target add thumbv7em-none-eabihf
cargo build --release
# Flash target/thumbv7em-none-eabihf/release/robotkit-nucleo-g474re
# using the board's ST-LINK tool; example with probe-rs:
probe-rs download --chip STM32G474RETx \
  target/thumbv7em-none-eabihf/release/robotkit-nucleo-g474re
```

The fingerprint constant in `src/fingerprint.rs` comes from the bench layout
and `robotkit/schema/device_wire6.lock.json`. Regenerate it with
`robotkit/tools/device_fingerprint.py` when either artifact changes.
The board advertises the RKD6 minimal profile: two virtual joints, degree-1
segments, an eight-segment queue, 40 kHz logical step tick, no physical step
output and position setpoints only. USART1 runs at 921600 baud. The board
uses DWT monotonic ticks and publishes state every 25 ms. No motor pins are
driven.

Copy the bench deployment directory to the Linux board, edit `device.path`
for the UART node, and run `robotd --server --deployment=...`. Before physical
actuators are attached, measure UART timing and stop behavior on the bench.
