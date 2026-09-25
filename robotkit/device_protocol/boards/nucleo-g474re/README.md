# NUCLEO-G474RE RKD5 bench adapter

This is the external MCU reference target for Linux-to-device UART bring-up.
It runs the unchanged `robotkit-device-protocol` core with two virtual wheel
joints. No pins drive motors. Targets change only the in-memory state; velocity
targets integrate into virtual position. The adapter provides USART1 RX/TX,
DWT-derived monotonic time, a 500 ms command watchdog, and 40 Hz STATE.

The board uses its HSI 16 MHz clock and USART1 at 460800 baud, 8N1. On the
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

The fingerprint constant in `src/fingerprint.rs` comes from
`../../../deployment/bench-nucleo-g474re/layout.json` and its adjacent
`device_wire.lock.json`. If either changes, regenerate it with
`tools/device_fingerprint.py` and update `robot.json` with the printed hex.
`robotd` verifies those same bytes at startup. The bench `target_error` and
virtual layout are test settings, not drivetrain calibration.

Copy the bench deployment directory to the Linux board, edit only the `device.path`
for its actual UART node, then start from the `robotkit` directory:

```sh
../haxeon/scripts/haxeon run --project robotd/haxeon.json -- \
  --server --deployment=/path/to/bench-nucleo-g474re/robot.json \
  --listen=192.168.10.20
```

From the `robotkit` directory on a client machine, the integration probe can exercise reset, velocity,
disconnect, emergency stop, reconnect, reset, and a new position command:

```sh
../haxeon/scripts/haxeon run --project tests/integration/haxeon.json -- \
  --device --host=192.168.10.20
```

Before any actuator is attached, capture UART traces and measure startup,
STATE cadence, round-trip latency, and watchdog stop time. Repeat after MCU
reset, Linux reboot, host kill, cable removal/reconnection, malformed frames,
and fingerprint mismatch. Physical timing and fail-safe behavior have not yet
been measured.
