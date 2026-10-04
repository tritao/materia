# NUCLEO-G474RE RKD6 minimal bench adapter

This is the external MCU reference target for Linux-to-device UART bring-up.
It runs the no_std RKD6 scheduled core with two virtual wheel joints. No pins drive motors. The adapter provides USART1 RX/TX and DWT-derived monotonic time.

The optional welding connector profile in `src/welder_profile.rs` reserves PA4/DAC1 channel 1
for wire speed, PA5/DAC1 channel 2 for voltage, PB2 for the optoMOS trigger, PB0/ADC1 channel 15
for Hall current, PB1/ADC1 channel 12 for isolated arc voltage, and PB10 for independent touch.
HAL trait checks verify the DAC and ADC assignments with `cargo check --offline`.
The DAC pins require isolated external amplifiers to produce 0–10 V; the sensor inputs require
conditioning to the ADC voltage range. None of these pins conflict with PC4/PC5 UART.
The bench image retains its virtual outputs; this pin profile does not enable or flash a welding driver.

`../welder_retrofit.rs` defines the shared retrofit I/O policy. Calibration specifies wire speed
and welding voltage at 10 V, Hall zero and A/V, isolated arc V/V, current threshold, no-arc timeout,
persistent-short threshold/duration and supply efficiency. Arc established comes from measured
current. A sustained high-current/low-voltage short reports the existing wire-stuck fault (3);
brief MIG shorts are allowed. Faults latch until an explicit reset with safe outputs.
The profile is tested against virtual I/O in `device_virtual/src/retrofit_tests.rs`.

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

The board's identity is its factory-programmed 96-bit unique ID (read at
0x1FFF7590, zero-padded to the protocol's 16 bytes). It is not compiled in, so
one firmware image serves every board of this type, and no configuration change
needs a reflash. A deployment names the board it is for in `device.controller`;
the board refuses a session for another id and always reports its own, so
`robotd identify <device path> <baud>` prints the id of the board on a port.
The firmware does not read the UID register on a host build, so this part is
checked on hardware only.
The board advertises the RKD6 minimal profile: two virtual joints, degree-1
segments, an eight-segment queue, 40 kHz logical step tick, no physical step
output and position setpoints only. USART1 runs at 921600 baud. The board
uses DWT monotonic ticks and publishes state every 25 ms. No motor pins are
driven.

Copy the bench deployment directory to the Linux board, edit `device.path`
for the UART node, and run `robotd --server --auth=... --deployment=...`. Before physical
actuators are attached, measure UART timing and stop behavior on the bench.
