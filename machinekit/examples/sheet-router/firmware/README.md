# SR15 host-checked firmware scope

Run `machinekit/examples/sheet-router/firmware/check.sh` from the repository root.
It checks the pin allocation, runs the focused no_std router policy's host tests,
and cross-checks the existing G474 bench firmware for `thumbv7em-none-eabihf`.
The bench firmware still has virtual motor outputs. These modules do not enable
or flash physical motion. Register/DMA bring-up and measurements remain SR18;
RKD6 State6 encoder/fault integration and wire-format freeze remain SR16.

## Resource screen

[ST DS12288 Rev 6](https://www.st.com.cn/resource/en/datasheet/stm32g474re.pdf),
tables 7, 12, 13 and 45, is the reviewed datasheet. `af-evidence.json` records a
manually reviewed subset; `check.py` checks the proposed map against it, verifies
Rust/JSON consistency, unique EXTI/DMA resources and the SR17 terminal reverse map.
It is an allocation check, not automated validation of a PCB drawing.
[RM0440](https://www.st.com/resource/en/reference_manual/dm00355726-stm32g4-series-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
provides the timer, DMA and watchdog register basis; DMA request IDs are also
cross-checked against the installed stm32g4xx-hal 0.1.0 source.

| Axis | Step timer / pin / AF | Encoder timer / A,B / AF | Direction |
|---|---|---|---|
| X | TIM15_CH1 / PA2 / 9 | TIM2 / PA0,PA1 / 1 | PC8 |
| Y left | TIM16_CH1 / PA6 / 1 | TIM3 / PB4,PB5 / 2 | PC9 |
| Y right | TIM17_CH1 / PA7 / 1 | TIM4 / PA11,PA12 / 10 | PC10 |
| Z | TIM20_CH1 / PB2 / 3 | TIM8 / PC6,PC7 / 4 | PC11 |
| Spare | TIM5_CH2 / PC12 / 1 | TIM1 / PA8,PA9 / 6 | PB0 |

Each step axis has its own counter, avoiding a shared PWM period. Encoders use
separate hardware quadrature counters. Step DMA occupies DMA1 channels 1–5;
USART1 RX/TX and SPI2 RX/TX occupy DMA2 channels 1–4. Fourteen fast inputs use
fourteen distinct EXTI lines. Remaining 44 inputs and 27 outputs fit three
16-bit input and two 16-bit output SPI expanders (assumed addressed MCP23S17
class, common SPI2 bus/CS, isolated/conditioned 24 V interfaces). The input
budget is 58 channels, below 64; output budget 27, below 32. Step/dir/encoders
are dedicated motion resources, not slow process output slots.

USART1 is PC4/PC5, SPI2 PB13/14/15 with PA3 CS; DAC1_OUT1 is PA4 with isolated
0–10 V conditioning. PA13/PA14 retain SWD. PB3 is a normally-low global output
permission; external pull-down and carrier gating must inhibit line drivers,
process valves, analog run and brake release even if expander latches retain
old bits. PB1 reserves an external-watchdog heartbeat; the independent MCU
watchdog does not rely on that optional extra hardware. SR17 STO is separate.
All 106 SR17 connector/terminal assignments have a device pin in `pin-map.json`.

The **MCU/package allocation fits**, including a fifth reserved axis. The stock
Nucleo's routing is not qualified: PC14/PC15 require LSE disabled/disconnected;
PA2/PA3 may have ST-Link VCP links, PA5 a user LED, and PB3/PB4 JTAG functions.
Inspect the exact board schematic/solder links before carrier use. Keep SWD,
disable full JTAG and UCPD dead-battery pull-downs where needed. Reserve PF0/PF1
for a stable HSE clock. Actual 170 MHz timer clocks, boost/flash setup and APB
clock multipliers must be checked during SR18; the minimal bench image remains
at its existing clock. No timing claim is made for that image's virtual outputs.

Fallback if those board pins cannot be freed: move fast/slow auxiliary I/O to a
second MCU with captured timestamps, keeping step/encoder timing on the G474.
If measured DMA deadlines fail, use a dedicated pulse/encoder coprocessor and
requalify SR18. Do not silently lower the 200 kHz requirement or count interrupts
as hardware quadrature. A fifth installed drive also requires extending SR17's
STO graph and power budget; the present four-drive machine uses four guard axes.

## Timing and stops

`robotkit/device_protocol/boards/router.rs` is shared no_std firmware policy.
At a proposed 10 MHz counter clock (170 MHz / 17), step high is 2 µs, low is
at least 3 µs, maximum 200 kHz/axis. Direction setup is 5 µs and hold is 5 µs
**after the previous falling edge**. A reversal therefore needs at least 12 µs
between rises. Queue preparation needs at least 5 µs lead. Invalid/late requests
leave the accepted pulse sequence unchanged. Both edges are admitted atomically.
At 5000 pulses/turn, XY at 40 m/min needs 166.667 kHz; Z at 10 m/min needs
83.333 kHz with the 10 mm screw lead. Drive pulse timings remain assumptions
until bought-drive data arrives.

The scheduler returns absolute edge/direction timestamps to the hardware adapter;
it is not a software bit-banging loop. Output-compare/DMA must consume them,
with no direction writes while pulses are active: stop/empty that axis queue,
observe hold, write direction, observe setup, then arm the next burst. The
adapter must account for any longer direction-change delay in execution timing.
The 3 ms queue horizon is below half a 16-bit counter wrap; disable/disarm a
channel across long idle gaps and re-arm it explicitly. A wrapping CCR alone
must never create an extra pulse. At five simultaneous maximum-rate axes the
CCR traffic is 8 MB/s writes plus 8 MB/s memory reads; arbitration/deadline
margin, not this average, must be measured. These register/DMA integration and
pulse-count/jitter checks are SR18 hardware gates.

The guard samples at 1 kHz. Commanded feedback is **emitted** pulses, not queued
end positions. Rational encoder scaling is checked without float rounding.
Assumed 10000 encoder edges and 5000 step pulses per motor turn give 2 counts
per pulse: .10 mm following-error trip is 50 counts on XY and 100 on Z. These
are trip thresholds, not a guarantee of post-stop positional error. Signed
16-bit encoder extension is checked through forward/reverse wraps; sample
before 32768 edges, with timestamp freshness <=2 ms. Calibrate origins, ratio
and polarity before enabling. The real pulse-train/counting test is SR18.

Following error, alarm/not-ready, stale feedback, expired lease, bad clock or
configuration and a missed 20 ms supervisor deadline latch a stop across all
axes: disable step/DMA permission and enables, spindle run/DAC, tool release
and brake release. Healthy disarmed polling may refresh IWDG but enables stay
low. A fault does not refresh it. Re-arm requires an explicit manual reset at
rest and valid fresh feedback/lease. Do not refresh from an ISR or blindly on
UART traffic. Dual-Y skew remains the existing independent scheduled-core check.

The compile-checked G474 adapter provides force-safe and IWDG start/refresh
functions. IWDG /4, reload 159 gives 18.824–21.695 ms over the datasheet LSI range
29.5–34 kHz; it is not an exact 20 ms timer. On a stalled CPU, reset plus the
carrier's external default-low permission must remove outputs. Measure this,
including expander latch isolation and Z brake behaviour, in SR18. The functions
are dormant in the minimal bench image until explicit carrier bring-up.

## Validation boundary

16 focused host tests cover pulse width/rate, direction reversal, rejected-pair
atomicity, counter wrap/horizon/overflow, encoder extension, following-error signs
and boundary, alarm/not-ready, feedback age/lease, watchdog latch, reset and bad
clock/configuration. Six pin-map negative fixtures reject collisions, wrong AF,
EXTI/DMA conflicts and broken terminal/device mappings. The full ARM bench target
cross-check passes. See `validation.json` and `pin-validation.json`.

An exploratory unfiltered `cargo test` also exposed existing f32/f64 mismatches
in unrelated integration tests (`tests/step_generator.rs`, `tests/virtual_board.rs`);
those files were left untouched. The focused `--lib router` gate above passes;
this packet does not claim the entire device-protocol integration suite passes.

SR18 owns bench parity against the virtual device, physical pulse counts/rate/
jitter, actual quadrature train, fast capture latency, forced-alarm/following-error
stops, watchdog output removal, Nucleo routing and isolated carrier validation.
