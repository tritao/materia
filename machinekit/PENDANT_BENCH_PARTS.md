# Pendant bench prototype: parts to order (stage 1)

For one pendant, internal use. The aim is to learn the real signals and the feel before any board is
designed (`PENDANT_PLAN.md`, phase P1). Everything here is an unchecked suggestion: part families and
search terms, not verified listings or prices. Cheap parts are bought twice so one wrong guess
(output type, pinout, a dead module) does not stall the bench.

## What the bench is

Nucleo-G474RE (owned) wired by jumpers to the controls on a flat board, a CAN-FD transceiver, and a
USB-CAN adapter to a PC or Pi. Firmware speaks the `PENDANT_*` frames from `operator_link`. The
Nucleo is powered over USB; the 24 V path is rehearsed separately.

## Order list

| # | Part | Qty | Ask for | Search terms | Check on arrival |
|---|---|---:|---|---|---|
| 1 | **MPG handwheel**, 60 mm, 100 PPR | 2 | 5 V supply, A and B outputs, preferably non-differential; a detent is a plus. Buy two from different listings if the outputs differ. | `60mm 100PPR MPG manual pulse generator handwheel 5V`, `MPG 100 PPR A B 5V` | Pinout (5 V, 0 V, A, B, maybe A-, B-); output type (open collector, TTL, line driver); supply current; counts per turn on a logic analyzer. |
| 2 | **Rotary switch, 12 position**, panel mount | 2 | 1 pole 12 throw, break-before-make (non-shorting), good detent. We read OFF plus 11 positions. | `12 position rotary switch 1 pole 12 throw panel mount`, `rotary switch 1P12T non-shorting` | Detent feel; each position closes exactly one contact; contact bounce. |
| 3 | **Rotary switch, 3 position**, panel mount | 2 | 1 pole 3 throw (x1, x10, x100). | `3 position rotary switch 1P3T panel mount` | As above. |
| 4 | **Illuminated momentary buttons**, 12 to 16 mm | 8 (+3 spare) | 6 with LEDs: green, yellow, red for the fixed keys and 3 neutral for F1 to F3, with LEDs for 5 V or 12 V with a series resistor. Plus 1 plain momentary for wheel enable (no LED). | `16mm momentary push button LED 5V`, `12mm illuminated momentary pushbutton` | LED forward voltage and current; contact bounce; the feel through a glove. |
| 5 | **Emergency stop**, 22 mm, with 1 NC contact block | 1 | Latching, twist or pull release, NC contact block. A branded, standards-marked part is worth the price even for a bench. | `22mm emergency stop mushroom latching 1NC`, `E-stop 22mm contact block NC` | Mounting hole size and depth; contact block rating; that it fits the case thickness. |
| 6 | **OLED module**, 128x64, I2C, 3.3 V | 2 | SSD1306 or SH1106, 0.96 to 1.3 in. | `1.3 inch OLED 128x64 I2C SH1106`, `0.96 OLED SSD1306 I2C` | I2C address; supply current; contrast; does it read at arm's length. |
| 7 | **CAN-FD transceiver breakout**, 3.3 V logic | 2 | TJA1051, TCAN1042 or MCP2562FD class. Not a classic-CAN-only part. | `TJA1051 breakout`, `TCAN1042 breakout`, `MCP2562FD breakout` | 3.3 V logic compatible; 120 ohm termination jumper or solder pad. |
| 8 | **USB-CAN FD adapter** | 1 | gs_usb or candleLight firmware, CAN FD capable. | `CANable 2.0`, `candleLight FD USB CAN` | Shows up as a `can0` interface and does 2 Mbit/s data phase. |
| 9 | **120 ohm resistors**, 1/4 W | 4 | Bus termination at both ends. | any | Measure about 60 ohm across the bus with both ends fitted. |
| 10 | **M12 A-coded 12-pin** panel connector and a **5 m shielded cable** to match | 1 each | The real connector and cable, to retire the sourcing risk and test the noise. | `M12 A-coded 12 pin panel mount connector`, `M12 12 pin cable 5m shielded` | Availability and price; pin numbering; wire gauge; cable shielding. |
| 11 | **24 V to 5 V DC-DC module** | 2 | Wide input, 24 V in, 5 V at 1 A. | `24V to 5V buck converter 3A module` | Output under load; ripple on a scope if one is to hand. |
| 12 | **Level shifting** | 1 pack | 5 V MPG outputs into 3.3 V inputs: resistors for dividers, or a 74LVC/74HCT buffer. Check first whether the Nucleo pins are 5 V tolerant. | `74LVC245 breakout`, `5V 3.3V level shifter 4 channel` | MPG high level; edge shape. |
| 13 | **Prototyping** | | Breadboard or perfboard, jumpers, pin headers, 2 to 3 m of 4-wire shielded cable, ferrules, heat shrink. | | |
| 14 | **Logic analyzer**, 8 channel USB (skip if owned) | 1 | Saleae-compatible. | `USB logic analyzer 8 channel 24MHz` | Sees the MPG A/B edges. |

A bench supply (24 V) and a multimeter are assumed. A scope helps with items 1, 7 and 11.

## Pin budget on the Nucleo

About 35 GPIOs: wheel A and B (one timer), 12 axis-switch inputs, 3 step-switch inputs, 7 button
inputs (6 keys and wheel enable), 6 LED outputs, 2 for I2C, 2 for FDCAN, 1 for the E-stop read-back.
The Morpho headers have enough. The final MCU (G0B1 class) has room too; if it did not, a resistor
ladder on one ADC pin or an I/O expander would replace the 12 axis-switch inputs.

## Wiring notes

- Pull-ups on all switch inputs; the common of the 12-position switch goes to 0 V, so a closed
  position pulls its input low. OFF is the position with no axis assigned, read as position 0.
- The E-stop NC contact goes in series with the loop on the pin pair set aside in the plan (v1
  chain); a separate isolated input only reads it back.
- Terminate the CAN bus at both ends (item 9).
- Debounce in firmware for the buttons; the rotary switches need a short settling time too.

## What to settle with the bench

1. The MPG: output type, supply, feel, detent. This sets the input stage on the board.
2. Read the 12-position switch on 12 GPIOs now; decide ladder or GPIO for the board later.
3. OLED: legibility, dim and blank behaviour, contrast at the handheld distance.
4. The connector and cable: price, pin numbering, and noise on a 5 m run at 2 Mbit/s data phase.
5. A rough control layout on a flat board, then a printed mock-up with the real parts.

## Rough budget

About 150 to 300 EUR in total, dominated by the E-stop, the M12 connector and cable, and the CAN FD
adapter (unchecked estimate).
