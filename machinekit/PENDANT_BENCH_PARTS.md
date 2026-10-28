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
| 1 | **MPG handwheel**, Vallder 100 PPR, 4-pin single-ended (A, B, 0V, VCC), SKU 1165001 | 2 | Diameters are known from the vendor drawing (80 flange, 59.5 grip; see `PENDANT_PLAN.md`). Ask Vallder only for the output type (push-pull or open collector). 5 V supply. About 21.67 EUR ex tax each. Fallback: the SmartPendant's "60 mm 6 pin 100 PPR" AliExpress listing. | `vallder.com/product/mpg-hand-wheel/`, `OSBAO-80-100B-5E-4P` | Output type; counts per turn on a logic analyzer; 5 V high level against the MCU pin tolerance; the stud and boss protrusion and the terminal strip height behind the panel; weight (276 g listed); panel thickness that the studs allow. |
| 2 | **Rotary switch, 12 position**, panel mount | 2 | 1 pole 12 throw, break-before-make (non-shorting), good detent. We read OFF plus 11 positions. | `12 position rotary switch 1 pole 12 throw panel mount`, `rotary switch 1P12T non-shorting` | Detent feel; each position closes exactly one contact; contact bounce. |
| 3 | **Rotary switch, 3 position**, panel mount | 2 | 1 pole 3 throw (x1, x10, x100). | `3 position rotary switch 1P3T panel mount` | As above. |
| 4 | **Illuminated momentary buttons**, 12 to 16 mm | 8 (+3 spare) | 6 with LEDs: green, yellow, red for the fixed keys and 3 neutral for F1 to F3, with LEDs for 5 V or 12 V with a series resistor. Plus 1 plain momentary for wheel enable (no LED). | `16mm momentary push button LED 5V`, `12mm illuminated momentary pushbutton` | LED forward voltage and current; contact bounce; the feel through a glove. |
| 5 | **Emergency stop**, 22 mm, with 1 NC contact block | 1 | Latching, twist or pull release, NC contact block. A branded, standards-marked part is worth the price even for a bench. | `22mm emergency stop mushroom latching 1NC`, `E-stop 22mm contact block NC` | Mounting hole size and depth; contact block rating; that it fits the case thickness. |
| 6 | **OLED module**, 128x64, 2.42 in, I2C | 2 | SSD1309 class, 3.3 V, I2C (many modules have a solder-pad or resistor choice between I2C and SPI; confirm it ships in, or can be set to, I2C). Active area about 55x27 mm (from memory, check the listing). | `2.42 inch OLED 128x64 I2C SSD1309`, `2.42 OLED 128x64 IIC` | I2C address (often 0x3C or 0x3D, set by a pad); supply current; contrast; legibility at arm's length; whether the window leaves room for the controls on the face. |
| 7 | **CAN-FD transceiver breakout**, 3.3 V logic | 2 | TJA1051, TCAN1042 or MCP2562FD class. Not a classic-CAN-only part. | `TJA1051 breakout`, `TCAN1042 breakout`, `MCP2562FD breakout` | 3.3 V logic compatible; 120 ohm termination jumper or solder pad. |
| 8 | **USB-CAN FD adapter** | 1 | gs_usb or candleLight firmware, CAN FD capable. | `CANable 2.0`, `candleLight FD USB CAN` | Shows up as a `can0` interface and does 2 Mbit/s data phase. |
| 9 | **120 ohm resistors**, 1/4 W | 4 | Bus termination at both ends. | any | Measure about 60 ohm across the bus with both ends fitted. |
| 10 | **M12 A-coded 12-pin connector set**: a 5 m shielded cable (plug at one end, free wires or a plug at the other), a **panel-mount receptacle** for the pendant side (bench rig), a **bulkhead receptacle** for the cabinet wall with a pigtail or terminals, and a **dummy plug** | 1 each | The real connector family. Same coding and pin count on both ends. Rear-mount with a molded pigtail is easiest to wire; screw or solder terminals work. Shield must be reachable at the bulkhead. | `M12 A-coded 12 pin panel mount receptacle`, `M12 12 pin bulkhead rear mount pigtail`, `M12 12 pin cable 5m shielded`, `M12 12 pin dummy plug` | Availability and price; pin numbering matches on both ends; wire gauge for 24 V and the loop; shield contact at the bulkhead; the dummy plug can take a jumper across the loop pins. |
| 11 | **24 V to 5 V DC-DC module** | 2 | Wide input, 24 V in, 5 V at 1 A. | `24V to 5V buck converter 3A module` | Output under load; ripple on a scope if one is to hand. |
| 12 | **Level shifting** | 1 pack | The MPG runs from 5 V and its outputs reach 5 V. Only needed if the MCU timer pins are not 5 V tolerant: resistors for dividers, or a 74LVC/74HCT buffer. Check the pin tolerance first. | `74LVC245 breakout`, `5V 3.3V level shifter 4 channel` | MPG high level; edge shape. |
| 13 | **Prototyping** | | Breadboard or perfboard, jumpers, pin headers, 2 to 3 m of 4-wire shielded cable, ferrules, heat shrink. | | |
| 14 | **Logic analyzer**, 8 channel USB (skip if owned) | 1 | Saleae-compatible. | `USB logic analyzer 8 channel 24MHz` | Sees the MPG A/B edges. |
| 15 | **Cabinet-side terminals** (v1 pendant interface) | 1 set | DIN rail, 2 to 3 spring or screw terminal blocks per signal group, a short DIN rail piece, end clamps, a PE terminal. | `DIN rail terminal block 2.5mm2`, `DIN rail PE terminal` | Wire gauge fits the pigtail; groups: loop, 24 V, CAN, shield. |
| 16 | **Fuse holder and fuses** | 1 + 5 | DIN or terminal fuse holder, 1 A or 2 A fast fuses for the pendant's 24 V. | `DIN rail fuse terminal block 5x20`, `fuse 5x20 1A` | Holder rating; indicator, if any. |
| 17 | **Cabinet wall mock-up** | 1 | A scrap of plate (about as thick as the real enclosure wall) with the M12 bulkhead hole, to test mounting, shield contact and cable entry. | | Hole size and torque; whether the shield bonds to the plate. |

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

1. The MPG (Vallder, drawing-confirmed dimensions): output type, pin tolerance, the rear protrusion and the allowed panel thickness, feel, detent, weight. This sets the input stage on the board and the face thickness.
2. Read the 12-position switch on 12 GPIOs now; decide ladder or GPIO for the board later.
3. OLED (2.42 in): legibility, dim and blank behaviour, contrast at the handheld distance, and the face size it forces on the case.
4. The connector set and the cabinet side: price, pin numbering on both ends, the dummy plug, shield bonding at the bulkhead, and noise on a 5 m run at 2 Mbit/s data phase.
5. A rough control layout on a flat board, then a printed mock-up with the real parts.

## Rough budget

About 150 to 300 EUR in total, dominated by the E-stop, the M12 connector and cable, and the CAN FD
adapter (unchecked estimate).
