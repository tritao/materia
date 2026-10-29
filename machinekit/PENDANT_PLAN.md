# Jog pendant plan

**Goal.** Our own handheld jog pendant, one SKU for every machine (drill-tap, hobber, router,
lathe and others), as phase CC4 of `CONTROL_CABINET_PLAN.md`. We buy the E-stop button, the MPG
wheel and the switches, and design the electronics, firmware and case.

Numbers and part families below are targets or unchecked suggestions unless stated.

## Decisions

1. **Our own design.** The SmartPendant (Devtronic-US/SmartPendant and nickshl/SmartPendant on
   GitHub) is a reference for ideas only: its BOM and schematic show input protection and part
   choices, its case uses a printed TPU cable relief and a button jig. Neither repo has a
   licence, so nothing is copied.
2. **One SKU.** The pendant has no per-machine wiring or labels. The selector positions are
   printed as numbers; what each means comes from the machine profile (CC0), sent by the host and
   shown by name on the OLED.
3. **Small display.** A monochrome OLED (128×64, I2C, 2.42") shows text only. No
   graphic UI and no touch. The main HMI already shows the DRO and program, and a second UI on
   the pendant would duplicate it.
4. **MCU: STM32H503RBT6** (decided 2026-10-29). Cortex-M33 at 250 MHz, 128 KB flash, 32 KB RAM, FDCAN and
   encoder-capable timers on chip, LQFP-64 with 49 I/O, enough for every control without shift registers. LCSC
   C6807804 (JLCPCB extended, 612 in stock and $3.70 at 1–9 on 2026-10-23). Chosen for a wired pendant over: the
   STM32G0B1 (almost no JLCPCB stock); RP2354A + MCP2518FD (cheaper chip, but CAN-FD needs a second chip, so more
   extended parts and setup fees); TI MSPM0G3507 (cheaper, 60 I/O, but young Rust support); classic-CAN parts such as
   the CH32V203 or the ESP32's TWAI (a classic controller disturbs an FD bus). Same Rust stack as before
   (embassy-stm32). The NUCLEO-H503RB is the matching bench board; the Nucleo-G474 stays usable for the CAN-FD bench.
   **Wireless** is not in v1. If it comes, it is jog-only (the cabinet keeps its hardwired E-stop and stops motion when
   the link drops) and an ESP32-C6-MINI-1 module (C6553337) is added next to the H503 rather than replacing it.
5. **Safety is outside the MCU.** v1 uses the uncertified bench chain: the pendant's E-stop (NC
   contact) is in series with the door switch (see `CONTROL_CABINET_PLAN.md`). The MCU only
   reads its state back.

## Controls

| Control | Part | Notes |
|---|---|---|
| MPG wheel | Bought off the shelf from Vallder: MPG hand wheel, 100 PPR, anodised aluminium, **4-pin single-ended** (A, B, 0V, VCC), SKU 1165001, model OSBAO-80-100B-5E-4P. Dimensions from the vendor drawing are in "MPG hand wheel" below. | Read on an MCU encoder timer; 400 counts per turn. Needs 5 V (at most 100 mA); outputs are 5 V level, so use 5 V tolerant timer pins or a level shifter. The output type (push-pull or open collector) is not on the page: verify on arrival and fit DNP pull-ups. |
| Axis | 12-position panel rotary switch: OFF plus positions 1 to 11 | The position is the state, so it survives power cycles, reads with a glove and does not depend on the display. Positions are printed as plain numbers; the profile maps each to an axis and the OLED shows the name ("3: Z"). If the OLED fails, the printed number and the main HMI still identify the axis. |
| Step | 3-position panel rotary switch | ×1, ×10, ×100. Universal across machines, so fixed. |
| Wheel enable | Plain hold button | Wheel counts are accepted only while it is held. A guard against bumps, not a safety function. |
| Buttons | 6 with LEDs | **Fixed (3):** Start, Hold, Stop/Reset, with standard icons, the same on every machine. **User keys F1 to F3 (3):** the function comes from the machine profile and the OLED's bottom line shows the current names next to the keys, so the one SKU needs no relabelling. Default profile assignment: F1 door open/close, F2 spindle jog (manual, hold-to-run), F3 work light or MQL toggle; the router can use F3 for the vacuum table. Unused keys stay dark. |
| E-stop | Bought, 22 mm, NC contacts. **Stand-in (AliExpress, 2.55 EUR):** an XB2-ES542 style, 40 mm twist-release mushroom with a snap-on ZB2-BE102C block (1NC; a 2NC variant exists). A seller's size drawing for the XB2-ES542 family gives 31 mm above the panel and 43 mm behind it (block 29 mm wide, base 42 mm wide), 52.4 g; the linked listing gives the block as 29.4 x 29.2 x 14.8 mm. Measure on arrival. **Compact option** (not AliExpress): EAO Series 84 (13.5 to 18 mm behind) or APEM EC Compact (18.7 mm), both stated EN 60947-5-5. | In the v1 chain. The cheap block is marked IEC 60947-5-1 and CCC, not 60947-5-5, so it is acceptable for the uncertified v1 chain only. Measure depth and height on arrival. |
| Display | Waveshare 2.42 in OLED module (SKU 25742): 128×64 mono, SSD1309, I2C (0 ohm resistors moved from the SPI default) | Text only; about 21 columns by 8 rows. The 2.42 in size is chosen over 1.3 in for legibility at handheld distance, at the cost of a larger face; the bench confirms it. Footprint for a one-character display as an option. |

An endless rotary encoder for the axis was considered and rejected: the selection would live only
in firmware and on the OLED, so a display fault would leave the operator jogging blind, and the
state would have to reset on power-up and lease changes. Eleven axes is more than any planned
CNC needs (the hobber has six). A machine with more than that, such as a seven-joint arm, uses
its own pendant. If the OLED proves troublesome, the switch and the HMI still work without it.

### Physical switches or membrane keys

Physical for v1. The E-stop, the enabling switch and the axis and step selectors have to be real devices whatever the keys
are. For the other seven keys (Start, Hold, Stop, F1 to F3, wheel enable), discrete 16 mm metal buttons win on a handheld:
raised, travelling keys can be found and pressed with a glove and do not fire when the face is held against the palm, they carry
their own LED, the OLED supplies the function-key legends so a printed overlay adds little, and for one printed unit they
are standard, swappable parts already in the model.

A membrane keypad is a later option, not a v1 choice. It gives one sealed thin face (IP65 without gland and nut seals), no
plate cut-outs, a board that can sit close behind the face, custom graphics and a lower cost at volume. It costs custom
tooling and a layout that is frozen, and it is worse with gloves and for accidental presses. The natural place for it is a
block of secondary keys (jog increments, axis keys) beside the physical safety and cycle keys, as on commercial pendants,
once the layout has been proven on the bench.

### Swappable legend overlay (a later revision)

Commercial wireless pendants (a WMPG-6 style unit is the reference picture) put the keys under one printed, coloured legend
sheet. For the one-SKU pendant this is the way to make the faces differ per machine without changing hardware: the
legends are the only machine-specific part.

- **Fixed and physical:** E-stop, enabling switch, the wheel, and the two selectors. Start, Hold and Stop stay physical buttons at
  first, because they are the keys a hand finds by feel.
- **Keypad zone:** a block of 10 to 12 keys (a stock matrix membrane keypad or a PCB with domes) replaces F1 to F3, under a
  printed overlay in a shallow recess in the plate's face, with a gasket for sealing.
- **Selector ring legends** move onto the overlay, so the 12-position axis switch prints the machine's axis names (X, Y, Z, A ...)
  instead of plain numbers.
- **One source for legends.** The operator profile already names each key's function per machine. The overlay artwork (SVG, from
  our layout and `StrokeText`) and the firmware key map are both generated from it, so a printed legend cannot disagree with
  what the key does.
- **Overlay identity.** The keypad connector carries an ID (a resistor value or a small memory). The pendant refuses to run a
  profile whose overlay is not the one fitted; a mislabelled key is a hazard, so this is a requirement, not a convenience.
- **Bench path:** a stock 4 x 4 matrix membrane keypad with overlays printed on adhesive polyester needs no tooling for the first
  unit. A custom membrane or dome PCB comes later if the layout settles.
- **Costs and risks:** worse with gloves than metal buttons; the overlay wears and must be replaceable; the recess needs a seal.
  Keep the keypad zone to non-safety functions: the wheel-enable key, which is held to jog, stays a physical button
  (decided 2026-10-29). A membrane matrix has no diodes, so three keys held at once can read as a fourth; the firmware
  rejects combinations it cannot resolve (two at once is fine). A dome-switch keypad PCB can carry diodes.
- **Board v1 reserves it** (see "Board v1 pin budget"), so the keypad is a connector change, not a redesign.
- **Work in the repo when it is taken up:** a `KeypadOverlay` panel part (cutout, recess, ID), overlay artwork generated from
  the profile, a keypad zone in `ReferencePendant`, and the ID check in the operator profile and its digest.

## MPG hand wheel (from the vendor drawing)

Read from the Vallder dimension drawing and the rear photos; the page itself gives no numbers.

| Item | Value |
|---|---|
| Back plate / flange | diameter 80 +/- 0.2 |
| Dial scale ring (outer) | diameter 77.5 |
| Knurled grip ring (inner) | diameter 59.5 |
| Rear boss | diameter 42 |
| Height above the panel | 7 (flange) + 20 (wheel) + 14 (the 10 mm spinner handle) = 41; 27 without the handle |
| Panel cutout | centre hole diameter 45 (+1/0); three holes of diameter 4.8 on a 72 +/- 0.2 pitch circle at 120 degrees |
| Fixing | 3 x M4 studs with nuts; a waterproof pad on the back plate (IP54 listed) |
| Terminals | M3 x 5 screws, 4-pin: A, B, 0V, VCC (a 6-pin version adds the complement terminals) |
| Behind the panel | small protrusion: two dimensions, 3.5 and 3 mm, read as the stud and boss; terminal strip height not given |

Consequences for the case: the wheel needs an 80 mm circle of face (the stud holes sit 36 mm from the
centre, so there is no useful overhang past the flange), which puts the face at about 90 mm or more in
one direction. The panel under it is thin (probably about 3 mm with an M4 nut): a printed face needs
ribs behind it. The case model takes every one of these as a parameter.

## Electrical

- **Power.** 24 V in with reverse-polarity protection and a TVS, a buck to 5 V and 3.3 V. A bought
  MPG typically needs 5 V at tens of mA; check its datasheet.
- **Inputs.** ESD protection on every input. Check whether the MPG outputs are 5 V TTL and
  whether the chosen MCU pins are 5 V tolerant; add a level shifter or Schmitt buffer if not.
  If the wheel has differential outputs, add a line receiver.
- **CAN-FD.** The H503's FDCAN with an isolated transceiver, since the cable is 3–5 m: candidate CA-IS2062A
  (C22438481, CAN-FD, 5 Mbps, isolated supply built in, 2.5 kVrms; check its datasheet before the parts review). A
  crystal clocks the FDCAN (the internal oscillator is not accurate enough for CAN). TI offers
  isolated CAN parts (for example the ISO1042); verify before choosing.
- **E-stop read-back.** An isolated input observes the chain state; it is never in the chain.
- **Display.** I2C inside the case, text only, dimmed after a few idle seconds and blanked when
  idle to limit burn-in, with large axis and step and one status line for the refusal reason.

## Connector and cable

12-pin M12 A-coded (availability and ratings to verify). v1 wires 8 pins:

| Pins | Use |
|---|---|
| 2 | E-stop loop (NC contact) |
| 3 | CAN_H, CAN_L, CAN ground |
| 2 | 24 V, 0 V |
| 1 | Shield (housing) |
| 4 reserved | Enabling switch channels for the certified upgrade |

- A detachable cable, so length is a separate part, not a pendant variant.
- Unplugging opens the E-stop loop. For removable use, a dummy plug at the cabinet bridges it.
- The case has a blanked mounting hole for a 3-position enabling switch. The certified upgrade
  fills the hole and wires the four reserved pins, with no new pendant.

## Cabinet side

The pendant cable never runs to a board. It ends at a bulkhead M12 receptacle on the enclosure wall
(door or side), and short internal wires split the signals to where each belongs:

```
Pendant ──M12 12-pin cable──▶ bulkhead receptacle on the enclosure wall
                                   │ rear pigtail or terminals
                                   ▼
  E-stop loop (2 wires) ───────▶ safety chain terminals
  24 V / 0 V, through a fuse ──▶ 24 V distribution
  CAN_H / CAN_L / ground ──────▶ FDCAN connector on the RKD6 cabinet node
  shield ──────────────────────▶ chassis / PE at the wall
```

- **Why not straight to a board.** The loop, power and data have different destinations, and the
  cable stays removable at the wall. A dummy plug with a jumper across the loop pins closes the
  chain when the pendant is unplugged.
- **Shield and protection at the entry.** The shield lands on the enclosure at the bulkhead (360
  degrees where the connector allows). ESD or TVS protection on the CAN lines sits at the same point.
- **CAN termination** (120 ohm) at the cabinet end, on the RKD6 board's connector or a terminal.
- **v1:** no PCB. The receptacle's pigtail goes to DIN rail terminals and a fuse holder. The four
  reserved pins get terminals for the later enabling-switch channels.
- **Later:** a small interface board behind the bulkhead (terminals, fuse, TVS, termination, status
  LED) in pcbkit, with the same function and less wiring.

## Protocol

CAN-FD frames, at most 64 bytes, between the pendant and the cabinet controller. These are small
pendant messages, not RKD6 frames, which can be several KB. CAN identifiers are placeholders until
the bus plan is fixed.

| Message | Direction | Content |
|---|---|---|
| `PENDANT_STATE` | pendant to cabinet controller | Sequence number, wheel count delta, axis switch position, step, button bits, wheel-enable held, E-stop chain read-back. Sent at a fixed rate (about 100 Hz) and doubling as the heartbeat. |
| `PENDANT_CONFIG` | cabinet controller to pendant | One label per frame: the axis name for a switch position, or an F-key label, from the machine profile. |
| `PENDANT_OUTPUT` | cabinet controller to pendant | LED states, lease state, one status line of text. |

Rules the controller enforces (the lease, hold-to-run, jog refusals, fail-safe defaults):

- Sequence numbers reject stale and duplicate frames.
- Heartbeat loss at the cabinet controller ends the pendant's lease and stops an executing jog.
  Heartbeat loss at the pendant lights a fault LED.
- Jog refusals (for example "B and C refused while the gearbox is engaged") come from the machine
  profile, compiled to the controller; the controller, not the host, evaluates the machine facts.
- A running program always blocks the pendant.

The wire schema, the controller-side code, the host link and how it is tested are described in
`operator_link/README.md`.

## Suppliers

For the switches, buttons and E-stop: Zhejiang Jiushi Electronics Co., Ltd. first, then Zhejiang Ninuo
Electrical Co., Ltd. (what Ninuo makes is unconfirmed). Details and what to ask for are in
`PENDANT_BENCH_PARTS.md`.

### MPG wheel: YUMO ISM6045 (smaller, documented)

**Yueqing Yumo Electric Co., Ltd (YUMO)**, model ISM6045 (`yumoelectric.com/ISM6045-100ppr-Cnc-Mpg-Pulse-Generator-pd90343536.html`),
100 PPR, MOQ 1, CE and RoHS, price on inquiry. The page gives a full electrical table with a choice of
output circuit, and the maker's drawing gives the size.

| Item | YUMO ISM6045 | Vallder (80 mm flange) |
|---|---|---|
| Overall diameter | 60 | 80 (grip 59.5) |
| Panel template | 43 boss; three M3 studs on a 50.8 circle | 45 hole; three M4 studs on a 72 circle |
| Above the panel | 26 (about 38 with the 12 mm handle) | 27 (41 with the handle) |
| Behind the panel | studs 10 mm, terminal boss 5.5 | studs about 6.5 (read), terminal strip height unknown |
| Weight | 250 g | 276 g |
| IP | 50 | 54 |
| Output | selectable: open collector, voltage, push-pull, or line driver | not stated |
| Price | inquire | about 21.67 EUR ex tax |

Electrical (page): 5 V (5 to 30 V for the open-collector, voltage and push-pull types), at most 80 mA
(150 mA line driver), 10 kHz, 600 r/min, -25 to 70 C. Line driver: high at least 3.4 V, low at most
0.4 V, edges 200 ns; push-pull: high at least Vcc minus 1.5 V, low at most 0.8 V. Six terminals: +5 V,
0 V, A, then A- and B or B and B- (line driver). Model `ISM6045-002-100B-5L` is the 5 V line driver; ask
YUMO for the code of the 5 V push-pull type. A single-ended push-pull output on 5 V is the simplest
for a wheel inside the case.

Why it matters: a 60 mm wheel with a 50.8 mm stud circle takes the face from 220 to 190 mm wide
(`ReferencePendant.compactFace`) and puts only 10 mm of studs behind the panel. It is the better fit
if the price is acceptable; ask YUMO for a quote. The Vallder wheel stays the cheap known-price fallback.

### MPG wheel: Puyang Yimihelung

**Puyang Yimihelung Precision Technology Co., Ltd.** sells a "Universal Electronic Handwheel for CNC
Systems" on Alibaba (product 1601665094833, `alibaba.com/product-detail/Universal-Electronic-Handwheel-for-CNC-Systems_1601665094833.html`)
with several variants. The listing has not been read: Alibaba returned a CAPTCHA to the page fetch.
Variants, diameters, PPR, output type, drawing and price are to be filled in from the listing. Until
then the Vallder wheel stays the plan; this is a candidate if its variants include a smaller wheel or
a better-documented one.

## Parts, layout and case

The bought parts, the panel layout and its checks are generic and live in `machinekit/haxe/src/machinekit/hmi/`
(see its README); the pendant adds only its own case and reference layout in `machinekit/pendant/`.

- **Layout.** `ReferencePendant.layout()` is a one-hand handheld like commercial CNC pendants (checked against two
  of them): a 114 x 210 mm face held with the palm behind it, 3 mm plate and wall. From the top: the axis and step
  switches beside the E-stop (top right corner), the display, F1 to F3 with the wheel-enable key, the Start, Hold and
  Stop keys, and the wheel at the bottom. The cheap E-stop is 43 mm deep, so the case is 48 mm and the pendant
  89 mm tall face up with the handle (the wheel stands 38 mm). The first layout was a 190 x 110 mm landscape slab; it
  fitted but did not suit holding. The width is set by the top row: the E-stop base (42 mm) and two 30 mm rotary bodies.
- **Devices are assemblies of bodies.** In the pendant assembly the E-stop (black operator, red head), the buttons
  (black housing, coloured cap), the rotary switches (body, knob) and the wheel (housing, wheel) are subassemblies of
  separately finished bodies. The first body is the purchase and keeps the device's BOM line; the others are included in
  it, and the device's mass is shared by volume. The wheel turns on a free `encoder` joint and each knob on a `position`
  joint, which the editor's Machine tab rotates. Still open: the E-stop's NC contact ports stay on the whole-device
  facade, not on the pendant's operator body, until the pendant is wired.
- **Case.** `PendantCase` plans and builds the shell: a rounded box (8 mm corners, 3 mm walls and back) open at the
  top; six 9 mm M3 screw posts rising to the plate, each merged into a side wall (a post alone on the back wall would flex and its layers would carry the load), each with a hole for an M3 heat-set insert (the board standoffs take M2.5 inserts); an M12 receptacle hole in the bottom wall (16.2 mm, an assumed M16
  rear-mount thread) with room for its body; a **board envelope** behind the shallow controls (1.6 mm, 10 mm of
  components on the back side) on four standoffs, each tied to a wall or a post by a 1.6 mm rib no longer than 16 mm, notched for the posts, the receptacle, the enabling switch and the
  deep E-stop that passes through its plane (slivers of board between notches are cut away); two strap slots
  through the back wall; the **enabling switch**, a side key: a 22.3 mm opening in the left wall (right if the left is
  full) for a 26 x 30 mm body lying against the back wall, where only the deep parts can reach; and on the plate a **perimeter tongue** that fits inside the
  walls, interrupted around every rear body and post. `PendantTests` checks that the posts avoid every part and
  body, the shell and plate are single solids, the shell's volume matches an analytic estimate, and the whole
  assembly (plate, shell, board, every part) has no overlaps under `AssemblyIntegrity`.

Findings so far, all on assumed rear bodies (measure the real parts):

- **The enabling switch sits on the left wall at y = +42 mm,** beside the display module (12 mm deep). The buttons and
  rotaries are 25 mm deep, which blocks a body at the back wall wherever they stand, so the side key goes where only
  the display is. That is high on the body for a thumb: check it in a printed mock-up, and if it is wrong move the
  display or shorten the button bodies. The planner falls back to the right wall and says so in `notes`.
- **The left edge now has a tongue,** which stops the plate gaping where no post fits; it is interrupted where the
  E-stop, knobs and buttons come near the wall.
- **Board:** it sits 26 mm behind the face (behind the 25 mm buttons and switches) with the deep E-stop through a
  notch, leaving about 10 mm for the connector and components. That is the board's real constraint, to confirm when
  the board is designed.
- **Strap slots** keep clear of parts that leave under 8 mm to the back wall.
- **Not yet modelled:** gasket, the M12 receptacle's own geometry and the real board, the wheel handle's removal.
- **Printing:** the shell is deep (51 mm) and prints best open side up; the plate prints face down.

## Mechanics

- Printed case first (CadKit, parametric), PA12 or similar for the product.
- Cut-outs: 22 mm E-stop, 60 mm wheel, panel switches and encoder, buttons, OLED window.
- Cable strain relief (a TPU relief as SmartPendant does), a handle strap, magnetic hang.
- v1 is not IP-rated; plan a gasket and sealed window for a product.

## Board v1 and the enclosure (P2)

The case model already decides where the board can be: `PendantCase` places a 1.6 mm board 26 mm behind
the face on four M2.5 standoffs, notched for the E-stop, posts, receptacle and enabling switch, with about
10 mm behind it. Today that is a placeholder solid (`PendantBoard`). The board itself is designed in pcbkit,
so the case has to hand pcbkit its constraints as data, and pcbkit has to hand the real board back. Nothing
is copied by hand in either direction.

**The board is `machinekit/examples/pendant/electronics/pendant/`** (a pcbkit board in materia, created 2026-10-29; pcbkit is
the `pcbkit/` submodule). Its requirements (`REQ-PENDANT-*`) and decisions (`DEC-PENDANT-*`, `DEC-SPEC`) are records there,
which `pcbkit status` grades; the sections below are their rationale. Open: `DEC-SPEC` (the specification review),
`DEC-PENDANT-PARTS` (the parts review), `DEC-PENDANT-CONNECTORS` and `DEC-PENDANT-LED-SUPPLY`.

**Decisions (2026-10-28, chosen for the long term):**

- **Component side.** Every part on the side facing the back wall, so JLCPCB assembles one side; the face
  side stays empty. It has about 1 mm under the button and switch bodies, so it could not take much anyway.
  In pcbkit terms the PCB's top side faces the back wall, so the case plan is mirrored on the board; the
  envelope record states the transform.
- **Axis switch wiring.** Read digitally: the common and all 12 positions go to the board (13 wires), and
  the board reads them as 12 inputs. Exactly one active input is a valid position; none or more than one is
  a fault (a broken wire, a worn contact, the moment between detents) and refuses jogging. This needs no
  parts at the switch, so nothing is hand-soldered onto its lugs, and no analogue thresholds can drift. A
  resistor ladder would save 11 wires but needs 11 resistors wired on the switch and an ADC band per
  position; a coded (BCD or Gray) switch is a dearer special part. The step switch is read the same way
  (3 positions and common).
- **M12 pigtail.** It lands on the pendant board (24 V, 0 V, CAN_H/L/ground, shield, and two taps on the
  E-stop loop for the isolated read-back). The loop itself stays wire-only between the E-stop block and the
  receptacle; the board never carries it.

**What the envelope record carries** (one pcbkit-defined, versioned JSON file, generated by `PendantCase`,
with a content hash so pcbkit marks the board stale when the case changes):

| Item | From |
|---|---|
| Outline: the rounded rectangle less the merged notches | `boardHalf()`, `notches()` |
| Mounting holes: positions, hole size, standoff and screw-head keep-outs on each side, hardware and whether it is conductive | `standoffs`, `BOARD_HOLE`, `hardware()` |
| Board plane: depth behind the face, thickness, which side faces the back wall | `boardTop`, `BOARD_THICKNESS` |
| Height zones per side: on the face side the clearance under each shallow control's rear body (about 1 mm under buttons and switches, about 14 mm under the OLED, the wheel's rear stud); on the back side the room to the back wall, with the rib tops (1 mm under the board) as no-part zones | control bodies and depths, `ribs`, `ribTop()` |
| Wire arrivals: for each control, where its terminals are (projected on the board) and how many wires, so each connector sits near its control | the control ports (EN1) |
| Connector access: low-profile, top-entry parts on the back side; latches reachable with the board in place | the stack above |
| Frame: the case plan (origin at the face centre, y up, seen from the face) to the board frame (top-left origin, y down, seen from the PCB's top side) | stated, mirrored per the component-side decision |
| Status: every value from an assumed part is a placeholder until the part is measured | the part sources |

**Board v1 pin budget** (STM32H503RBT6, 49 I/O). Decided 2026-10-29: the axis (12) and step (3) switches are read through
two 74HC165 shift registers (C5613, preferred) on SPI1, about $0.22, which keeps 11 pins free. The pin-by-pin assignment,
with alternate functions from ST's data (stm32-metapac), is in `pendant_firmware/README.md`, "Pendant pin assignment":
the firmware and the board follow that table.

| Function | Pins |
|---|---:|
| Selector shift registers (load, clock, data) | 3 |
| Keys (contacts) | 7 |
| Key LEDs (through transistors: 16 mm button LEDs) | 7 |
| Wheel encoder (TIM2, through a 5 V-tolerant buffer) | 2 |
| I2C1 (OLED, keypad ID memory) and the OLED's reset | 3 |
| FDCAN1 (PB12/PB13; PA11/PA12 are USB) | 2 |
| E-stop read-back | 1 |
| Enabling switch (reserved) | 4 |
| Keypad ID ADC pin and 2 matrix spares (reserved) | 3 |
| HSE crystal, SWD, USB | 6 |
| **Used / free** | **38 / 11** |

Reserved for the later membrane keypad zone:

- **Matrix group:** the contacts and LEDs of F1, F2 and F3 (PC6 to PC11) and two spare pins (PC12, PD2) form a group of 8
  plain GPIO that can act as a 4 x 4 matrix's rows (outputs) and columns (inputs with pull-ups and interrupts). The keypad
  variant reuses those 8 pins; it has no key LEDs.
- **ID:** an ADC pin (PC0, for a resistor ID) and the I2C bus (a small memory on the keypad tail) both reach the keypad
  connector, so either ID method works.
- **Connector:** a keypad connector footprint beside the button harness, unfitted in v1 (a stock 4 x 4 keypad has a 2.54 mm
  8-pin tail; a custom membrane usually a flat-flex tail, chosen with the keypad), with ESD protection on the matrix lines.

**Board v1 parts (proposed 2026-10-29, for the parts review).** From the JLCPCB catalogue snapshot of 2026-10-23 (USD at 1–9
pieces); the trial board's locked parts are reused where they fit. Every pick is pending until the parts review.

| Function | Part | LCSC | Tier | Price | Why |
|---|---|---|---|---:|---|
| MCU | STM32H503RBT6 | C6807804 | ext | 3.70 | decided |
| Isolated CAN-FD | CA-IS2062A | C22438481 | ext | 1.64 | datasheet read: ISO 11898-2:2016, FD to 5 Mbps, built-in isolated 5 V from a 5 V supply, logic 2.5–5.5 V, 2.5 kVrms, bus ±42 V; preliminary datasheet, certifications pending |
| CAN bus ESD | PESD2CAN | C284078 | ext | 0.05 | 24 V standoff, bus side |
| HSE crystal | 16 MHz 3225 (X322516MLB4SI) | C13738 | basic | 0.09 | FDCAN and USB clocks through the PLL |
| 24 V entry fuse | 1812L050/60GR PTC | C19078719 | ext | — | trial part, 60 V |
| Reverse protection | SS210 Schottky (100 V, 2 A) | C14996 | basic | 0.03 | 0.85 V maximum drop against the M7's 1.1 V, and 100 V reverse, above the TVS clamp. Chosen 2026-10-29 over the trial's M7 (C95872), whose record the trial's blocks depend on |
| Surge | SMBJ30CA | C19077585 | pref | — | trial part |
| 24 V to 5 V | LMR51606XDBVR (65 V, 0.6 A synchronous buck, SOT-23-6) | C21260865 | ext | 0.54 | rated above the TVS's clamping voltage (about 48 V at peak surge; the basic XL1509-5.0, C61063, is rated only to 40 V). Chosen 2026-10-29 over the TPS54360 (C44377, first proposal): internally compensated and synchronous, so no catch diode, timing resistor or compensation network; 65 V against 60 V; TI's own 24 V to 5 V example values (33 uH SWPA6045S330MT, C83451, whose 1.65 A saturation is above the part's 1.4 A peak limit). Pin-compatible with the LMR51610 (1 A, C20539658, same price) if the load grows, which then needs an inductor above its 1.95 A limit. Stock 9.9k against the TPS54360's 65k. An EN divider (100k/10k) starts it at 13.5 V and stops it at 11 V |
| 5 V to 3.3 V | XC6206P332MR-G (200 mA LDO) | C5446 | basic | 0.14 | MCU and logic need well under 100 mA |
| Wheel phases | SN74LVC2G17DBVR (Schmitt buffer at 3.3 V, 5 V-tolerant inputs) | C7394037 | ext | 0.05 | the wheel's 5 V outputs, whatever the pins' tolerance |
| Selectors | 2 x 74HC165 | C5613 | pref | 0.11 | decided |
| Key LEDs | ULN2003ADR (7 channels) | C7512 | basic | 0.15 | one part for the 7 LEDs, from the LED supply the buttons need (12 or 24 V LEDs with their own resistor, to be measured) |
| Input ESD | SRV05-4 (4 channels) | C85364 | pref | 0.20 | on the lines that leave the board: keys, selectors, enabling channels |
| E-stop read-back | `din_type3` block (LTV-817S, C109227 basic) | — | — | — | the trial's 24 V input block, if the loop runs at 24 V |
| Harness connectors | PH 2 mm, SMD side entry: JST S4B-PH-SM4-TB (keys, wheel, step), S8B-PH-SM4-TB (pigtail, OLED); the axis switch's 13-way is Megastar's PH-compatible ZX-PH2.0-WT13P | C265102, C265121, C22366240 | ext | 0.31, 0.37, 0.11 | low (about 5.5 mm), wires leave parallel to the board, and SMD so no leads stand out on the face side, which has about 1 mm under the buttons (pcbkit's tail check, E2). JST's own 13-way (C566195) has no EasyEDA record, so pcbkit could check neither its pins nor its assembly placement; the Megastar part has one and JST's land pattern within 0.05 mm (second source C42382963, same pattern). Its fit with a genuine JST PHR-13 housing is a bench check |
| CAN termination | 2 x 60.4 ohm 1206 (split, 4.7 nF to the bus ground) | C57572 | ext | — | the pendant is a bus end; a half can see 2.8 V (the 5.5 V bus envelope split), 130 mW, over an 0603's rating |

Notes for the parts review:

- **Connector sizes cost setup fees.** Each different connector size is another extended part (about $3 per order). Keep
  the set small: for example one size for each key (contact and LED, 4 pins), one for the wheel and step switch (4 pins),
  one for the axis switch (13 pins, if PH makes it in SMD) and one for the M12 pigtail (8 pins).
- **To measure on the bought parts:** the button LEDs' voltage and current, the wheel's current and output type.
- **Programming:** see "Programming and updates" below.
- **Still to check on the H503 datasheet:** which pins are 5 V tolerant (the part record now lists the FT/TT type of every assigned pin).
- **E-stop read-back wiring:** tap 1 is the loop after the E-stop's NC contact, tap 2 the loop's own 0 V. The isolated input then only loads the loop toward its 0 V (it can pull a weak input low, never hold it high); it must never sit across the contact, where it would be in series with the chain when the contact opens.

**Programming and updates (decided 2026-10-29).** The pendant's only external connection is the M12 cable, so:

- **Development and bench: SWD.** A Tag-Connect TC2030 footprint (pads only, no part) on SWDIO/SWCLK (PA13/PA14), NRST and
  3.3 V/GND, for a debug probe (ST-LINK or any probe-rs probe; `probe-rs run`). BOOT0 has a pull-down and a test pad, so the
  ROM bootloader can be forced if the firmware is broken.
- **Field updates: over CAN-FD through the cable**, pushed by the cabinet controller, with the case closed. Either ST's
  ROM bootloader (the H5 system bootloader; whether it offers FDCAN on the H503 is to confirm in AN2606) entered on command
  by the running firmware, or a small bootloader of ours in flash that checks the image before switching to it. Open.
- **USB:** PA11/PA12 on test pads and an unfitted USB-C footprint, for the bench only; no opening in the case.

**Steps:**

| Step | Work |
|---|---|
| EN1 | Ports for every wired control in machinekit: switch and LED contacts on the push buttons, the rotary switches' contacts, the OLED's I2C header, the receptacle's pins (8 of 12 used, 4 reserved for the enabling switch), each with a terminal position in the control's frame. **Done 2026-10-28 for the face's controls:** every wired port names a `Terminal` connector frame where its wires attach (push button: `no` and `led` at the body's rear; rotary switch: `contacts`, positions + 1 wires; OLED: the 7-pin `header` at its pin ends, wires along +x; wheel: its four terminals; E-stop: `nc<n>` on each block's rear face), and expanded bodies keep those frames. `PendantBoard` derives one matching port per control port from the layout and `PendantBoard.wire` connects them all (56 wires); the E-stop's contacts stay off the board. The receptacle and the enabling switch are still case openings, not parts: their ports come when they are modelled (with EN3). |
| EN2 | pcbkit (its PLAN, "Enclosure envelope" E1–E3): the envelope schema and reader, height zones per side and region that are not tied to a plug-in module, part keep-outs (not just copper keep-outs), and generating the board's mechanical record and intent placement from the envelope. |
| EN3 | `PendantCase.envelope()` writes the record next to the pendant example; pcbkit becomes a pinned submodule of materia (the dependency direction decided 2026-10-25) and the pendant board gets its folder beside the pendant. |
| EN4 | **Circuit done 2026-10-29** (`circuit.py`, every review pending): 109 parts (93 assembled, 13 footprints not fitted, 3 test pads), 39 locked identities, the pin table as data (`pendant.pins.json`, pcbkit W5, also read by the firmware build), lint, ERC, block, gate, supplier pin map and 5 V rail checks clean. **Power phase done 2026-10-29:** the power check derives the whole tree from part data (pcbkit: buck, series diode, zero-ohm link and clamp kinds, external loads): 24 V in 18 to 30 V, 273 mA worst case; 5 V 4.64 to 4.95 V (divider 49.9k/10k), 353 mA of the LMR51606's 600 mA; 3.3 V 103 mA of the XC6206's 200 mA; the wheel (100 mA), OLED (50 mA) and key LEDs (7 x 20 mA) as external loads until measured. Unknowns: the cabinet supply's capacity, the transceiver's VDDL current, the LDO's thermal data. Next: placement once EN3 delivers the envelope. The board v1 design in pcbkit, inside the envelope: STM32H503RBT6 (pins as in `pendant_firmware/README.md`), two 74HC165 for the selectors, isolated FDCAN (CA-IS2062A candidate), 24 V entry with reverse protection and TVS, 5 V and 3.3 V, the wheel's encoder input at 5 V levels, protected switch inputs, LED drivers, the OLED's I2C, the E-stop read-back. Every review pending, as for any pcbkit board. |
| EN5 | Back from pcbkit (its PLAN E4): the real board's outline, holes, connector positions and a STEP model. `PendantBoard` uses them instead of the envelope solid, with one port per connector; the controls are wired to those ports and `AssemblyIntegrity` checks the board against the case and the controls. |

## Phases

| Phase | Work |
|---|---|
| P0 | This spec; pin budget; message definitions with CC1. |
| P1 | Bench: Nucleo-G474 with the bought wheel, switches, buttons and an OLED on a proto board; Rust firmware (`embassy-stm32`, skeleton in `pendant_firmware/`); CAN-FD to the host. Parts and checks: `PENDANT_BENCH_PARTS.md`. |
| P2 | Board v1 in `pcbkit`, constrained by the case: "Board v1 and the enclosure" (EN1–EN5). |
| P3 | Case v1 in CadKit, printed. |
| P4 | Wiring with the v1 chain; ESD and noise tests with the real cable. |
| P5 | Certified upgrade: dual-channel E-stop and 3-position enabling switch on the reserved pins. |

## Open questions

- **Button set.** Settled for v1 as three fixed keys and F1 to F3 profile keys. More user keys
  wait until a real machine shows what is missing.
- **MPG.** Settled on the Vallder 100 PPR 4-pin single-ended wheel (diameters now known from its drawing). To check on the real part: output type (push-pull or open collector), 5 V tolerance of the timer pins, the stud and boss protrusion behind the panel (two small drawing dimensions, 3.5 and 3 mm, read as about 6.5 mm in total, so the panel may be limited to about 3 mm), wheel feel, detent, and weight (276 g listed). SmartPendant's case is 160 x 65 x 20 mm, 55 mm with its wheel and handle; ours is larger because this wheel needs an 80 mm circle of face.
- **E-stop contact block.** Rating and terminals.
- **pcbkit.** The H503 and the CA-IS2062A need part data and pin maps in pcbkit (EN4).
- **Display burn-in.** Settings to confirm on a real module (the Waveshare listing gives no lifetime figures).
- **Z index.** Whether the wheel's index channel is needed. Probably not.
- **Rust support.** Checked: embassy-stm32 0.6 has an FDCAN driver with CAN-FD data bit rates and `write_fd`, with active fixes to FD timing, and `pendant_firmware` builds for both the Nucleo-G474 and the STM32G0B1 (the G0B1 shares its FDCAN interrupt lines with TIM16 and TIM17); the `h503` target (2026-10-29) replaced the G0B1 one and builds a linked image. Compile-checked only: nothing has run on hardware, and the FDCAN kernel clock is still the default.
