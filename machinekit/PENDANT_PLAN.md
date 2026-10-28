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
4. **Small MCU.** An STM32G0B1-class part with FDCAN and an encoder timer. The Nucleo-G474 is the
   bench stand-in.
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
| Display | 128×64 mono OLED, 2.42 in (SSD1309 class), I2C | Text only; about 21 columns by 8 rows. The 2.42 in size is chosen over 1.3 in for legibility at handheld distance, at the cost of a larger face; the bench confirms it. Footprint for a one-character display as an option. |

An endless rotary encoder for the axis was considered and rejected: the selection would live only
in firmware and on the OLED, so a display fault would leave the operator jogging blind, and the
state would have to reset on power-up and lease changes. Eleven axes is more than any planned
CNC needs (the hobber has six). A machine with more than that, such as a seven-joint arm, uses
its own pendant. If the OLED proves troublesome, the switch and the HMI still work without it.

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
- **CAN-FD.** One FDCAN port with an isolated transceiver, since the cable is 3–5 m. TI offers
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

## Face model

`machinekit/haxe/src/machinekit/pendant/` models the top face: each bought part as a `PendantControl`
(cutout, keep-out, height above and depth behind the plate, mass, and where each number came from:
`vendor-drawing`, `measured`, `datasheet` or `assumed`), a `PendantFace` with a checker, and
`PendantFacePlate`, the plate as a solid with every cutout (the printable layout mock-up, STEP export
through `Shape.exportStep`). `ReferencePendant` is the first layout and `PendantFaceSvg` draws it for
review. `PendantFaceTests` asserts the layout passes its own checks, that each check catches what it
should, and the plate's volume.

First layout: a 220 x 110 mm face, 3 mm plate and wall, wheel on the right, E-stop, display and
wheel-enable across the top left, axis and step switches with F1 and F2 below, then F3 and the fixed
Start, Hold and Stop/Reset keys. What the checker already shows:

- **The wheel sets the face size.** Its 80 mm flange and 72 mm stud circle need about 90 mm of face
  in one direction, which makes the face about 220 x 110 mm, far from a 160 x 65 slab.
- **Depth was the problem.** A standard 22 mm E-stop with its contact block is 82 mm deep (Harmony
  XB5AS8445 datasheet) and rejected by the checker. A compact one is 13.5 to 18.7 mm behind the panel,
  so the deepest parts become the switches and buttons (assumed 25 mm), a 30 mm case and the wheel's 41
  mm (27 without the handle): 74 mm tall face up. Shallower buttons and rotary switches are the next lever.
- **Assumed numbers:** everything except the wheel (its drawing) is a placeholder until measured.

## Mechanics

- Printed case first (CadKit, parametric), PA12 or similar for the product.
- Cut-outs: 22 mm E-stop, 60 mm wheel, panel switches and encoder, buttons, OLED window.
- Cable strain relief (a TPU relief as SmartPendant does), a handle strap, magnetic hang.
- v1 is not IP-rated; plan a gasket and sealed window for a product.

## Phases

| Phase | Work |
|---|---|
| P0 | This spec; pin budget; message definitions with CC1. |
| P1 | Bench: Nucleo-G474 with the bought wheel, switches, buttons and an OLED on a proto board; Rust firmware (`embassy-stm32` or similar); CAN-FD to the host. Parts and checks: `PENDANT_BENCH_PARTS.md`. |
| P2 | Board v1 in `pcbkit`. |
| P3 | Case v1 in CadKit, printed. |
| P4 | Wiring with the v1 chain; ESD and noise tests with the real cable. |
| P5 | Certified upgrade: dual-channel E-stop and 3-position enabling switch on the reserved pins. |

## Open questions

- **Button set.** Settled for v1 as three fixed keys and F1 to F3 profile keys. More user keys
  wait until a real machine shows what is missing.
- **MPG.** Settled on the Vallder 100 PPR 4-pin single-ended wheel (diameters now known from its drawing). To check on the real part: output type (push-pull or open collector), 5 V tolerance of the timer pins, the stud and boss protrusion behind the panel (two small drawing dimensions, 3.5 and 3 mm, read as about 6.5 mm in total, so the panel may be limited to about 3 mm), wheel feel, detent, and weight (276 g listed). SmartPendant's case is 160 x 65 x 20 mm, 55 mm with its wheel and handle; ours is larger because this wheel needs an 80 mm circle of face.
- **E-stop contact block.** Rating and terminals.
- **pcbkit.** Whether it covers a G0B1 with FDCAN and an isolated transceiver today.
- **Display burn-in.** Settings to confirm on a real module (the Waveshare listing gives no lifetime figures).
- **Z index.** Whether the wheel's index channel is needed. Probably not.
- **Rust support.** Check the current FDCAN driver status for the chosen MCU before committing.
