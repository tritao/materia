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
3. **Small display.** A monochrome OLED (128×64, I2C, about 1" to 1.3") shows text only. No
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
| MPG wheel | Bought, 60 mm, 100 PPR | Read on an MCU encoder timer; 400 counts per turn. Check the output type and supply. |
| Axis | 12-position panel rotary switch: OFF plus positions 1 to 11 | The position is the state, so it survives power cycles, reads with a glove and does not depend on the display. Positions are printed as plain numbers; the profile maps each to an axis and the OLED shows the name ("3: Z"). If the OLED fails, the printed number and the main HMI still identify the axis. |
| Step | 3-position panel rotary switch | ×1, ×10, ×100. Universal across machines, so fixed. |
| Wheel enable | Plain hold button | Wheel counts are accepted only while it is held. A guard against bumps, not a safety function. |
| Buttons | 6 with LEDs | **Fixed (3):** Start, Hold, Stop/Reset, with standard icons, the same on every machine. **User keys F1 to F3 (3):** the function comes from the machine profile and the OLED's bottom line shows the current names next to the keys, so the one SKU needs no relabelling. Default profile assignment: F1 door open/close, F2 spindle jog (manual, hold-to-run), F3 work light or MQL toggle; the router can use F3 for the vacuum table. Unused keys stay dark. |
| E-stop | Bought 22 mm mushroom, NC contact block | In the v1 chain. |
| Display | 128×64 mono OLED | Text only; footprint for a one-character display as an option. |

An endless rotary encoder for the axis was considered and rejected: the selection would live only
in firmware and on the OLED, so a display fault would leave the operator jogging blind, and the
state would have to reset on power-up and lease changes. Eleven axes is more than any planned
CNC needs (the hobber has six). A machine with more than that, such as a seven-joint arm, uses
its own pendant. If the OLED proves troublesome, the switch and the HMI still work without it.

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
- **MPG choice.** Output type, supply, wheel feel, detent and price from a real part.
- **E-stop contact block.** Rating and terminals.
- **pcbkit.** Whether it covers a G0B1 with FDCAN and an isolated transceiver today.
- **Display burn-in.** Settings to confirm on a real module (the Waveshare listing gives no lifetime figures).
- **Z index.** Whether the wheel's index channel is needed. Probably not.
- **Rust support.** Check the current FDCAN driver status for the chosen MCU before committing.
