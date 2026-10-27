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
2. **One SKU.** The pendant has no per-machine wiring or labels. What each selector position
   means comes from the machine profile (CC0), sent by the host.
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
| Axis | Endless detented rotary encoder with push | Turning steps through the axes the profile defines; push returns to OFF. Axis name shows on the OLED. Resets to OFF on power-up and on any lease change. |
| Step | 3-position panel rotary switch | ×1, ×10, ×100. Universal across machines, so fixed. |
| Wheel enable | Plain hold button | Wheel counts are accepted only while it is held. A guard against bumps, not a safety function. |
| Buttons | About 6 with LEDs | Start, hold, stop, and user keys. The set is not decided (see open questions). Unused buttons stay dark. |
| E-stop | Bought 22 mm mushroom, NC contact block | In the v1 chain. |
| Display | 128×64 mono OLED | Text only; footprint for a one-character display as an option. |

Fallback if the axis encoder or display proves troublesome: a 7-position rotary switch (OFF plus
six axes) with a legend overlay generated from the profile. The cable, messages and profile map
are the same either way.

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

## Protocol (draft; defined with CC1)

CAN-FD frames, at most 64 bytes. These are small pendant messages, not RKD6 frames, which can be
several KB. Values are placeholders.

| Message | Direction | Content |
|---|---|---|
| `PENDANT_STATE` | pendant to cabinet controller | Sequence number, wheel count delta, axis index, step index, button bits, wheel-enable held, E-stop chain read-back, flags. Sent at a fixed rate (about 100 Hz) and doubling as the heartbeat. |
| `PENDANT_CONFIG` | cabinet controller to pendant | Axis names for the profile (short strings, up to about 12), step labels. |
| `PENDANT_OUTPUT` | cabinet controller to pendant | LED states, status line text, refusal code, lease state. |

- Sequence numbers reject stale and duplicate frames.
- Heartbeat loss at the cabinet controller ends the pendant's lease and stops motion. A heartbeat
  loss at the pendant lights a fault LED.
- Jog rules (for example "B and C refused while the gearbox is engaged") live on the host.

## Mechanics

- Printed case first (CadKit, parametric), PA12 or similar for the product.
- Cut-outs: 22 mm E-stop, 60 mm wheel, panel switches and encoder, buttons, OLED window.
- Cable strain relief (a TPU relief as SmartPendant does), a handle strap, magnetic hang.
- v1 is not IP-rated; plan a gasket and sealed window for a product.

## Phases

| Phase | Work |
|---|---|
| P0 | This spec; pin budget; message definitions with CC1. |
| P1 | Bench: Nucleo-G474 with the bought wheel, switches, buttons and an OLED on a proto board; Rust firmware (`embassy-stm32` or similar); CAN-FD to the host. |
| P2 | Board v1 in `pcbkit`. |
| P3 | Case v1 in CadKit, printed. |
| P4 | Wiring with the v1 chain; ESD and noise tests with the real cable. |
| P5 | Certified upgrade: dual-channel E-stop and 3-position enabling switch on the reserved pins. |

## Open questions

- **Button set.** Starting list: start, hold, stop and two user keys. Which functions do your
  machines need (spindle, door, tool change)?
- **MPG choice.** Output type, supply, wheel feel, detent and price from a real part.
- **E-stop contact block.** Rating and terminals.
- **pcbkit.** Whether it covers a G0B1 with FDCAN and an isolated transceiver today.
- **Display burn-in.** Settings to confirm on a real module (the Waveshare listing gives no lifetime figures).
- **Z index.** Whether the wheel's index channel is needed. Probably not.
- **Rust support.** Check the current FDCAN driver status for the chosen MCU before committing.
