# pendant

The jog pendant's controller board, v1 (`machinekit/PENDANT_PLAN.md`, "Board v1 and the enclosure"). Its requirements
(`REQ-PENDANT-*`) and decisions (`DEC-PENDANT-*`) are `requirements.json` and `decisions.json`; `pcbkit status` grades
them. Every review is pending; the specification, parts, connector, LED supply and bootloader decisions are open.

## Circuit (EN4, 2026-10-29)

`circuit.py` is the source; sheets:

| Sheet | Contents |
|---|---|
| power | 24 V from the pigtail: PTC 1812L050/60GR, SMBJ30CA, SS210 Schottky reverse diode, LMR51606 buck to 5 V (4.79 V nominal, 4.64 to 4.95 V over tolerances; EN divider: on at 13.5 V, off at 11 V), XC6206 LDO to 3.3 V, the LED supply link (24 V fitted, 5 V alternative not fitted) |
| mcu | STM32H503RBT6 on the pin table, decoupling, 2 x 2.2 uF VCAP, 16 MHz crystal with 12 pF loads, NRST 100 nF, BOOT0 10k pull-down |
| can | CA-IS2062A isolated CAN FD with its own isolated 5 V, split termination (2 x 60.4 ohm 1206, 4.7 nF), PESD2CAN on the bus, the cable shield to 0 V through 1 Mohm and 4.7 nF 2 kV |
| cable | the M12 pigtail header: 24 V, 0 V, CAN_H, CAN_L, CAN ground, shield, E-stop taps 1 and 2 |
| estop | pcbkit's `din_type3` block (24 V Type 3 input, LTV-817S) reading tap 1 against tap 2 |
| wheel | the wheel's header (5 V, 0 V, A, B), ESD, 1k series, SN74LVC2G17 Schmitt buffer at 3.3 V (5 V-tolerant inputs), pull-up footprints until the wheel's output type is measured |
| selectors | the axis switch's 13-way header (common + 12 positions), the step switch's 4-way, 10k pull-ups, ESD, two 74HC165 chained on SPI1 |
| keys | seven 4-way key headers (contact, 0 V, LED supply, LED cathode), pull-ups, ESD, ULN2003A for the seven LEDs |
| reserved | not fitted: the enabling switch's header and ESD (pull-ups fitted), the membrane keypad's 13-way header and ESD; the keypad ID pull-up fitted |
| oled | the OLED's 8-way header (I2C mode, CS and DC low: address 0x3C), I2C pull-ups, reset pull-up |
| prog | Tag-Connect TC2030 pads (SWD, NRST), BOOT0 test pad, the bench-only USB-C footprint (not fitted) with USB test pads |

**Pin table.** `pendant.pins.json` assigns every port pin of the MCU (38 nets, 11 free). The circuit wires the MCU from it,
`checks.py` proves the netlist follows it, and the firmware's `build.rs` generates its pin module from it (pcbkit PLAN W5).

**Selector bits.** After a load, the chain shifts out `u_sr1` D7 first: `u_sr1` holds AXIS_0 to AXIS_7 (D0 to D7), `u_sr2`
AXIS_8 to AXIS_11 (D0 to D3), STEP_1, STEP_10, STEP_100 (D4 to D6) and a spare pulled high (D7); a closed contact reads 0.

**E-stop read-back.** Tap 1 is the loop after the E-stop's NC contact, tap 2 the loop's own 0 V. The input then only loads
the loop toward its 0 V and is never across the contact.

**Isolation.** The CA-IS2062A joins the logic and bus domains (its lands keep 3.15 mm across; functional isolation, a
24 V system); the pigtail header carries the logic supply, the bus and the E-stop loop on neighbouring contacts.

## Checks

`checks.py`: the shared lint, ERC, parts lock, block and gate checks, the supplier pin maps of every locked part, the pin
table, and the power check. The power check derives every rail from part data and the declared external loads (the wheel,
the OLED module and the key LEDs, budgets until they are measured):

| Rail | Range | Worst-case load | Limit |
|---|---|---|---|
| +24V_IN (cabinet) | 18 to 30 V | 273 mA | unknown (a physical review) |
| +24V_F (after the PTC) | 17.3 to 30 V | 273 mA | 500 mA hold |
| +24V (after the SS210) | 16.45 to 30 V | 273 mA | 2 A |
| +5V (LMR51606, 49.9k/10k) | 4.64 to 4.95 V | 353 mA (CAN 150 mA dominant, wheel 100 mA, the LDO 103 mA) | 600 mA |
| +3V3 (XC6206) | 3.234 to 3.366 V | 103 mA | 200 mA |
| LED_SUPPLY (0 ohm from +24V) | 16.45 to 30 V | 140 mA | — |

Unknowns: the cabinet supply's capacity, the CAN transceiver's VDDL current (its datasheet gives VDDP and VDDL together),
the LDO's thermal resistance.

## Mechanical interface and floorplan

`pendant.envelope.json` is the case's envelope, derived by machinekit's `BoardEnvelope` from the pendant assembly (see
`machinekit/PENDANT_PLAN.md`, EN3): a 104 x 200 mm outline less the notches, four M2.5 mounting holes (h1 to h4) with the
screw heads on the top side and the standoffs on the bottom, the room on each side (17 mm to the back wall over the top
side, 6.5 mm under the buttons' and selectors' bodies on the bottom side) and where each control's wires arrive. The
circuit applies it: the outline, the holes as code-owned anchors, the zones, and each header near its control's wires
(within 40 mm, a harness budget). The case's parts are assumed until measured, so the mechanical review stays open.
