# pendant

The jog pendant's controller board, v1 (`machinekit/PENDANT_PLAN.md`, "Board v1 and the enclosure"). Its requirements
(`REQ-PENDANT-*`) and decisions (`DEC-PENDANT-*`) are `requirements.json` and `decisions.json`; `pcbkit status` grades
them. Every review is pending; the specification, parts, connector, LED supply and bootloader decisions are open.

## Circuit (EN4, 2026-10-29)

`circuit.py` is the source; sheets:

| Sheet | Contents |
|---|---|
| power | 24 V from the pigtail: PTC 1812L050/60GR, SMBJ30CA, M7 reverse diode, LMR51606 buck to 5 V (4.88 V nominal, EN divider: on at 13.5 V, off at 11 V), XC6206 LDO to 3.3 V, the LED supply link (24 V fitted, 5 V alternative not fitted) |
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
table, and the 5 V rail (the buck's divider over its reference and resistor tolerances, inside the CAN transceiver's VDDP
range). The power phase is not graded yet: the ICs' part records need supply data, and pcbkit's power check has no model of
a regulated switching buck, so it cannot see that the buck drives 5 V.

## Mechanical interface and floorplan

The outline, mounting holes, height zones and connector arrivals come from the case's envelope record (EN3), before any
placement.
