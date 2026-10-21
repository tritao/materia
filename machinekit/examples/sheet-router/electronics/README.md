# SR17 electrical design data

`schematic.json` is the reviewable schematic graph; `wire-list.csv` maps every
assembly/controller channel to a unique terminal and carrier connector pin.
`safety-wires.csv` and `power-wires.csv` detail the hardwired paths separately.
The 49 SR17 extension channels (drive interfaces, probe, air, brake, extractor,
reset/EDM) are labelled separately from the 57 channels exported from both beds.
A carrier connector pin is a physical design assignment; MCU/expander assignment
and timer capacity are checked in SR15 before carrier layout. This is schematic
data, not a released KiCad board or a claim of achieved PL.

Reproduce: `electronics/check.sh` from the repository root exports the live CAD
channels for both beds, regenerates the schematic/wires and runs `check.py`.
`check.py` alone validates committed data, including ten negative fixtures:
release bypasses, either STO lane bypassed, duplicate channel/terminal/pin and
an unwired assembly channel. Graph reachability and cut-vertex tests require
every e-stop/trip contact to interrupt its lane at all four drives and the VFD;
every release-power path requires zero speed, independently confirmed stopped,
instantaneous safety permission and the CPU request. It cannot prove actual
contact reliability, insulation, installation or braking performance.

## Supply and demand — awaiting SR0 workshop facts

Baseline assumption: 400 V line-to-line, three-phase, 25 A, PE, no neutral needed
(400 V-input isolated control PSU). Earthing system, prospective fault current,
upstream protective devices/RCD compatibility, cable route/derating and actual
available power remain SR0 facts. Never create a cabinet N–PE bond by assumption.
The provisional 4 mm² feeder/PE, 2.5 mm² motor branches, .75 mm² controls and
breaker values in the data require installation and manufacturer checks.

Connected nameplate demand exceeds 25 A: 6 kW spindle, 5.5 kW blower, four
750 W servos and 2.2 kW extraction are about **31 A**, before controls, using the
stated efficiencies and power factors. Therefore unrestricted simultaneous
rated operation is NOT admitted on this assumed supply. The SR13 recipe needs
at most .648 kW spindle power; its force admission bounds the 18 krpm process
to .848 kW. A provisional **1.2 kW spindle process cap** gives approximately
22.5 A plus .5 A controls with all other nameplates. Confirm measured phase
currents, startup/inrush, duty and diversity in SR18/SR19. Keep this process
cap or confirm a larger supply and re-rate the feeder before heavier recipes.
Do not rely on the 25 A breaker as a production load controller.

## Safety chain

Cabinet, pendant and rack e-stops and two gantry trip bars have independent NC
A/B circuits into a monitored, manually reset safety relay. Immediate outputs
request a controlled stop in all drives and the VFD; assumed three-second delayed
outputs remove both STO permissions and drop redundant K1/K2 contactors.
Mirror NC feedback from both contactors is in the reset EDM loop; reset is edge
monitored, not automatic restart. A relay fault removes STO immediately (category
0). Z's spring brake applies at 2.9 s, before STO at 3 s, provisionally. Keep the
24 V safety supply upstream of K1/K2; fuse each branch. Verify brake retention,
its sequencing and stopping distance with bought drives. A timed software stop
alone is not the controlled part of the safety function.

The release valve's fused 24 V passes in series through instantaneous safety
permission, VFD zero-speed contact, an independent spindle-stopped contact and
the CPU request. Both speed sources must fail open/disagree safe, with separated
wiring and diagnostic checking. A forced-on CPU output cannot defeat them.
The zero-speed contacts' suitability and safe speed threshold need supplier
qualification. De-energised valve means clamp; verify stored air/bleed and
spring/drawbar behaviour on the ATC bench.

PL d/category 3 is a target. MTTFd 30 years, DC 90% and CCF 65 points are explicit
placeholders; achieved PL is null. Obtain component B10d/PFHd, switching counts,
mission time, diagnostic interval and common-cause evidence and do the full
safety calculation/fault tests before release. SF1 stop/STO, SF2 safe release,
SF3 unexpected restart and SF4 gravity retention are listed in the data.
No safety function depends on MCU code, SPI expander or RKD6 alone.

[The manufacturer's SS1 definition](https://www.pilz.com/en-GB/lexicon/sicherer-stopp-1-ss1)
establishes controlled braking followed by STO for stop category 1; supplier
implementations and measured stopping time determine the actual chain.

## Cabinet and wiring layout

800 × 1000 × 300 mm cabinet, four DIN rows: incoming isolator/protection and
K1/K2 on top; drives/VFD on the left power side; safety/24 V/fused valves in the
second control row; carrier, isolated serial and field terminals on the right.
Keep motor/VFD/braking-resistor conductors away from encoder, probe and analog
routes; cross at right angles. Terminate motor/encoder shields with 360-degree
clamps at cabinet entry and drive/motor bonding points per supplier instructions;
PE is a separate protective conductor. Bond cabinet door, frame, gantry and
motor/VFD chassis to the PE bar; do not use bearings or shields as PE.

Differential line drivers/receivers serve step/dir and encoder A/B. Condition
24 V sensors through isolation, drive valves through fused protected outputs,
and isolate/scale the DAC to 0–10 V. Put suppression at inductive loads without
slowing safety contactor/brake dropout beyond measured limits. Ventilation,
VFD/servo clearances and heat load await bought-part loss data; reserve filtered
forced ventilation and keep conductive dust out. Wire identity, ferrules,
terminal jumpers, shielding and PE continuity form the SR18/SR21 inspection.

Release remains gated by SR0 supply facts, supplier safety/rating data and
SR18/SR21 hardware/fault validation; the graph check is design verification.
