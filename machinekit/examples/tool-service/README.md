# Cobot electrical tool service

This is an opt-in power/data interface and CAD mounting study for the generic
cobot family. It does not reproduce an OEM robot tool socket or certify a harness.

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/tool-service/haxeon.json
```

Electrical checks can also run without rebuilding the CAD previews:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/tool-service/haxeon.checks.json
```

The preview command checks the circuit contract, saved junction recipe and mounting on
Reach500/850/900/1300. It exports `build/cobot-service.mtrg`, `build/junction.mtrg`,
STEP geometry, an isolated `build/wrist-service.mtrg` and `build/report.json`. The junction preview is a separate component
reference; its installation on the dual DH adapter is not implemented here.

## Interface and connection graph

```text
External controller 24 V supply and RS485 master
              |
CobotArm.toolServicePowerIn / toolServiceSignalIn
              |
     Wrist service socket (one physical output connector)
              |
CobotArm.toolPower / toolSignal
              |
     Dual-gripper passive junction
          |                 |
       Gripper A         Gripper B
```

`new CobotArm(cls)` retains its existing layout. Call
`arm.installToolService(new ToolServiceSpec())` to add the socket and its removable
split collar. The collar follows the terminal housing taper, reserves 0.5 mm for a
liner and has two M3 clamp-bolt holes. Four M3 holes mount the socket to the upper
half. The mount is provisional: liners, tightening torque, clamp screws, socket
screws, shell stresses and dynamic retention still need engineering.

The module inlet and each outlet have two logical ports at one physical connector:
power and signal. Directed bridges let service tracing follow both through the
socket and junction. A wrist socket has one outlet; a junction has two. Neither is
a DC converter or an active protocol gateway. The combined peak current is shared,
not independently available in full at every branch.

The proposed contract is 24 V, 5 A at the single wrist outlet/shared harness, 2 A per junction branch and
Modbus RTU/RS485. These capacities are design targets, not ratings of selected
hardware. The current [DH PGC specification](https://en.dh-robotics.com/product/pgc)
lists 0.4 A rated and 1.2 A peak per PGC-140-50, so two simultaneous peaks total
2.4 A. Confirm these values against the exact purchased revision; older catalogues
list different peak currents. Bus addresses 1 and 2 are proposed configuration,
not detected or programmed device settings.

The eight-contact connector shapes are **unselected planning envelopes**. There
are no asserted pin assignments, connector coding, sex, IP rating or vendor SKU.
`pinoutVerified` remains false. Power and signal declarations do not count contacts
twice or establish the physical pin mapping. Controller-side inlet ports mark a
logical boundary; no physical passage through the joints is implied. In particular,
the existing 48 V motor supply must not be connected to the 24 V grippers.

## Validation and remaining work

`ToolServiceSpec.checkLoads` / `ToolServiceModule.checkLoads` explicitly check
voltage, protocol, independent Modbus addresses, finite positive peak current,
branch limits and simultaneous shared supply/harness capacity. The module also
checks outlet count for directly connected loads. `checkPassThrough` budgets the
single wrist output feeding a junction; only the downstream grippers have bus
addresses, not the passive junction. `checkWiredLoads` traces to the actual rated source and checks the upstream wrist
module capacity as well. It rejects an unrated external boundary. The source must
be dedicated to the supplied complete tool-load list, or separately budgeted for
other loads. Call these when assembling a circuit: generic port matching alone
does not validate electrical budgets or pinouts. This is currently a Modbus
addressing model, not a model of arbitrary fieldbus devices.

The example connects logical endpoints for both grippers and verifies power and
signal service-network tracing through the junction and wrist to the external controller boundary,
recipe preservation, valid single-solid CAD, mutual support clearances and zero
volume overlap with the terminal housing, flange, joint-six housing and rotor on
all four arm sizes at the local wrist mounting. This does not establish clearance
from other moving links, a tool, the machine, or across the full joint range.
Computed service mass uses aluminium CAD volume, not a measured electronics mass.

Next work is exact DH cable/mating connector selection and pinout; selected 24 V
source and serial master; power protection, voltage drop and shielding; RS485
termination/bias and short branch topology; dual-adapter junction mounting; and a
physical harness with joint-motion limits and a checked service-loop envelope.
The previous large-return cable prototype has a confirmed CAD clash with the arm
at a tested pose and must not be reused as a validated route. Its test evidence is
in the tending example's `build/electric-access/collision-audit.json`.

Manufacturer interfaces for the imported UR5e/FR5 robots are separate, source-backed
profiles in [the reference-arm example](../reference-arms/README.md#manufacturer-wrist-electrical-interfaces).
Do not apply this custom 5 A service contract or collar geometry to those robots.

Purchased Fairino Standard and Expert adapters are modeled separately from the
custom service-module experiment. See [FAIRINO-ADAPTERS.md](FAIRINO-ADAPTERS.md)
for product identities, BOM behavior, provisional geometry and the preview command.
