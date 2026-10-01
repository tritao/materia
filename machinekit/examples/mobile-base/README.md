# Mobile base

A differential-drive mobile robot generated with MachineKit: an aluminium
base plate, two NEMA 23 steppers driving 150 mm wheels directly (continuous
joints `wheel_l`, `wheel_r`), swivel casters front and rear, a battery, an
upper deck on tube posts and a planar lidar.

- `MobileBase.hx` — the assembly (`PosedAssembly`) and its example-only parts.
- `MobileBasePreview.hx` — the project entrypoint and `MobileBaseChecks`
  (floor contact, track width, wheel rotation sense, no interference), run by
  the MachineKit smoke suite.
- `PLAN.md` — where this example is going: driving, missions, lidar, arm.

Open it from the editor's Start page, or check it alone with
`haxeon/scripts/haxeon run --project machinekit/examples/mobile-base/haxeon.json`
(CadKit's native libraries on `LD_LIBRARY_PATH`).
