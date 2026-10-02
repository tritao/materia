# Robot welder

A fixed MIG welding cell generated with MachineKit: the six-axis arm from
`robot-arm` on its pedestal carrying a robot torch (22° or 45° swan neck, nozzle,
contact tip, breakaway mount), a wire feeder on its upper arm, a welding power
source and a shielding gas cylinder on the floor, and a welding table with a
workpiece held on it: a plate T-joint and a small tube frame.

Services reach the torch through ports, the way a real cell is cabled: the gas
cylinder feeds the power source, which feeds the feeder with weld current, gas
and control, and the feeder feeds the torch and the wire. The power source's
`mains` inlet (230 V single-phase) and its `control` input are the cell's own
exposed ports.

- `ArmWeldingTool.hx` — the arm's welding end effector (an `ArmTool`): adapter
  plate and torch, with the `tcp` working frame at the wire tip.
- `WeldingCell.hx` — the cell.
- `WeldingWorkpiece.hx` — the workpiece (plate T-joint and tube frame) and the
  `Weldment` that says which joints are welded. Its seams are not authored: they
  are found from the members' faces (`machinekit.welding.WeldSeams`).
- `WeldSeamChecks.hx` — checks of the derived seams: names, frames, edits, gaps.
- `RobotWelderPreview.hx` — the project entrypoint and `RobotWelderChecks`
  (services supplied, torch pose, reach of the derived seam frames, torch and arm
  clearances, bill of materials), run by the MachineKit smoke suite.
- `PLAN.md` — where this example is going: seams from the CAD, a simulated
  welder, missions, then the mobile welder.

Open it from the editor's Start page, or check it alone with
`haxeon/scripts/haxeon run --project machinekit/examples/robot-welder/haxeon.json`
(CadKit's native libraries on `LD_LIBRARY_PATH`).
