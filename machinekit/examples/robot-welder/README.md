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
exposed ports. The work lead goes to a magnetic work clamp on the weldment's
base plate, and the scene's `torch` robot tool is derived from that: the arc
returns through the plate and everything welded to it, and nothing else.

The scene's mission welds one seam: the plate's T-joint with the upright (`materia.post.project.json` is the same cell
with a mission that welds the four sides of a tube post as one step, a path of four segments). The weld step is relative to
the workpiece's reference member, so it follows the workpiece where it stands. When the
simulation runs, the arm approaches the seam, strikes the arc and waits for it, travels
the seam at the speed that deposits the leg the weldment asks for, fills the crater,
stops the wire and retracts, and the weld metal grows along the seam as the simulated
welder deposits it. The recipe (`machinekit.welding.WeldingRecipe`) derives the wire
speed, voltage and travel speed from the seam's leg size.

- `ArmWeldingTool.hx` — the arm's welding end effector (an `ArmTool`): adapter
  plate and torch, with the `tcp` working frame at the wire tip.
- `WeldingCell.hx` — the cell.
- `WeldingWorkpiece.hx` — the workpiece (plate T-joint and tube frame) and the
  `Weldment` that says which joints are welded. Its seams are not authored: they
  are found from the members' faces (`machinekit.welding.WeldSeams`).
- `WeldSeamChecks.hx` — checks of the derived seams: names, frames, edits, gaps.
- `RobotWelderPreview.hx` — the project entrypoint (which writes the weld mission) and `RobotWelderChecks`
  (services supplied, torch pose, reach of the derived seam frames, torch and arm
  clearances, bill of materials), run by the MachineKit smoke suite.
- `PLAN.md` — where this example is going: seams from the CAD, a simulated
  welder, missions, then the mobile welder.

Open it from the editor's Start page, or check it alone with
`haxeon/scripts/haxeon run --project machinekit/examples/robot-welder/haxeon.json`
(CadKit's native libraries on `LD_LIBRARY_PATH`).
