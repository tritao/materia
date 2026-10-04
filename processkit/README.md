# ProcessKit

ProcessKit owns work geometry and task interpretation above RobotKit's generic
robot, skill and tool interfaces. RobotKit does not depend on ProcessKit.

- `processkit.work`: surfaces, raster coverage, height maps and earthwork regions.
- `processkit.path`: process toolpaths and points.
- `processkit.skill`: finishing, scanning, registration, welding and earthwork skills.
- `processkit.manipulation`: surface patches and process-path reachability planning.
- `processkit.motion`: MotionKit lowering for surface and toolpath plans.
- `processkit.perception`: work-surface registration and simulated surface scans.
- `processkit.tool` and `processkit.simulation`: weld contracts, arc models and simulated welders.
- `processkit.cadbridge`: CAD face and BIM wall conversion to work surfaces.

The package also owns the welding plan runners and process-device interfaces.
Consumers declare a `processkit` dependency and import these types directly;
there are no RobotKit or MotionKit re-export aliases. Mechanical geometry,
generic manipulation and tool control remain in RobotKit. The low-level runtime
continues to validate the frozen external sensor frame contract at publication;
process state, planning and simulated arc behavior belong here.
