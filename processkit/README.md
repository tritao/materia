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

Screwdriving uses `ScrewdriverOutputs`, `ScrewdriverFeedback` and
`ScrewdriverProcessDevice` for simulation and commissioned controllers.
`DiscreteScrewdriver` requires an atomic I/O acquisition containing the actual
selected-program code, ready/OK/NOK inputs, seven measured process values,
sequence, clock identity and acquisition time. The angle is cumulative spindle
rotation from the engagement datum, in radians. Digital result bits and an
Open Protocol angle-from-snug report cannot replace that measurement.

For a revision-1 network controller, compose `OpenProtocolScrewdriver` with an
`OpenProtocolClient`, the commissioned measured I/O implementation and its
`ScrewdriverIoProfile`. The profile's binary codes identify the same parameter
sets in both interfaces. Selection must be acknowledged before network enable;
start, reverse and vacuum remain physical I/O commands. Other controller
revisions require their own supported profile.

`ScrewdriverDeviceOwner` services either controller independently of planning.
Pass the owner as outputs, feedback and status to `ScrewdriverProcessDevice`,
using the fitted tool's exact channel names and qualified slot. Prepare and wait
for readiness before applying channel events. A controlled stop preserves
vacuum and discards queued drive requests; reprepare explicitly before restart.
The owner's clock must match the acquisition clock. The I/O provider owns an
independent watchdog, since closing TCP does not prove a physical stop.
Shutdown waits for fresh measured zero speed and, for network controllers, an
acknowledged disable; an unacknowledged shutdown is reported as a fault.

`ScrewdriverRuntimeBinding` connects that owner to a declared external
`tool_screwdriver` sensor and the runtime's exact typed channels. It exposes the
common process device and its owner for the existing runner. Apply each newly
fired channel record once; the binding routes only its own tool's records.
Acquisitions publish on the owner thread with the provider's sequence, clock
and acquisition time. Repeated polls do not refresh the frame. Close the
binding before disposing its robot runtime. Hardware transport/profile choices
remain deployment configuration; the authored CAD mission stays unchanged.
