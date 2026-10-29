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

## Klipper resonance measurements

`processkit.moonraker.ResonanceImport.read` downloads recorded CSV artifacts
through Moonraker's file API. `PrinterLink.readResonance` exposes this on the
application's existing connection. Importing runs no G-code, moves no hardware,
and writes no printer configuration.

Acquisition has two inputs:

1. A spectrum from Klipper's `TEST_RESONANCES ... OUTPUT=resonances` or
   `SHAPER_CALIBRATE`, with `freq,psd_x,psd_y,psd_z,psd_xyz` columns. Extra shaper
   columns are accepted. The recommended shaper frequency is not a measured
   mechanical natural frequency, and its configured damping is not a damping
   measurement.
2. An unshaped free-decay capture from `ACCELEROMETER_MEASURE`, with Klipper's
   `#time,accel_x,accel_y,accel_z` columns. Identify an interval **after excitation
   stops**, while the motors hold position; pass its capture timestamps to the
   importer. A moving frequency sweep is not free decay. Record the physical
   joint pose and payload at acquisition, rather than using the printer's live
   position later.

Klipper writes these files under `/tmp`; Moonraker does not expose that directory
as a file root. Publish the selected artifacts under a configured `config` or
`gcodes` directory before importing. The caller supplies those relative paths;
there is no automatic copying, path discovery or interpretation of arbitrary
G-code log messages. Downloads encode individual path segments and reject
traversal. A capture is limited to 16 MiB and one million rows.

The fitter uses the spectrum to select a mode, then fits a continuous damped
sinusoid to the free trace, solving amplitude, phase and sensor bias linearly.
It rejects irregular sample clocks, insufficient cycles/sample resolution,
no identifiable amplitude decay, mismatched modes and waveform RMS error above
5%. This single-mode fit is deliberately narrower than Klipper's multimode
shaper selection. It does not claim a confidence interval from that RMS score.
The result preserves both content hashes, the selected decay interval and the
fit method.

`cadbridge.BeltResonanceCalibration` combines the measurement with the matching
assembly/RobotModel and its acquisition pose (physical RobotKit coordinates in
SI). It identifies one **common rigidity correction** for an explicit group of
carriage belts: for the CoreXY printer use `['beltA', 'beltB']`. A partial group,
an unrelated mechanism, explicit competing overrides or an elastic network is
refused. A single carriage mode cannot identify each CoreXY belt independently.
Mass/inertia and mode attribution remain model inputs; the resulting effective
cord rigidity is inferred from that measured mode, not a material test.

```haxe
link.readResonance("config", "resonances/x.csv", "resonances/ring_x.csv",
  "x", freeStart, freeEnd, measurement -> {
    var update = new cadbridge.BeltResonanceCalibration(
      assembly, robotModel, ["beltA", "beltB"], measurement, acquisitionPose);
    sys.io.File.saveContent("printer-calibration.json", update.encode());
    update.apply();
  });
```

`beforeFrequency`, `afterFrequency`, `rigidityFactor`, `fitted`, and a copied
`values` map expose the result before applying it. Applying changes the local
model, retaining its position-dependent span laws; every relation of each
selected belt updates together with basis `Measured`. Motor/mass/efficiency
assumptions remain. Changed source or model data invalidates the proposal.
Rebuild planning/simulation and derive fixed shaping from the resulting full
frequency range; this operation never installs the shaper on firmware.

Normal MachineAssembly descriptions preserve the calibration. Source-defined
printers may contain CAD members without rebuild recipes; the standalone overlay
above also covers them. After regenerating the original source assembly/model,
use `BeltResonanceCalibration.applySaved(assembly, robotModel, savedText)`.
It checks both source and physical-model hashes (including mass), and refuses
reapplication to an already calibrated or changed model. Use
`assembly.clearBeltCalibration(beltId)` before deliberately authoring a competing
transmission override.

Build the focused check programs with:
`./haxeon/scripts/haxeon build --project robotkit/cadbridge/tests/haxeon.resonance.json` and
`./haxeon/scripts/haxeon build --project processkit/tests/moonraker/haxeon.json`.

Upstream formats and assumptions:
[Klipper resonance measurement](https://www.klipper3d.org/Measuring_Resonances.html),
[Klipper shaper calibration implementation](https://github.com/Klipper3d/klipper/blob/master/klippy/extras/shaper_calibrate.py),
[Moonraker file API](https://moonraker.readthedocs.io/en/latest/external_api/file_manager/).

## Moonraker networking

`MoonrakerClient(host, port, events, timeout, secure)` uses NativeKit HTTP; `MoonrakerSocket` uses its WebSocket message
transport at `/websocket`. Pass the same `EventPump` that the application owns and drain it every frame, then call
`poll(now)` on the clients. Client callbacks run from `poll`, after native response data has been copied into managed memory.
Closing disposes event subscriptions and native handles and suppresses late completions. Reconnect by creating new clients.
The optional `secure` flag selects HTTPS/WSS with normal certificate verification; it defaults to plain HTTP/WS for Klipper's
usual local endpoint. NativeKit supplies the browser transport and fetch backend under its ABI32 bindings; browser CORS and
mixed-content rules apply. The editor still accepts its existing host/port address format.

Multipart bodies and Moonraker JSON-RPC id matching remain here. HTTP framing, WebSocket negotiation, message reassembly,
masking and control frames belong to NativeKit. The old wire helpers exist only in the loopback fake-server tests.
`sentBytes` counts submitted request-body bytes; NativeKit owns wire-header accounting.

The Moonraker test's native provider builds NativeKit with HTTP enabled, without the robotd aggregate. Linux builds require
libcurl development files. Run the loopback suite with `haxeon build --project processkit/tests/moonraker/haxeon.json` and
its generated binary. `bash processkit/tests/moonraker/web/check-compile.sh` checks the production client against ABI32.

## Enclosure guard interface

`processkit.guard.EnclosureGuard` consumes door requests, separate closed/open and
lock feedback, spindle/axis standstill, the E-stop chain, boundary status, and
setup/hold-to-run inputs. Its outputs are door direction, lock command, cutting
permission, reduced-speed setup permission, and a stop request for hazardous
spindle/process-axis motion. The host must apply those permissions to its drive
controls. Setup speed and feedback timeout are required explicit policy inputs.

The interface is a simulation/control contract. Physical actuator sizing,
stop-distance validation and safety qualification belong to each machine.
Run its focused interlock regression with:

```sh
./haxeon/scripts/haxeon run --jobs=1 --project=processkit/tests/guard/haxeon.json
```
