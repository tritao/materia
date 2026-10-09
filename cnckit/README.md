# CncKit

CncKit parses LinuxCNC G-code into ToolpathKit's `ToolpathProgram`. The
`CncController` owns the dialect, G54–G59 work offsets, G28/G30 home
positions, H/D mappings, and tool library. Physical frame, axes, speed,
tolerances, and travel belong to `toolpathkit.motion.MachineBinding`.

`CncCompiler.compileDetailed(source, controller, ?start, ?travel)` returns a
program and source-positioned diagnostics. The optional start is a machine
position; omitted travel means no travel check. The interpreter creates a
program setup for each G54–G59 offset it uses, with positions read from the
controller. The first setup is active at the start. A measured origin is
represented by a setup at that measured position in the program consumed by
the motion adapter.

`CncWriter.write(program, controller, ?travel)` exports supported operations
to LinuxCNC millimetre G-code. It validates optional setup stock and fixture
data, and checks machine travel when an envelope is supplied.

```haxe
var result = CncCompiler.compileDetailed(source, controller, start,
  binding.travel);
var lowered = ToolpathMotion.lower(result.program, binding);
```

## Printer G-code

`CncDialect.Printer` reads and writes the Marlin, Klipper and RepRap family.
A printer `CncController` carries `PrinterSettings`: the filament diameter, which turns `E`
into the bead laid along the path, and the firmware retract for `G10`/`G11`.

- `CncCompiler.compileDetailed(source, controller)` with a printer controller returns the printing
  operations of ToolpathKit: `E` becomes `Deposit` (a new one when the extrusion per distance
  changes by more than `ratioTolerance`, 0.5 %), `FilamentMove` (E-only moves, retracts) and
  `Move`s; `M104`/`M109`/`M140`/`M190`/`M141`/`M191` become `Heat`, `M106`/`M107` `Fan`; `M220`/`M221`
  are folded into the moves they scale. Slicer comments become `Layer`, `Feature` and `PrintObject`
  markers (PrusaSlicer, OrcaSlicer, Cura, `EXCLUDE_OBJECT_*`, `M486`). `G28` only sets
  `CncCompileResult.homes`: homing is the player's job. `G92` sets the extruder position only.
  Motion limits, stored settings and macros are reported once as `CNC_PRINTER_IGNORED` warnings.
- `PrinterWriter.write(program, controller, flavour)` writes the same operations as Klipper or
  Marlin G-code in millimetres with relative extrusion; reading the file back gives the same
  deposited volume (within the controller's tolerance) and geometry (within 1 um).

Run CncKit tests with:

```sh
./haxeon/scripts/haxeon run --project=cnckit/tests/haxeon.json
```

See [PLAN.md](PLAN.md) for the historical CNC roadmap and
[ToolpathKit's ADR](../toolpathkit/docs/ADR-001-toolpath-ir.md) for the shared
format decision.
