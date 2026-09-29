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

Run CncKit tests with:

```sh
./haxeon/scripts/haxeon run --project=cnckit/tests/haxeon.json
```

See [PLAN.md](PLAN.md) for the historical CNC roadmap and
[ToolpathKit's ADR](../toolpathkit/docs/ADR-001-toolpath-ir.md) for the shared
format decision.
