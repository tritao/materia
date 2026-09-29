# ToolpathKit motion adapter

`toolpathkit-motion` lowers a `ToolpathProgram` into MotionKit paths and
execution plans. `MachineBinding` owns the physical frame, XYZ axes, rapid
speed, tolerances, corner limit, and optional `TravelEnvelope`; it does not
store setup positions. `ToolpathMotionBinding` intersects logical axis travel
with model joint limits and makes the resulting envelope available as
`binding.machine.travel`. The program supplies setup positions, including
measured or probed origins.

```haxe
var result = CncCompiler.compileDetailed(gcode, controller, start,
  binding.machine.travel);
var lowered = ToolpathMotion.lower(result.program, binding.machine);
var compiled = binding.compile(result.program, initialJoints, firstPlanId);
```

The adapter also contains CNC execution scenarios, machining feed hold and
restart tests. With the MotionKit native vendors linked in a worktree, run:

```sh
./haxeon/scripts/haxeon run --project=toolpathkit/motion/tests/haxeon.json
```
