# ToolpathKit migration plan

The T0–T7 migration is complete. The table below records what landed and the
cleanup that followed. Use an isolated worktree for edits and run the suites
listed at the end before integrating further format changes.

## Historical inputs

The interpreter fixes for G53 in cycles, M2/M30 shutdown, invalid G28/cycle
under cutter compensation, and G80 with G0 landed before T0. StockKit's
cutter types and consumers were then integrated during T1–T7.

## T0: Skeleton and decision

Create the dependency-free package and test project. Record the format decisions in [ADR 001](ADR-001-toolpath-ir.md). Gate: the empty test project runs.

## T1: Move format files

Move `CncOp`, `CncGeometry`, `CncGeometryTools`, `CncPlane`, and `CncPoint` to `toolpathkit.path` as `ToolpathOp`, `PathGeometry`, `GeometryTools`, `ArcPlane`, and `Point3`. Move `CncTool` and the StockKit cutter types to `toolpathkit.tool`. Replace `CncSpan` in operations with `Provenance` (`GCode(line, column, length)` or `Cam(operationIndex)`). Update all consumers in one commit, without aliases. Gate: preserve pre-move assertion counts in CncKit, CamKit, MotionKit CNC/C7, and StockKit.

## T2: Enrich the format

Replace `Rapid` and `Feed` with `Move(kind, geometry, feed, tolerance, provenance)`. Kinds are Rapid, Cut, Plunge, Ramp, Link, and Retract. Convert G64 P to per-move tolerance and G61 to zero. Use typed spindle direction and RPM, coolant mist/flood, and tool-library IDs. Extend CAM provenance to `Cam(operationId, featureRef)` and fill real move kinds and feature references in CamKit. Gate: CAM tests check kinds and references; CncKit golden geometry and counts stay stable.

## T3: Motion adapter

Create `toolpathkit/motion`, package `toolpathkit-motion`, depending on ToolpathKit, MotionKit, MotionKit Robot and later ProcessKit. Move CNC lowering, pose primitives, source maps, channels and robot binding into the adapter. Move pure travel geometry checks into ToolpathKit. Move CNC/C7 scenario tests into adapter tests. Replace direct CNC and CAM motion compilation with producer to format to adapter execution. Gate: CncKit depends only on ToolpathKit; CamKit depends on ToolpathKit, CadKit and ManufacturingKit; MotionKit and MotionKit Robot have no CncKit dependency; C7 passes 2,845 assertions.

## T4: Machine model

Put setup work frame, stock bounds, fixtures and safe Z in `toolpathkit.setup`; ToolLibrary and TravelEnvelope in ToolpathKit; MachineBinding in ToolpathKit Motion; and G54–G59, G28/G30, H/D mapping in CncController. CncKit emits work-frame moves and `SetSetup`; CamKit emits its setup. Gate: CamSetup stock/clamp checks use ToolpathKit setup and G54/G55 round-trip passes.

## T5: G-code writer

Move CamGCodeWriter to `cnckit.CncWriter`. Map setups through the dialect, reject unsupported geometry and tool orientation, and add hand-built format to G-code to format and CAD to CAM to G-code round trips. Gate: both round trips agree within tolerance.

## T6: Direct execution

Represent spindle and coolant as position-tied events while retaining a spindle-speed barrier. Blend with per-corner tolerance, returning corner indices as data. Integrate feed hold, path-slice restart and spindle-fault stop with ProcessKit; assess whether machining needs its own recipe. Gate: a C7 pocket scenario supports feed hold and mid-path restart without exact stops at spindle/coolant changes.

## T7: StockKit

Change StockKit to consume ToolpathProgram and tag material intervals using provenance and move kind. Gate: StockKit passes without CncKit dependency.

## Sequence

T0 → T1 → T2 → T3 → T4 → T5. T6 starts after T3 and may proceed alongside T4/T5. T7 needs T1's tool types and finishes after T4. Defer splines, adaptive engagement feed and 4/5-axis or robot tool orientation until a consumer needs them.

## Progress log

| Step | Landed work | Commits |
| --- | --- | --- |
| T0 | Dependency-free package, test project, ADR 001 | `2f2b2d4a` |
| T1 | Shared path geometry, operations, tools, provenance; consumers migrated together | `00b7cb73` |
| T2 | Typed move roles, spindle/coolant, CAM feature provenance, compensation kept at the CNC boundary | `91915bd1`, `f0835d84`, `8a05a9f7`, `e1060c27` |
| T3 | Independent motion adapter, machine binding, lowering, source map and robot binding | `2dac185e`, `408ca168`, `95d2a2cc`, `548ce0bc` |
| T4 | Setup, fixtures, travel and controller mapping | `ef660431`, `81dbd6ef` |
| T5 | CNC G-code writer and direct round trips | `b4e8618e` |
| T6 | Position-tied process events, corner diagnostics, machining feed hold and restart | `1962e63c` |
| T7 | StockKit consumes `ToolpathProgram` and preserves move provenance | `b2696fb7` |

At the T7 handoff (`b2696fb7`), the recorded gates were ToolpathKit 9,
CncKit 304, CamKit 12,221, StockKit 7,049 plus 2,574 core rays,
toolpathkit-motion 10 machining-run plus 2,975 scenario assertions, and
MotionKit's focused CNC run 30. The full MotionKit count was re-baselined at
6,693 after its session-owned simulation migration. These historical counts
are the comparison point for the final gate below.

Deferred format work remains splines, adaptive engagement feed, 4/5-axis
machining, and robot tool orientation. A consumer should drive each addition.

## Cleanup after T7

### C — Package-owned StockKit native build

`eed1956b` declares `stockkit_core` in StockKit's manifest and removes the
duplicate app build. Haxeon commits `b74dc375` and `91f0153a` support a
package-owned CMake shared library at runtime and float enum payload
constants. StockKit, CamKit and the app run without a manual StockKit CMake
build or a StockKit `LD_LIBRARY_PATH` entry.

### A — One owner for machine and setup values

`b0c3fcad` makes `ToolpathProgram` carry setups, splits optional stock data
from required setup placement, and turns `TravelEnvelope` into a value that
checks a program with provenance. `MachineBinding` supplies physical data and
travel, never setup positions. `CncController` owns dialect, G54–G59 offsets,
homes, H/D mappings and tools; `CncCompiler.compileDetailed` accepts the
controller, optional start and optional travel. CamKit returns
`ToolpathProgram` directly. Regression checks cover G54/G55 placement,
probed-origin differences, unknown setups and G-code-line travel errors.

### B — MotionKit test layering

`49aa24f0` moves the CNC execution scenarios into toolpathkit-motion tests
and removes CNC and toolpath dependencies from MotionKit's test manifest.
`testCircularSegments` uses only MotionKit and now runs in the full suite;
its optional `MOTIONKIT_CIRCULAR_ONLY` mode is also available. The empty C7
mode is gone. Every public PlannerTests, ProcessTests and ProgramTests method
is in the full run or deliberately moved. The recovery, blend and MachineKit
focused modes only repeat tests already present in the full run.

### D — Direct base-package suites

`15b77c85` adds 50 direct ProjectKit assertions for units, assembly codecs
and frames, scene artifact versions and materials. `1daaace5` grows the
standalone ToolpathKit suite from 9 to 89 assertions for geometry, offsets,
setup and travel validation, and cutter shapes. Profile offset geometry now
lives in ToolpathKit and CamKit's existing 12,223 assertions still pass.

## Final gate

The requested single-worktree workflow uses clean generated test builds on
`feat/toolpath-cleanup`, with MotionKit vendor links and CadKit's native
directories on `LD_LIBRARY_PATH`. Haxeon builds StockKit core automatically;
no manual StockKit CMake step or StockKit runtime path is used. The assertion
counts below record that clean-build gate and were checked again after merging
current main into the branch. The app gained a worker demo test group from main.

| Suite | Final result |
| --- | ---: |
| ProjectKit | 50 |
| ToolpathKit | 89 |
| toolpathkit-motion core, machining-run, scenarios | 9 + 10 + 2,975 |
| CncKit | 306 |
| CamKit | 12,223 |
| StockKit, core rays | 100,667 + 17,494 rays |
| MotionKit full | 6,723 |
| App | 6 test groups passed; runner does not report an assertion total |
