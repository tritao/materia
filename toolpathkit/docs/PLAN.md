# ToolpathKit migration plan

Use the `toolpathkit` worktree for the entire migration. Record each step as a separate commit. Run the CncKit, CamKit, ToolpathKit, MotionKit CNC/C7 and, once integrated, StockKit gates after each step.

## Inputs

- Interpreter fixes for G53 in cycles, M2/M30 shutdown, invalid G28/cycle under cutter compensation, and G80 with G0 are on `main`.
- CamSetup and CamFixture are on `main`.
- StockKit's CutterProfile and consumers are still in the `stockkit` worktree. Integrate that work before renaming tools.

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
