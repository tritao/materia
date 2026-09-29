# CncKit roadmap

This is the historical CNC roadmap. The initial dialect is LinuxCNC; G92 is rejected; CAM belongs in a separate `camkit`; and rotary A is deferred until MachineKit supports it. The current shared format and follow-up work are tracked in [ToolpathKit's plan](../toolpathkit/docs/PLAN.md).

## Phase 0 — Restore tests

Trace Haxeon's HashLink and `haxeon_runtime.hdll` selection. Fix mismatched native runtime loading in Haxeon. Pass CncKit and the focused MotionKit CNC and C7 suites from this worktree.

## Phase 1 — Correctness

Apply LinuxCNC block order: F/S, T, M6, M3/M4/M5, M7/M8/M9, G4, modal changes, motion, then M0/M1/M2/M30. Add `CncDialect.LinuxCnc`. Preserve partially blended paths and issue line-tied exact-stop warnings. Replace one-billion-second waits with indefinite barriers; centralize channel names. Make the blend corner limit a machine setting. Cover circles, arc direction and validity, G91 arc centres, comments, lowercase, program end, ordering, blend fallback, and G43/G49 relative mode.

## Phase 2 — Parse, interpret, lower

Split `CncCompiler` into `parse/` (`CncLexer`, `CncBlock`, `CncSpan`), `interp/` (`CncState`, `CncInterpreter`), `ir/` (`CncOp`), and `lower/` (`CncLowering`). Return `CncCompileResult` with program, ops, source map, and all `CncDiagnostic`s. Recover at the next line after an error. Keep `compile()` as a first-error throwing wrapper. Geometry in `CncOp` is in metres so editor preview does not need MotionKit. Preserve Phase 1 and MotionKit C6/C7 behavior.

## Phase 3 — Real files

Accept `%`, O and N; warn on ignored `/` block delete; accept G40 only when compensation is off, G80 cancellation, and G94; reject G93/G95. Add one-block G53 machine moves, G28/G30 stored-home moves, and reject G92. Add R arcs, G18/G19, helices, and G81/G82/G83/G73 with G98/G99 retract modes. First add a MotionKit `CircularSegment` that supports any plane and axial rise through blending (initially exact stop), TOPP-RA, joint conversion, and deviation checks. Add FreeCAD and Fusion LinuxCNC fixtures with golden counts, length, bounds, and geometry tolerance.

## Phase 4 — Machine model

Introduce `CncTool` with number, length, diameter. Check travel envelopes during CNC compilation and derive them from MotionBinding joint limits. Add G41/G42 offsets for lines and arcs in the active plane, with lead-in/out and gouge checks, plus dedicated fixtures.

## Phase 5 — CAM producer (superseded format boundary)

Create `camkit` for 2.5D profile, offset pocket, and drill operations from cadkit faces, edges, or sketches and the tool table. The original instruction to emit `CncOp` directly is superseded by [ADR 001](../toolpathkit/docs/ADR-001-toolpath-ir.md): CamKit returns `ToolpathProgram` with its tools and setups. CncKit writes G-code and compiles it back to that shared program format. Target MachineKit gantry parts and manufacturingkit sheet profiles first.

## Dependency order and progress

0 → 1 → 2 → 3a (file syntax, coordinates, cycles and R arcs) → 4 → 5. MotionKit 3b (`CircularSegment`) depends on 0 and may be built alongside 2. CncKit 3c (G18/G19 and helices) depends on 2 and 3b. Update this log after each phase.

- [x] Phase 0 — CncKit 20 baseline assertions; MotionKit CNC 50 and C7 2,845 assertions after restoring `ObjectMap.remove`. Haxeon launcher rebuilds a mismatched native release pair.
- [x] Phase 1 — LinuxCNC ordering, indefinite waits, partial-blend warnings and configurable corner limit. CncKit 38 assertions; MotionKit CNC 50 and C7 2,845 assertions.
- [x] Phase 2 — Parser, transactional modal interpreter, metre-based CNC IR, and MotionKit lowering with distance-aware source map. Structured diagnostics recover at the next line; `compile()` keeps its first-error behavior. CncKit 51, MotionKit CNC 50, and C7 2,845 assertions pass.
- [x] Phase 3a — LinuxCNC file syntax, line-only G53, stored G28/G30 homes, R arcs, and G73/G81/G82/G83 cycles with G98/G99. Real posted FreeCAD and Fusion output fixtures have golden motion counts, length, bounds, and lowered-path geometry checks. CncKit 194, MotionKit CNC 50, and C7 2,845 assertions pass.
- [x] Phase 3b — MotionKit `CircularSegment` supports XY/XZ/YZ planes and axial rise, with unit tangents, Cartesian second derivatives, and 3D distance checks. Circular corners remain exact stops in blending and timing. Direct MotionSystem, ProgramCompiler/TOPP-RA, and task-space checks pass in all planes. MotionKit CNC 80, C7 2,845, full bootstrap 9,496, and CncKit 194 assertions pass.
- [x] Phase 3c — G17/G18/G19 arcs and helices, including plane-specific I/J/K centres and R form, stay as metre geometry in preview and lower through MotionKit `CircularSegment`. G18 direction follows LinuxCNC's positive-Y viewpoint. CncKit 214, MotionKit CNC 89, and C7 2,845 assertions pass.
- [x] Phase 4 — `CncTool` stores number, length, and diameter. The binding intersects logical axis travel with model joint limits; compilation checks line and arc extrema and reports `CNC_TRAVEL` at the G-code span. G41/G42 with a declared cutter offsets planar lines and arcs, trims inside corners, rounds outside corners, and rejects short lead moves or gouges. G19 cutter compensation remains unsupported per LinuxCNC. Dedicated geometry fixtures cover inside, outside, line-to-arc, and arc offsets. CncKit 240, MotionKit CNC 92, C7 2,845, and full bootstrap 9,508 assertions pass.
- [x] Phase 5 — `camkit` accepts solved cadkit sketches, CAD edges and faces, and manufacturingkit sheet placements. Its former metre-based CNC operation output was replaced by `ToolpathProgram` under ADR 001; profile and pocket cuts support depth steps. Direct lowering preserves operation spans for editor highlighting, and the LinuxCNC writer round-trips geometry and process commands through CncKit. The historical gate had 551 CamKit assertions; current counts are in ToolpathKit's plan.
- [x] CAM setup export gate — `CamSetup` records stock bounds, safe Z and rectangular fixture keep-outs in work coordinates. G-code export validates machine travel, stock bottom, lateral rapid clearance and the cutter-radius swept path through fixtures, including arcs. A generated clamp fixture covers collision, depth and clearance rejection. CamKit 12,204 assertions pass.
- [x] CNC interpreter follow-up — G53 is rejected during an active drilling cycle; G28/G30 and drilling cycles fail at their own lines under active cutter compensation; G80 and G0-G3 conflict in one block. M2/M30 turn off active spindle and coolant outputs before End. MotionKit blend results carry corner indices directly for source warnings. CncKit 250, MotionKit CNC 92, C7 2,845 and CamKit 12,204 assertions pass.
