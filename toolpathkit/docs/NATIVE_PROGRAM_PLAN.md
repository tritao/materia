# Native toolpath programs

Run machining programs on our own control stack from `ToolpathProgram` itself, with G-code
as an export for other controllers rather than the hand-off format.

Status: planned 2026-10-20. Nothing below is implemented, scheduled or referenced from other
plans yet.

## What exists

- CamKit produces `ToolpathProgram` (work-frame moves, typed move kinds, per-move tolerance,
  exact arcs, `Cam(operationId, featureRef)` provenance). `cnckit.CncWriter` exports it as
  LinuxCNC-dialect G-code, and `CncCompiler` reads G-code back into a program; the round trip
  is tested (PLAN.md T5).
- `toolpathkit-motion` lowers a program onto MotionKit (`ToolpathMotion.lower`,
  `MachineBinding`), with source maps, position-tied spindle and coolant events, corner
  diagnostics, feed hold and path-slice restart (`MachiningRun`, `MachiningContinuation`; T6).
- The app plays programs in simulation (`CncProgramPlayer`, `ToolpathPlayer`,
  `StockSimulationSession`), and the RobotKit runtime streams RKD6 segments to a virtual device.
- Gaps:
  - **No file format for `ToolpathProgram`.** G-code text is the only persisted form, so
    fabrication packages ship `.ngc` files and lose provenance, tool identity and setup data.
  - **G-code-shaped operations.** `ToolChange(number)` and `ToolLengthOffset(number, length)`
    model T and G43 H; `Tool` is keyed by an integer number; with G43 active, `Move` points
    name the tip and the controlled point sits `length` above it.
  - **No binding of a program to a machine, a source package or a release decision.**
  - **No hardware.** Nothing in the repository has driven a real motor; the G474 board is a
    UART stub (see `machinekit/examples/sheet-router/PLAN.md` SR-D7, SR15, SR16).

## Decisions

- **NP-D1. The program is the artifact.** Producers persist `ToolpathProgram` through one strict
  codec. G-code is generated from it for external controllers and is never read back as the
  source of a package. `CncCompiler` stays the way external G-code enters our stack.
- **NP-D2. Tool identity, not tool numbers.** A program references tools by stable ID with a
  declared cutter and nominal reach. `ToolChange(toolId)` replaces `ToolChange(number)`;
  `ToolLengthOffset` leaves the format. Move points are always the tool tip in the active
  setup's work frame. The measured length is a machine fact bound at run time (tool setter or
  a measured tool table). The run fails when the measurement falls outside the declared reach
  or stick-out the program was planned with. `CncCompiler` maps T/H/G43 into this model, using
  the `CncController` tool table, and `CncWriter` maps it back to T/H numbers through a
  controller tool table.
- **NP-D3. Setups declare how their origin is found.** A setup keeps its nominal work origin,
  stock and fixtures, plus an origin source: nominal, or probed by a named routine. At run time
  a probed origin replaces the nominal one, and travel, fixture and stock checks rerun against
  the measured pose before motion. G54–G59 remain a G-code export concern only.
- **NP-D4. One unit system.** The file stores SI metres, seconds and radians, as the IR does. No
  unit field and no millimetre variant. Displays convert.
- **NP-D5. Strict, versioned, fingerprinted.**
  - One format version is accepted and others are rejected, with no migration path.
  - Every op is validated on decode as the constructor does now.
  - The program carries:
    - its producer and source fingerprints (package, plan, panel or part);
    - the machine profile ID and fingerprint it was planned for;
    - its own content fingerprint.
- **NP-D6. Release is a property of the program.** A program carries `release`, true only when
  its source package's readiness allowed it. The runtime refuses unreleased programs outside an
  explicit test or dry-run mode, and refuses a machine-profile or source mismatch.
- **NP-D7. Provenance is mandatory.** Every motion op carries provenance; CAM ops carry operation
  and feature references. Feed hold, resume, errors and as-run records report against these
  references, not line numbers.
- **NP-D8. As-run records.** Each run writes a fingerprinted record:
  - program fingerprint;
  - measured tool lengths and setup origins;
  - overrides, holds and resumes, by op reference;
  - faults, start and end times.

  Downstream evidence (for example tooling coupon trials) references this record.

## Format sketch

```json
{
  "format": "materia.toolpath-program", "version": 1,
  "producer": { "kit": "camkit", "source": "sha256:…", "plan": "sha256:…", "item": "base-side-left" },
  "machine": { "profile": "sheet-router", "fingerprint": "sha256:…" },
  "release": false,
  "setups": [{ "id": "sheet-1", "origin": { "nominal": [0.02, 0.02, 0.0], "source": "probe:sheet-corner" },
               "stock": { "size": [2.44, 1.22, 0.0182], "measured": true }, "fixtures": ["vacuum-zones-1-4"] }],
  "tools": [{ "id": "upcut-6-2f", "cutter": { "diameter": 0.006, "fluteLength": 0.022 },
              "reach": { "min": 0.030, "max": 0.040 } }],
  "ops": [
    { "op": "toolChange", "tool": "upcut-6-2f", "ref": { "operation": "panel-profile" } },
    { "op": "spindle", "dir": "cw", "rpm": 18000, "ref": { "operation": "panel-profile" } },
    { "op": "move", "kind": "rapid", "line": [0.12, 0.08, 0.025], "ref": { "operation": "panel-profile" } },
    { "op": "move", "kind": "plunge", "line": [0.12, 0.08, -0.001], "feed": 0.01, "tol": 2e-5,
      "ref": { "operation": "panel-profile", "feature": "joint-tab-3" } },
    { "op": "move", "kind": "cut", "arc": { "plane": "xy", "center": [0.14, 0.08], "end": [0.16, 0.08, -0.001], "cw": true },
      "feed": 0.0667, "tol": 2e-5, "ref": { "operation": "panel-profile" } },
    { "op": "end" }
  ],
  "fingerprint": "sha256:…"
}
```

## Phases

### NP0 — Tool identity and tip-frame moves

Replace `ToolChange(number)` and `ToolLengthOffset` with tool IDs (NP-D2), and key `Tool` and
`ToolLibrary` by ID with declared reach. Update CncKit (T/H/G43 mapping through the controller
tool table), CamKit, toolpathkit-motion lowering, StockKit, the printer ops and the app players
in one change, without aliases.

Acceptance:
- CncKit golden fixtures keep their motion counts, length, bounds and lowered geometry, and
  CamKit counts are unchanged.
- G-code → program → G-code round-trips T/H/G43 through a controller tool table.
- A program with no `ToolLengthOffset` lowers identically to its G43 equivalent.

### NP1 — Program codec

A `toolpathkit` codec for the whole program: setups with origin sources, tools, every
`ToolpathOp` including printer, gear and signal ops, provenance, producer and machine bindings,
and fingerprint (NP-D4, NP-D5).

Acceptance:
- Encode/decode is identity on every existing CamKit, CncKit and printer fixture.
- Unknown fields, versions and op kinds are rejected.
- The fingerprint changes with any op, tool, setup or binding and with nothing else.

### NP2 — Package artifacts

CamKit producers (sheet profiles, tooling panels) write programs through the codec, with
mandatory provenance (NP-D7). Fabrication packages hash them, and G-code becomes a generated
export beside them, checked by the existing CncCompiler round trip.

Acceptance:
- Every package program decodes, lowers and matches its G-code export within tolerance.
- A tampered program fails the manifest check.

### NP3 — Machine binding and release

A machine profile ID and fingerprint on `MachineBinding`. Load-time checks for profile, source
and release (NP-D6). Tool measurement binding and probed setup origins, with travel, fixture
and stock checks rerun on the measured values (NP-D2, NP-D3).

Acceptance:
- Wrong profile, stale source, unreleased program, out-of-reach measured tool and a probed
  origin that breaks travel are each rejected before motion, with the op reference.

### NP4 — Runtime job

The RobotKit runtime accepts a program as a job. It is lowered and timed on the host through
toolpathkit-motion and streamed as RKD6 segments. Feed hold, feed and spindle overrides, single
block and resume from an op reference use `MachiningRun`/`MachiningContinuation`. The editor
gets an operator view (job, current op and feature, holds, faults). As-run records follow
NP-D8.

Acceptance:
- On the virtual device, a sheet-router panel program runs end to end, holds and resumes at a
  named feature without gouging (checked in StockKit), and writes an as-run record whose
  fingerprints match the job.

### NP5 — First hardware run

Runs the NP4 job on the real board once its hardware release exists.

Acceptance:
- A coupon program cuts on the physical machine, with its as-run record and measured
  dimensions attached as evidence.

## Dependencies

- **Jerk-limited path timing:** NP4 timing quality depends on it (`motionkit/plans/ONLINE_MOTION.md`
  OM-R1).
- **Hardware:** NP5 depends on the board's step, encoder, drive I/O and safety work and the first
  frozen RKD6 release (`machinekit/examples/sheet-router/PLAN.md` SR15, SR16).
- **No dependency on hardware for NP0–NP3,** which are worth doing on their own: they make the
  program the single source of truth in packages.

## Trade-offs

- **For:**
  - one representation from CAM to motion, with no lossy text round trip and no tool-table or
    work-offset drift;
  - feature-level holds, resumes and errors;
  - package integrity that reaches the spindle;
  - simulation that plans exactly what the machine runs.
- **Against:**
  - nothing runs on hardware before NP5, and safety functions become ours to get right;
  - external controllers still need G-code, so the export stays maintained;
  - a format, a runtime and an operator view to maintain, where an existing controller gives
    those today.

## Out of scope

- Splines, adaptive engagement feed, and 4/5-axis or robot tool orientation (deferred as in
  PLAN.md).
- Controller-side G-code features beyond what `CncCompiler` already reads.
- A G-code sender for third-party controllers.
- EtherCAT drives.
