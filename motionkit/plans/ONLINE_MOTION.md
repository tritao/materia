# Online motion and jerk

Path timing uses TOPP-RA (our fork, `motionkit/native/vendor/toppra`) with TR3 streaming. It times a
given geometric path and is the only planner for prints, toolpaths, seams and certified transfers. It does
not cover free state-to-state motion toward targets that change every cycle, and nothing in the stack plans
jerk directly. This plan adds Ruckig for the first and records the second as research.

Packages involved: `motionkit` (generator, certification, lowering), `robotkit` (runtime submission,
stops), `app` (MissionPlayer force search, jog), `machinekit` (example acceptance).

## Division of responsibility

| Job | Planner |
|---|---|
| Timing a given path (print, toolpath, seam, certified transfer), including feed hold, starvation and stops on that path | TOPP-RA fork + TR3 windows |
| Free motion to changing targets: jog, teleop, force-search approach, visual or seam-tracking corrections, stopping free motion from any state | Ruckig |
| Jerk on long paths | OM-R1 (required since 2026-10-20) |

- **Stops on a path stay on the path.** Feed hold, stream starvation and stop requests during a process path
  brake along that path with TOPP-RA's controllable sets and TR3's safe-stop windows. Ruckig never takes a
  process path off its geometry (no bead, seam or cut leaves its path to stop).
- **Certified routes stay TOPP-RA.** A Ruckig move does not follow a certified collision-free route; it is used
  only where the motion has its own collision admission (free space, small corrections inside a checked envelope).
- **One downstream pipeline.** Ruckig output is piecewise polynomial; it goes through the same continuous
  certification against the caller's original limits, polynomial lowering, integer clock and device
  compilation as TOPP-RA output. No second execution path.
- **TOPP-RA fork scope.** Fork changes remain robustness, speed and streaming within TOPP-RA's formulation,
  upstreamable where generic. State-to-state generation and jerk are not added to the fork.

## Decisions

- OM-D1. Ruckig community edition (MIT), vendored as a submodule like TOPP-RA. Ruckig Pro (commercial:
  waypoints, intermediate positions) is not used; anything needing waypoints is a path and goes to TOPP-RA.
- OM-D2. Limits come from the same sources as path timing (motor-derived joint limits, motor-space rows where
  applicable); jerk limits are explicit per drive, with provenance like other assumed drive data.
- OM-D3. Multi-axis moves use Ruckig's time synchronization; phase synchronization (straight line in joint
  space) when the collision admission assumes a joint-space line.

## Steps

**OM0. Survey.** Every place that currently hand-builds short reactive motion or stops free motion
(force search's streamed chunks in `MissionPlayer`, jog, starvation stops of free motion, any retargeting),
with its current behaviour, limits and tests. Record jerk/acceleration at the drives for each as the baseline.

**OM1. Vendor and wrap.** Ruckig submodule; a MotionKit online generator taking current state (position,
velocity, acceleration) and target state with limits; output as polynomial segments; continuous certification
against the original limits; typed failure reasons. Native tests including mid-motion replanning with nonzero
acceleration (no acceleration jump at the replan).

**OM2. Runtime integration.** Submission through the existing executor and integer clock; retargeting replaces
the unexecuted suffix atomically with revision tokens, like TR3 window retirement.

**OM3. Stopping free motion.** Jerk-limited time-optimal stop from any state for jog and free moves; process-path
stops are unchanged (TOPP-RA/TR3). Test both, including that a stop during a process path stays on the path.

**OM4. Force search.** Replace the hand-built streamed chunks with the online generator; same limits and
tolerances, force-search result unchanged within its existing bound, smoother drive acceleration recorded.

**OM5. Live retargeting.** Jog/teleop from the editor and a seam-tracking correction inside a checked envelope;
latency and drive jerk recorded.

**OM-R1. Research: jerk on paths.** TOPP-RA's per-stage LPs are convex only up to acceleration. Evaluate
jerk-limited path timing (third-order TOPP variants, iterative convexification, or certified post-smoothing
against our current pressure-advance/shaping/lowering chain) on the printer corner, Paint and weld seam cases.
Named post-smoothing candidate: MoveIt 2's Ruckig smoothing (Ruckig between consecutive timed waypoint states,
slowing segments until jerk limits hold). It does not preserve geometry between waypoints, so on process paths
it is admissible only with a continuously certified deviation bound taken from the process tolerance (bead
width, seam tolerance, cutter allowance), recertification against the original limits, and measured added
time and drive jerk versus no smoothing. TOTG is not a candidate: TOPP-RA covers it and keeps exact geometry,
general constraints and streaming. Adopt only with continuous certification and measured benefit at the drives;
otherwise record the result.

**OM-R1 is now required (2026-10-20), driven by the sheet router.** The router's motion-error screen
(`machinekit/examples/sheet-router/fem/README.md`, commit 1ba79b551) shows the tool error comes from
TOPP-RA's acceleration steps, not from structural frequency. On the 26 Hz gantry, TOPP-RA contours give
0.073 mm contour / 0.085 mm rapid-to-cut residual. A 40 Hz structure still gives 0.084 / 0.037 mm.
Jerk-limited motion gives 0.011 / 0.008 mm. Deliver:
- Jerk-limited path timing for process paths (CNC contours first), keeping exact geometry, or certified
  post-smoothing with the deviation bound above.
- An optional per-axis input shaper (ZV/ZVD) stage in the same certification and lowering chain, tuned from
  measured modes. ZVD on TOPP-RA gives 0.026 / 0.004 mm for 38 ms per move and holds under ±15% frequency
  error; ZV does not.

Acceptance: the router's governing-pose contour and transition cases at ≤ 0.05 mm motion error with the
added cycle time reported, plus no regression on the printer corner, Paint and weld seam cases. The router's
SR13/SR16 control steps depend on this.

**OM-R1 breakdown.** Axis jerk along a path is `q''' = q'·s''' + 3·q''·s'·s'' + q'''(s)·s'^3`. It has two
independent sources, and each needs its own fix:
- **Timing (`s'''`).** TOPP-RA stages have constant path acceleration, so `s''` jumps at every stage join. No
  TOPP-RA constraint can bound this.
- **Geometry (`q''` jumps, `q'''`).** Circular corner fillets and G-code line–arc joins step in curvature. Axis
  acceleration then jumps by `κ·s'^2`, whatever the timing.

Current state, so steps don't redo it:
- Ruckig is vendored and wrapped (`mk_generate_state_to_state`) and is used for free entry/exit moves.
- Jerk limits exist in `PlanningLimits`/`ToolpathMotionBinding` but path timing ignores them.
- `InputShaper`/`mk_shape_trajectory` support ZV/MZV/EI, not ZVD.
- The router's "jerk-limited 0.011 mm" figure was a 1-DOF straight-edge Ruckig probe with exact stops; it is
  not yet evidence for curved multi-axis contours.

Steps:
- **J0. Measure.** Add axis jerk to continuous certification and the plan report. Record baselines on the
  router contour, printer corner, Paint and weld seam cases.
- **J1. Ruckig for rapids and rest-to-rest straight moves.** Time G0/air moves, and straight cuts with exact
  stops, with Ruckig phase synchronization. On linear axes a rest-to-rest phase-synchronized move is an exact
  straight line, so geometry and `RapidClearance` are unchanged. This targets the rapid-to-cut residual
  directly.
- **J2. Curvature-continuous corners.** `CornerBlender` emits G2 blends (clothoid pair or the existing
  `QuinticBlend`) within the same tolerance budget instead of circular fillets. Unavoidable curvature steps
  (line–arc joins from G-code) get a short in-tolerance clothoid transition. Where no transition fits, a speed
  cap keeps the acceleration step `Δκ·s'^2` within an explicit per-drive bound.
- **J3. Jerk filter on the time law.** Keep TOPP-RA. Filter s(t) with a twice-applied moving average of length
  `T = a/J`, which bounds `s'''` and keeps the tool exactly on the geometry.
  - Plan TOPP-RA against speed caps pulled earlier along s by `s'·T`, so the lagging filtered law cannot enter
    a slower zone too fast.
  - Certify velocity, acceleration and jerk continuously; the existing stretch is the fallback.
  - Streaming carries a fixed delay `T`. TR3 safe stops include the filter tail.
- **J4. Shaping.** Add ZVD to `InputShaper`. Allow shaping on s(t) (geometry-exact; equal to per-axis shaping
  on straight lines) as well as per axis. Tune from measured modes.
- **J5. Ruckig on s for stops.** Feed hold, stream-starvation stops and resume use 1-DOF Ruckig on the path
  parameter: jerk-limited and still on the path, consistent with "stops on a path stay on the path".
- **J6. Acceptance.** Rerun the router dynamic-error screen on real multi-axis contours with arcs and blended
  corners, replacing the straight-edge probe. Target ≤ 0.05 mm, with cycle-time cost reported. The printer,
  Paint and weld cases must not regress.
- **J7. Research, only if J3 costs too much cycle time.** Ruckig-on-s look-ahead: blocks with constant caps
  and look-ahead junction speeds, which is the classic S-curve feed planner. Or third-order TOPP.

Not candidates for process paths:
- per-axis Ruckig on contours (leaves the geometry);
- MoveIt-style Ruckig waypoint smoothing without a certified deviation bound.

All outputs are polynomials that go through the existing certification, lowering, integer clock and device
compilation; there is no second execution path. Per-drive jerk limits need real values (OM-D2) instead of the
`25 × acceleration` placeholder. Order: J0 first; J1, J2 and J3 can run in parallel; J4–J6 follow; J7 only if
needed.

**Order.** OM0 -> OM1 -> OM2 -> OM3 -> OM4 -> OM5; OM-R1 independent. Starts after 3d-printing lands;
independent of `simkit/plans/FIDELITY.md` and the pellet printer, so it may run in parallel with them.
