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
| Jerk on long paths | Neither yet (OM-R1) |

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

**Order.** OM0 -> OM1 -> OM2 -> OM3 -> OM4 -> OM5; OM-R1 independent. Starts after 3d-printing lands;
independent of `simkit/plans/FIDELITY.md` and the pellet printer, so it may run in parallel with them.
