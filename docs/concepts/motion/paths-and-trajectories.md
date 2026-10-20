---
title: Paths vs. trajectories
description: Geometry vs. timing, and the two planners that turn one into the other.
status: draft
---

# Paths vs. trajectories

## The problem

"Move the tool from here to there" leaves two questions open:

- **Where** does it go on the way? Along a weld seam, the edge of a part, a
  toolpath, or any route that is convenient?
- **When** is it at each point? How fast can it go there, given what the
  motors and the mechanism can do?

Some motion has a fixed answer to *where*: a cut, a bead or a seam must follow
its geometry exactly, and only *when* is free. Other motion only cares about
the end points: a move to a pick position can take whatever route is fastest
and smoothest. These are different problems, and Materia uses a different
planner for each.

## The model

A **path** is geometry only: joint positions as a function of distance along
the path, `q(s)`. It says where, never when.

A **trajectory** adds time: joint positions as a function of time, `q(t)`,
and with them velocity, acceleration and jerk.

### Path timing: TOPP-RA

When the path is fixed, the planner's only freedom is how fast to move along
it, `s(t)`. **TOPP-RA** (time-optimal path parameterization by reachability
analysis) finds the fastest `s(t)` that keeps every joint within its velocity
and acceleration limits, including the acceleration that curvature demands at
speed. The geometry is never changed. If the limits cannot be met at some
point, the path slows down there; it never cuts the corner.

Path timing is used for everything whose geometry matters: linear and
circular moves, toolpaths, prints, seams and certified transfer routes.

### Free moves: Ruckig

When only the end points matter, **Ruckig** plans state-to-state motion: from
the current position, velocity and acceleration to a target, within velocity,
acceleration **and jerk** limits. With phase synchronization, every joint
follows one profile scaled to its distance, so a move between two rests is a
straight line in joint space.

Ruckig is used for free joint moves, jogging, and motion toward targets that
change while moving. It never takes a process path off its geometry.

### The difference, computed

The figure below is computed by MotionKit itself (`docs/figures`): one move of
a two-axis gantry from (0.2, 0) to (0, 0.2) m, with the same axis limits for
both planners (0.5 m/s, 2 m/s², and 40 m/s³ for Ruckig). TOPP-RA times a
quarter-circle path; Ruckig makes a free move.

<figure class="materia-figure" data-figure="paths-vs-trajectories">
<figcaption>TOPP-RA keeps the tool exactly on the arc and slows for its curvature; Ruckig takes the shorter straight route, arriving sooner but up to 59 mm away from the arc. Drag the slider, press Play, or hover a time chart.</figcaption>
</figure>

*The figure is interactive on the documentation site. Its data is
[paths-vs-trajectories.json](../../figures/data/paths-vs-trajectories.json).*

Read it as:

- **Same end points, different routes.** The path planner cannot leave the
  arc; the free move goes straight.
- **Different limits bind.** On the arc, the speed is capped by the
  acceleration that curvature needs at speed (v²/r), so TOPP-RA never reaches
  the axes' full speed. Ruckig, on a straight line, runs both axes at their
  limits.
- **Jerk.** Ruckig holds jerk at its limit. TOPP-RA does not plan jerk at all:
  its jerk is whatever its timing produces.

### Both become the same thing

Whichever planner runs, its result is lowered to the same form: bounded joint
polynomials in an [execution plan](execution-plans.md), checked against the
same limits. Downstream, nothing knows which planner made it.

## In Materia

| Piece | Where |
|---|---|
| Path samples `q(s)`, `q'(s)`, `q''(s)` (with the curvature jump where a line meets an arc) | `motionkit.planner.JointPathSamples` |
| Path timing limits: per-joint velocity and acceleration, per-span feed caps, start and end path speeds | `motionkit.planner.PathTimingLimits` |
| TOPP-RA timing, whole or streamed in windows | `motionkit.planner.ToppraPathTiming` (fork in `motionkit/native/vendor/toppra`) |
| A timed path: its trajectory plus the distance-to-time law | `motionkit.planner.TimedPath` |
| Ruckig state-to-state moves | `motionkit.trajectory.Trajectory.generateStateToState` |
| Which program step uses which | `ProgramCompiler`: `MoveL`, `MoveC` and `FollowPath` are timed paths; `MoveJ` is a free move |

## What it isn't

- **A path is not a toolpath.** A toolpath (`ToolpathProgram`) is a machining
  program with tools and setups; it is compiled into motion programs, whose
  moves become paths (see [Toolpaths](../process/toolpaths.md)).
- **Path timing does not limit jerk yet.** Plans made by path timing record
  jerk as *unchecked*, not as passed. Jerk-limited path timing is required
  work (OM-R1 in `motionkit/plans/ONLINE_MOTION.md`).
- **A free move is not collision-checked by construction.** Ruckig moves are
  used where the motion has its own collision admission; certified routes stay
  with path timing.

## See also

- [The life of a machine](../start/life-of-a-machine.md), step 6
- [Execution plans](execution-plans.md)
- [Machine-axis views](machine-axes.md)
- [Mechanical truth](../modelling/mechanical-truth.md): where the limits come from
