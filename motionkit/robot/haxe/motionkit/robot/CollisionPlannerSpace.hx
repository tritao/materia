package motionkit.robot;

import collisionkit.CollisionBuild;
import collisionkit.CollisionDescription;
import collisionkit.CollisionMargins;
import collisionkit.kinematics.ModelBodies;
import kinematicskit.MotionReach;
import motionkit.planner.BoxClearanceSpace;

/**
 * A `PlannerSpace` over a collisionkit world (COLLISION.md CL-D5, CL-D7):
 * a model's bodies in a built description, checked against the world with
 * `margins` (the safety margins plus the planner's slack).
 *
 * An edge is clear when its midpoint is clear with each body inflated by its
 * `MotionReach` bound over half the edge's per-joint travel: then no point
 * of any body comes closer along the whole edge. Otherwise it is halved, to
 * `depthLimit`; an edge whose midpoint collides as it is, or that is still
 * open at the limit, is blocked. Each bounded batch of pending intervals
 * uses one native call for the inflated checks and one for flagged
 * midpoints as they are.
 *
 * The same inflated check proves a whole box of configurations clear: every
 * configuration within the per-joint travel of the centre stays inside the
 * inflated bodies (`boxesClear`).
 */
class CollisionPlannerSpace implements BoxClearanceSpace {
  /** Bound world-pose buffers and visit deep halves before expanding every edge. */
  static inline var EDGE_BATCH_STATES:Int = 64;
  static inline final MAX_BATCH_POSES = 8388608;
  public final bodies:ModelBodies;
  public final build:CollisionBuild;
  public final description:CollisionDescription;
  public final margins:CollisionMargins;
  public final reach:MotionReach;
  public var depthLimit:Int = 14;
  final bodyLower:Array<Float>;
  final bodyUpper:Array<Float>;
  /** Per described body of the model (`bodies.all()` order): its model body and radius about its frame. */
  final modelBody:Array<Int> = [];
  final radius:Array<Float> = [];
  // Native queries consume these synchronously. Retain their capacity across
  // RRT edges instead of reallocating full-world pose arrays at every halving.
  final poseBuffer:Array<Float> = [];
  final inflationBuffer:Array<Float> = [];
  final exactBuffer:Array<Float> = [];
  final stateScratch:kinematicskit.KinematicState;
  final modelPoses:Array<Float> = [];
  /** Halvings run for the edges checked so far (diagnostics). */
  public var queries(default, null):Int = 0;

  public function new(description:CollisionDescription, build:CollisionBuild, bodies:ModelBodies, margins:CollisionMargins) {
    this.description = description;
    this.build = build;
    this.bodies = bodies;
    this.margins = margins;
    stateScratch = new kinematicskit.KinematicState(bodies.model);
    reach = new MotionReach(bodies.model);
    var model = bodies.model;
    bodyLower = [for (dof in 0...model.dofCount()) model.dofLower[dof]];
    bodyUpper = [for (dof in 0...model.dofCount()) model.dofUpper[dof]];
    for (d in 0...build.bodies.length)
      if (build.bodies[d] != d) throw "A collision planner space needs a world built from its description alone";
    var all = bodies.all();
    for (i in 0...all.length) {
      modelBody.push(i < bodies.bodies.length ? i : bodies.followed[i - bodies.bodies.length]);
      radius.push(description.bodyRadius(all[i]));
    }
    // Bodies outside the model (the environment, other articulations) do not move: no inflation.
  }

  public function dimension():Int return bodies.model.dofCount();
  public function lower():Array<Float> return bodyLower.copy();
  public function upper():Array<Float> return bodyUpper.copy();

  /** A selected chain cannot move other DOFs; their observed positions still set its reach bounds. */
  @:allow(motionkit.robot.ManipulatorPlannerSpace)
  function holdOtherDofs(reference:Array<Float>, active:Array<Int>):Void {
    if (reference.length != dimension()) throw "Held collision state has the wrong model dimension";
    for (i in 0...reference.length) {
      if (!Math.isFinite(reference[i])) throw "Held collision state must be finite";
      if (active.indexOf(i) < 0) { bodyLower[i] = reference[i]; bodyUpper[i] = reference[i]; }
    }
  }

  public function invalid(q:Array<Float>):Null<String> {
    var found = violation(q);
    if (found == null) return null;
    return '${name(found.a)} / ${name(found.b)} (${found.distance} m, needs ${found.required} m)';
  }

  /** Structured diagnostics for callers that need the actual contact pair. */
  public function violation(q:Array<Float>):Null<collisionkit.CollisionViolation> {
    bodies.place(build, bodies.stateInto(q, stateScratch));
    return build.world.violation(margins);
  }

  function name(worldObject:Int):String {
    var described = build.objectOf(worldObject);
    return described < 0 ? 'object $worldObject' : description.objects[described].name;
  }

  public function edgesClear(edges:Array<Float>, count:Int):Array<Bool> {
    var n = dimension();
    var result = [for (_ in 0...count) true];
    // Pending halves: edge index, start, end, depth.
    var pending:Array<{edge:Int, a:Array<Float>, b:Array<Float>, depth:Int}> = [];
    for (e in 0...count)
      pending.push({edge: e, a: edges.slice(2 * e * n, (2 * e + 1) * n), b: edges.slice((2 * e + 1) * n, (2 * e + 2) * n), depth: 0});
    var worldBodies = build.bodies.length;
    var all = bodies.all();
    // Respect the native per-array limit even for unusually large worlds.
    var batchLimit = Std.int(Math.max(1, Math.min(EDGE_BATCH_STATES, MAX_BATCH_POSES / (7 * worldBodies))));
    while (pending.length > 0) {
      var live = [];
      while (pending.length > 0 && live.length < batchLimit) {
        var p = pending.pop();
        if (result[p.edge]) live.push(p);
      }
      if (live.length == 0) break;
      var poses = poseBuffer, inflation = inflationBuffer, middles:Array<Array<Float>> = [];
      poses.resize(0); inflation.resize(0);
      for (p in live) {
        var middle = [for (k in 0...n) 0.5 * (p.a[k] + p.b[k])];
        middles.push(middle);
        var travel = [for (k in 0...n) 0.5 * Math.abs(p.b[k] - p.a[k]) * (1 + 1e-9) + 1e-12];
        appendWorld(middle, poses);
        var offset = inflation.length;
        for (_ in 0...worldBodies) inflation.push(0.0);
        for (i in 0...all.length) inflation[offset + build.bodies[all[i]]] = reach.bound(modelBody[i], radius[i], travel);
      }
      queries += live.length;
      var inflated = build.world.violations(poses, margins, inflation);
      var flagged = [for (k in 0...live.length) if (inflated[k]) k];
      if (flagged.length == 0) continue;
      var exact = exactBuffer;
      exact.resize(0);
      for (k in flagged) appendWorld(middles[k], exact);
      var real = build.world.violations(exact, margins);
      for (i in 0...flagged.length) {
        var p = live[flagged[i]];
        if (real[i] || p.depth >= depthLimit) {
          result[p.edge] = false;
          continue;
        }
        pending.push({edge: p.edge, a: p.a, b: middles[flagged[i]], depth: p.depth + 1});
        pending.push({edge: p.edge, a: middles[flagged[i]], b: p.b, depth: p.depth + 1});
      }
    }
    return result;
  }

  public function boxesClear(centres:Array<Float>, halfWidths:Array<Float>, count:Int):Array<Bool> {
    var n = dimension();
    if (centres == null || halfWidths == null || centres.length != count * n || halfWidths.length != count * n)
      throw "Collision planner boxes have the wrong dimension";
    if (count == 0) return [];
    var worldBodies = build.bodies.length, all = bodies.all();
    var poses = poseBuffer, inflation = inflationBuffer;
    poses.resize(0); inflation.resize(0);
    for (box in 0...count) {
      var travel = [for (k in 0...n) {
        var half = halfWidths[box * n + k];
        if (!(half >= 0.0) || !Math.isFinite(half)) throw "Collision planner box half widths must be finite and nonnegative";
        half * (1 + 1e-9) + 1e-12;
      }];
      appendWorld(centres.slice(box * n, (box + 1) * n), poses);
      var offset = inflation.length;
      for (_ in 0...worldBodies) inflation.push(0.0);
      for (i in 0...all.length) inflation[offset + build.bodies[all[i]]] = reach.bound(modelBody[i], radius[i], travel);
    }
    queries += count;
    var blocked = build.world.violations(poses, margins, inflation);
    return [for (box in 0...count) !blocked[box]];
  }

  /**
   * Every world body's pose (seven numbers each, in world body order) with
   * the model at `q`: the other bodies where the description has them.
   */
  function appendWorld(q:Array<Float>, out:Array<Float>):Void {
    bodies.posesInto(bodies.stateInto(q, stateScratch), modelPoses);
    var model = modelPoses;
    var all = bodies.all();
    var start = out.length;
    for (body in 0...build.bodies.length) {
      var pose = description.bodies[body].pose;
      out.push(pose.x); out.push(pose.y); out.push(pose.z); out.push(pose.qx); out.push(pose.qy); out.push(pose.qz); out.push(pose.qw);
    }
    for (i in 0...all.length) {
      var o = start + 7 * build.bodies[all[i]];
      for (k in 0...7) out[o + k] = model[7 * i + k];
    }
  }
}
