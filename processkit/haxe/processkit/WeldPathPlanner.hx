package processkit;

import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;
import motionkit.robot.CompiledProgram;
import motionkit.trajectory.PlanDiagnostic.PlanCheckResult;
import robotkit.manipulation.ClearanceWorld;
import processkit.skill.WeldPlan;
import processkit.skill.WeldPlan.WeldSegment;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** How the torch comes in to the start of a weld: a joint move to `joints`, then straight moves through `moves` (the last is the approach pose). */
class WeldEntry {
  public final name:String;
  public final joints:Array<Float>;
  public final moves:Array<Pose3>;

  public function new(name:String, joints:Array<Float>, moves:Array<Pose3>) {
    this.name = name;
    this.joints = joints;
    this.moves = moves;
  }
}

/** A weld planned for an arm: the plan with the torch's rolls chosen, and how it comes in and leaves. */
class PlannedWeld {
  public final plan:WeldPlan;
  /** Continuous joint selection has no single constant roll per segment. */
  public final selected:Bool;
  /** The roll of the torch about its wire chosen for each segment, in radians. */
  public final rolls:Array<Float>;
  public final entry:WeldEntry;
  /** Where the torch ends up after the weld, and how it gets there. */
  public final retreat:Pose3;
  public final exitName:String;
  /** How the torch turns into each segment (`WeldCorner.AROUND` and the like); the first segment has no turn. */
  public final styles:Array<Int>;
  /** How many poses of the motion were checked for reach, and for clearance. */
  public final checked:Int;
  /** Verified joint configuration after retreat, for planning the next motion on the same branch. */
  public final endJoints:Array<Float>;

  public function new(plan:WeldPlan, rolls:Array<Float>, styles:Array<Int>, entry:WeldEntry, retreat:Pose3, exitName:String, checked:Int,
      endJoints:Array<Float>,selected:Bool=false) {
    this.selected=selected;
    this.styles = styles;
    this.plan = plan;
    this.rolls = rolls;
    this.entry = entry;
    this.retreat = retreat;
    this.exitName = exitName;
    this.checked = checked;
    this.endJoints = endJoints.copy();
  }
}

/** A pose of the planned motion, and whether the torch is working the seam there (within the contact zone). */
private typedef Step = {
  var pose:Pose3;
  var contact:Bool;
}

private typedef Chained = {
  var q:Null<Array<Float>>;
  var reason:Null<String>;
  var collision:Bool;
}

private typedef Way = {
  var name:String;
  /** Out along the open side of the corner rather than the wire. */
  var open:Bool;
  var scale:Float;
  /** Over the work first, then down. */
  var high:Bool;
}

/**
 * Plans a weld for an arm before it runs: which roll of the torch about its wire each segment is welded at, and how the
 * torch comes in and leaves, so that every pose of the motion is reachable and the torch and arm are clear of the work.
 *
 * A seam's frame fixes the wire (the work and travel angles) and, by its +X along the travel, also the torch's roll about
 * the wire: the swan neck leads. That roll does not matter to the weld, but it decides whether the arm can hold the pose
 * and what the neck and the arm come near, and the sides of a tube, with the travel turning a quarter at each corner, ask
 * for four different ones. So each segment is tried at the rolls in `ROLLS`, the one nearest the previous segment's first,
 * and also tries a roll derived from the previous orientation to avoid extra wrist twist. It takes the first at which
 * the arm can follow the corner into it and the segment, every `STEP` metres, without leaving
 * its branch, and at which `clearance` finds nothing near. If the segments after cannot be done from where that leaves the
 * arm, the search goes back and tries the next roll (at most `BUDGET` segment tries per entry configuration).
 *
 * The first segment is entered, and the last left, along the first of these that is reachable and clear: out along the wire
 * by the plan's approach distance, along the open side of the corner by that distance, and both again at twice the
 * distance. If a joint move from where the arm stands to the entry would sweep the arm through the work, the entry is made
 * from over it: a joint move to `VIA_HEIGHT` above the approach pose, then straight down.
 *
 * The search checks a joint-space sweep to the entry, the straight moves, the segments with their corner turns, the lift
 * and the retreat, sampled every `STEP` metres of the tip (and every 0.02 rad of a joint in the joint move). Where the tip is
 * within `CONTACT_ZONE` of the weld path the torch is held to the contact margin of the clearance, elsewhere to its margin.
 * `compileMotion`, when supplied, then validates the complete program and its actual joint trajectories before a candidate
 * is accepted. A weld no roll and no entry can do fails with the reasons: the part, its neighbour, the distance and where the tip was.
 */
class WeldPathPlanner {
  /** Spacing of the poses checked along a straight stretch, in metres. */
  public static inline var STEP:Float = 0.002;
  /** The tip is working the seam within this distance of the weld path, in metres. */
  public static inline var CONTACT_ZONE:Float = 0.025;
  /** How far over the approach pose a high entry comes from, in metres. */
  public static inline var VIA_HEIGHT:Float = 0.10;
  /** How far the torch lifts while the arc burns back, in metres. */
  public static inline var LIFT:Float = 0.005;
  /** The most segment tries from each reachable entry configuration. */
  public static inline var BUDGET:Int = 1024;
  /** The welding cell's margin for arm and air motions, in metres. */
  public static inline var AIR_MARGIN:Float = 0.003;
  static inline var AIR_STEP:Float = 0.02;
  /** How many goals of the joint move to the entry are tried. */
  static inline var GOALS:Int = 12;
  /** The rolls about the wire tried for a segment, in radians: a quarter turns, and the eighths between. */
  public static final ROLLS:Array<Float> = [0.0, Math.PI / 4, -Math.PI / 4, Math.PI / 2, -Math.PI / 2, 3 * Math.PI / 4, -3 * Math.PI / 4, Math.PI];

  final solver:KinematicsSolver;
  final tolerance:IkTolerance;
  final velocity:Array<Float>;
  final wrist:WristLimits;
  final clearance:Null<ClearanceWorld>;
  final maxJointJump:Array<Float>;
  final compileMotion:Null<(PlannedWeld, Array<Float>) -> CompiledProgram>;
  var checked:Int = 0;
  var budget:Int = 0;
  var attempts:Int = 0;
  /** The compiled entry moves are shared by every downstream roll from this entry. */
  var entryMotionBlocked:Bool = false;
  /** The weld path of the plan being planned: where the torch works. */
  var path:Array<Segment3> = [];
  final collisions:Array<String> = [];
  final unreachable:Array<String> = [];

  /**
   * `velocity` weighs the joints when the joint move to the entry picks among the arm's configurations (the nearest to
   * where it stands); `wrist` limits the corner turns (`WeldCorner`); `clearance` is null for a cell without hulls.
   * `maxJointJump` is the compiler's continuity bound; `compileMotion` returns validated plans without robot submission.
   */
  public function new(solver:KinematicsSolver, tolerance:IkTolerance, velocity:Array<Float>, wrist:WristLimits, ?clearance:ClearanceWorld,
      ?maxJointJump:Array<Float>, ?compileMotion:(PlannedWeld, Array<Float>) -> CompiledProgram) {
    if (solver == null || tolerance == null || velocity == null || wrist == null) throw "A weld planner needs a solver, tolerances, joint speeds and the wrist's limits";
    this.solver = solver;
    this.tolerance = tolerance;
    this.velocity = velocity;
    this.wrist = wrist;
    this.clearance = clearance;
    this.compileMotion = compileMotion;
    this.maxJointJump = maxJointJump == null ? [for (_ in 0...solver.jointCount()) 0.2] : maxJointJump.copy();
    if (this.maxJointJump.length != solver.jointCount()) throw "A weld planner needs one continuity bound per joint";
    for (jump in this.maxJointJump) if (!(jump > 0.0) || !Math.isFinite(jump)) throw "Weld continuity bounds must be finite and positive";
  }

  /**
   * The weld `requested` planned for an arm standing at joints `start`; throws the reasons when it cannot be. The planner
   * can be used again for another weld.
   */
  public function plan(requested:WeldPlan, start:Array<Float>):PlannedWeld {
    checked = 0;
    budget = BUDGET;
    attempts = 0;
    entryMotionBlocked = false;
    collisions.splice(0, collisions.length);
    unreachable.splice(0, unreachable.length);
    path = [for (item in requested.segments) ({from: item.start.translation, to: item.stop.translation} : Segment3)];
    var found = search(requested, start, 0, [], [], [], null, 0.0, null);
    // A closed run has no required starting corner. The CAD tour's nearest corner may strand the wrist later in the run.
    if (found == null && requested.segments.length > 1 && requested.stop().translation.sub(requested.start().translation).norm() <= WeldPlan.JOIN) {
      for (corner in 1...requested.segments.length) {
        var segments = requested.segments.slice(corner).concat(requested.segments.slice(0, corner));
        var rotated = new WeldPlan(segments, requested.parameters);
        budget = BUDGET;
        found = search(rotated, start, 0, [], [], [], null, 0.0, null);
        if (found != null) break;
      }
    }
    if (found == null) {
      var reversed = requested.reversed();
      budget = BUDGET;
      found = search(reversed, start, 0, [], [], [], null, 0.0, null);
      if (found == null && reversed.segments.length > 1 && reversed.stop().translation.sub(reversed.start().translation).norm() <= WeldPlan.JOIN) {
        for (corner in 1...reversed.segments.length) {
          var rotated = new WeldPlan(reversed.segments.slice(corner).concat(reversed.segments.slice(0, corner)), reversed.parameters);
          budget = BUDGET;
          found = search(rotated, start, 0, [], [], [], null, 0.0, null);
          if (found != null) break;
        }
      }
    }
    if (found == null) {
      var names:Array<String> = [];
      for (segment in requested.segments) if (segment.name != "" && names.indexOf(segment.name) < 0) names.push(segment.name);
      var label = names.length == 0 ? "the weld" : 'the weld of ${names.join(", ")}';
      // Keep both kinds of failure visible; a large number of collisions must not hide the reach failures.
      var all:Array<String> = [];
      for (index in 0...Std.int(Math.max(collisions.length, unreachable.length))) {
        if (index < collisions.length) all.push(collisions[index]);
        if (index < unreachable.length) all.push(unreachable[index]);
      }
      var reasons:Array<String> = [];
      for (reason in all) if (reasons.indexOf(reason) < 0) reasons.push(reason);
      var shown = reasons.slice(0, 80);
      throw 'Cannot plan $label: no roll of the torch is both reachable and clear of the work ($attempts tries); ' + shown.join("; ") +
        (reasons.length > shown.length ? '; and ${reasons.length - shown.length} more' : "");
    }
    return found;
  }

  /** The search over segment `index` and the ones after: null when no choice of rolls gets through them. */
  function search(requested:WeldPlan, start:Array<Float>, index:Int, rolled:Array<WeldSegment>, rolls:Array<Float>, styles:Array<Int>,
      carry:Null<Array<Float>>, previous:Float, entry:Null<WeldEntry>):Null<PlannedWeld> {
    var segments = requested.segments;
    var segment = segments[index];
    var candidates = ROLLS.copy();
    candidates.sort(function(x, y) return Reflect.compare(Math.abs(x - previous), Math.abs(y - previous)));
    if (index > 0) {
      var continued = WeldCorner.continuationRoll(rolled[index - 1].stop.rotation, segment.start.rotation);
      candidates = [continued].concat([for (roll in candidates) if (Math.abs(roll - continued) > 1e-6) roll]);
    }
    for (roll in candidates) {
      if (index > 0 && entryMotionBlocked) return null;
      if (index > 0 && budget <= 0) return null;
      if (index > 0) budget--;
      attempts++;
      var spin = Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), roll);
      var turned = new WeldSegment(new Transform3(segment.start.translation, segment.start.rotation.multiply(spin)),
        new Transform3(segment.stop.translation, segment.stop.rotation.multiply(spin)), segment.name, segment.open);
      var label = '${segment.name == "" ? 'segment $index' : segment.name} at roll ${Math.round(roll * 180 / Math.PI)} degrees';
      if (index == 0) {
        var found = enter(requested, turned, start, label, function(chosen, q) {
          // A reachable approach can lead into an IK branch that cannot finish the chain. Try each entry against
          // the whole weld; no earlier entry consumes the budget of the configurations that follow it.
          budget = BUDGET;
          attempts++;
          return follow(requested, start, index, turned, rolled, rolls, styles, q, roll, chosen, label);
        });
        if (found != null) return found;
      } else {
        var found = follow(requested, start, index, turned, rolled, rolls, styles, cast carry, roll, cast entry, label);
        if (found != null) return found;
      }
    }
    return null;
  }

  /** Follow a segment and the rest of the chain from this entry or the preceding segment's IK branch. */
  function follow(requested:WeldPlan, start:Array<Float>, index:Int, turned:WeldSegment, rolled:Array<WeldSegment>,
      rolls:Array<Float>, styles:Array<Int>, q:Array<Float>, roll:Float, entry:WeldEntry, label:String):Null<PlannedWeld> {
    for (style in 0...(index == 0 ? 1 : WeldCorner.STYLES)) {
      if (entryMotionBlocked) return null;
      if (budget <= 0) return null;
      if (style > 0) {
        budget--;
        attempts++;
      }
      var steps = weldSteps(requested, index, turned, rolled, style);
      var done = chain(steps, [q], label + (index == 0 ? "" : ", turning " + styleName(style)));
      if (done.q == null) continue;
      var finished = rolled.concat([turned]);
      var rollsNow = rolls.concat([roll]), stylesNow = styles.concat([style]);
      if (index == requested.segments.length - 1) {
        var out = leave(requested, turned, done.q, label);
        if (out == null) continue;
        var candidate = new PlannedWeld(new WeldPlan(finished, requested.parameters), rollsNow, stylesNow, entry, out.retreat, out.name, checked, out.q);
        var end = verify(candidate, start, label);
        if (end == null) continue;
        return new PlannedWeld(candidate.plan, rollsNow, stylesNow, entry, out.retreat, out.name, checked, end);
      }
      var rest = search(requested, start, index + 1, finished, rollsNow, stylesNow, done.q, roll, entry);
      if (rest != null) return rest;
    }
    return null;
  }

  /** Check the compiler's actual trajectories before accepting a sampled path, without sending anything to the robot. */
  function verify(candidate:PlannedWeld, start:Array<Float>, label:String):Null<Array<Float>> {
    var compile = compileMotion;
    if (compile == null) return candidate.endJoints.copy();
    var compiled:Null<CompiledProgram> = null;
    try {
      compiled = compile(candidate, start);
      var speed = 0.0;
      for (limit in velocity) speed = Math.max(speed, limit);
      var interval = AIR_STEP / speed;
      if (clearance != null) for (block in compiled.blocks) for (index in 0...block.plans.length) {
        var trajectory = block.plans[index];
        var samples = Std.int(Math.max(1.0, Math.ceil(trajectory.durationSeconds / interval)));
        for (sample in 0...samples + 1) {
          var q = trajectory.evaluate(trajectory.durationSeconds * sample / samples).positions;
          var tip = solver.forward(q);
          var contact = block.opIndices[index] != 0 && near(new Vec3(tip.x, tip.y, tip.z));
          var hit = clearance.violation(q, contact);
          checked++;
          if (hit != null) {
            // Later seam rolls cannot change the same entry or approach. Try
            // another entry rather than rechecking this blocked trajectory.
            if (block.opIndices[index] <= candidate.entry.moves.length + 1) entryMotionBlocked = true;
            throw describe(hit, tip) + ' in operation ${block.opIndices[index]} at ${trajectory.durationSeconds * sample / samples}s';
          }
        }
      }
      var end = candidate.endJoints.copy();
      for (block in compiled.blocks) for (trajectory in block.plans)
        end = trajectory.evaluate(trajectory.durationSeconds).positions.copy();
      compiled.dispose();
      return end;
    } catch (error:Dynamic) {
      if (compiled != null) compiled.dispose();
      unreachable.push('$label, compiled motion: $error');
      return null;
    }
  }

  /**
   * How the first segment is entered: the first way in that the arm can follow, from a joint move to the entry's first pose
   * (goals nearest to where it stands) to the segment's start. `accept` checks whether the whole weld can be finished
   * from that entry. Null, noting the reasons, when none can.
   */
  function enter(requested:WeldPlan, segment:WeldSegment, start:Array<Float>, label:String,
      accept:(WeldEntry, Array<Float>) -> Null<PlannedWeld>):Null<PlannedWeld> {
    var wireIn = segment.start.rotation.rotate(new Vec3(0.0, 0.0, 1.0));
    var distance = requested.parameters.approach;
    for (way in ways()) {
      var out = way.open ? segment.open : wireIn.negate();
      if (out == null) continue;
      var approach = segment.start.translation.add(out.scale(distance * way.scale));
      var upright = new Vec3(0.0, 0.0, VIA_HEIGHT);
      var waypoints = way.high ? [approach.add(upright), approach] : [approach];
      var poses = [for (point in waypoints) pose(point, segment.start.rotation)];
      var continued = solver.solvePose(poses[0], start, tolerance, motionkit.path.OrientationPolicy.FreeAboutTool);
      // Prove the current branch before paying for global discovery. Failed continuations retain every other sampled goal.
      for (discovery in [false, true]) {
        var goals:Array<Array<Float>> = [];
        if (!discovery) {
          if (continued != null) goals.push(continued);
        } else {
          for (goal in solver.sampleCandidates(poses[0], GOALS, tolerance, motionkit.path.OrientationPolicy.FreeAboutTool)) {
            var duplicate = false;
            if (continued != null) {
              var separation = 0.0;
              for (joint in 0...goal.length) separation += (goal[joint] - continued[joint]) * (goal[joint] - continued[joint]);
              duplicate = separation < tolerance.candidateSeparation * tolerance.candidateSeparation;
            }
            if (!duplicate) goals.push(goal);
          }
        }
        if (discovery && continued == null && goals.length == 0) unreachable.push('$label, ${way.name}: no entry IK configuration at ' + where(poses[0]));
        goals.sort(function(a, b) return Reflect.compare(cost(a, start), cost(b, start)));
        for (goal in goals) {
          var air = clearance == null ? null : clearance.sweep(start, goal, false, AIR_STEP, AIR_MARGIN);
          if (air != null) {
            collisions.push('$label, ${way.name}: the joint move to the entry (tip bound for ' + where(poses[0]) + ') ' + describeHit(air));
            continue;
          }
          var steps:Array<Step> = [{pose: poses[0], contact: false}];
          var from = waypoints[0];
          for (index in 1...waypoints.length) {
            line(from, segment.start.rotation, waypoints[index], segment.start.rotation, steps, true);
            from = waypoints[index];
          }
          line(from, segment.start.rotation, segment.start.translation, segment.start.rotation, steps, true);
          var done = chain(steps, [goal], '$label, ${way.name}');
          if (done.q != null) {
            entryMotionBlocked = false;
            var accepted = accept(new WeldEntry(way.name, goal, poses.slice(1)), cast done.q);
            if (accepted != null) return accepted;
          }
        }
      }
    }
    return null;
  }

  /** The ways the torch comes in or leaves, in the order they are tried. */
  function ways():Array<Way> {
    return [
      {name: "along the wire", open: false, scale: 1.0, high: false},
      {name: "along the open side", open: true, scale: 1.0, high: false},
      {name: "along the wire from twice as far", open: false, scale: 2.0, high: false},
      {name: "along the open side from twice as far", open: true, scale: 2.0, high: false},
      {name: "over the work, then along the wire", open: false, scale: 1.0, high: true},
      {name: "over the work, then along the open side", open: true, scale: 1.0, high: true}
    ];
  }

  /** How the torch leaves the end of the last segment: the first way that is reachable and clear from the weld's end. */
  function leave(requested:WeldPlan, segment:WeldSegment, from:Array<Float>, label:String):Null<{retreat:Pose3, name:String, q:Array<Float>}> {
    var wireOut = segment.stop.rotation.rotate(new Vec3(0.0, 0.0, 1.0));
    var distance = requested.parameters.approach;
    var end = segment.stop.translation;
    for (way in ways()) {
      if (way.high) continue;
      var out = way.open ? segment.open : wireOut.negate();
      if (out == null) continue;
      var away = end.add(out.scale(distance * way.scale));
      var steps:Array<Step> = [];
      var lift = end.sub(wireOut.scale(LIFT));
      line(end, segment.stop.rotation, lift, segment.stop.rotation, steps, false);
      line(lift, segment.stop.rotation, away, segment.stop.rotation, steps, true);
      var done = chain(steps, [from], '$label, leaving ${way.name}');
      if (done.q != null) return {retreat: pose(away, segment.stop.rotation), name: way.name, q: done.q};
    }
    return null;
  }

  /**
   * The poses of segment `index` (already rolled as `segment`) that the arm must follow: for any but the first, the turn at
   * the corner from the segment before (`rolled` ends with it), then the segment itself up to where the turn to the next
   * begins (the last segment, to its end).
   */
  function weldSteps(requested:WeldPlan, index:Int, segment:WeldSegment, rolled:Array<WeldSegment>, style:Int):Array<Step> {
    var steps:Array<Step> = [];
    var a = segment.start.translation, b = segment.stop.translation;
    var length = segment.length();
    var forward = b.sub(a).scale(1.0 / length);
    var last = index == requested.segments.length - 1;
    var travel = requested.parameters.travelSpeed;
    var from = a;
    var rotation = segment.start.rotation;
    if (index > 0) {
      var before = rolled[index - 1];
      var beforeLength = before.length();
      var back = before.stop.translation.sub(before.start.translation).scale(1.0 / beforeLength);
      var angle = before.stop.rotation.angularDistance(segment.start.rotation);
      var turnLength = WeldCorner.turnLength(angle, travel, wrist);
      var outer = WeldCorner.given(turnLength, beforeLength);
      var inner = WeldCorner.given(turnLength, length);
      // The part of the segment before that the corner covers, and what of it is turning.
      var covered = Math.min(WeldCorner.MAX_TURN, WeldCorner.SHARE * beforeLength);
      var corner = before.stop.translation;
      line(corner.sub(back.scale(covered)), before.stop.rotation, corner.sub(back.scale(outer)), before.stop.rotation, steps, false);
      // The two halves of the turn, the wire swinging along the shortest arc (`WeldCorner.orientationAt`).
      if (outer > 0.0) turn(corner.sub(back.scale(outer)), corner, before.stop.rotation, segment.start.rotation, 0.0, 0.5, back, forward, style, steps, true);
      if (inner > 0.0) {
        turn(a, a.add(forward.scale(inner)), before.stop.rotation, segment.start.rotation, 0.5, 1.0, back, forward, style, steps, true);
        from = a.add(forward.scale(inner));
      }
    }
    var until = last ? b : b.sub(forward.scale(Math.min(WeldCorner.MAX_TURN, WeldCorner.SHARE * length)));
    if (until.sub(from).norm() > 1e-6) line(from, rotation, until, segment.stop.rotation, steps, steps.length > 0);
    return steps;
  }

  /** The poses along a stretch of a turn from `r1` to `r2` at a corner between travel directions `travelA` and `travelB`, from fraction `s0` of it to `s1`. */
  function turn(from:Vec3, to:Vec3, r1:Quat, r2:Quat, s0:Float, s1:Float, travelA:Vec3, travelB:Vec3, style:Int, into:Array<Step>, skipFirst:Bool):Void {
    var steps = Std.int(Math.max(1.0, Math.ceil(to.sub(from).norm() / STEP)));
    for (step in (skipFirst ? 1 : 0)...steps + 1) {
      var fraction = step / steps;
      var at = from.add(to.sub(from).scale(fraction));
      into.push({pose: pose(at, WeldCorner.orientationAt(r1, r2, s0 + (s1 - s0) * fraction, travelA, travelB, style)), contact: near(at)});
    }
  }

  /** The poses along a straight stretch, the orientation turning from one end's to the other's. */
  function line(from:Vec3, fromRotation:Quat, to:Vec3, toRotation:Quat, into:Array<Step>, skipFirst:Bool):Void {
    var steps = Std.int(Math.max(1.0, Math.ceil(to.sub(from).norm() / STEP)));
    for (step in (skipFirst ? 1 : 0)...steps + 1) {
      var fraction = step / steps;
      var at = from.add(to.sub(from).scale(fraction));
      into.push({pose: pose(at, fromRotation.slerp(toRotation, fraction)), contact: near(at)});
    }
  }

  /**
   * Whether the arm can follow `steps` in turn, from one of `seeds`, within reach and clear; the configuration it ends in.
   * Otherwise the reason, noted for the failure message.
   */
  function chain(steps:Array<Step>, seeds:Array<Array<Float>>, label:String):Chained {
    var reason:Null<String> = null;
    var collided = false;
    for (seed in seeds) {
      var q:Null<Array<Float>> = seed;
      var failure:Null<String> = null;
      var blocked = false;
      for (index in 0...steps.length) {
        var step = steps[index];
        var previous = cast(q, Array<Float>).copy();
        q = solver.solvePose(step.pose, cast q, tolerance, motionkit.path.OrientationPolicy.FreeAboutTool);
        if (q == null) {
          failure = 'unreachable at ' + where(step.pose);
          break;
        }
        var jump = false;
        for (joint in 0...q.length) if (Math.abs(q[joint] - previous[joint]) > maxJointJump[joint]) jump = true;
        if (jump) {
          failure = 'IK branch discontinuity at ' + where(step.pose);
          q = null;
          break;
        }
        var hit = clearance == null ? null : clearance.violation(q, step.contact);
        checked++;
        if (hit != null) {
          failure = describe(hit, step.pose);
          blocked = true;
          q = null;
          break;
        }
      }
      if (failure == null) return {q: q, reason: null, collision: false};
      if (reason == null || blocked) {
        reason = failure;
        collided = blocked;
      }
    }
    if (reason != null) (collided ? collisions : unreachable).push('$label: $reason');
    return {q: null, reason: reason, collision: collided};
  }

  static function styleName(style:Int):String
    return style == WeldCorner.AROUND ? "around the corner" : style == WeldCorner.ARC ? "along the wire's arc" : "by rotation";

  static function describeHit(hit:robotkit.manipulation.ClearanceViolation):String
    return '${hit.a} is ${Math.round(hit.distance * 10000) / 10} mm from ${hit.b} (needs ${Math.round(hit.required * 10000) / 10} mm)';

  static function describe(hit:robotkit.manipulation.ClearanceViolation, pose:Pose3):String
    return describeHit(hit) + ' with the tip at ' + where(pose);

  static function where(pose:Pose3):String
    return '(${Math.round(pose.x * 10000) / 10}, ${Math.round(pose.y * 10000) / 10}, ${Math.round(pose.z * 10000) / 10}) mm';

  function cost(q:Array<Float>, from:Array<Float>):Float {
    var sum = 0.0;
    for (joint in 0...from.length) {
      var d = (q[joint] - from[joint]) / velocity[joint];
      sum += d * d;
    }
    return sum;
  }

  static function pose(at:Vec3, rotation:Quat):Pose3 return new Pose3(at.x, at.y, at.z, rotation.x, rotation.y, rotation.z, rotation.w);

  /** Whether `point` is within the contact zone of the weld path. */
  function near(point:Vec3):Bool {
    for (segment in path) {
      var along = segment.to.sub(segment.from);
      var length2 = along.dot(along);
      var t = length2 > 0.0 ? Math.max(0.0, Math.min(1.0, point.sub(segment.from).dot(along) / length2)) : 0.0;
      if (point.sub(segment.from.add(along.scale(t))).norm() <= CONTACT_ZONE) return true;
    }
    return false;
  }
}

private typedef Segment3 = {
  var from:Vec3;
  var to:Vec3;
}
