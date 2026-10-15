import collisionkit.CollisionBuild;
import collisionkit.CollisionDescription;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionMargins;
import collisionkit.CollisionPose;
import collisionkit.kinematics.ModelBodies;
import collisionkit.native.NativeCollisionWorld;
import haxe.Int64;
import kinematicskit.JointKind;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicModelBuilder;
import kinematicskit.KinematicState;
import kinematicskit.Transform;
import kinematicskit.Vector3;
import motionkit.MotionOptions;
import motionkit.planner.MotionPlan;
import motionkit.planner.RrtConnect;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.robot.CollisionPlannerSpace;
import motionkit.robot.StructuredJointPathPlanner;
import robotkit.collision.CollisionClearance;
import collisionkit.CollisionDescription;
import robotkit.manipulation.Manipulator;

private class CountingSpace implements motionkit.planner.PlannerSpace {
  public var points = 0;
  public var edges = 0;
  public var blocked = false;
  public function new() {}
  public function dimension():Int return 1;
  public function lower():Array<Float> return [-2.0];
  public function upper():Array<Float> return [2.0];
  public function invalid(q:Array<Float>):Null<String> { points++; return q[0] > 1.0 ? "past boundary" : null; }
  public function edgesClear(values:Array<Float>, count:Int):Array<Bool> {
    edges += count;
    return [for (i in 0...count) !blocked && Math.max(values[2*i], values[2*i+1]) <= 1.0];
  }
}

/**
 * Free-space planning (COLLISION.md CL-D7, CL7): RRT-Connect over a
 * collisionkit world through a narrow passage; start and goal in collision,
 * named; determinism from a seed; and on main's weld arm, a planned joint
 * move around a post whose timed path runs along the checked edges and
 * passes validation.
 */
@:access(WeldPlanningTests)
class PlannerTests {
  static inline var D6 = 0.0823;
  static final START = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0];
  static var assertions = 0;

  public static function main():Void {
    testNarrowPassage();
    testLargeEdgeBatch();
    testExactQueryCache();
    testSavedRoutes();
    testTimingCache();
    testScratchRoots();
    testEndpointsInCollision();
    testEndpointRounding();
    testDeterministicFromSeed();
    testPlannedMoveThroughValidation();
    testFlyByThroughGap();
    var bench = Sys.getEnv("PLANNER_BENCH_DIR");
    if (bench != null && bench != "") benchmark(bench);
    Sys.println('Planner tests passed ($assertions assertions)');
  }

  static function testExactQueryCache():Void {
    var raw = new CountingSpace(), cached = new motionkit.planner.CachedPlannerSpace(raw, 2);
    check(cached.invalid([1.0]) == null && cached.invalid([1.0]) == null && raw.points == 1, "valid point answers are cached");
    check(cached.invalid([1.0 + 2.220446049250313e-16]) != null && raw.points == 2, "adjacent floating-point values have distinct cache keys");
    var flags = cached.edgesClear([0.0, 0.5, 0.0, 0.5, 0.0, 1.5], 3);
    check(flags[0] && flags[1] && !flags[2] && raw.edges == 2, "duplicate edges are checked once, including blocked results");
    cached.edgesClear([0.0, 1.5], 1);
    check(raw.edges == 2, "blocked edge answer is cached");
    raw.blocked = true; cached.clear();
    check(!cached.edgesClear([0.0, 0.5], 1)[0], "clearing the context discards old answers");
    var fresh = new motionkit.planner.CachedPlannerSpace(raw);
    check(!fresh.edgesClear([0.0, 0.5], 1)[0], "another scene instance cannot inherit cached clearance");
    cached.invalid([-1.0]); cached.invalid([-0.5]); cached.invalid([0.0]);
    check(cached.invalid([1.5]) != null, "bounded cache eviction preserves query semantics");
  }

  static function testSavedRoutes():Void {
    var scene = maze(), from = [-0.8, 0.6], to = [0.8, 0.5];
    var path = [from, [-0.1, 0.0], [0.1, 0.0], to];
    check(motionkit.planner.SavedPathValidation.failure(scene.space, path, from, to) == null, "saved complete route through the gap is recertified");
    check(motionkit.planner.SavedPathValidation.failure(scene.space, [from, to], from, to) != null, "valid endpoints cannot hide an interior wall crossing");
    check(motionkit.planner.SavedPathValidation.failure(scene.space, path, from, [0.8, 0.4]) != null, "changed route endpoint is rejected");
    check(motionkit.planner.SavedPathValidation.failure(scene.space, [from, [0.0, 0.6], to], from, to) != null, "colliding saved waypoint is rejected");
    check(motionkit.planner.SavedPathValidation.failure(scene.space, [from, [2.0, 0.0], to], from, to) != null, "saved overtravel is rejected");
    check(motionkit.planner.SavedPathValidation.failure(scene.space, [from, [Math.NaN, 0.0], to], from, to) != null, "nonfinite route is rejected");
    check(motionkit.planner.SavedPathValidation.failure(scene.space, null, from, to) != null, "missing route requires planning");
    scene.world.dispose();
  }

  static function testTimingCache():Void {
    var rates = [for (_ in 0...2) {velocity:0.7, acceleration:1.5, jerk:5.0}];
    var timing = new motionkit.robot.JointPathTiming(rates);
    var path = [[0.0,0.0], [0.2,0.3], [0.2,0.3], [-0.3,0.8], [0.0,0.0]];
    var original = timing.trajectory(path), expected = original.durationSeconds(); original.dispose();
    check(timing.duration(path) == expected && timing.duration(path) == expected, "cached duration exactly matches the original nanosecond trajectory");
    check(timing.durationCacheHits > 0, "repeated edge timings reuse native results");
    var lower = timing.lowerBound(path[0], path[1]);
    check(timing.lowerBound(path[0], path[1]) == lower && timing.lowerBoundCacheHits > 0, "repeated lower bounds are cached");
    var slower = new motionkit.robot.JointPathTiming([for (_ in 0...2) {velocity:0.1, acceleration:0.2, jerk:0.5}]);
    check(slower.duration(path) > expected, "timing caches do not cross rate configurations");
  }

  static function testScratchRoots():Void {
    var scene = maze(), q = [-0.8, 0.0];
    check(scene.space.invalid(q) == null, "scratch FK starts at the reference root");
    scene.space.bodies.reference.setRootPose(0, Transform.translation(0.8,0.6,0));
    check(scene.space.invalid(q) != null, "scratch FK observes an updated reference root");
    scene.space.bodies.reference.setRootPose(0, Transform.identity());
    check(scene.space.invalid(q) == null, "scratch FK clears after restoring the root");
    var expected = scene.space.bodies.poses(scene.space.bodies.state(q)), out:Array<Float> = [42.0];
    scene.space.bodies.posesInto(scene.space.bodies.state(q), out);
    check(expected.join(",") == out.join(","), "caller-owned FK output matches allocating FK");
    scene.world.dispose();
  }

  static function testLargeEdgeBatch():Void {
    var scene = maze(), edges:Array<Float> = [];
    // More edges than the bounded native buffers, interleaving wall crossings,
    // same-side travel and travel through the gap. Clear chunks must not end
    // the query early, and one blocked edge must not discard another edge.
    for (i in 0...257) {
      var a = i % 3 == 2 ? [-0.8, 0.0] : [-0.8, 0.6];
      var b = i % 3 == 0 ? [-0.8, -0.6] : i % 3 == 1 ? [0.8, 0.6] : [0.8, 0.0];
      edges = edges.concat(a).concat(b);
    }
    var checked = scene.space.edgesClear(edges, 257);
    check(checked.length == 257, "all large-batch edges have a result");
    for (i in 0...257) check(checked[i] == (i % 3 != 1), 'large-batch edge $i agrees with the wall and gap');
    check(scene.space.boxesClear([0.0, 0.6], [0.01, 0.01], 1)[0] == false, "reused box buffer detects the wall");
    check(scene.space.edgesClear([-0.8, 0.0, 0.8, 0.0], 1)[0], "small edge batch clears after a large blocked batch");
    check(scene.space.boxesClear([-0.8, 0.0], [0.01, 0.01], 1)[0], "reused box buffer clears after the blocked query");
    check(!scene.space.edgesClear([-0.8, 0.6, 0.8, 0.6], 1)[0], "reused edge buffer still rejects a later crossing");
    scene.world.dispose();
  }

  static function testEndpointRounding():Void {
    var scene = maze();
    var planner = new RrtConnect(scene.space);
    var start = [-1.0 - 2e-16, 0.0];
    var goal = [-0.8, 1.0 + 2e-16];
    var plan = planner.plan(start, goal);
    check(plan.found(), "One-ULP endpoint residue is snapped to the physical stop");
    check(start[0] < -1.0 && goal[1] > 1.0, "Endpoint snapping preserves caller coordinates");
    check(!planner.plan([-1.000001, 0.0], [-0.8, 0.0]).found(), "Real start overtravel remains invalid");
    check(!planner.plan([-0.8, 0.0], [-0.8, 1.000001]).found(), "Real goal overtravel remains invalid");
    scene.world.dispose();
  }

  /** A 10 cm sphere carried by x and y slides, and a wall across x = 0 with a 16 cm gap at y = 0. */
  static function maze():{space:CollisionPlannerSpace, world:NativeCollisionWorld} {
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base"), slideX = builder.addBody("x"), slideY = builder.addBody("y");
    builder.addJoint("x", JointKind.Prismatic, base, slideX, Transform.identity(), Transform.identity(), new Vector3(1, 0, 0), -1, 1);
    builder.addJoint("y", JointKind.Prismatic, slideX, slideY, Transform.identity(), Transform.identity(), new Vector3(0, 1, 0), -1, 1);
    var model = builder.build();
    var description = new CollisionDescription();
    var bodies = ModelBodies.describe(description, model, "point", 0, new KinematicState(model, [-0.8, 0.5]), "start", true);
    description.addObject("ball", bodies.bodies[2], CollisionPose.identity(), CollisionGeometry.Sphere(0.05));
    bodies.declare();
    description.addFixed("wall-low", CollisionPose.translation(0, -0.58, 0), CollisionGeometry.Box(0.02, 0.5, 0.2), 1);
    description.addFixed("wall-high", CollisionPose.translation(0, 0.58, 0), CollisionGeometry.Box(0.02, 0.5, 0.2), 1);
    var world = new NativeCollisionWorld();
    var build = description.build(world);
    // ds 5 mm, with a 2 mm planning slack.
    return {space: new CollisionPlannerSpace(description, build, bodies, new CollisionMargins(2, 0.007)), world: world};
  }

  static function onSegment(p:Array<Float>, a:Array<Float>, b:Array<Float>):Float {
    var ab = 0.0, ap = 0.0;
    for (k in 0...p.length) {
      ab += (b[k] - a[k]) * (b[k] - a[k]);
      ap += (p[k] - a[k]) * (b[k] - a[k]);
    }
    var t = ab == 0 ? 0.0 : Math.min(1, Math.max(0, ap / ab));
    var d = 0.0;
    for (k in 0...p.length) d += Math.pow(p[k] - (a[k] + t * (b[k] - a[k])), 2);
    return Math.sqrt(d);
  }

  /**
   * Through the wall's gap without stopping: the corners before and after the gap are passed
   * moving where the curve can be covered by clear boxes, and every point of the timed motion
   * stays clear. Stopping at the corners is the certified fallback, so fly-by is never slower.
   */
  static function testFlyByThroughGap():Void {
    var scene = maze();
    var path = [[-0.8, 0.6], [-0.1, 0.0], [0.1, 0.0], [0.8, 0.5]];
    var edges:Array<Float> = [];
    for (k in 1...path.length) edges = edges.concat(path[k - 1]).concat(path[k]);
    for (clear in scene.space.edgesClear(edges, path.length - 1)) check(clear, "the gap path's straight edges are clear");
    var rates = [for (_ in 0...2) {velocity: 0.5, acceleration: 2.0, jerk: 20.0}];
    var timing = new motionkit.robot.JointPathTiming(rates);
    var stopped = timing.trajectory(path), blended = timing.blended(path, scene.space);
    check(blended.durationSeconds() < stopped.durationSeconds() - 0.1,
      'fly-by through the gap is faster than stopping at its corners (${blended.durationSeconds()} vs ${stopped.durationSeconds()} s)');
    var worst = "";
    for (i in 0...2001) {
      var q = blended.evaluate(blended.durationSeconds() * i / 2000).positions.copy();
      var found = scene.space.invalid(q);
      if (found != null && worst == "") worst = '$found at $q';
    }
    check(worst == "", 'the fly-by motion stays clear of the wall: $worst');
    // The runtime admits a plan only if its segments join within 1e-9 in position.
    var pieces = blended.segments(), jump = 0.0;
    for (k in 1...pieces.length) {
      var before = pieces[k - 1], seconds = Int64.toInt(before.durationNs) * 1e-9;
      for (joint in 0...2) {
        var reached = 0.0, power = 1.0;
        for (c in before.coefficients[joint]) { reached += c * power; power *= seconds; }
        jump = Math.max(jump, Math.abs(reached - pieces[k].coefficients[joint][0]));
      }
    }
    check(jump <= 1e-9, 'fly-by segments join in position (worst jump $jump)');
    var end = blended.evaluate(blended.durationSeconds());
    check(Math.abs(end.positions[0] - 0.8) < 1e-9 && Math.abs(end.positions[1] - 0.5) < 1e-9 &&
      Math.abs(end.velocities[0]) < 1e-9 && Math.abs(end.velocities[1]) < 1e-9, "the fly-by motion stops at its goal");
    Sys.println('Fly-by gap: ${blended.durationSeconds()} s, stopping at each corner ${stopped.durationSeconds()} s');
    stopped.dispose(); blended.dispose();
    scene.world.dispose();
  }

  static function testNarrowPassage():Void {
    var scene = maze();
    var start = [-0.8, 0.6], goal = [0.8, 0.5];
    check(scene.space.edgesClear(start.concat(goal), 1)[0] == false, "the straight way is blocked by the wall");
    var planner = new RrtConnect(scene.space, 11);
    planner.step = 0.2;
    var plan = planner.plan(start, goal);
    check(plan.found(), 'a path through the gap is found (${plan.failure})');
    var path:Array<Array<Float>> = cast plan.waypoints;
    check(path[0].join(",") == start.join(",") && path[path.length - 1].join(",") == goal.join(","), "from start to goal");
    // Every edge is clear (rechecked) and the path goes through the gap.
    var edges:Array<Float> = [];
    for (i in 1...path.length) edges = edges.concat(path[i - 1]).concat(path[i]);
    check(scene.space.edgesClear(edges, path.length - 1).indexOf(false) < 0, "its edges are clear");
    var shortcuts:Array<Float> = [], shortcutCount = 0;
    for (i in 0...path.length - 2) for (j in i + 2...path.length) {
      shortcuts = shortcuts.concat(path[i]).concat(path[j]); shortcutCount++;
    }
    check(shortcutCount == 0 || scene.space.edgesClear(shortcuts, shortcutCount).indexOf(true) < 0,
      "the certified path has no avoidable intermediate stops");
    var crossing = false;
    for (i in 1...path.length) if ((path[i - 1][0] < 0) != (path[i][0] < 0)) {
      var t = -path[i - 1][0] / (path[i][0] - path[i - 1][0]);
      var y = path[i - 1][1] + t * (path[i][1] - path[i - 1][1]);
      crossing = crossing || Math.abs(y) < 0.08;
    }
    check(crossing, "it crosses the wall in the gap");
    Sys.println('CL7 narrow passage: ${path.length} waypoints, length ${plan.length()}, ${plan.iterations} iterations, ${plan.nodes} nodes, ${plan.edgeChecks} edge checks, ${Math.round(100 * plan.checkSeconds / Math.max(plan.totalSeconds, 1e-9))}% of ${plan.totalSeconds} s in checks');
    scene.world.dispose();
  }

  static function testEndpointsInCollision():Void {
    var scene = maze();
    var planner = new RrtConnect(scene.space, 3);
    var inWall = [0.0, 0.6];
    var a = planner.plan(inWall, [0.8, 0.5]);
    check(!a.found() && a.failure.indexOf("start in collision") >= 0 && a.failure.indexOf("ball") >= 0
      && a.failure.indexOf("wall-high") >= 0, 'a start in the wall is named, not planned (${a.failure})');
    var b = planner.plan([-0.8, 0.6], inWall);
    check(!b.found() && b.failure.indexOf("goal in collision") >= 0, 'so is a goal (${b.failure})');
    scene.world.dispose();
  }

  static function testDeterministicFromSeed():Void {
    var scene = maze();
    function path(seed:Int):String {
      var planner = new RrtConnect(scene.space, seed);
      planner.step = 0.2;
      var plan = planner.plan([-0.8, 0.6], [0.8, 0.5]);
      return plan.found() ? [for (q in (cast plan.waypoints : Array<Array<Float>>)) q.join(",")].join(";") : "none";
    }
    var first = path(5), again = path(5), other = path(6);
    check(first != "none" && first == again, "the same seed plans the same path");
    check(other != first, "another seed plans another");
    scene.world.dispose();
  }

  /**
   * The OMPL comparison (CL-D7), our side: each scene written out for the
   * out-of-tree OMPL benchmark (the same model, world, margins and motion
   * bound, so its checker is ours), and our planner run on it from 20
   * seeds: success, time, path length and the share of time in checks.
   */
  static function benchmark(dir:String):Void {
    var maze = maze();
    var scenes = [{name: "maze", space: maze.space, start: [-0.8, 0.6], goal: [0.8, 0.5], step: 0.2, lower: null, upper: null}];
    var post = postScene();
    scenes.push({name: "weld-arm-post", space: post.space, start: START, goal: post.goal, step: 0.3, lower: post.lower, upper: post.upper});
    for (scene in scenes) {
      var lower:Array<Float> = scene.lower == null ? scene.space.lower() : cast scene.lower;
      var upper:Array<Float> = scene.upper == null ? scene.space.upper() : cast scene.upper;
      writeScene('$dir/${scene.name}.scene', scene.space, scene.start, scene.goal, scene.step, lower, upper);
      var rows:Array<String> = [];
      for (seed in 1...21) {
        var planner = new RrtConnect(new BoundedSpace(scene.space, lower, upper), seed);
        planner.step = scene.step;
        var plan = planner.plan(scene.start, scene.goal);
        rows.push('$seed ${plan.found() ? 1 : 0} ${plan.totalSeconds} ${plan.found() ? plan.length() : -1} ${plan.checkSeconds} ${plan.edgeChecks} ${plan.nodes}');
      }
      sys.io.File.saveContent('$dir/${scene.name}.ours', "seed found seconds length checkSeconds edgeChecks nodes\n" + rows.join("\n") + "\n");
      Sys.println('CL7 benchmark scene ${scene.name} written');
    }
    maze.world.dispose();
    post.world.dispose();
  }

  /** The weld arm and a post its torch would sweep through, for the benchmark. */
  static function postScene():{space:CollisionPlannerSpace, world:NativeCollisionWorld, goal:Array<Float>, lower:Array<Float>, upper:Array<Float>} {
    var arm:Manipulator = WeldPlanningTests.arm().arm;
    var model = arm.model;
    var goal = START.copy();
    goal[0] += 1.2;
    var halfway = START.copy();
    halfway[0] += 0.6;
    var tip = arm.linkPoses(halfway, ["wrist_3_link"])[0].transformPoint(new robotkit.spatial.Vec3(0, D6, -0.09));
    var description = new CollisionDescription();
    var bodies = ModelBodies.describe(description, model, "ur", 0, arm.stateOf(START), "start", true);
    description.addObject("torch", bodies.bodies[model.bodyIndex("wrist_3_link")], CollisionPose.identity(),
      CollisionGeometry.Convex(WeldPlanningTests.box(-0.012, 0.012, D6 - 0.012, D6 + 0.012, -0.14, -0.03)));
    bodies.declare();
    description.addFixed("table", CollisionPose.identity(), CollisionGeometry.Convex(WeldPlanningTests.box(-0.8, 0.8, -0.8, 0.8, -0.1, 0.0)), 1);
    description.addFixed("post", CollisionPose.identity(), CollisionGeometry.Convex(WeldPlanningTests.box(tip.x - 0.03, tip.x + 0.03,
      tip.y - 0.03, tip.y + 0.03, 0.0, tip.z + 0.05)), 1);
    var world = new NativeCollisionWorld();
    var build = description.build(world);
    return {space: new CollisionPlannerSpace(description, build, bodies, new CollisionMargins(2, 0.007)), world: world, goal: goal,
      lower: [for (j in 0...6) Math.min(START[j], goal[j]) - 0.8], upper: [for (j in 0...6) Math.max(START[j], goal[j]) + 0.8]};
  }

  /** A scene as whitespace-separated numbers and words (read by the benchmark). */
  static function writeScene(path:String, space:CollisionPlannerSpace, start:Array<Float>, goal:Array<Float>, step:Float,
      lower:Array<Float>, upper:Array<Float>):Void {
    var out = new StringBuf();
    function line(label:String, values:Array<String>):Void out.add('$label ${values.length} ${values.join(" ")}\n');
    function reals(values:Array<Float>):Array<String> return [for (v in values) Std.string(v)];
    var model = space.bodies.model, description = space.description;
    line("ints", [for (v in kinematicskit.native.NativeKinematics.packInts(model)) Std.string(v)]);
    line("reals", reals(kinematicskit.native.NativeKinematics.packReals(model)));
    var reference = space.bodies.reference;
    var roots:Array<Float> = [];
    for (body in 0...model.bodyCount()) {
      var pose = model.bodyParentJoint[body] < 0 ? reference.rootPose(body) : Transform.identity();
      roots = roots.concat([pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw]);
    }
    line("roots", reals(roots));
    var all = space.bodies.all();
    out.add('bodies ${description.bodies.length}\n');
    for (d in 0...description.bodies.length) {
      var b = description.bodies[d], p = b.pose;
      var index = all.indexOf(d);
      var follows = index < 0 ? -1 : index < space.bodies.bodies.length ? index : space.bodies.followed[index - space.bodies.bodies.length];
      out.add('${b.group} ${b.fixed ? 1 : 0} ${p.x} ${p.y} ${p.z} ${p.qx} ${p.qy} ${p.qz} ${p.qw} $follows ${index < 0 ? 0 : description.bodyRadius(d)}\n');
    }
    out.add('objects ${description.objects.length}\n');
    for (o in description.objects) {
      var f = o.offset;
      var shape = switch o.geometry {
        case Box(x, y, z): 'box 3 $x $y $z';
        case Sphere(r): 'sphere 1 $r';
        case Capsule(r, h): 'capsule 2 $r $h';
        case Cylinder(r, h): 'cylinder 2 $r $h';
        case Convex(points): 'convex ${points.length} ${points.join(" ")}';
        case _: throw "benchmark scenes hold primitives and convex sets only";
      }
      out.add('${o.body} $shape ${f.x} ${f.y} ${f.z} ${f.qx} ${f.qy} ${f.qz} ${f.qw} ${o.inflation}\n');
    }
    out.add('rules ${description.bodyRules.length}\n');
    for (r in description.bodyRules) out.add('${r.a} ${r.b} ${(r.rule : Int)} ${(r.reason : Int)}\n');
    out.add('articulations ${description.articulations.length}\n');
    for (a in description.articulations) out.add('${a.bodies.length} ${a.bodies.join(" ")}\n');
    line("margins", reals(space.margins.flat()));
    line("distance", reals(space.reach.distance));
    line("angular", reals(space.reach.angular));
    line("lower", reals(lower));
    line("upper", reals(upper));
    line("start", reals(start));
    line("goal", reals(goal));
    out.add('step $step\ndepth ${space.depthLimit}\n');
    sys.io.File.saveContent(path, out.toString());
  }

  /** Main's weld arm turns its base 1.2 rad past a post its torch would sweep through: planned, timed and validated. */
  static function testPlannedMoveThroughValidation():Void {
    var arm:Manipulator = WeldPlanningTests.arm().arm;
    var model = arm.model;
    var goal = START.copy();
    goal[0] += 1.2;
    var torch:collisionkit.CollisionDescription = CollisionClearance.convex("torch", "wrist_3_link", WeldPlanningTests.box(-0.012, 0.012, D6 - 0.012, D6 + 0.012, -0.14, -0.03), true);
    var table:collisionkit.CollisionDescription = CollisionClearance.convex("table", "base_link", WeldPlanningTests.box(-0.8, 0.8, -0.8, 0.8, -0.1, 0.0), false);
    // The post stands where the torch passes halfway round.
    var halfway = START.copy();
    halfway[0] += 0.6;
    var tip = arm.linkPoses(halfway, ["wrist_3_link"])[0].transformPoint(new robotkit.spatial.Vec3(0, D6, -0.09));
    var post:collisionkit.CollisionDescription = CollisionClearance.convex("post", "base_link", WeldPlanningTests.box(tip.x - 0.03, tip.x + 0.03, tip.y - 0.03, tip.y + 0.03, 0.0, tip.z + 0.05), false);
    // The planner's world: the arm's bodies from its model, the same hulls.
    var description = new CollisionDescription(0.005);
    var bodies = ModelBodies.describe(description, model, "ur", 0, arm.stateOf(START), "start", true);
    description.addObject("torch", bodies.bodies[model.bodyIndex("wrist_3_link")], CollisionPose.identity(), CollisionGeometry.Convex(CollisionClearance.vertices(torch.objects[0])));
    bodies.declare();
    description.addFixed("table", CollisionPose.identity(), CollisionGeometry.Convex(CollisionClearance.vertices(table.objects[0])), 1);
    description.addFixed("post", CollisionPose.identity(), CollisionGeometry.Convex(CollisionClearance.vertices(post.objects[0])), 1);
    var world = new NativeCollisionWorld();
    var build = description.build(world);
    var space = new CollisionPlannerSpace(description, build, bodies, new CollisionMargins(2, 0.007));
    // Bound the search to a neighbourhood of the move.
    var planner = new RrtConnect(new BoundedSpace(space, [for (j in 0...6) Math.min(START[j], goal[j]) - 0.8],
      [for (j in 0...6) Math.max(START[j], goal[j]) + 0.8]), 2);
    // Validation: the adapter on the same hulls, with CollisionClearance's margins.
    var clearance = new CollisionClearance(arm, description, START, () -> new NativeCollisionWorld(), bodies);
    check(clearance.sweep(START, goal) != null, "the direct turn sweeps the torch through the post");
    var compiler = processkit.WeldingPlanRunner.planning(arm, 2.0).compiler.withJointPathPlanner(
      new StructuredJointPathPlanner(arm, null, null, clearance, 8, false));
    compiler.motionPlanner = planner;
    var compiled = compiler.compile(new MotionProgram([MotionOp.MoveJ(MoveTarget.JointTarget(goal),
      new MotionOptions(0, 0, 0, true), Blend.ExactStop)]), START, Int64.ofInt(1));
    var plan = compiled.blocks[0].plans[0];
    check(plan.report.collision.status == TrajectoryCoreConstants.MK_CHECK_PASSED, "the planned move passes validation");
    // The timed path runs along the planned edges: replan with the same seed and compare.
    var replay = planner.plan(START, goal);
    var path:Array<Array<Float>> = cast replay.waypoints;
    var motion = motionkit.trajectory.Trajectory.fromSegments(plan.segments());
    var worst = 0.0;
    for (i in 0...200) {
      var q = motion.evaluate(motion.durationSeconds() * i / 199).positions;
      var nearest = Math.POSITIVE_INFINITY;
      for (k in 1...path.length) nearest = Math.min(nearest, onSegment(q, path[k - 1], path[k]));
      worst = Math.max(worst, nearest);
    }
    motion.dispose();
    check(path.length > 2 && worst < 1e-6, 'the timed path follows the ${path.length - 1} checked edges (worst ${worst} rad)');
    Sys.println('CL7 planned move: ${path.length - 1} edges, ${plan.report.collisionPair.nameA}/${plan.report.collisionPair.nameB} ${plan.report.collision.value} m, method ${plan.report.collision.method}');
    // The same route as planned joint moves joined by fly-bys: one motion through the detour, validated, faster.
    compiler.flyByCertificate = space;
    var ops = [for (k in 1...path.length) MotionOp.MoveJ(MoveTarget.JointTarget(path[k]), new MotionOptions(0, 0, 0, true),
      k + 1 < path.length ? Blend.FlyBy : Blend.ExactStop)];
    var flown = compiler.compile(new MotionProgram(ops), START, Int64.ofInt(2));
    var flownPlans = [for (block in flown.blocks) for (each in block.plans) each];
    check(flownPlans.length == 1 && flownPlans[0].report.collision.status == TrajectoryCoreConstants.MK_CHECK_PASSED,
      "the fly-by route compiles to one validated motion");
    var flownMotion = motionkit.trajectory.Trajectory.fromSegments(flownPlans[0].segments());
    var stoppedMotion = motionkit.trajectory.Trajectory.fromSegments(plan.segments());
    check(flownMotion.durationSeconds() <= stoppedMotion.durationSeconds(),
      'flying by the detour is no slower (${flownMotion.durationSeconds()} vs ${stoppedMotion.durationSeconds()} s)');
    Sys.println('CL7 fly-by: ${flownMotion.durationSeconds()} s against ${stoppedMotion.durationSeconds()} s stopping at each waypoint');
    flownMotion.dispose(); stoppedMotion.dispose();
    flown.dispose();
    compiled.dispose();
    world.dispose();
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}

/** A planner space narrowed to joint bounds of its own. */
class BoundedSpace implements motionkit.planner.PlannerSpace {
  final inner:motionkit.planner.PlannerSpace;
  final low:Array<Float>;
  final high:Array<Float>;

  public function new(inner:motionkit.planner.PlannerSpace, low:Array<Float>, high:Array<Float>) {
    this.inner = inner;
    var outer = [inner.lower(), inner.upper()];
    this.low = [for (k in 0...low.length) Math.max(low[k], outer[0][k])];
    this.high = [for (k in 0...high.length) Math.min(high[k], outer[1][k])];
  }

  public function dimension():Int return inner.dimension();
  public function lower():Array<Float> return low.copy();
  public function upper():Array<Float> return high.copy();
  public function invalid(q:Array<Float>):Null<String> return inner.invalid(q);
  public function edgesClear(edges:Array<Float>, count:Int):Array<Bool> return inner.edgesClear(edges, count);
}
