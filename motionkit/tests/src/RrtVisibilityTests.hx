import motionkit.planner.PlannerSpace;
import motionkit.planner.RrtConnect;
import motionkit.robot.JointPathTiming;

/** Exact planar wall and gap, independent of the planner's edge subdivision. */
private class WallGapSpace implements PlannerSpace {
  public function new() {}
  public function dimension():Int return 2;
  public function lower():Array<Float> return [-1.0, -1.0];
  public function upper():Array<Float> return [1.0, 1.0];
  public function invalid(q:Array<Float>):Null<String>
    return Math.abs(q[0]) <= 0.05 && Math.abs(q[1]) >= 0.15 ? "wall" : null;
  public function edgesClear(edges:Array<Float>, count:Int):Array<Bool> {
    var result:Array<Bool> = [];
    for (edge in 0...count) {
      var offset = edge * 4;
      var x = edges[offset], y = edges[offset + 1];
      var dx = edges[offset + 2] - x, dy = edges[offset + 3] - y;
      var clear = true;
      if (dx == 0) {
        if (Math.abs(x) <= 0.05)
          clear = Math.abs(y) < 0.15 && Math.abs(y + dy) < 0.15;
      } else {
        var a = (-0.05 - x) / dx, b = (0.05 - x) / dx;
        var enter = Math.max(0.0, Math.min(a, b)), exit = Math.min(1.0, Math.max(a, b));
        if (enter <= exit)
          clear = Math.abs(y + dy * enter) < 0.15 && Math.abs(y + dy * exit) < 0.15;
      }
      result.push(clear);
    }
    return result;
  }
}

/** A thin one-dimensional barrier immediately beyond the goal tree's root. */
private class BlockedConnectionSpace implements PlannerSpace {
  public var checked(default, null):Int = 0;
  public function new() {}
  public function dimension():Int return 1;
  public function lower():Array<Float> return [-100.0];
  public function upper():Array<Float> return [100.0];
  public function invalid(q:Array<Float>):Null<String>
    return q[0] >= 89.75 && q[0] <= 89.85 ? "barrier" : null;
  public function edgesClear(edges:Array<Float>, count:Int):Array<Bool> {
    checked += count;
    return [for (edge in 0...count)
      Math.max(edges[2 * edge], edges[2 * edge + 1]) < 89.75 ||
      Math.min(edges[2 * edge], edges[2 * edge + 1]) > 89.85];
  }
}

class RrtVisibilityTests extends MotionKitTestSupport {
  public function new() { super(); }
  public function testTimedRefinement():Void {
    var space = new WallGapSpace();
    var timing = new JointPathTiming([
      {velocity: 2.0, acceleration: 1.8, jerk: 6.0},
      {velocity: 2.0, acceleration: 1.8, jerk: 6.0}]);
    var path = [[-0.8, 0.6], [-0.1, 0.05], [0.1, 0.05], [0.8, 0.5]];
    var original = timing.duration(path), refined = timing.refine(space, path);
    check(timing.duration(refined) < original * 0.99, "certified bend refinement reduces executed motion time");
    near(refined[0][0], -0.8, "refinement preserves the start", 0);
    near(refined[0][1], 0.6, "refinement preserves the start heading coordinate", 0);
    near(refined[refined.length - 1][0], 0.8, "refinement preserves the goal", 0);
    near(refined[refined.length - 1][1], 0.5, "refinement preserves the goal heading coordinate", 0);
    near(path[1][0], -0.1, "refinement does not mutate its caller's certificate", 0);
    near(path[1][1], 0.05, "refinement does not mutate its caller's configuration", 0);
    for (index in 1...refined.length)
      check(space.edgesClear(refined[index - 1].concat(refined[index]), 1)[0],
        "the exact wall intersection remains clear on every refined edge");
    near(timing.duration(timing.refine(space, path, 0)), original,
      "zero passes retain the original route time", 1e-10);
    var refused = false;
    try { var invalid = timing.refine(space, [[-0.8, 0.6], [0.0, 0.5], [0.8, 0.5]]); }
    catch (_:Dynamic) refused = true;
    check(refused, "refinement rejects an invalid input waypoint");
    refused = false;
    try { var invalidEdge = timing.refine(space, [[-0.8, 0.6], [0.8, 0.5]]); }
    catch (_:Dynamic) refused = true;
    check(refused, "refinement rejects a wall-crossing edge with valid endpoints");
    near(timing.duration([[0.2, 0.3], [0.2, 0.3]]), 0.0, "a stationary route has zero duration", 0);
    refused = false;
    try { var wrongDimension = timing.duration([[0.2], [0.2]]); }
    catch (_:Dynamic) refused = true;
    check(refused, "stationary timing retains joint-dimension validation");
  }
  public function testBlockedConnection():Void {
    var space = new BlockedConnectionSpace(), planner = new RrtConnect(space, 1);
    planner.step = 0.3; planner.maxIterations = 1;
    var plan = planner.plan([-90.0], [90.0]);
    check(!plan.found(), "a thin barrier cannot acquire an unchecked connection");
    check(space.checked < 60,
      'a blocked connection must stop probing its unusable suffix, checked ${space.checked} edges');
  }
  public function testPoseSampling():Void {
    var angle = 0.23;
    var start = new motionkit.kinematics.Pose3(0, 0, 0, 0, 0, 0, 1);
    var end = new motionkit.kinematics.Pose3(0.003, 0, 0, Math.sin(angle / 2), 0, 0, Math.cos(angle / 2));
    var line = new motionkit.path.PoseLine(new motionkit.path.PoseWaypoint(start, 1e-4, 1e-4),
      new motionkit.path.PoseWaypoint(end, 1e-4, 1e-4), motionkit.path.OrientationPolicy.Interpolated, 0.1, 0.08);
    var pieces = motionkit.path.PoseSampling.pieces(line, 0.0075, 0.1);
    check(pieces >= 3, "short translation retains the tool's rotation in its selection grid");
    for (index in 0...pieces) {
      var from = line.waypointAt(line.length() * index / pieces).pose;
      var to = line.waypointAt(index + 1 == pieces ? line.length() : line.length() * (index + 1) / pieces).pose;
      check(motionkit.path.PoseMath.angle(from, to) <= 0.1 + 1e-8, "selection samples respect angular spacing");
      check(motionkit.path.PoseMath.distance(from, to) <= 0.0075 + 1e-8, "selection samples retain Cartesian spacing");
    }
    var fixed = new motionkit.path.PoseLine(line.start, line.end,
      motionkit.path.OrientationPolicy.Fixed, 0.1, 0.08);
    check(motionkit.path.PoseSampling.pieces(fixed, 0.0075, 0.1) == 1,
      "an unused endpoint rotation does not change a fixed-orientation grid");
  }
  public function testVisibility():Void {
    var space = new WallGapSpace();
    var start = [-0.8, 0.6], goal = [0.8, 0.5];
    check(!space.edgesClear(start.concat(goal), 1)[0], "direct motion hits the wall");
    for (seed in [3, 11, 17]) {
      var planner = new RrtConnect(space, seed);
      planner.step = 0.2;
      var plan = planner.plan(start, goal), path = plan.waypoints;
      if (path == null) throw 'Visibility fixture has no path: ${plan.failure}';
      check(path[0].join(",") == start.join(",") && path[path.length - 1].join(",") == goal.join(","),
        "simplification preserves commanded endpoints");
      for (i in 1...path.length)
        check(space.edgesClear(path[i - 1].concat(path[i]), 1)[0], "retained edge clears the wall");
      for (i in 0...path.length - 2) for (j in i + 2...path.length)
        check(!space.edgesClear(path[i].concat(path[j]), 1)[0], "no unnecessary intermediate stop remains");
      var rawPlanner = new RrtConnect(space, seed);
      rawPlanner.step = 0.2; rawPlanner.visibilityCheckLimit = 0;
      var raw = rawPlanner.plan(start, goal);
      var boundedPlanner = new RrtConnect(space, seed);
      boundedPlanner.step = 0.2; boundedPlanner.visibilityCheckLimit = 1;
      var bounded = boundedPlanner.plan(start, goal), boundedPath = bounded.waypoints;
      if (boundedPath == null) throw "Bounded simplification lost its valid path";
      check(bounded.edgeChecks - raw.edgeChecks <= 1, "visibility work respects its collision-check budget");
      for (i in 1...boundedPath.length)
        check(space.edgesClear(boundedPath[i - 1].concat(boundedPath[i]), 1)[0],
          "budget exhaustion preserves certified edges");
    }
  }

  public function testJointPathTiming():Void {
    var timing = new JointPathTiming([{velocity: 0.7, acceleration: 2.0, jerk: 20.0},
      {velocity: 1.2, acceleration: 3.0, jerk: 40.0}]);
    var motion = timing.trajectory([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]);
    var duration = motion.durationSeconds();
	var bound = timing.lowerBound([0.0, 0.0], [1.0, 1.0]);
	check(bound > 1.0 / 0.7 && bound <= duration, "independent joint timing bounds a certified detour");
	near(bound, timing.lowerBound([1.0, 1.0], [0.0, 0.0]), "rest-to-rest bound is symmetric", 1e-8);
	near(timing.lowerBound([0.0, 0.0], [0.0, 0.0]), 0.0, "a stationary path has zero lower bound", 1e-8);
    check(duration > 1.0 / 0.7 + 1.0 / 1.2, "timing includes acceleration and stops, not just distance over speed");
    var corner = motion.evaluate(timing.duration([[0.0, 0.0], [1.0, 0.0]]));
    near(corner.positions[0], 1.0, "the first certified edge ends at the corner", 1e-8);
    near(corner.positions[1], 0.0, "the second edge starts at the same corner", 1e-8);
    for (velocity in corner.velocities) near(velocity, 0.0, "the corner is a rest-to-rest stop", 1e-8);
    for (sample in 0...101) {
      var state = motion.evaluate(duration * sample / 100);
      check(Math.abs(state.positions[1]) < 1e-8 || Math.abs(state.positions[0] - 1.0) < 1e-8,
        "the timed path follows its certified edges without cutting the corner");
      check(Math.abs(state.velocities[0]) <= 0.7 + 1e-8 && Math.abs(state.velocities[1]) <= 1.2 + 1e-8,
        "timing respects each joint's velocity ceiling");
    }
    var end = motion.evaluate(duration);
    for (position in end.positions) near(position, 1.0, "the timed path reaches its endpoint", 1e-8);
    for (velocity in end.velocities) near(velocity, 0.0, "the endpoint is stopped", 1e-8);
    motion.dispose();
    // Unequal signed displacements and unequal limits distinguish phase
    // synchronization from merely arriving at the same time.
    var diagonal = timing.trajectory([[0.25, -0.75], [1.25, -2.75]]);
    for (sample in 0...101) {
      var state = diagonal.evaluate(diagonal.durationSeconds() * sample / 100);
      near(2 * (state.positions[0] - 0.25) + state.positions[1] + 0.75, 0.0,
        "the timed diagonal remains on its certified straight edge", 1e-8);
      near(2 * state.velocities[0] + state.velocities[1], 0.0,
        "joints follow the same scaled phase rather than independent time profiles", 1e-8);
      check(Math.abs(state.velocities[0]) <= 0.7 + 1e-8 && Math.abs(state.velocities[1]) <= 1.2 + 1e-8 &&
        Math.abs(state.accelerations[0]) <= 2.0 + 1e-8 && Math.abs(state.accelerations[1]) <= 3.0 + 1e-8 &&
        Math.abs(state.jerks[0]) <= 20.0 + 1e-8 && Math.abs(state.jerks[1]) <= 40.0 + 1e-8,
        "phase timing respects each joint's velocity, acceleration and jerk limits");
    }
    diagonal.dispose();
  }
}
