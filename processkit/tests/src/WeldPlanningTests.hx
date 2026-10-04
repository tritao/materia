import motionkit.kinematics.IkTolerance;
import motionkit.robot.ManipulatorKinematics;
import processkit.WeldCorner;
import processkit.WeldCorner.WristLimits;
import processkit.WeldPathPlanner;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.Manipulator;
import robotkit.model.Frame;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.skill.WeldPlan;
import robotkit.skill.WeldPlan.WeldParameters;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/**
 * The corner turn is derived from the angle and the wrist's limits; and a weld is planned for an arm (a UR5-style 6R arm
 * with a torch body on its wrist) so that it is reachable and clear: the torch's roll is chosen to keep its neck off a wall
 * beside the seam, and a seam that cannot be welded without hitting something is refused with the part and the pose.
 */
class WeldPlanningTests {
  static var assertions = 0;
  static final WRIST:WristLimits = {angularSpeed: 3.0, angularAcceleration: 2.0};
  static final PARAMETERS:WeldParameters = {wireSpeed: 8.0, voltage: 24.0, travelSpeed: 0.0115, approach: 0.04, startDwell: 0.15, craterDwell: 0.15,
    burnback: 0.1};

  public static function run():Int {
    assertions = 0;
    testCornerTurn();
    testRollAvoidsTheWall();
    testEntryBranchReachesTheWholeWeld();
    testClosedRunChoosesAReachableCorner();
    testBranchJumpIsRefused();
    testCompiledMotionRejectsAnEntry();
    testEntryUsesTheCurrentConfiguration();
    testCollidingWeldIsRefused();
    Sys.println('ProcessKit weld planning tests passed ($assertions assertions)');
    return assertions;
  }

  static function testCornerTurn():Void {
    var travel = 0.0115;
    check(WeldCorner.turnLength(0.0, travel, WRIST) == 0.0, "a straight join has no turn");
    check(WeldCorner.turnLength(1e-9, travel, WRIST) == 0.0, "an orientation change of nothing has no turn");
    var shallow = WeldCorner.turnLength(0.3, travel, WRIST);
    var quarter = WeldCorner.turnLength(Math.PI / 2, travel, WRIST);
    var half = WeldCorner.turnLength(Math.PI, travel, WRIST);
    check(shallow >= WeldCorner.MIN_TURN && shallow < quarter, 'a shallower corner turns over less than a quarter turn ($shallow m, $quarter m)');
    check(quarter < half || half == WeldCorner.MAX_TURN, 'a sharper corner turns over more, up to the limit ($quarter m, $half m)');
    check(half <= WeldCorner.MAX_TURN && shallow >= WeldCorner.MIN_TURN, "turns stay within their bounds");
    // The time of a quarter turn at 0.5 of a wrist of 3 rad/s and 2 rad/s^2: the speed limit (1.5) is not reached, so 2 sqrt(a / 1).
    near(WeldCorner.turnTime(Math.PI / 2, WRIST), 2.0 * Math.sqrt(Math.PI / 2), 1e-9, "a quarter turn takes 2 sqrt(a / alpha')");
    near(quarter, travel * WeldCorner.turnTime(Math.PI / 2, WRIST) / 2.0, 1e-9, "the tip travels at its speed while the torch turns, half on each side");
    // A stronger wrist turns faster, so the stretch is shorter; a faster weld needs more of it.
    var strong = WeldCorner.turnLength(Math.PI / 2, travel, {angularSpeed: 6.0, angularAcceleration: 8.0});
    check(strong < quarter, "a stronger wrist needs less path to turn");
    check(WeldCorner.turnLength(Math.PI / 2, travel * 1.5, WRIST) > quarter, "a faster weld needs more path to turn");
    // A turn at speed w' over a long way reaches the speed limit: a - w'^2 / alpha' > 0.
    var long = WeldCorner.turnTime(2 * Math.PI, WRIST);
    near(long, 2 * Math.PI / 1.5 + 1.5 / 1.0, 1e-9, "a long turn reaches the speed limit");
    // The turn keeps the wire on the shortest arc between its two directions. Two sides of a post: wires 75 degrees apart, the
    // second rolled a quarter turn. Half way, the wire is the bisector of the two, steeper than the rotations' own interpolation.
    var first = Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), 0.0).multiply(Quat.fromAxisAngle(new Vec3(1.0, 0.0, 0.0), Math.PI * 0.75));
    var second = Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), Math.PI / 2).multiply(first);
    var w1 = first.rotate(new Vec3(0.0, 0.0, 1.0)), w2 = second.rotate(new Vec3(0.0, 0.0, 1.0));
    var wire = WeldCorner.orientationAt(first, second, 0.5).rotate(new Vec3(0.0, 0.0, 1.0));
    var bisector = w1.add(w2).normalized();
    near(wire.dot(bisector), 1.0, 1e-9, "half way through a turn the wire is the bisector of the two");
    // Round a corner whose edge is vertical (travel along +Y, then along -X), the wire keeps its elevation: the nozzle keeps its
    // distance from the edge. The great arc between the wires would steepen it half way.
    var yaw = Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), Math.PI / 2);
    var leaning = Quat.fromAxisAngle(new Vec3(1.0, 0.0, 0.0), Math.PI * 0.75);
    var away = yaw.multiply(leaning);
    var around = WeldCorner.orientationAt(leaning, away, 0.5, new Vec3(0.0, 1.0, 0.0), new Vec3(-1.0, 0.0, 0.0)).rotate(new Vec3(0.0, 0.0, 1.0));
    near(around.z, leaning.rotate(new Vec3(0.0, 0.0, 1.0)).z, 1e-9, "round a vertical corner the wire keeps its elevation");
    var shortcut = WeldCorner.orientationAt(leaning, away, 0.5).rotate(new Vec3(0.0, 0.0, 1.0));
    check(shortcut.z < around.z - 0.05, "where the shortest arc between the wires would have steepened it");
    near(WeldCorner.orientationAt(first, second, 0.0).angularDistance(first), 0.0, 1e-9, "a turn starts at its first orientation");
    near(WeldCorner.orientationAt(first, second, 1.0).angularDistance(second), 0.0, 1e-9, "and ends at its second");
    // The roll turns evenly: a quarter turn in all, an eighth at half way, while the wire stays put when the wires are equal.
    var rolled = WeldCorner.orientationAt(first, Quat.fromAxisAngle(w1, Math.PI / 2).multiply(first), 0.5);
    near(rolled.rotate(new Vec3(0.0, 0.0, 1.0)).dot(w1), 1.0, 1e-9, "a turn of the roll alone leaves the wire");
    near(first.angularDistance(rolled), Math.PI / 4, 1e-9, "and turns it evenly");
    var continued = WeldCorner.continuationRoll(first, second);
    var held = second.multiply(Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), continued));
    near(held.rotate(new Vec3(0.0, 0.0, 1.0)).dot(w2), 1.0, 1e-9, "continuation roll preserves the next seam's wire direction");
    near(first.angularDistance(held), Math.atan2(w1.cross(w2).norm(), w1.dot(w2)), 1e-9, "continuation uses only the wire's swing, with no added twist");
    // A short segment gives no more than its share to a corner.
    near(WeldCorner.given(0.02, 0.020), WeldCorner.SHARE * 0.020, 1e-12, "a short segment gives its share");
    near(WeldCorner.given(0.02, 0.2), 0.02, 1e-12, "a long segment gives the whole turn");
    check(WeldCorner.given(0.0, 0.2) == 0.0, "no turn takes nothing from a segment");
  }

  /** A UR5-style arm with joints, its flange the tool frame (+Z the wire out of the torch), and its links' ids. */
  static function arm():{arm:Manipulator, links:Array<String>} {
    var d1 = 0.089159, shoulderOffset = 0.13585, elbowOffset = -0.1197, a2 = 0.425, a3 = 0.39225, d4 = 0.10915, d5 = 0.09465, d6 = 0.0823;
    var model = new RobotModel("planning-arm");
    var names = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"];
    var links = [for (name in names) model.addLink(new Link(name))];
    var offsets = [new Vec3(0.0, 0.0, d1), new Vec3(0.0, shoulderOffset, 0.0), new Vec3(0.0, elbowOffset, a2), new Vec3(0.0, 0.0, a3),
      new Vec3(0.0, d4, 0.0), new Vec3(0.0, 0.0, d5)];
    var axes = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]];
    for (i in 0...6) {
      var joint = model.addJoint(new Joint('j$i', JointType.Revolute, links[i], links[i + 1]));
      joint.parentFramePosition = offsets[i].toArray();
      joint.axis = axes[i];
      joint.limits.lower = -2.0 * Math.PI;
      joint.limits.upper = 2.0 * Math.PI;
      joint.limits.velocity = 3.0;
    }
    var flange = model.addFrame(new Frame("flange", links[6]));
    flange.position = [0.0, d6, 0.0];
    return {arm: new Manipulator(model, links[0].id, flange.id), links: names};
  }

  static function box(x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float):Array<Float> {
    var result:Array<Float> = [];
    for (x in [x0, x1]) for (y in [y0, y1]) for (z in [z0, z1]) {
      result.push(x);
      result.push(y);
      result.push(z);
    }
    return result;
  }

  /**
   * The cell: the torch is a bar behind the wire tip (along -Z of the tool frame) with a neck sticking out to the tool's
   * +X; a table below the seam; and, when `wall` is given, a wall as a fixed body beside the seam's end.
   */
  static function cell(wallX:Null<Float>):{planner:WeldPathPlanner, clearance:ArmClearance, start:Array<Float>} {
    var made = arm();
    var d6 = 0.0823;
    var bodies = [
      {name: "torch", link: "wrist_3_link", vertices: box(-0.012, 0.012, d6 - 0.012, d6 + 0.012, -0.14, -0.03), tool: true},
      {name: "neck", link: "wrist_3_link", vertices: box(0.0, 0.10, d6 - 0.012, d6 + 0.012, -0.14, -0.10), tool: true},
      {name: "table", link: "base_link", vertices: box(-0.8, 0.8, -0.8, 0.8, -0.1, 0.0), tool: false}
    ];
    if (wallX != null) bodies.push({name: "wall", link: "base_link", vertices: box(wallX, wallX + 0.05, 0.05, 0.35, 0.0, 0.5), tool: false});
    var start = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0];
    var clearance = new ArmClearance(made.arm, bodies, start);
    var solver = new ManipulatorKinematics(made.arm, 1e-8);
    var planner = new WeldPathPlanner(solver, new IkTolerance(2e-4, 1e-3, 300, 0.03), [for (_ in 0...6) 3.0], WRIST, clearance);
    return {planner: planner, clearance: clearance, start: start};
  }

  /** A straight seam along +X at height 0.15, the wire pointing down. */
  static function seam():WeldPlan {
    var down = Quat.fromAxisAngle(new Vec3(1.0, 0.0, 0.0), Math.PI);
    return WeldPlan.straight(new Transform3(new Vec3(0.35, 0.2, 0.15), down), new Transform3(new Vec3(0.45, 0.2, 0.15), down), PARAMETERS);
  }

  static function testRollAvoidsTheWall():Void {
    var free = cell(null);
    var open = free.planner.plan(seam(), free.start);
    near(open.rolls[0], 0.0, 1e-9, "with nothing beside it the torch keeps the seam frame's own roll");
    check(open.entry.name == "along the wire", 'it comes in along the wire (${open.entry.name})');
    check(open.checked > 20, 'every pose along the way was checked (${open.checked})');
    // A wall 30 mm past the seam's end, on the side the neck points to at roll 0: the neck would hit it, the other side is clear.
    var walled = cell(0.48);
    var turned = walled.planner.plan(seam(), walled.start);
    check(Math.abs(turned.rolls[0]) > 0.5, 'beside a wall the torch is rolled so that its neck points away (${turned.rolls[0]} rad)');
    Sys.println('weld planning: open seam roll ${open.rolls[0]} via "${open.entry.name}", ${open.checked} poses; beside a wall roll ${turned.rolls[0]}, ${turned.checked} poses');
    check(turned.checked > open.checked / 2, 'and the rolls tried before it were checked too (${turned.checked} poses)');
  }

  static function testEntryBranchReachesTheWholeWeld():Void {
    var planner = new WeldPathPlanner(new EntryBranchFixture(), new IkTolerance(2e-4, 1e-3, 300, 0.03), [3.0], WRIST);
    var planned = planner.plan(seam(), [0.0]);
    check(planned.entry.joints[0] == 1.0, "entries that only reach the start are replaced by a later IK branch that reaches the whole seam");
    near(planned.rolls[0], 0.0, 1e-9, "choosing the other entry branch keeps the requested weld orientation");
  }

  static function testCollidingWeldIsRefused():Void {
    // A wall right at the seam's end: no roll takes the neck and the torch clear of it, nor any way in or out.
    var tight = cell(0.452);
    var message:Null<String> = null;
    try tight.planner.plan(seam(), tight.start) catch (error:Dynamic) message = Std.string(error);
    check(message != null, "a weld that cannot be done clear of the wall is refused");
    var text = message == null ? "" : message;
    check(text.indexOf("wall") >= 0 && (text.indexOf("torch") >= 0 || text.indexOf("neck") >= 0), 'the reason names the parts: $text');
    check(text.indexOf("mm") >= 0 && text.indexOf("tip ") >= 0, 'and the pose of the tip: $text');
    Sys.println('weld planning: refused: $text');
  }

  static function testClosedRunChoosesAReachableCorner():Void {
    var rotation = seam().start().rotation;
    var points = [new Vec3(0.35, 0.2, 0.15), new Vec3(0.45, 0.2, 0.15), new Vec3(0.45, 0.3, 0.15), new Vec3(0.35, 0.3, 0.15)];
    var segments = [for (i in 0...4) new robotkit.skill.WeldPlan.WeldSegment(new Transform3(points[i], rotation),
      new Transform3(points[(i + 1) % 4], rotation), 'side$i')];
    var requested = new WeldPlan(segments, PARAMETERS);
    var planner = new WeldPathPlanner(new EntryCornerFixture(), new IkTolerance(2e-4, 1e-3, 300, 0.03), [3.0], WRIST);
    var planned = planner.plan(requested, [0.0]);
    check(planned.plan.segments[0].name == "side1", "a closed run may start at the next reachable corner");
    check([for (segment in planned.plan.segments) segment.name].join(",") == "side1,side2,side3,side0", "every seam keeps its direction and is welded exactly once");
    near(planned.plan.length(), requested.length(), 1e-9, "changing the entry corner preserves the deposited length");
    check(requested.segments[0].name == "side0", "planning leaves the CAD mission unchanged");
  }

  static function testBranchJumpIsRefused():Void {
    var planner = new WeldPathPlanner(new BranchJumpFixture(), new IkTolerance(2e-4, 1e-3, 300, 0.03), [3.0], WRIST);
    var message = "";
    try planner.plan(seam(), [0.0]) catch (error:Dynamic) message = Std.string(error);
    check(message.indexOf("IK branch discontinuity") >= 0, "reachable individual poses with a joint branch jump are refused before welding");
  }

  static function testCompiledMotionRejectsAnEntry():Void {
    var probe = {calls: 0};
    var planner = new WeldPathPlanner(new ContinuousBranchFixture(), new IkTolerance(2e-4, 1e-3, 300, 0.03), [3.0], WRIST, null, null,
      function(planned, start) {
        probe.calls++;
        if (planned.entry.joints[0] < 1.0) throw "The compiler rejects this entry branch";
        return new motionkit.robot.CompiledProgram([], []);
      });
    var planned = planner.plan(seam(), [0.0]);
    check(probe.calls == 3, "each complete candidate is compiled before it can be accepted");
    check(planned.entry.joints[0] == 1.0, "a compiler rejection tries a different entry before starting the weld");
  }

  static function testEntryUsesTheCurrentConfiguration():Void {
    var planner = new WeldPathPlanner(new CurrentEntryFixture(), new IkTolerance(2e-4, 1e-3, 300, 0.03), [3.0], WRIST);
    var planned = planner.plan(seam(), [0.3]);
    near(planned.entry.joints[0], 0.3, 1e-9, "the current arm configuration can reach an entry missed by broad IK sampling");
  }

  static function near(actual:Float, expected:Float, tolerance:Float, message:String):Void {
    assertions++;
    if (!(Math.abs(actual - expected) <= tolerance)) throw 'Assertion failed: $message (expected $expected, got $actual)';
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'Assertion failed: $message';
  }
}

/** Three reachable entry branches; only the third can reach the last half of the seam. */
private class EntryBranchFixture implements motionkit.kinematics.KinematicsSolver {
  public function new() {}
  public function jointCount():Int return 1;
  public function fork():motionkit.kinematics.KinematicsSolver return this;
  public function forward(q:Array<Float>):motionkit.kinematics.Pose3 throw "The fixture only solves poses";
  public function sampleCandidates(target:motionkit.kinematics.Pose3, maxCount:Int,
      tolerance:motionkit.kinematics.IkTolerance):Array<Array<Float>> return [[0.0], [0.5], [1.0]];
  public function solvePose(target:motionkit.kinematics.Pose3, seed:Array<Float>,
      tolerance:motionkit.kinematics.IkTolerance):Null<Array<Float>>
    return seed[0] < 1.0 && target.x >= 0.4 ? null : seed.copy();
  public function solveDifferential(q:Array<Float>, twist:motionkit.kinematics.Twist6,
      ?redundancyRate:Array<Float>):Null<Array<Float>> throw "The fixture only solves poses";
  public function solvePath(request:motionkit.kinematics.PathRequest):Array<Null<Array<Float>>> throw "The fixture only solves poses";
}

/** The weld poses are reachable, but an approach is available only on the far side of the perimeter. */
private class EntryCornerFixture extends EntryBranchFixture {
  public function new() super();
  override public function sampleCandidates(target:motionkit.kinematics.Pose3, maxCount:Int,
      tolerance:motionkit.kinematics.IkTolerance):Array<Array<Float>> return target.x >= 0.4 ? [[1.0]] : [];
}

/** Each pose has an IK answer, but reaching the second half requires a discontinuous joint change. */
private class BranchJumpFixture extends EntryBranchFixture {
  public function new() super();
  override public function solvePose(target:motionkit.kinematics.Pose3, seed:Array<Float>,
      tolerance:motionkit.kinematics.IkTolerance):Null<Array<Float>> return [seed[0] + (target.x >= 0.4 ? 1.0 : 0.0)];
}

private class ContinuousBranchFixture extends EntryBranchFixture {
  public function new() super();
  override public function solvePose(target:motionkit.kinematics.Pose3, seed:Array<Float>,
      tolerance:motionkit.kinematics.IkTolerance):Null<Array<Float>> return seed.copy();
}

private class CurrentEntryFixture extends ContinuousBranchFixture {
  public function new() super();
  override public function sampleCandidates(target:motionkit.kinematics.Pose3, maxCount:Int,
      tolerance:motionkit.kinematics.IkTolerance):Array<Array<Float>> return [];
}
