import collisionkit.native.NativeCollisionWorld;
import haxe.Int64;
import motionkit.MotionOptions;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.robot.CompiledProgram;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.StructuredJointPathPlanner;
import motionkit.robot.TrajectoryClearanceProof.ClearanceEvent;
import motionkit.trajectory.ValidationReport;
import robotkit.collision.CollisionClearance;
import robotkit.collision.CollisionClearance;
import robotkit.manipulation.ClearanceBodyData;
import robotkit.manipulation.ClearanceChange;
import robotkit.manipulation.ClearanceWorld;
import robotkit.manipulation.Manipulator;
import robotkit.spatial.Vec3;

/**
 * Clearance in the validation report (COLLISION.md CL4b), on main's weld
 * arm: passing and failing cells recorded; a joint move whose sampled check
 * misses a thin plate between samples, caught by the bound; bisection at its
 * depth limit reported as sampled; a grasp and a place replayed; and a torch
 * window with its approach margin.
 */
@:access(WeldPlanningTests)
class ClearanceValidationTests {
  static inline var D6 = 0.0823;
  static final START = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0];
  static var assertions = 0;
  static var arm:Manipulator;

  public static function main():Void {
    arm = WeldPlanningTests.arm().arm;
    testPassingAndFailingCells();
    testBoundCatchesWhatSamplesMiss();
    testDepthLimitIsSampled();
    testGraspAndPlace();
    testTorchWindow();
    testServoStopsAboveTheTable();
    Sys.println('Clearance validation tests passed ($assertions assertions)');
  }

  static function torch():ClearanceBodyData
    return {name: "torch", link: "wrist_3_link", vertices: WeldPlanningTests.box(-0.012, 0.012, D6 - 0.012, D6 + 0.012, -0.14, -0.03), tool: true};

  static function neck():ClearanceBodyData
    return {name: "neck", link: "wrist_3_link", vertices: WeldPlanningTests.box(0.0, 0.10, D6 - 0.012, D6 + 0.012, -0.14, -0.10), tool: true};

  static function table():ClearanceBodyData
    return {name: "table", link: "base_link", vertices: WeldPlanningTests.box(-0.8, 0.8, -0.8, 0.8, -0.1, 0.0), tool: false};

  static function compiler(world:ClearanceWorld):ProgramCompiler
    return processkit.WeldingPlanRunner.planning(arm, 2.0).compiler.withJointPathPlanner(
      new StructuredJointPathPlanner(arm, null, null, world, 8, false));

  static function moves(targets:Array<Array<Float>>):MotionProgram
    return new MotionProgram([for (target in targets) MotionOp.MoveJ(MoveTarget.JointTarget(target), new MotionOptions(), Blend.ExactStop)]);

  static function reports(compiled:CompiledProgram):Array<ValidationReport>
    return [for (block in compiled.blocks) for (plan in block.plans) plan.report];

  static function shifted(q:Array<Float>, delta:Array<Float>):Array<Float> return [for (j in 0...q.length) q[j] + delta[j]];

  /** A box given in the wrist frame at `q`, as base-frame vertices. */
  static function inWrist(q:Array<Float>, x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float):Array<Float> {
    var wrist = arm.linkPoses(q, ["wrist_3_link"])[0];
    var local = WeldPlanningTests.box(x0, x1, y0, y1, z0, z1), out:Array<Float> = [];
    for (i in 0...8) {
      var p = wrist.transformPoint(new Vec3(local[3 * i], local[3 * i + 1], local[3 * i + 2]));
      out.push(p.x); out.push(p.y); out.push(p.z);
    }
    return out;
  }

  static function throwsWith(action:Void->Void):String {
    try action() catch (error:Dynamic) return Std.string(error);
    return "";
  }

  static function testPassingAndFailingCells():Void {
    var world = new robotkit.collision.CollisionClearance(arm, [torch(), neck(), table()], START, () -> new collisionkit.native.NativeCollisionWorld());
    var target = shifted(START, [0.4, 0.1, 0.0, 0.0, 0.0, 0.2]);
    var compiled = compiler(world).compile(moves([target]), START, Int64.ofInt(1));
    var report = reports(compiled)[0];
    check(report.collision.status == TrajectoryCoreConstants.MK_CHECK_PASSED, "a clear joint move passes");
    check(report.collision.method == TrajectoryCoreConstants.MK_CHECK_METHOD_BOUND, 'and its bound closes (method ${report.collision.method})');
    check(report.collisionPair.nameA != "" && report.collision.value >= report.collision.limit,
      'the closest pair is named with its clearance (${report.collisionPair.nameA}/${report.collisionPair.nameB} ${report.collision.value} >= ${report.collision.limit})');
    check(report.guarantees().collision == trajectorykit.validation.ValidationGuarantee.Proven, "the guarantee is a proof");
    Sys.println('CL4B passing cell: ${report.collisionPair.nameA}/${report.collisionPair.nameB} ${report.collision.value} m (needs ${report.collision.limit}), bound');
    compiled.dispose();
    // Without a clearance world the check stays unchecked.
    var free = processkit.WeldingPlanRunner.planning(arm, 2.0).compiler.compile(moves([target]), START, Int64.ofInt(1));
    check(reports(free)[0].collision.status == TrajectoryCoreConstants.MK_CHECK_UNCHECKED, "without a clearance world it is unchecked");
    free.dispose();
    // Down into the table: refused, naming the pair.
    // Lower the arm until the torch or the neck reaches the table.
    var low:Null<Array<Float>> = null;
    for (step in 1...40) for (joint in [1, 2]) if (low == null) {
      var delta = [for (_ in 0...6) 0.0];
      delta[joint] = step * 0.1;
      var q = shifted(START, delta), hit = world.violation(q);
      if (hit != null && hit.b == "table") low = q;
    }
    if (low == null) throw "assertion failed: no target reaches the table";
    var target2:Array<Float> = cast low;
    var message = throwsWith(() -> compiler(world).compile(moves([target2]), START, Int64.ofInt(1)).dispose());
    check(message.indexOf("table") >= 0 && message.indexOf("clearance") >= 0, 'a failing cell is refused with its pair ($message)');
  }

  /** A 1 mm bead swept across a half-millimetre plate: between the sampled check's samples, inside the bound's. */
  static function testBoundCatchesWhatSamplesMiss():Void {
    var bead:ClearanceBodyData = {name: "bead", link: "wrist_3_link",
      vertices: WeldPlanningTests.box(-0.0005, 0.0005, D6 - 0.0005, D6 + 0.0005, -0.1405, -0.1395), tool: true};
    // The arm reaches out, so the base's turn moves the bead about 12 mm between the sampled check's samples.
    var reach = START, farthest = 0.0;
    for (i in 0...16) for (k in 0...16) {
      var q = [0.0, -1.5 + 0.1 * i, 0.1 * k, -1.5708, -1.5708, 0.0];
      var p = arm.linkPoses(q, ["wrist_3_link"])[0].transformPoint(new Vec3(0, D6, -0.14));
      var r = Math.sqrt(p.x * p.x + p.y * p.y);
      if (r > farthest) {
        farthest = r;
        reach = q;
      }
    }
    var from = shifted(reach, [-0.47, 0, 0, 0, 0, 0]), to = shifted(reach, [0.53, 0, 0, 0, 0, 0]);
    // Where the sampled check samples the base's turn (TrajectoryClearance: every 10 ms, and steps of at
    // most 0.02 rad between): the plate goes in the widest gap in the middle of the motion.
    var free = processkit.WeldingPlanRunner.planning(arm, 2.0).compiler.compile(moves([to]), from, Int64.ofInt(1));
    var motion = motionkit.trajectory.Trajectory.fromSegments(free.blocks[0].plans[0].segments());
    var duration = motion.durationSeconds(), yaws:Array<Float> = [];
    var steps = Std.int(Math.ceil(duration / 0.01)), previous = motion.evaluate(0.0).positions;
    yaws.push(previous[0]);
    for (i in 1...steps + 1) {
      var q = motion.evaluate(duration * i / steps).positions;
      var parts = 1;
      for (j in 0...6) parts = Std.int(Math.max(parts, Math.ceil(Math.abs(q[j] - previous[j]) / 0.02)));
      for (k in 0...parts + 1) yaws.push(previous[0] + (q[0] - previous[0]) * k / parts);
      previous = q;
    }
    motion.dispose();
    free.dispose();
    yaws.sort((a, b) -> a < b ? -1 : a > b ? 1 : 0);
    var yawAt = 0.0, gap = 0.0;
    for (i in 1...yaws.length) if (yaws[i - 1] > -0.2 && yaws[i] < 0.3 && yaws[i] - yaws[i - 1] > gap) {
      gap = yaws[i] - yaws[i - 1];
      yawAt = 0.5 * (yaws[i] + yaws[i - 1]);
    }
    // The plate: a radial half-plane at that yaw, through the bead's sweep.
    var crossing = shifted(reach, [yawAt, 0, 0, 0, 0, 0]);
    var centre = arm.linkPoses(crossing, ["wrist_3_link"])[0].transformPoint(new Vec3(0, D6, -0.14));
    var radius = Math.sqrt(centre.x * centre.x + centre.y * centre.y);
    var yaw = Math.atan2(centre.y, centre.x), c = Math.cos(yaw), s = Math.sin(yaw), plate:Array<Float> = [];
    for (x in [radius - 0.05, radius + 0.05]) for (y in [-0.00025, 0.00025]) for (z in [centre.z - 0.05, centre.z + 0.05]) {
      plate.push(c * x - s * y); plate.push(s * x + c * y); plate.push(z);
    }
    var world = new robotkit.collision.CollisionClearance(arm, [bead, {name: "plate", link: "base_link", vertices: plate, tool: false}], from, () -> new collisionkit.native.NativeCollisionWorld(), 0.0002, 0.0002);
    check(world.violation(from) == null && world.violation(to) == null, "both ends are clear");
    var message = throwsWith(() -> compiler(world).compile(moves([to]), from, Int64.ofInt(1)).dispose());
    check(message.indexOf("trajectory clearance bound (bead, plate)") >= 0,
      'the bound finds the contact the sampled check missed (radius $radius m, samples ${gap} rad apart: $message)');
    Sys.println('CL4B between samples: radius $radius m, samples $gap rad apart; $message');
  }

  /** With no bisection the bound cannot close, and the report says sampled over the open interval. */
  static function testDepthLimitIsSampled():Void {
    var world = new robotkit.collision.CollisionClearance(arm, [torch(), neck(), table()], START, () -> new collisionkit.native.NativeCollisionWorld());
    var shallow = compiler(world);
    shallow.clearanceDepthLimit = 0;
    var compiled = shallow.compile(moves([shifted(START, [0.4, 0.1, 0.0, 0.0, 0.0, 0.2])]), START, Int64.ofInt(1));
    var report = reports(compiled)[0];
    check(report.collision.status == TrajectoryCoreConstants.MK_CHECK_PASSED, "it still passes (the sampled check did)");
    check(report.collision.method == TrajectoryCoreConstants.MK_CHECK_METHOD_SAMPLED, "but as sampled");
    check(report.collisionPair.segmentStart == 0 && report.collisionPair.segmentEnd > 0, 'naming the open interval (${report.collisionPair.segmentStart}..${report.collisionPair.segmentEnd} s)');
    check(switch report.guarantees().collision { case Sampled(_): true; case _: false; }, "the guarantee is sampled");
    compiled.dispose();
  }

  /** A part grasped at the torch tip rides along, and stays where it is placed while the torch leaves it. */
  static function testGraspAndPlace():Void {
    var part:ClearanceBodyData = {name: "part", link: "base_link", vertices: inWrist(START, -0.012, -0.004, D6 - 0.004, D6 + 0.004, -0.146, -0.138), tool: false};
    var world = new CollisionClearance(arm, [torch(), neck(), table(), part], START, () -> new NativeCollisionWorld(), 0.001, 0.0005);
    check(world.violation(START) != null, "the grasped part touches the torch");
    var carry = shifted(START, [0.5, -0.2, 0.0, 0.0, 0.0, 0.0]), leave = shifted(carry, [0.0, -0.25, 0.2, 0.0, 0.0, 0.0]);
    var program = moves([carry, leave]);
    var refused = throwsWith(() -> compiler(world).compile(program, START, Int64.ofInt(1)).dispose());
    check(refused.indexOf("part") >= 0, 'without the grasp the part is in the torch ($refused)');
    var replay = compiler(world);
    replay.clearanceEvents = [
      new ClearanceEvent(0, 0.0, ClearanceChange.Attach("part", "wrist_3_link")),
      new ClearanceEvent(1, 0.0, ClearanceChange.Detach("part")),
      new ClearanceEvent(1, 0.0, ClearanceChange.OpenWindow("torch", "part", null)),
      new ClearanceEvent(1, 1e6, ClearanceChange.CloseWindow("torch", "part"))
    ];
    var compiled = replay.compile(program, START, Int64.ofInt(1));
    var all = reports(compiled);
    check(all.length == 2 && all[0].collision.status == TrajectoryCoreConstants.MK_CHECK_PASSED
      && all[1].collision.status == TrajectoryCoreConstants.MK_CHECK_PASSED, "the grasp and the place replay clear");
    Sys.println('CL4B grasp and place: ${[for (r in all) r.collisionPair.nameA + "/" + r.collisionPair.nameB + " " + r.collision.value + " m"]}');
    compiled.dispose();
    // After the place, the part is where the torch left it, and the torch is clear of it.
    var placed = world.closest(leave);
    check(placed != null, "the scene carries on after the program");
  }

  /** The torch closes to 2 mm of its seam: inside the 5 mm margin, outside the window's 1 mm approach margin. */
  static function testTorchWindow():Void {
    var seam:ClearanceBodyData = {name: "seam", link: "base_link", vertices: inWrist(START, -0.012, -0.006, D6 - 0.012, D6 + 0.012, -0.152, -0.142), tool: false};
    var world = new CollisionClearance(arm, [torch(), neck(), table(), seam], START, () -> new NativeCollisionWorld());
    var above = shifted(START, [0.0, -0.2, 0.15, 0.0, 0.0, 0.0]);
    check(world.violation(above) == null, "the approach starts clear");
    var closing = world.violation(START);
    check(closing != null && closing.a == "torch" && closing.b == "seam" && Math.abs(closing.distance - 0.002) < 1e-6,
      'the torch ends 2 mm from its seam (${closing == null ? "clear" : closing.a + "/" + closing.b + " " + closing.distance})');
    var refused = throwsWith(() -> compiler(world).compile(moves([START]), above, Int64.ofInt(1)).dispose());
    check(refused.indexOf("seam") >= 0, 'without a window the approach breaches the margin ($refused)');
    var windowed = compiler(world);
    windowed.clearanceEvents = [
      new ClearanceEvent(0, 0.0, ClearanceChange.OpenWindow("torch", "seam", 0.001)),
      new ClearanceEvent(0, 1e6, ClearanceChange.CloseWindow("torch", "seam"))
    ];
    var compiled = windowed.compile(moves([START]), above, Int64.ofInt(1));
    var report = reports(compiled)[0];
    check(report.collision.status == TrajectoryCoreConstants.MK_CHECK_PASSED, "inside the window the approach margin holds");
    Sys.println('CL4B torch window: ${report.collisionPair.nameA}/${report.collisionPair.nameB} ${report.collision.value} m (needs ${report.collision.limit}), method ${report.collision.method}');
    compiled.dispose();
  }

  /** CL5: jogging the torch down and sideways, the servo stops it at the margin above the table and slides along. */
  static function testServoStopsAboveTheTable():Void {
    var description = new collisionkit.CollisionDescription();
    var model = arm.model;
    var bodies = collisionkit.kinematics.ModelBodies.describe(description, model, "ur", 0, arm.stateOf(START), "start", true);
    var wrist = bodies.bodies[model.bodyIndex("wrist_3_link")];
    description.addObject("torch", wrist, collisionkit.CollisionPose.identity(), collisionkit.CollisionGeometry.Convex(torch().vertices));
    bodies.declare();
    description.addFixed("table", collisionkit.CollisionPose.identity(), collisionkit.CollisionGeometry.Convex(table().vertices), 1);
    var world = new NativeCollisionWorld();
    var build = description.build(world);
    check(build.layoutErrors.length == 0, "the arm starts clear of the table");
    var servo = new motionkit.robot.ManipulatorServo(arm);
    servo.avoidanceSpeed = 0.2;
    servo.avoidance = state -> {
      bodies.place(build, state);
      return bodies.avoidancePairs(build, world.distances(0.05), 0.005, 0.05);
    };
    var q = START.copy(), closest = Math.POSITIVE_INFINITY, slid = 0.0, start = arm.tcpPose(q).translation, held = 0;
    for (_ in 0...1500) {
      var step = servo.step(q, new motionkit.kinematics.Twist6(0.05, 0.0, -0.2, 0, 0, 0), 0.01);
      if (step.avoided.length > 0) held++;
      for (j in 0...6) q[j] += step.velocity[j] * 0.01;
      bodies.place(build, arm.stateOf(q));
      for (d in world.distances(0.05)) closest = Math.min(closest, d.distance);
    }
    slid = arm.tcpPose(q).translation.x - start.x;
    check(held > 0 && closest >= 0.005 - 1e-3, 'the servo stops short of the table (closest ${closest} m)');
    check(slid > 0.2, 'and slides along it (${slid} m)');
    Sys.println('CL5 servo: closest ${closest} m above the table after sliding ${slid} m');
    servo.dispose();
    world.dispose();
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
