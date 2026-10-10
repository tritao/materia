package motionkit.robot;

import motionkit.MotionOptions;
import motionkit.event.EventValue;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import motionkit.program.Blend;
import motionkit.path.OrientationPolicy;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import robotkit.manipulation.KinematicGroup;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.execution.FiredProcessEvent;
import robotkit.core.Robot;

/**
 * Picks and places with an arm: a motion program that comes to the contact point along an approach
 * at `approachHeight`, sets the tool's hold channel there, dwells while it seals or lets go, then
 * rises again. A free-orientation tool returns home; a tool with a requested orientation remains at
 * the raised pose so a following operation can carry the held part without folding through the
 * home posture. Motion runs through `ManipulatorMotion` like every other arm program. To pick, it
 * presses `pressDepth` past the contact, as a compliant suction cup is pressed onto a part,
 * so the cup meets the part within the motion's position tolerance.
 * Without an explicit orientation the tool keeps its home Z direction and spin is free.
 * A requested orientation fixes its heading as well. Other joints, such as a mobile base's
 * wheels, hold still while the program runs.
 */
class HandlingPlanRunner implements robotkit.skill.HandlingRunner {
  public static inline var FRAME:String = "arm-base";
  /** Default vertical clearance between the grasp plane and the transport pose. */
  public static inline var DEFAULT_APPROACH_HEIGHT:Float = 0.12;

  public final motion:ManipulatorMotion;
  public final channel:String;
  public var configuration:Null<motionkit.kinematics.SixAxisConfiguration> = null;
  /** The arm's home joints, in its chain order, and the tool's pose there in the base frame. */
  public final home:Array<Float>;
  public final homePose:Pose3;
  public final approachHeight:Float;
  public final travelSpeed:Float;
  public final contactSpeed:Float;
  public final dwell:Float;
  public final pressDepth:Float;
  /** Open a physical gripper and allow its valve/stroke dwell before approaching a new part. */
  public final openBeforePick:Bool;

  public static function create(robot:Robot, manipulator:KinematicGroup,
      eventSource:Void -> {events:Array<FiredProcessEvent>, overflow:Bool}, channel:String,
      planning:PlanningLimits, ?approachHeight:Float = DEFAULT_APPROACH_HEIGHT, ?travelSpeed:Float = 0.3, ?contactSpeed:Float = 0.08,
      ?dwell:Float = 0.4, ?pressDepth:Float = 0.003, openBeforePick:Bool = false, ?ikTolerance:IkTolerance):HandlingPlanRunner {
    var count = manipulator.group.count();
    planning.requireGroup(manipulator);
    var limits = planning.validation();
    var solver = new ManipulatorKinematics(manipulator, 1e-8);
    var indices = [for (target in manipulator.toJointTargets([for (_ in 0...count) 0.0])) target.joint];
    var positions = robot.snapshot().setpointPositions;
    // A positioning axis stays at its current station while the arm returns
    // to its home posture. Arm coordinates remain relative to the saved pose.
    var home = [for (index in 0...count) manipulator.external[index] ? positions.get(indices[index]) : 0.0];
    // The authored poses use the home orientation. Prefer its spin softly in the
    // geometric ladder; omitted external ranges hold independent station axes.
    var preferred = solver.forward(home);
    var pathPlanner = new StructuredJointPathPlanner(manipulator,null,null,null,8,false,
      (sample,candidate) -> {
        // Numeric fallback candidates do not carry discrete roll indices.
        var angle = motionkit.path.PoseMath.angle(solver.forward(candidate.q),preferred);
        return angle*angle;
      });
    var compiler = new ProgramCompiler(solver, limits, FRAME,
      planning.velocity, planning.acceleration, planning.jerk,
      planning.startTolerances(0.005),
      null, 0.0075, 0.2, 0.002, 0.02, ikTolerance == null ? new IkTolerance(2e-3, 5e-3, 300, 0.03) : ikTolerance,pathPlanner);
    compiler.planningAssumptions = planning.assumptions.copy();
    compiler.planCheck = planning.check();
    var motion = new ManipulatorMotion(robot, compiler, function(_) return null, eventSource, indices);
    // Arm joint values are relative to the robot's starting pose.
    return new HandlingPlanRunner(motion, channel, home, solver.forward(home), approachHeight, travelSpeed,
      contactSpeed, dwell, pressDepth, openBeforePick);
  }

  public function new(motion:ManipulatorMotion, channel:String, home:Array<Float>, homePose:Pose3,
      approachHeight:Float, travelSpeed:Float, contactSpeed:Float, dwell:Float, pressDepth:Float, openBeforePick:Bool = false) {
    if (motion == null || channel == null || channel.length == 0 || home == null || homePose == null ||
        !(approachHeight > 0) || !(travelSpeed > 0) || !(contactSpeed > 0) || !(dwell >= 0) || !(pressDepth >= 0))
      throw "HandlingPlanRunner needs motion, a channel, a home pose and positive speeds";
    this.motion = motion;
    this.channel = channel;
    this.home = home.copy();
    this.homePose = homePose;
    this.approachHeight = approachHeight;
    this.travelSpeed = travelSpeed;
    this.contactSpeed = contactSpeed;
    this.dwell = dwell;
    this.pressDepth = pressDepth;
    this.openBeforePick = openBeforePick;
  }

  public function run(contact:Vec3, hold:Bool, ?orientation:Quat, ?approach:Vec3):Void
    motion.run(program(contact, hold, orientation, approach));

  /** The program `run` executes. */
  public function program(contact:Vec3, hold:Bool, ?orientation:Quat, ?approach:Vec3):MotionProgram {
    if (contact == null) throw "HandlingPlanRunner needs a contact point";
    var lift = approach == null ? new Vec3(0, 0, 1) : approach.normalized();
    var q = orientation == null ? new Quat(homePose.qx, homePose.qy, homePose.qz, homePose.qw) : orientation;
    var policy = orientation == null ? OrientationPolicy.FreeAboutTool : OrientationPolicy.Fixed;
    var abovePoint = contact.add(lift.scale(approachHeight));
    var atPoint = contact.sub(lift.scale(hold ? pressDepth : 0.0));
    var above = new Pose3(abovePoint.x, abovePoint.y, abovePoint.z, q.x, q.y, q.z, q.w);
    var at = new Pose3(atPoint.x, atPoint.y, atPoint.z, q.x, q.y, q.z, q.w);
    var ops:Array<MotionOp> = [];
    if (hold && openBeforePick) {
      ops.push(MotionOp.SetOutput(channel, EventValue.Digital(false)));
      ops.push(MotionOp.Dwell(dwell));
    }
    ops = ops.concat([
      MotionOp.MoveL(above, FRAME, travelSpeed, Blend.ExactStop,
        orientation == null ? policy : OrientationPolicy.Interpolated),
      MotionOp.MoveL(at, FRAME, contactSpeed, Blend.ExactStop, policy),
      MotionOp.SetOutput(channel, EventValue.Digital(hold)),
      MotionOp.Dwell(dwell),
      MotionOp.MoveL(above, FRAME, contactSpeed, Blend.ExactStop, policy)
    ]);
    if (orientation == null)
      ops.push(MotionOp.MoveJ(MoveTarget.JointTarget(home), new MotionOptions(), Blend.ExactStop));
    return new MotionProgram(ops, configuration);
  }

  /** Derive a step's approach while retaining the tool's actuation policy. */
  public function withApproachDistance(distance:Float):HandlingPlanRunner {
    return new HandlingPlanRunner(motion, channel, home, homePose, distance, travelSpeed,
      contactSpeed, dwell, pressDepth, openBeforePick);
  }

  public function update(dtSeconds:Float):Void motion.update(dtSeconds);
  public function abort():Void motion.abort();
  public function running():Bool return motion.running;
  public function completed():Bool return motion.completed;
  public function failure():Null<String> return motion.failure;
}
