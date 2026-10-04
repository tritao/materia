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
import robotkit.manipulation.Manipulator;
import robotkit.spatial.Vec3;
import robotkit.execution.FiredProcessEvent;
import robotkit.core.Robot;

/**
 * Picks and places with an arm: a motion program that comes down onto the contact point from above
 * at `approachHeight`, sets the tool's hold channel there, dwells while it seals or lets go, rises
 * and returns to the arm's home joints, run by `ManipulatorMotion` like every other arm program. To
 * pick, it presses `pressDepth` past the contact, as a compliant suction cup is pressed onto a part,
 * so the cup meets the part within the motion's position tolerance.
 * The tool keeps its home Z direction, with home orientation preferred and spin free. The arm's other joints, such as a mobile base's
 * wheels, hold still while the program runs.
 */
class HandlingPlanRunner implements robotkit.skill.HandlingRunner {
  public static inline var FRAME:String = "arm-base";

  public final motion:ManipulatorMotion;
  public final channel:String;
  /** The arm's home joints, in its chain order, and the tool's pose there in the base frame. */
  public final home:Array<Float>;
  public final homePose:Pose3;
  public final approachHeight:Float;
  public final travelSpeed:Float;
  public final contactSpeed:Float;
  public final dwell:Float;
  public final pressDepth:Float;

  public static function create(robot:Robot, manipulator:Manipulator,
      eventSource:Void -> {events:Array<FiredProcessEvent>, overflow:Bool}, channel:String,
      planning:PlanningLimits, ?approachHeight:Float = 0.12, ?travelSpeed:Float = 0.3, ?contactSpeed:Float = 0.08,
      ?dwell:Float = 0.4, ?pressDepth:Float = 0.003):HandlingPlanRunner {
    var count = manipulator.group.count();
    planning.requireGroup(manipulator);
    var limits = planning.validation();
    var solver = new ManipulatorKinematics(manipulator, 1e-8);
    var home = [for (_ in 0...count) 0.0];
    solver.preferredOrientation = solver.forward(home);
    var compiler = new ProgramCompiler(solver, limits, FRAME,
      planning.velocity, planning.acceleration, planning.jerk,
      planning.startTolerances(0.005),
      null, 0.0075, 0.2, 0.002, 0.02, new IkTolerance(2e-3, 5e-3, 300, 0.03));
    compiler.planningAssumptions = planning.assumptions.copy();
    compiler.planCheck = planning.check();
    var indices = [for (target in manipulator.toJointTargets([for (_ in 0...count) 0.0])) target.joint];
    var motion = new ManipulatorMotion(robot, compiler, function(_) return null, eventSource, indices);
    // Joint values are relative to the robot's starting pose, so home is all zeros.
    return new HandlingPlanRunner(motion, channel, home, solver.forward(home), approachHeight, travelSpeed,
      contactSpeed, dwell, pressDepth);
  }

  public function new(motion:ManipulatorMotion, channel:String, home:Array<Float>, homePose:Pose3,
      approachHeight:Float, travelSpeed:Float, contactSpeed:Float, dwell:Float, pressDepth:Float) {
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
  }

  public function run(contact:Vec3, hold:Bool):Void motion.run(program(contact, hold));

  /** The program `run` executes. */
  public function program(contact:Vec3, hold:Bool):MotionProgram {
    if (contact == null) throw "HandlingPlanRunner needs a contact point";
    var q = homePose;
    var above = new Pose3(contact.x, contact.y, contact.z + approachHeight, q.qx, q.qy, q.qz, q.qw);
    var at = new Pose3(contact.x, contact.y, contact.z - (hold ? pressDepth : 0.0), q.qx, q.qy, q.qz, q.qw);
    return new MotionProgram([
      MotionOp.MoveL(above, FRAME, travelSpeed, Blend.ExactStop, OrientationPolicy.FreeAboutTool),
      MotionOp.MoveL(at, FRAME, contactSpeed, Blend.ExactStop, OrientationPolicy.FreeAboutTool),
      MotionOp.SetOutput(channel, EventValue.Digital(hold)),
      MotionOp.Dwell(dwell),
      MotionOp.MoveL(above, FRAME, contactSpeed, Blend.ExactStop, OrientationPolicy.FreeAboutTool),
      MotionOp.MoveJ(MoveTarget.JointTarget(home), new MotionOptions(), Blend.ExactStop)
    ]);
  }

  public function update(dtSeconds:Float):Void motion.update(dtSeconds);
  public function abort():Void motion.abort();
  public function running():Bool return motion.running;
  public function completed():Bool return motion.completed;
  public function failure():Null<String> return motion.failure;
}
