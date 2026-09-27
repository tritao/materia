package motionkit.robot;

import haxe.Int64;
import motionkit.MotionOptions;
import motionkit.kinematics.IkTolerance;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.trajectory.ValidationLimits;
import robotkit.manipulation.Manipulator;
import robotkit.process.Toolpath;
import robotkit.world.Robot;

/** Plans joint moves through the authored poses of a toolpath. */
class ToolpathPlanRunner implements robotkit.skill.ToolpathPlanRunner {
  public final motion:ManipulatorMotion;
  public final manipulator:Manipulator;
  public final positionTolerance:Float;
  public final orientationTolerance:Float;
  public final ikMaxIterations:Int;
  public final ikDamping:Float;
  public final maxJointJump:Float;
  var cutActive:Bool = false;

  public static function create(robot:Robot, manipulator:Manipulator, frameId:String,
      maxAcceleration:Float, maxJointJump:Float, positionTolerance:Float,
      orientationTolerance:Float, ikMaxIterations:Int, ikDamping:Float):ToolpathPlanRunner {
    var count = manipulator.group.count();
    var limits = new ValidationLimits(count, Int64.ofInt(1), Int64.ofInt(0));
    for (joint in 0...count) {
      var bound = manipulator.group.limitsOf(joint);
      if (bound.lower < bound.upper) limits.position(joint, bound.lower, bound.upper);
      limits.velocity(joint, bound.velocity > 0.0 ? bound.velocity : 10.0);
      limits.acceleration(joint, maxAcceleration);
      limits.jerk(joint, 20.0);
    }
    var solver = new ManipulatorKinematics(manipulator, 1e-8);
    var compiler = new ProgramCompiler(solver, limits, frameId,
      [for (joint in 0...count) {
        var speed = manipulator.group.limitsOf(joint).velocity;
        speed > 0.0 ? speed : 10.0;
      }], [for (_ in 0...count) maxAcceleration], [for (_ in 0...count) 20.0],
      null, 0.01, maxJointJump, positionTolerance,
      orientationTolerance, new IkTolerance(positionTolerance,
        orientationTolerance, ikMaxIterations, ikDamping));
    var indices = [for (target in manipulator.toJointTargets(
      [for (_ in 0...count) 0.0])) target.joint];
    var motion = new ManipulatorMotion(robot, compiler, function(_) return null,
      function() return {events:[], overflow:false}, indices);
    return new ToolpathPlanRunner(motion, manipulator, positionTolerance,
      orientationTolerance, ikMaxIterations, ikDamping, maxJointJump);
  }

  public function new(motion:ManipulatorMotion, manipulator:Manipulator,
      positionTolerance:Float, orientationTolerance:Float,
      ikMaxIterations:Int, ikDamping:Float, maxJointJump:Float) {
    this.motion = motion;
    this.manipulator = manipulator;
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
    this.ikMaxIterations = ikMaxIterations;
    this.ikDamping = ikDamping;
    this.maxJointJump = maxJointJump;
  }

  public function run(toolpath:Toolpath, seed:Array<Float>):Void {
    if (toolpath == null || toolpath.points.length < 2)
      throw "Toolpath plan needs at least two points";
    if (toolpath.frameId != motion.compiler.frameId)
      throw 'Toolpath frame ${toolpath.frameId} does not match ${motion.compiler.frameId}';
    if (seed == null || seed.length != manipulator.group.count())
      throw "Toolpath plan needs one seed value per joint";
    var previous = seed.copy();
    var previousPose = manipulator.tcpPose(seed);
    var ops:Array<MotionOp> = [];
    for (point in toolpath.points) {
      var ik = manipulator.solveIkForTcp(point.work_T_tcp, previous,
        positionTolerance, orientationTolerance, ikMaxIterations, ikDamping);
      if (!ik.converged) throw "Toolpath waypoint is unreachable";
      var jump = 0.0;
      for (joint in 0...previous.length)
        jump = Math.max(jump, Math.abs(ik.q[joint] - previous[joint]));
      if (jump > maxJointJump) throw 'Toolpath waypoint joint jump $jump exceeds $maxJointJump';
      var linearDistance = point.work_T_tcp.translation
        .sub(previousPose.translation).norm();
      var angularDistance = previousPose.rotation
        .angularDistance(point.work_T_tcp.rotation) * 0.1;
      var pathDistance = Math.sqrt(linearDistance * linearDistance +
        angularDistance * angularDistance);
      var jointSpeed = pathDistance > 1e-9 ? jump * point.feedRate / pathDistance : 0.0;
      ops.push(MotionOp.MoveJ(MoveTarget.JointTarget(ik.q),
        new MotionOptions(jointSpeed), Blend.ExactStop));
      previous = ik.q;
      previousPose = point.work_T_tcp;
    }
    cutActive = false;
    motion.run(new MotionProgram(ops));
  }

  public function update(dtSeconds:Float):Void {
    var before = motion.progress().op;
    motion.update(dtSeconds);
    cutActive = before == 1 || motion.progress().op == 1;
  }
  public function abort():Void motion.abort();
  public function running():Bool return motion.running;
  public function completed():Bool return motion.completed;
  public function failure():Null<String> return motion.failure;
  public function cuttingMoveActive():Bool return cutActive;
}
