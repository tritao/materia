package motionkit.robot;

import robotkit.manipulation.IkOptions;
import motionkit.MotionOptions;
import haxe.Int64;
import motionkit.event.HoldPolicy;
import motionkit.event.PathEvent;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.IkTolerance;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.trajectory.ValidationLimits;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.WorkPatch;
import robotkit.process.Toolpath;
import robotkit.process.ToolpathPoint;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.world.FiredProcessEvent;
import robotkit.world.Robot;

/** Lowers one raster patch into approach, process and retract plans. */
class SurfacePlanRunner implements robotkit.skill.SurfacePlanRunner {
  public final motion:ManipulatorMotion;
  public final manipulator:Manipulator;
  public final channel:String;
  public final feed:Float;

  public static function create(robot:Robot, manipulator:Manipulator,
      eventSource:Void -> {events:Array<FiredProcessEvent>, overflow:Bool},
      feed:Float, maxAcceleration:Float, maxJointJump:Float,
      ?modelRevision:Int64, ?calibrationRevision:Int64,
      ?cartesianResolution:Float = 0.01):SurfacePlanRunner {
    var count = manipulator.group.count();
    var limits = new ValidationLimits(count,
      modelRevision == null ? Int64.ofInt(1) : modelRevision,
      calibrationRevision == null ? Int64.ofInt(0) : calibrationRevision);
    for (joint in 0...count) {
      var bound = manipulator.group.limitsOf(joint);
      if (bound.lower < bound.upper) limits.position(joint, bound.lower, bound.upper);
      limits.velocity(joint, bound.velocity > 0.0 ? bound.velocity : 2.0);
      limits.acceleration(joint, maxAcceleration);
      limits.jerk(joint, 20.0);
    }
    var solver = new ManipulatorKinematics(manipulator, 1e-8);
    var compiler = new ProgramCompiler(solver, limits, "arm-base",
      [for (joint in 0...count) {
        var speed = manipulator.group.limitsOf(joint).velocity;
        speed > 0.0 ? speed : 2.0;
      }], [for (_ in 0...count) maxAcceleration], [for (_ in 0...count) 20.0],
      StartTolerances.uniform(count, 0.005, maxAcceleration * 0.01, 20.0 * 0.01),
      null, Math.min(cartesianResolution, 0.0075), maxJointJump, 0.005, 0.02,
      new IkTolerance(2e-3, 5e-3, 300, 0.03));
    var indices = [for (target in manipulator.toJointTargets(
      [for (_ in 0...count) 0.0])) target.joint];
    var motion = new ManipulatorMotion(robot, compiler,
      function(_) return null, eventSource, indices);
    return new SurfacePlanRunner(motion, manipulator, "surface.process", feed);
  }

  public function new(motion:ManipulatorMotion, manipulator:Manipulator,
      channel:String, feed:Float) {
    if (motion == null || manipulator == null || channel == null || channel.length == 0 ||
        !Math.isFinite(feed) || feed <= 0.0)
      throw "SurfacePlanRunner needs motion, manipulator, channel and feed";
    this.motion = motion; this.manipulator = manipulator;
    this.channel = channel; this.feed = feed;
  }

  public function runPatch(patch:WorkPatch, base_T_work:Transform3,
      seed:Array<Float>):Void {
    motion.run(programForPatch(patch, base_T_work, seed));
  }

  public function programForPatch(patch:WorkPatch, base_T_work:Transform3,
      seed:Array<Float>):MotionProgram {
    if (patch == null || base_T_work == null || patch.toolpath.points.length < 2)
      throw "Surface patch needs a path and base transform";
    var points:Array<ToolpathPoint> = [];
    for (point in patch.toolpath.points)
      points.push(new ToolpathPoint(base_T_work.compose(point.work_T_tcp),
        point.feedRate, point.processOn, null, null, 0.005, 0.02));
    var path = ToolpathPosePath.convert(new Toolpath("arm-base", points), channel);
    var events = [for (event in path.events) new PathEvent(event.distance,
      event.channel, event.value, event.leadSeconds, HoldPolicy.RestoreOnResume)];
    var start = points[0].work_T_tcp;
    var end = points[points.length - 1].work_T_tcp;
    var ik = manipulator.solve(start, seed, new IkOptions(2e-3, 5e-3, 300, 0.03));
    if (!ik.converged) throw "Surface patch approach pose is unreachable";
    var localEnd = patch.toolpath.points[patch.toolpath.points.length - 1].work_T_tcp;
    var retractLocal = new Transform3(
      localEnd.translation.add(new Vec3(0.0, 0.0, 0.03)), localEnd.rotation);
    var retract = base_T_work.compose(retractLocal);
    return new MotionProgram([
      MotionOp.MoveJ(MoveTarget.JointTarget(ik.q), new MotionOptions(), Blend.ExactStop),
      MotionOp.Dwell(0.6),
      MotionOp.FollowPath(path, "arm-base", feed, events),
      MotionOp.MoveL(pose(retract), "arm-base", feed, Blend.ExactStop)
    ]);
  }

  public function update(dtSeconds:Float):Void motion.update(dtSeconds);
  public function hold():Void motion.hold();
  public function resume():Void motion.resume();
  public function abort():Void motion.abort();
  public function running():Bool return motion.running;
  public function completed():Bool return motion.completed;
  public function failure():Null<String> return motion.failure;
  public function firedEvents():Array<FiredProcessEvent> return motion.firedEvents();
  public function jointIndices():Array<Int> return motion.jointIndices.copy();

  static function pose(transform:Transform3):Pose3 {
    var p = transform.translation, q = transform.rotation;
    return new Pose3(p.x, p.y, p.z, q.x, q.y, q.z, q.w);
  }
}
