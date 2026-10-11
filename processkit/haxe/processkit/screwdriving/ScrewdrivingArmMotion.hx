package processkit.screwdriving;

import motionkit.robot.ManipulatorMotion;
import motionkit.kinematics.Pose3;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.Blend;
import motionkit.path.OrientationPolicy;
import robotkit.spatial.Transform3;

/** Executes checked position programs with a TCP at the uncompressed bit. */
class ScrewdrivingArmMotion implements ScrewdrivingMotion {
  public final motion:ManipulatorMotion;
  /** A mission may pin one arm configuration through pickup, driving and retract. */
  public var configuration:Null<motionkit.kinematics.SixAxisConfiguration> = null;
  final travelSpeed:Float;
  final contactSpeed:Float;
  final prepareTravel:Null<Void->Null<String>>;
  public function new(motion:ManipulatorMotion, travelSpeed:Float = 0.08, contactSpeed:Float = 0.01, ?prepareTravel:Void->Null<String>) {
    if (motion == null || !Math.isFinite(travelSpeed) || travelSpeed <= 0 || !Math.isFinite(contactSpeed) || contactSpeed <= 0)
      throw "Screwdriving arm motion needs checked motion and positive speeds";
    this.motion = motion; this.travelSpeed = travelSpeed; this.contactSpeed = contactSpeed; this.prepareTravel = prepareTravel;
  }
  var pending:Null<Transform3> = null;
  var waiting = 0.0;
  var waitProblem:Null<String> = null;
  public function move(target:Transform3, contact:Bool):Void {
    var p = target.translation, q = target.rotation;
    var pose = new Pose3(p.x,p.y,p.z,q.x,q.y,q.z,q.w);
    var prepare = prepareTravel;
    if (!contact && prepare != null) {
      var problem = prepare();
      if (problem != null) {pending = target; waitProblem = problem; return;}
      pending = null; waiting = 0;
      motion.run(new MotionProgram([MotionOp.MoveJ(motionkit.program.MoveTarget.PoseTarget(pose,motion.compiler.frameId,null),
        new motionkit.MotionOptions(0.4,0,0,true),Blend.ExactStop)],configuration));
      return;
    }
    motion.run(new MotionProgram([MotionOp.MoveL(new Pose3(p.x,p.y,p.z,q.x,q.y,q.z,q.w),motion.compiler.frameId,
      contact ? contactSpeed : travelSpeed,Blend.ExactStop,
      contact ? OrientationPolicy.FreeAboutTool : OrientationPolicy.Interpolated)],configuration));
  }
  public function update(durationSeconds:Float):Void {
    motion.update(durationSeconds);
    var target = pending;
    if (target != null && motion.failure == null) {
      waiting += durationSeconds;
      if (waiting > 2) throw 'Lost-screw escape clearance did not become available: $waitProblem';
      move(target,false);
    }
  }
  public function completed():Bool return pending == null && motion.completed;
  public function failure():Null<String> return motion.failure;
  public function stop():Void {pending = null; waiting = 0; motion.stop();}
  public function stopped():Bool return motion.failure == null && motion.sessionState() == motionkit.robot.SessionState.Idle;
  public function abort():Void {pending = null; waiting = 0; motion.abort();}
}
