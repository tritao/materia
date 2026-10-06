package motionkit.robot;

import motionkit.kinematics.Pose3;
import motionkit.path.PoseMath;
import robotkit.manipulation.KinematicGroup;

/** Immutable request preferences; FK always uses the planning worker's group. */
class JointPathPreferences {
  final posture:Null<Array<Float>>;
  final orientation:Null<Pose3>;
  final preferTarget:Bool;

  public function new(source:ManipulatorKinematics) {
    posture = source.preferredPosture == null ? null : source.preferredPosture.copy();
    if (posture != null) {
      if (posture.length != source.jointCount()) throw "Preferred posture must match compiled joints";
      for (value in posture) if (!Math.isFinite(value)) throw "Preferred posture must be finite";
    }
    var pose = source.preferredOrientation;
    orientation = pose == null ? null : new Pose3(pose.x,pose.y,pose.z,pose.qx,pose.qy,pose.qz,pose.qw);
    preferTarget = source.preferTargetOrientation;
  }

  public function cost(group:KinematicGroup,target:Pose3,q:Array<Float>):Float {
    var value = 0.0;
    if (posture != null) for (joint in 0...q.length) {
      var delta = q[joint]-posture[joint];
      value += delta*delta;
    }
    var desired = orientation == null && preferTarget ? target : orientation;
    if (desired != null) {
      var fk = group.tcpPose(q);
      var reached = new Pose3(fk.translation.x,fk.translation.y,fk.translation.z,
        fk.rotation.x,fk.rotation.y,fk.rotation.z,fk.rotation.w);
      var angle = PoseMath.angle(reached,desired);
      value += angle*angle;
    }
    return value;
  }
}
