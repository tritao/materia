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
    return postureCost(q)+orientationCost(group,target,q);
  }

  function postureCost(q:Array<Float>):Float {
    var value = 0.0;
    if (posture != null) for (joint in 0...q.length) {
      var delta = q[joint]-posture[joint];
      value += delta*delta;
    }
    return value;
  }

  function orientationCost(group:KinematicGroup,target:Pose3,q:Array<Float>):Float {
    var value=0.0;
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
  /** Exact analytic branches and physical turns share one task orientation
   * per lattice cell (their native FK witnesses are within 1e-9). Posture and
   * caller costs still use every physical joint vector independently. */
  public function forProblem(group:KinematicGroup,problem:CandidateProblem):
      (Int,motionkit.robot.CartesianCandidateSampler.LatticeCandidate)->Float {
    var analytic=problem.family=="EAIK" || problem.family=="XYZ" ||
      problem.family=="XYZ+C" || problem.family=="XYZ+C+A";
    var cells=1.0*problem.rollCount*problem.tiltCount*problem.azimuthCount;
    if(!analytic || cells>2147483647.0)
      return (sample,candidate)->cost(group,problem.request.poses[sample],candidate.q);
    var cache=[for(_ in problem.samples)new Map<Int,Float>()];
    return (sample,candidate)->{
      var key=candidate.roll+problem.rollCount*(candidate.azimuth+problem.azimuthCount*candidate.tilt);
      var angle=cache[sample].get(key);
      if(angle==null){angle=orientationCost(group,problem.request.poses[sample],candidate.q);cache[sample].set(key,angle);}
      return postureCost(candidate.q)+angle;
    };
  }

}
