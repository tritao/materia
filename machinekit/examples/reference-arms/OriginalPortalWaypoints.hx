import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.IkTolerance;
import motionkit.path.PoseMath;

/** Explicit study waypoints in front of the machine; no fixture/axis changes. */
class OriginalPortalWaypoints {
  public static function generate(solvers:Map<String,ReferenceKinematics>,goals:Map<String,Array<Float>>,
      base:AssemblyFrame,worlds:Array<robotkit.collision.CollisionClearance>):Dynamic {
    var inverse=AssemblyFrames.inverse(base),points:Array<Array<Float>> = [],targets:Array<Dynamic> = [];
    for(channel in ["A","B"]) {
      var solver=solvers.get(channel),q=goals.get(channel),goal=solver.forward(q);
      var world=AssemblyFrames.compose(base,{x:goal.x,y:goal.y,z:goal.z,qx:goal.qx,qy:goal.qy,qz:goal.qz,qw:goal.qw});
      for(y in [-0.35,-0.5,-0.65]) for(lift in [-0.1,0.0,0.1]) {
        var frame={x:world.x,y:y,z:world.z+lift,qx:world.qx,qy:world.qy,qz:world.qz,qw:world.qw};
        var local=AssemblyFrames.compose(inverse,frame),wanted=new Pose3(local.x,local.y,local.z,local.qx,local.qy,local.qz,local.qw);
        var candidates=solver.sampleCandidates(wanted,8,new IkTolerance(1e-6,1e-6,400,0.005)),accepted=0;
        for(candidate in candidates) {
          var bounded=true;
          for(i in 0...6) if(!Math.isFinite(candidate[i]) || candidate[i]<-Math.PI || candidate[i]>Math.PI) bounded=false;
          if(!bounded) continue;
          var actual=solver.forward(candidate);
          if(PoseMath.distance(actual,wanted)>1e-6 || PoseMath.angle(actual,wanted)>1e-6) throw "Portal IK residual";
          var clear=true;for(check in worlds) if(check.violation(candidate)!=null) {clear=false;break;}
          if(clear) {points.push(candidate);accepted++;}
        }
        targets.push({channel:channel,worldFrameMetres:frame,candidates:candidates.length,clearInFourOpenDoorStates:accepted});
      }
    }
    return {points:points,targets:targets,scope:"Explicit front-entry TCP waypoints, all IK candidates checked in all four open-door holding states. Every connecting path still requires continuous review."};
  }
}
