import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import motionkit.path.PoseMath;

/** Select compatible clearance/approach IK branches for the same physical targets. */
class OriginalOutfeedBranches {
  static function distance(a:Array<Float>,b:Array<Float>):Float {
    var d=0.0;for(i in 0...6) d+=(a[i]-b[i])*(a[i]-b[i]);return d;
  }
  static function options(solver:ReferenceKinematics,q:Array<Float>,worlds:Array<robotkit.collision.CollisionClearance>):Array<Array<Float>> {
    var wanted=solver.forward(q),out:Array<Array<Float>> = [];
    for(candidate in solver.sampleCandidates(wanted,16,new IkTolerance(1e-6,1e-6,400,0.005))) {
      var bounded=true;for(v in candidate) if(!Math.isFinite(v) || v< -Math.PI || v>Math.PI) bounded=false;
      if(!bounded) continue;
      var actual=solver.forward(candidate);
      if(PoseMath.distance(actual,wanted)>1e-6 || PoseMath.angle(actual,wanted)>1e-6) throw "Outfeed branch IK residual";
      var clear=true;for(world in worlds) if(world.violation(candidate)!=null) {clear=false;break;}
      if(clear) out.push(candidate);
    }
    return out;
  }
  public static function select(solver:ReferenceKinematics,start:Array<Float>,clearance:Array<Float>,approach:Array<Float>,worlds:Array<robotkit.collision.CollisionClearance>):Dynamic {
    var upper=options(solver,clearance,worlds),lower=options(solver,approach,worlds);
    upper.sort((a,b) -> distance(start,a)<distance(start,b) ? -1 : distance(start,a)>distance(start,b) ? 1 : 0);
    var checks=0,failures:Array<Dynamic> = [];
    for(a in upper) {
      var near=lower.copy();near.sort((c,d) -> distance(a,c)<distance(a,d) ? -1 : distance(a,c)>distance(a,d) ? 1 : 0);
      for(b in near.slice(0,2)) {
        var descent=CellRegistration.curveReview(worlds,[a,b]);checks++;
        if(descent.clear!=true) {failures.push({leg:"descent",proof:descent});continue;}
        var transfer=CellRegistration.curveReview(worlds,[start,a]);checks++;
        if(transfer.clear==true) return {clearance:a,approach:b,clearanceCandidates:upper.length,approachCandidates:lower.length,
          checks:checks,transfer:transfer,descent:descent,
          clearancePositionError:PoseMath.distance(solver.forward(a),solver.forward(clearance)),
          clearanceAngleError:PoseMath.angle(solver.forward(a),solver.forward(clearance)),
          approachPositionError:PoseMath.distance(solver.forward(b),solver.forward(approach)),
          approachAngleError:PoseMath.angle(solver.forward(b),solver.forward(approach)),scope:"Same Cartesian target frames; both segments continuously checked in all four closed-door holding states"};
        failures.push({leg:"transfer",proof:transfer});
      }
    }
    return {clearance:null,approach:null,clearanceCandidates:upper.length,approachCandidates:lower.length,checks:checks,failures:failures};
  }
}
