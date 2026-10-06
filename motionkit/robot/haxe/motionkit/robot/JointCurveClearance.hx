package motionkit.robot;

import motionkit.planner.JointPathSamples;
import motionkit.planner.JointPathPolynomial;
import motionkit.planner.JointPathClearanceProof;
import motionkit.kinematics.Pose3;
import robotkit.manipulation.ArmClearance;

/** Continuous quintic-path certificate, with hull displacement reserved for
 * native lowering. Bernstein subdivision bounds every configuration between
 * samples; it never treats a sampled chord sweep as a continuous proof. */
class JointCurveClearance implements JointPathClearanceProof {
  public final world:ArmClearance;
  public final contact:Bool;
  public final policy:Null<Pose3->Bool>;
  final guard:Null<(Pose3,Float)->Bool>;
  final tolerance:Float;
  final snapshot:JointPathSamples;
  var checked:Bool = false;
  public var queries(default,null):Int = 0;
  public var maximumDelta(default,null):Float = 0.0;
  public var failedSpan(default,null):Int = -1;
  public var failure(default,null):Null<ArmClearance.ClearanceViolation> = null;
  public function new(world:ArmClearance,path:JointPathSamples,tolerance:Float,contact:Bool,
      ?policy:Pose3->Bool,?guard:(Pose3,Float)->Bool) {
    if(world==null || path==null || !Math.isFinite(tolerance) || tolerance<=0)
      throw "Curve clearance requires a world and finite positive lowering tolerance";
    if(policy!=null && guard==null)throw "Contact permission requires a conservative neighborhood guard";
    this.world=world;this.tolerance=tolerance;this.contact=contact;this.policy=policy;this.guard=guard;
    snapshot=new JointPathSamples(path.s,path.q,path.qPrime,path.qDoublePrime,path.qDoublePrimeBefore);
  }
  public function samePolicy(world:ArmClearance,contact:Bool,policy:Null<Pose3->Bool>,guard:Null<(Pose3,Float)->Bool>):Bool
    return this.world==world && this.contact==contact &&
      (this.policy==null ? policy==null : policy!=null && Reflect.compareMethods(this.policy,policy)) &&
      (this.guard==null ? guard==null : guard!=null && Reflect.compareMethods(this.guard,guard));
  public function covers(path:JointPathSamples,tolerance:Float):Bool {
    if(!checked || failedSpan>=0 || !Math.isFinite(tolerance) || tolerance<=0 || tolerance>this.tolerance ||
        path.s.length!=snapshot.s.length || path.jointCount!=snapshot.jointCount)return false;
    // Re-aligned distance grids may change Hermite coefficients slightly.
    // Compare polynomial ranges, not an arbitrary tolerance on the grid.
    for(span in 0...path.s.length-1){
      if(!Math.isFinite(path.s[span]) || !Math.isFinite(path.s[span+1]) || path.s[span+1]<=path.s[span])return false;
      var a=JointPathPolynomial.coefficients(snapshot,span),b=JointPathPolynomial.coefficients(path,span);
      for(j in 0...a.length){var error=0.0;for(k in 0...6)error+=Math.abs(a[j][k]-b[j][k]);
        if(!Math.isFinite(error) || error>this.tolerance*0.01)return false;
      }
    }
    return true;
  }
  public function check():Bool {
    for(span in 0...snapshot.s.length-1){
      var control=JointPathPolynomial.bernstein(JointPathPolynomial.coefficients(snapshot,span));
      if(!piece(control,0)){failedSpan=span;return false;}
    }
    checked=true;return true;
  }
  /** Phase-gate audit: compare actual lowered motion with the authored quintic
   * at distances from the native time law, and measure hull displacement. */
  public function auditTrajectory(path:JointPathSamples,trajectory:motionkit.trajectory.Trajectory,
      distances:Array<Float>):Void {
    if(!covers(path,tolerance) || distances.length<2)throw "Lowering audit requires the certified curve and exact time-law distances";
    var span=0,worstError=0.0,worstRatio=0.0,samples=0;
    // The compiler's map is at controller resolution; use a bounded 100 ms
    // audit cadence plus both ends. The continuous proof remains analytical.
    var stride=Std.int(Math.max(1,Math.floor(0.1*(distances.length-1)/trajectory.durationSeconds())));
    var indices=[for(i in 0...distances.length)if(i%stride==0 || i==distances.length-1)i];
    for(i in indices){var distance=distances[i];
      while(span<path.s.length-2 && distance>path.s[span+1])span++;
      var q=JointPathPolynomial.at(JointPathPolynomial.coefficients(path,span),
        (distance-path.s[span])/(path.s[span+1]-path.s[span]));
      var actual=trajectory.evaluate(trajectory.durationSeconds()*i/(distances.length-1)).positions;
      for(j in 0...q.length){var error=Math.abs(q[j]-actual[j]);worstError=Math.max(worstError,error);
        if(error>1.01*tolerance+1e-10)throw 'Lowered joint $j exceeds certified error: $error > ${1.01*tolerance}';}
      worstRatio=Math.max(worstRatio,world.auditDisplacement(q,actual,[for(_ in q)1.01*tolerance]));samples++;
    }
    Sys.println("PROCESS_PATH_CLEARANCE_AUDIT "+haxe.Json.stringify({samples:samples,maximumJointError:worstError,
      maximumHullBoundRatio:worstRatio,loweringTolerance:tolerance}));
  }
  function piece(control:Array<Array<Float>>,depth:Int):Bool {
    var predicate:(Pose3,Float)->Bool=cast guard;
    var halves=JointPathPolynomial.split(control),q=[for(row in halves[0])row[5]];
    var errors=[for(j in 0...q.length){var e=0.0;for(v in control[j])e=Math.max(e,Math.abs(v-q[j]));e+1.01*tolerance;}];
    var delta=world.displacementBounds(q,errors);
    var contactHere=contact;
    if(policy!=null){var tcp=world.arm.tcpPose(q);contactHere=predicate(new Pose3(tcp.translation.x,tcp.translation.y,tcp.translation.z,
      tcp.rotation.x,tcp.rotation.y,tcp.rotation.z,tcp.rotation.w),world.tcpDisplacementBound(q,errors));}
    queries++;
    var found=world.violation(q,contactHere,null,delta);
    if(found==null){
      var lowering=world.displacementBounds(q,[for(_ in q)1.01*tolerance]);
      for(v in lowering)maximumDelta=Math.max(maximumDelta,v);
      return true;
    }
    if(depth>=20){failure=found;return false;}
    // A failing midpoint under the lowering margin cannot be certified by
    // further subdivision. Contact uses the same shrunken neighborhood.
    var baseErrors=[for(_ in q)1.01*tolerance],baseContact=contact;
    if(policy!=null){var tcp=world.arm.tcpPose(q);baseContact=predicate(new Pose3(tcp.translation.x,tcp.translation.y,tcp.translation.z,
      tcp.rotation.x,tcp.rotation.y,tcp.rotation.z,tcp.rotation.w),world.tcpDisplacementBound(q,baseErrors));}
    queries++;
    var actual=world.violation(q,baseContact,null,world.displacementBounds(q,baseErrors));
    if(actual!=null){failure=actual;return false;}
    return piece(halves[0],depth+1) && piece(halves[1],depth+1);
  }
}
