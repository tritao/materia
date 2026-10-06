package motionkit.robot;

import motionkit.kinematics.SixAxisConfiguration;

import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;

/** Geometric branch enumeration, independent of timing and process runners. */
interface AnalyticIk {
  function family():String;
  function jointCount():Int;
  function forward(q:Array<Float>):Pose3;
  function branches(target:Pose3, seed:Array<Float>, ?freedom:OrientationPolicy):Array<AnalyticBranch>;
}

/** A geometric branch and its legal periodic lift; singularities stay explicit. */
class AnalyticBranch {
  public final q:Array<Float>;
  public final branch:Int;
  public final singular:Bool;
  /** Numeric fallback cannot classify geometric singularities. */
  public final singularityKnown:Bool;
  public final configuration:Null<SixAxisConfiguration>;

  public function new(q:Array<Float>, branch:Int, singular:Bool, singularityKnown:Bool = true, ?configuration:SixAxisConfiguration) {
    this.q = q.copy(); this.branch = branch; this.singular = singular;
    this.singularityKnown = singularityKnown;this.configuration=configuration;
  }
}
