package motionkit.robot;

import motionkit.kinematics.IkTolerance;
import robotkit.manipulation.KinematicGroup;

/** Selects an FK-verified model-derived family; unsupported geometry stays explicit. */
class BranchIk {
  public static function of(group:KinematicGroup, ?tolerance:IkTolerance):AnalyticIk {
    if (group == null) throw "Branch IK requires a kinematic group";
    var reasons:Array<String> = [];
    try return new CartesianAnalyticIk(group)
    catch (error:Dynamic) reasons.push("Cartesian: " + Std.string(error));
    try return new EaikAnalyticIk(group)
    catch (error:Dynamic) reasons.push("EAIK: " + Std.string(error));
    return new NumericBranchIk(group,reasons.join("; "),tolerance);
  }
}
