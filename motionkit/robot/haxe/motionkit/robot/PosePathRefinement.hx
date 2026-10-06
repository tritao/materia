package motionkit.robot;

import motionkit.path.PosePath;
import motionkit.path.PosePrimitive;
import motionkit.robot.AnalyticPathRefiner.RefinementTarget;

/** Task data from authored primitives, retaining both sides of path knots.
 * The path must already be expressed in the group's task reference frame. */
class PosePathRefinement {
  public final path:PosePath;
  public function new(path:PosePath) {
    if(path==null || path.length()<=0)throw "Refinement needs a nonempty pose path";
    this.path=path;
  }
  public function at(distance:Float):RefinementTarget {
    if(!Math.isFinite(distance) || distance<0 || distance>path.length())throw "Refinement distance outside pose path";
    var start=0.0,index=0,local=distance;
    for(i in 0...path.primitives.length){
      var end=start+path.primitives[i].length();
      if(distance<end || i==path.primitives.length-1){index=i;local=distance-start;break;}
      start=end;
    }
    var primitive=path.primitives[index],policy=primitive.orientationPolicy();
    switch policy {
      case Cone(_, _): throw "Cone refinement requires derivatives of its projected orientation centre";
      case Free: throw "Full-free refinement requires an explicit orientation centre";
      default:
    }
    var outgoing=primitive.derivativesAt(local),incoming=outgoing;
    if(index>0 && local==0){
      var previous=path.primitives[index-1];
      incoming=previous.derivativesAt(previous.length());
      for(axis in 0...3)if(Math.abs(incoming.linear[axis]-outgoing.linear[axis])>1e-6 ||
          Math.abs(incoming.angular[axis]-outgoing.angular[axis])>1e-6)
        throw "Refinement must split or blend a task-velocity discontinuity";
    }
    return new RefinementTarget(primitive.waypointAt(local).pose,policy,
      outgoing.linear.concat(outgoing.angular),outgoing.linearSecond.concat(outgoing.angularSecond),
      incoming.linearSecond.concat(incoming.angularSecond));
  }
}
