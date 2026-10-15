import TendingReach.TendingReachResult;
import TendingReach.TendingReachPose;
import TendingCyclePlan.TendingTransferRequest;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import motionkit.planner.PlannerSpace;
import motionkit.planner.SavedPathValidation;
import motionkit.planner.RrtConnect;
import motionkit.robot.JointPathTiming;

/** Saved data proposes a solution; current targets and physical-state spaces decide whether it survives. */
class TendingReachReuse {
  static function frame(f:AssemblyFrame):Array<Float> return [f.x,f.y,f.z,f.qx,f.qy,f.qz,f.qw];
  public static function copyFrame(f:AssemblyFrame):AssemblyFrame
    return {x:f.x,y:f.y,z:f.z,qx:f.qx,qy:f.qy,qz:f.qz,qw:f.qw};
  public static function layoutKey(r:TendingReachResult):String return haxe.Json.stringify({
    cls: Std.string(r.cls), riser:r.riser, placement:frame(r.armPlacement), flange:frame(r.armBaseFlange),
    park:r.parkPose, approach:r.handlingApproachMm, clearance:r.trayClearanceMm,
    table:frame(r.tableFrame), offset:r.tableOffset, width:r.tableWidth, depth:r.tableDepth, top:r.tableTop,
    trays:[for (t in r.trays) {id:t.id, at:frame(t.frame), pitchX:t.pitchX, pitchY:t.pitchY, width:t.width, depth:t.depth,
      slots:[for (s in t.slots) {name:s.name, at:frame(s.frame)}]}]
  });

  public static function validPosition(q:Array<Float>, count:Int):Bool {
    if (q == null || q.length != count) return false;
    for (v in q) if (!Math.isFinite(v)) return false;
    return true;
  }
  public static function findPose(r:TendingReachResult, name:String, tool:String):Null<TendingReachPose> {
    var result:Null<TendingReachPose> = null;
    for (p in r.poses) if (p.name == name && p.tool == tool) {
      if (result != null) return null;
      result = p;
    }
    return result;
  }
  /** Use the study's 1e-10 tolerances and its existing small-angle rotation residual. */
  public static function sameFrame(a:AssemblyFrame, b:AssemblyFrame):Bool {
    if (a == null || b == null) return false;
    for (v in frame(a).concat(frame(b))) if (!Math.isFinite(v)) return false;
    var an=a.qx*a.qx+a.qy*a.qy+a.qz*a.qz+a.qw*a.qw;
    var bn=b.qx*b.qx+b.qy*b.qy+b.qz*b.qz+b.qw*b.qw;
    if (Math.abs(an-1)>1e-4 || Math.abs(bn-1)>1e-4) return false;
    var dx=(a.x-b.x)/1000, dy=(a.y-b.y)/1000, dz=(a.z-b.z)/1000;
    if (Math.sqrt(dx*dx+dy*dy+dz*dz) > 1e-10) return false;
    var error = [0.0,0.0,0.0];
    kinematicskit.Rotations.logOfProduct(b.qx,b.qy,b.qz,b.qw,-a.qx,-a.qy,-a.qz,a.qw,error,0);
    return Math.sqrt(error[0]*error[0]+error[1]*error[1]+error[2]*error[2]) <= 1e-10;
  }

  public static function sameRequest(a:TendingTransferRequest, b:TendingTransferRequest):Bool
    return a.station==b.station && a.tool==b.tool && a.fromTool==b.fromTool && a.fromPose==b.fromPose &&
      a.toPose==b.toPose && a.door==b.door && a.role==b.role;
  public static function findRoute(r:TendingReachResult, request:TendingTransferRequest):Null<Array<Array<Float>>> {
    var result:Null<Array<Array<Float>>> = null;
    for (t in r.transfers) if (sameRequest(t.request, request)) {
      if (result != null) return null;
      result = t.positions;
    }
    return result;
  }
  public static function route(space:PlannerSpace, proposal:Null<Array<Array<Float>>>, from:Array<Float>, to:Array<Float>,
      timing:JointPathTiming):{positions:Null<Array<Array<Float>>>, reused:Bool, failure:Null<String>} {
    var problem = SavedPathValidation.failure(space, proposal, from, to);
    if (problem == null) return {positions:[for (q in proposal) q.copy()], reused:true, failure:null};
    var plan = new RrtConnect(space,17).plan(from,to);
    return {positions:plan.waypoints == null ? null : timing.refine(space,plan.waypoints), reused:false, failure:plan.failure};
  }
}
