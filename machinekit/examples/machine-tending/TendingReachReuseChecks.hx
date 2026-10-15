import motionkit.planner.PlannerSpace;
import motionkit.planner.SavedPathValidation;
import motionkit.robot.JointPathTiming;

/** Independent analytic wall with a gap; endpoints alone cannot certify a crossing. */
private class ReuseWall implements PlannerSpace {
  public var pointQueries = 0;
  public var edgeQueries = 0;
  public function new() {}
  public function dimension():Int return 2;
  public function lower():Array<Float> return [-1.0,-1.0];
  public function upper():Array<Float> return [1.0,1.0];
  public function invalid(q:Array<Float>):Null<String> {
    pointQueries++;
    return Math.abs(q[0]) <= 0.05 && Math.abs(q[1]) >= 0.15 ? "wall" : null;
  }
  public function edgesClear(edges:Array<Float>, count:Int):Array<Bool> {
    edgeQueries += count;
    return [for (i in 0...count) {
      var x=edges[4*i], y=edges[4*i+1], dx=edges[4*i+2]-x, dy=edges[4*i+3]-y;
      var clear=true;
      if (dx == 0) clear = Math.abs(x)>0.05 || Math.abs(y)<0.15 && Math.abs(y+dy)<0.15;
      else {
        var a=(-0.05-x)/dx, b=(0.05-x)/dx;
        var enter=Math.max(0,Math.min(a,b)), exit=Math.min(1,Math.max(a,b));
        if (enter<=exit) clear=Math.abs(y+dy*enter)<0.15 && Math.abs(y+dy*exit)<0.15;
      }
      clear;
    }];
  }
}

class TendingReachReuseChecks {
  static var assertions = 0;
  static function require(value:Bool, reason:String):Void {
    assertions++;
    if (!value) throw reason;
  }
  static function saved():TendingReach.TendingReachResult
    return TendingReachLayout.decode(TendingReachCertificate.JSON).result;

  public static function run():Void {
    var wall = new ReuseWall(), from=[-0.8,0.6], to=[0.8,0.5];
    var timing = new JointPathTiming([for (_ in 0...2) {velocity:1.0,acceleration:2.0,jerk:8.0}]);
    var path=[from,[-0.1,0.0],[0.1,0.0],to], before=haxe.Json.stringify(path);
    var good=TendingReachReuse.route(wall,path,from,to,timing);
    require(good.reused && good.positions != null, "valid route is retained");
    require(wall.pointQueries==4 && wall.edgeQueries==3, "retained route is checked without invoking path search");
    require(haxe.Json.stringify(good.positions)==before && haxe.Json.stringify(path)==before, "reuse preserves the path and caller data");
    var crossing=[from,to];
    require(SavedPathValidation.failure(wall,crossing,from,to) != null, "clear endpoints do not hide an interior collision");
    var repaired=TendingReachReuse.route(wall,crossing,from,to,timing);
    require(!repaired.reused && repaired.positions != null, "only the invalid route is replanned");
    require(SavedPathValidation.failure(wall,repaired.positions,from,to)==null, "replacement crosses through the independently defined gap");
    require(crossing.length==2, "repair does not rewrite the caller's proposal");
    var absent=TendingReachReuse.route(wall,null,from,to,timing);
    require(!absent.reused && absent.positions != null, "a missing route is planned");
    var f=materia.assembly.AssemblyFrames.identity(), malformed=TendingReachReuse.copyFrame(f);
    malformed.qw=2;
    require(!TendingReachReuse.sameFrame(f,malformed), "malformed quaternion is refused");
    var opposite=TendingReachReuse.copyFrame(f); opposite.qw=-1;
    require(TendingReachReuse.sameFrame(f,opposite), "quaternion sign does not change the target rotation");

    var changed=saved(); changed.trays[0].width+=1;
    refused(changed,"layout, fixtures");
    changed=saved(); changed.poses[0].position[0]+=0.1;
    refused(changed,"forward kinematics changed");
    changed=saved(); changed.poses[0]=changed.poses[1];
    refused(changed,"missing or duplicate pose");
    changed=saved();
    var count=changed.transfers.length;
    var removed:TendingReach.TendingReachTransfer = cast changed.transfers.shift();
    var repairedCell=TendingReach.revalidate(new EnclosedBenchMill(),TendingCyclePlan.bearingBlocks(),changed);
    require(repairedCell.transfers.length==count && TendingReachReuse.findRoute(repairedCell,removed.request)!=null,
      "the real cell repairs its missing route");
    for (retained in changed.transfers)
      require(haxe.Json.stringify(TendingReachReuse.findRoute(repairedCell,retained.request))==haxe.Json.stringify(retained.positions),
        "repair retains every unaffected real-cell route exactly");
    require(changed.transfers.length==count-1, "real-cell repair leaves its input certificate untouched");
    Sys.println('Tending reuse checks passed ($assertions assertions)');
  }
  static function refused(proposal:TendingReach.TendingReachResult, reason:String):Void {
    var caught=false;
    try TendingReach.revalidate(new EnclosedBenchMill(),TendingCyclePlan.bearingBlocks(),proposal) catch (error:Dynamic) {
      var message=Std.string(error);
      if (message.indexOf("Saved reach cannot be reused:")<0 || message.indexOf(reason)<0) throw error;
      caught=true;
    }
    require(caught,'invalid saved data must be refused: $reason');
  }
}
