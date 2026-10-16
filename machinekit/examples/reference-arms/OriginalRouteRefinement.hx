/** Connect reviewed transfer poses and front-entry IK poses with continuous checks. */
class OriginalRouteRefinement {
  final cache:Map<String,Dynamic> = [];
  final nodes:Map<String,Dynamic> = [];
  public var nodeChecks(default,null):Int = 0;
  public var edgeChecks(default,null):Int = 0;
  public var cacheHits(default,null):Int = 0;
  public var maxEdgeChecks:Int = 640;
  public function new() {}
  static function key(q:Array<Float>):String return [for(v in q) Std.string(v)].join(",");
  static function distance(a:Array<Float>,b:Array<Float>):Float {
    var value=0.0;for(i in 0...a.length) value+=(a[i]-b[i])*(a[i]-b[i]);return Math.sqrt(value);
  }
  function edge(door:String,worlds:Array<robotkit.collision.CollisionClearance>,a:Array<Float>,b:Array<Float>):Dynamic {
    var ak=key(a),bk=key(b),k=door+"/"+(ak<bk ? ak+"/"+bk : bk+"/"+ak);
    if(cache.exists(k)) {cacheHits++;return cache.get(k);}
    if(edgeChecks>=maxEdgeChecks) return {clear:false,error:"Roadmap edge budget exhausted"};
    for(q in [a,b]) {
      var nk=door+"/"+key(q);
      if(!nodes.exists(nk)) {
        var invalid:Dynamic=null;
        for(world in worlds) {nodeChecks++;invalid=world.violation(q);if(invalid!=null) break;}
        nodes.set(nk,{clear:invalid==null,violation:invalid,error:null});
      }
      var state=nodes.get(nk);
      if(state.clear!=true) {cache.set(k,state);return state;}
    }
    edgeChecks++;
    var proof=CellRegistration.curveReview(worlds,[a,b]);cache.set(k,proof);return proof;
  }
  public function refine(door:String,worlds:Array<robotkit.collision.CollisionClearance>,start:Array<Float>,end:Array<Float>,pool:Array<Array<Float>>):Dynamic {
    var before=edgeChecks,hitsBefore=cacheHits,seen:Map<String,Bool> = [];
    seen.set(key(start),true);seen.set(key(end),true);
    var nodes:Array<Array<Float>> = [];
    for(q in pool) if(!seen.exists(key(q))) {seen.set(key(q),true);nodes.push(q);}
    nodes.sort((a,b) -> {
      var delta=distance(start,a)+distance(a,end)-distance(start,b)-distance(b,end);
      return delta<0 ? -1 : delta>0 ? 1 : 0;
    });
    var fromStart:Array<Array<Float>> = [],toEnd:Array<Array<Float>> = [];
    for(q in nodes) {
      var a=edge(door,worlds,start,q),b=edge(door,worlds,q,end);
      if(a.clear==true) fromStart.push(q);
      if(b.clear==true) toEnd.push(q);
      if(a.clear==true && b.clear==true) return result(worlds,[start,q,end],before,hitsBefore,"one_reviewed_waypoint");
      if(edgeChecks>=maxEdgeChecks) break;
    }
    // Bridge two observed poses when neither alone connects the endpoints.
    var bridges:Array<{a:Array<Float>,b:Array<Float>,cost:Float}> = [];
    for(a in fromStart) for(b in toEnd) bridges.push({a:a,b:b,cost:distance(start,a)+distance(a,b)+distance(b,end)});
    bridges.sort((a,b) -> a.cost<b.cost ? -1 : a.cost>b.cost ? 1 : 0);
    for(pair in bridges.slice(0,64)) {
      if(edge(door,worlds,pair.a,pair.b).clear==true)
        return result(worlds,[start,pair.a,pair.b,end],before,hitsBefore,"two_reviewed_waypoints");
      if(edgeChecks>=maxEdgeChecks) break;
    }
    return {path:null,continuous:null,planner:{method:"observed_pose_roadmap",edgeChecks:edgeChecks-before,
      cacheHits:cacheHits-hitsBefore,fromStart:fromStart.length,toEnd:toEnd.length,
      failure:edgeChecks>=maxEdgeChecks ? "Roadmap edge budget exhausted" : "No bridge found within observed-pose roadmap"}};
  }
  function result(worlds:Array<robotkit.collision.CollisionClearance>,path:Array<Array<Float>>,before:Int,hitsBefore:Int,method:String):Dynamic {
    // Recheck the complete concatenated quintic in all four physical states.
    var proof=CellRegistration.curveReview(worlds,path);
    return {path:proof.clear ? path : null,continuous:proof,planner:{method:method,edgeChecks:edgeChecks-before,
      cacheHits:cacheHits-hitsBefore,failure:proof.clear ? null : "Concatenated route failed continuous recheck"}};
  }
}
