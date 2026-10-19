/** Electric route binding. Guarded edges remain explicit and cannot enter ordinary moveVia playback. */
class TendingElectricRuntimeProfile {
  public static function same(a:Array<Float>,b:Array<Float>,tolerance:Float=1e-8):Bool {
    if(a.length!=6 || b.length!=6)return false;
    for(j in 0...6)if(!Math.isFinite(a[j]) || !Math.isFinite(b[j]) || Math.abs(a[j]-b[j])>tolerance)return false;
    return true;
  }
  public static function bind(row:Dynamic,raw:Dynamic):Dynamic {
    if(row.transfer.found!=true || row.approachStateTransferClear!=true || row.withdrawal.freeEdgesClear!=true || row.jawStroke.clear!=true)
      throw "Electric runtime requires validated access, withdrawal and jaw stroke";
    if(row.constrainedApproachClear!=true && row.requiresGuardedContact!=true)throw "Approach is blocked";
    var source:Array<Array<Float>> = cast raw.routeQ,offsets:Array<Float> = cast row.jointTurnOffsetsRad;
    if(source.length<2 || offsets.length!=6)throw "Missing electric joint path";
    var approach:Array<Array<Float>> = [];
    for(q in source) {
      if(q.length!=6)throw "Electric waypoint needs six joints";
      var adjusted=[for(j in 0...6)q[j]+offsets[j]];
      if(!same(adjusted,adjusted))throw "Nonfinite electric joint target";
      approach.push(adjusted);
    }
    var transfer:Array<Array<Float>> = cast row.transfer.waypoints;
    for(q in transfer)if(!same(q,q))throw "Invalid transfer waypoint";
    if(transfer.length<2 || !same(transfer[transfer.length-1],approach[0]))throw "Transfer/approach discontinuity";
    var incoming:Array<Dynamic> = [],outgoing:Array<Dynamic> = [];
    function edge(a:Array<Float>,b:Array<Float>,guarded:Bool,separating:Bool):Dynamic return {
      kind:guarded ? "guardedJointEdge" : "jointEdge",from:a.copy(),to:b.copy(),
      contactPolicy:guarded ? row.guardLimits : null,separating:separating,
      stockOwner:separating ? row.withdrawal.stockOwner : row.activeStockOwner,
      jawOpeningMm:separating ? row.withdrawal.jawOpeningMm : row.jawOpeningMm};
    for(i in 1...transfer.length)incoming.push(edge(transfer[i-1],transfer[i],false,false));
    for(i in 1...approach.length)incoming.push(edge(approach[i-1],approach[i],
      row.requiresGuardedContact==true && i-1>=Std.int(row.failedApproachEdge),false));
    var guarded:Array<Int> = cast row.withdrawal.guardedEdges;
    for(i in 1...approach.length) {
      var end=approach.length-i;
      outgoing.push(edge(approach[end],approach[end-1],guarded.indexOf(end)>=0,true));
    }
    for(i in 1...transfer.length) {
      var end=transfer.length-i;outgoing.push(edge(transfer[end],transfer[end-1],false,true));
    }
    for(edges in [incoming,outgoing])for(i in 1...edges.length)
      if(!same(edges[i-1].to,edges[i].from))throw "Electric edge discontinuity";
    if(!same(incoming[incoming.length-1].to,outgoing[0].from) || !same(outgoing[outgoing.length-1].to,transfer[0]))throw "Electric round trip does not close";
    var locating:Dynamic=null;
    if(Reflect.hasField(row,"locating") && row.locating!=null) {
      if(row.station!="vise-load" || row.locating.commandCorridorClear!=true || row.locating.returnCorridorClear!=true || row.locating.returnPolicy!="reverse executed search samples")
        throw "Locating needs certified command and open-jaw return corridors";
      var samples:Array<Array<Float>> = cast raw.locatingQ;
      if(samples==null || samples.length<2)throw "Missing locating command samples";
      for(q in samples)if(!same(q,q))throw "Locating sample needs six finite arm coordinates";
      locating={samples:[for(q in samples)[for(j in 0...6)q[j]+offsets[j]]],limits:row.locating};
      var bound:Array<Array<Float>> = cast locating.samples;
      if(!same(bound[0],incoming[incoming.length-1].to))throw "Locating/insertion discontinuity";
    }
    var placing:Dynamic=null;
    if(Reflect.hasField(row.guardLimits,"beforeRelease") && row.guardLimits.beforeRelease!=null) {
      var policy:Dynamic=row.guardLimits.beforeRelease,t:Float=policy.entryFraction;
      if(!StringTools.startsWith(Std.string(row.station),"outfeedTray-") || !Math.isFinite(t) || t<0 || t>=1 || policy.returnCorridorClear!=true)
        throw "Support placement requires a reviewed final-segment subset and return";
      var last:Dynamic=incoming[incoming.length-1],a:Array<Float> = cast last.from,b:Array<Float> = cast last.to;
      var entry=[for(j in 0...6)a[j]+t*(b[j]-a[j])],prefix=incoming.slice(0,incoming.length-1);
      if(t>0)prefix.push(edge(a,entry,false,false));
      var returning=outgoing.copy(),first:Dynamic=outgoing[0];
      if(!same(cast first.from,b) || !same(cast first.to,a))throw "Support approach/return segment differs";
      returning[0]=edge(entry,a,first.kind=="guardedJointEdge",true);
      placing={limits:policy,samples:[entry,b],incoming:prefix,outgoing:returning};
    }
    return {caseReference:Std.string(row.station)+"/"+Std.string(row.side),incoming:incoming,outgoing:outgoing,
      jaw:{openPerJawMm:25,graspPerJawMm:17,requiresGraspConfirmationBeforeUnclamp:true,
        requiresSupportConfirmationAfterRelease:row.activeStockOwner=="tool"},
      seating:row.activeStockOwner=="tool" ? row.guardLimits.afterRelease : null,locating:locating,placing:placing,
      registration:Reflect.hasField(row.guardLimits,"beforePickup") ? row.guardLimits.beforePickup : null,
      clamping:row.station=="vise-load" ? row.guardLimits.afterClamp : null,
      doorPolicy:StringTools.startsWith(Std.string(row.station),"vise-") ? "confirmed open" : "confirmed closed",
      park:transfer[0].copy(),simulationOnly:true,ordinaryMissionCompatible:false,
      stopPolicy:"latched stop; retain payload and jaw state; inhibit subsequent operations; no automatic retry"};
  }
  public static function ordinaryWaypoints(profile:Dynamic):Array<Array<Float>> {
    if(Reflect.hasField(profile,"registration") && profile.registration!=null)throw "Observed grasp registration cannot enter ordinary waypoint playback";
    if(profile.seating!=null)throw "Electric placement needs post-release support feedback";
    var edges:Array<Dynamic> = cast profile.incoming;
    for(edge in edges)if(edge.kind=="guardedJointEdge")throw "Guarded electric route needs a contact-feedback executor";
    return [for(edge in edges)cast edge.to];
  }
}
