import materia.tooling.casting.CastingMoldPlan;
import machinekit.tooling.CastingMoldCad.CastingMoldReview;

/** Phase-8 ownership and manual EPS route checks, using the fresh native review. */
class HobberMoldChecks {
 /** Source geometry can prove the lift polygon and vertical demand, but cannot
  * certify bought lifting equipment or invent its permitted sling angle. */
 public static function liftingReview(planned:CastingMoldPlan):Array<materia.tooling.ToolingValidation> {
  var ids=materia.tooling.casting.CastHandling.liftingIds(planned.document);
  var inverse=materia.assembly.AssemblyFrames.inverse(planned.document.setup.sourceToMold);
  var center=materia.assembly.AssemblyFrames.transformPoint(inverse,
   planned.handling.centerOfMassMoldMm.x,planned.handling.centerOfMassMoldMm.y,planned.handling.centerOfMassMoldMm.z);
  var points:Array<{id:String,x:Float,y:Float}>=[];
  for(id in ids) {
   var insert=planned.document.input.inserts.filter(insert->insert.id==id)[0];
   var occupied=materia.casting.InsertAnchorage.InsertAnchorageCodec.occupiedHoleIds(insert.anchorage);
   var holes=materia.tooling.casting.InsertFixturePlanner.holes(insert)
    .filter(hole->hole.thread!=null && occupied.indexOf(hole.id)<0);
   var entry=materia.assembly.AssemblyFrames.compose(insert.pose,holes[0].frame);
   points.push({id:id,x:entry.x,y:entry.y});
  }
  points.sort((a,b)->Reflect.compare(Math.atan2(a.y-center.y,a.x-center.x),Math.atan2(b.y-center.y,b.x-center.x)));
  var inside=points.length>=3;
  for(index in 0...points.length) {
   var a=points[index],b=points[(index+1)%points.length];
   if((b.x-a.x)*(center.y-a.y)-(b.y-a.y)*(center.x-a.x)<=1e-7)inside=false;
  }
  var checks=[materia.tooling.export.ToolingLoads.check("tooling.handling.lift-polygon",ids,
   inside ? materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass : materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail,
   inside ? "Source casting CoM lies strictly inside the declared attachment polygon in the upright lifting projection."
    : "Source casting CoM does not lie strictly inside the declared attachment polygon.",[])];
  var rule=materia.casting.InsertAnchorage.InsertAnchorSelection.sourceDemandRule("hobber-source-anchors");
  var vertical=planned.handling.castingMassKg*rule.gravityMps2*rule.demandFactor/Math.min(rule.liftingLoadPaths,points.length);
  var x0=Math.POSITIVE_INFINITY,x1=Math.NEGATIVE_INFINITY,y0=Math.POSITIVE_INFINITY,y1=Math.NEGATIVE_INFINITY;
  for(point in points){x0=Math.min(x0,point.x);x1=Math.max(x1,point.x);y0=Math.min(y0,point.y);y1=Math.max(y1,point.y);}
  var rectangle=points.length==4 && x1>x0 && y1>y0;
  for(point in points)
   if(Math.min(Math.abs(point.x-x0),Math.abs(point.x-x1))>1e-7
    || Math.min(Math.abs(point.y-y0),Math.abs(point.y-y1))>1e-7)rectangle=false;
  var shares:Array<String>=[];
  if(rectangle && inside) {
   var fx=(center.x-x0)/(x1-x0),fy=(center.y-y0)/(y1-y0);
   var force=0.0,mx=0.0,my=0.0,weight=planned.handling.castingMassKg*rule.gravityMps2;
   for(point in points) {
    var load=weight*(Math.abs(point.x-x0)<1e-7 ? 1-fx : fx)*(Math.abs(point.y-y0)<1e-7 ? 1-fy : fy);
    force+=load;mx+=load*point.x;my+=load*point.y;
    shares.push(point.id+"="+load+" N vertical (equalized nominal)");
   }
   if(Math.abs(force-weight)>weight*1e-8 || Math.abs(mx-weight*center.x)>weight*Math.max(1,Math.abs(center.x))*1e-8
    || Math.abs(my-weight*center.y)>weight*Math.max(1,Math.abs(center.y))*1e-8)
    throw "Hobber lifting reactions do not balance casting weight and moments";
  }
  checks.push(materia.tooling.export.ToolingLoads.check("tooling.handling.load-share",ids,
   rectangle && inside ? materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending
    : materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
   rectangle && inside ? shares.join("; ")+". These vertical reactions balance weight and CoM moments. "
    +"Actual four-leg sharing requires declared equalizing rigging; the source anchors retain the conservative two-loaded-point demand."
    : "The example load-share calculation requires four rectangular, CoM-enclosing source attachment positions.",[]));

  checks.push(materia.tooling.export.ToolingLoads.check("tooling.handling.point-ratings",ids,
   materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending,
   "Conservative two-loaded-point vertical demand is "+vertical+" N per attachment, including the source demand factor. "
    +"Declare each lifting attachment's rated load and the equalizing/load-sharing rigging before comparing individual capacities; thread strength is not a lifting rating.",[]));
  checks.push(materia.tooling.export.ToolingLoads.check("tooling.handling.sling-angles",ids,
   materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending,
   "Declare suspension geometry and the bought lifting hardware's permitted sling angle. Inclined-leg tension is vertical load / cos(angle from vertical); no angle or equipment limit is assumed.",[]));
  return checks;
 }

 public static function supportRecords(planned:CastingMoldPlan):Void {
   var coreRecords=machinekit.tooling.ToolingFabricationExport.coreSupportRecords(planned.cores,planned.panels.members.concat(planned.supportPanels));
   if(coreRecords.length!=planned.cores.length)throw 'Every EPS unit needs a shop support record';
   for(index in 0...coreRecords.length) {
    var record=coreRecords[index],manual=planned.cores[index].support.breakoutAfterLift;
    if(manual==Reflect.hasField(record,'withdrawal'))
     throw 'EPS shop removal must match its checked/manual route: '+record.coreId;
   }
 }

 public static function run(planned:CastingMoldPlan,review:CastingMoldReview):Void {
   for(check in liftingReview(planned))
    if(check.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail)throw check.rule+": "+check.message;
   if(review.symmetricDifferenceMm3>1e-3)throw 'Hobber native reconstruction differs from source';
   var cured=review.permanentGroups.filter(group->group.id=='cured-casting');
   var lifted=planned.cores.filter(core->core.support.breakoutAfterLift);
   if(cured.length!=(lifted.length>0 ? 1 : 0))throw 'Manually broken EPS must belong to the checked cured composite';
   var map=new Map<String,Array<String>>();
   for(group in review.assemblyGroups.concat(review.permanentGroups))map.set(group.id,group.members);
   function leaves(id:String):Array<String>{
    var group=map.get(id);
    return group==null ? [id] : [for(member in group)for(leaf in leaves(member))leaf];
   }
   var installed=[for(motion in review.assemblyMotions)for(id in leaves(motion.id))id];
   for(core in planned.cores)for(id in materia.tooling.casting.MoldCores.assemblyMembers(core)){
    if(installed.filter(member->member==id).length!=1)throw 'EPS assembly stock must be checked exactly once: '+id;
    var inCured=cured.length==1 && cured[0].members.indexOf(id)>=0;
    if(inCured!=core.support.breakoutAfterLift)throw 'Cured lifting ownership differs from the EPS removal route: '+id;
   }
   var breakup=review.demoldingSteps.filter(step->step.kind==machinekit.tooling.CastingMoldCad.DemoldingStepKind.BreakoutEPS);
   if(breakup.length!=planned.cores.filter(core->core.support.breakoutAfterLift).length
    || breakup.filter(step->step.status!=materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending).length!=0)
    throw 'Sacrificial breakup must be an explicit unqualified manual step after lifting';
   supportRecords(planned);
   var memberIds=[for(member in machinekit.tooling.CastingMoldCad.assembly(planned).components())member.id];
   var playback=machinekit.tooling.MoldPlayback.frames(planned,review,memberIds);
   var bondRelease=playback.filter(frame->frame.action==machinekit.tooling.MoldPlayback.MoldPlaybackAction.ReleaseCoreBonds);
   if(bondRelease.length!=1 || bondRelease[0].status!=materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending)
    throw 'EPS bond release must stay one explicit manual Pending step';
   for(id in bondRelease[0].activeMemberIds)if(bondRelease[0].visibleMemberIds.indexOf(id)<0)
    throw 'Releasing an EPS bond must not remove the EPS: '+id;
   var breakoutFrames=playback.filter(frame->frame.action==machinekit.tooling.MoldPlayback.MoldPlaybackAction.BreakoutCore);
   if(breakoutFrames.length!=breakup.length)throw 'Every manual EPS breakup needs its own playback snapshot';
   for(frame in breakoutFrames)for(id in frame.activeMemberIds)if(frame.visibleMemberIds.indexOf(id)>=0)
    throw 'Manual breakup must remove the sacrificial EPS: '+id;
   var finalFrame=playback[playback.length-1];
   if(finalFrame.visibleMemberIds.length!=1 || finalFrame.visibleMemberIds[0]!='cured-casting')
    throw 'All sacrificial EPS must be removed in the final playback';
   Sys.println('Hobber playback: checked stock installation, manual bond release, EPS breakup and cured ownership pass');
   Sys.println('Hobber native reconstruction and both continuous removal graphs pass');
 }
}
