import materia.tooling.casting.CastingMoldPlan;
import machinekit.tooling.CastingMoldCad.CastingMoldReview;

/** Phase-8 ownership and manual EPS route checks, using the fresh native review. */
class HobberMoldChecks {
 public static function supportRecords(planned:CastingMoldPlan):Void {
   var coreRecords=machinekit.tooling.ToolingFabricationExport.coreSupportRecords(planned.cores,planned.panels.members.concat(planned.supportPanels));
   if(coreRecords.length!=planned.cores.length)throw 'Every EPS unit needs a shop support record';
   for(record in coreRecords)if(Reflect.hasField(record,'withdrawal'))
    throw 'Retained and manually broken EPS must not claim an intact withdrawal: '+record.coreId;
 }

 public static function run(planned:CastingMoldPlan,review:CastingMoldReview):Void {
   if(review.symmetricDifferenceMm3>1e-3)throw 'Hobber native reconstruction differs from source';
   var cured=review.permanentGroups.filter(group->group.id=='cured-casting');
   if(cured.length!=1)throw 'Retained and later-broken EPS must belong to the checked cured composite';
   var map=new Map<String,Array<String>>();
   for(group in review.assemblyGroups.concat(review.permanentGroups))map.set(group.id,group.members);
   function leaves(id:String):Array<String>{
    var group=map.get(id);
    return group==null ? [id] : [for(member in group)for(leaf in leaves(member))leaf];
   }
   var installed=[for(motion in review.assemblyMotions)for(id in leaves(motion.id))id];
   for(core in planned.cores)for(id in materia.tooling.casting.MoldCores.assemblyMembers(core)){
    if(installed.filter(member->member==id).length!=1)throw 'EPS assembly stock must be checked exactly once: '+id;
    if(cured[0].members.indexOf(id)<0)throw 'EPS was omitted from the cured lifting solid: '+id;
   }
   for(strut in planned.document.input.precastStruts)
    if(installed.filter(member->member==strut.id).length!=1)throw 'Source strut omitted from assembly: '+strut.id;
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
   var retained=[for(core in planned.cores)if(core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.PrecastStruts)
    for(id in materia.tooling.casting.MoldCores.assemblyMembers(core))id];
   if(finalFrame.visibleMemberIds.length!=retained.length+1 || finalFrame.visibleMemberIds.indexOf('cured-casting')<0)
    throw 'Final playback must retain the casting and only its enclosed EPS';
   for(id in retained)if(finalFrame.visibleMemberIds.indexOf(id)<0)throw 'Retained EPS disappeared from playback: '+id;
   for(frame in playback)if(frame.phase==machinekit.tooling.MoldPlayback.MoldPlaybackPhase.Demolding)
    for(strut in planned.document.input.precastStruts)if(frame.visibleMemberIds.indexOf(strut.id)>=0)
     throw 'Precast compound must not appear twice after cure: '+strut.id;
   Sys.println('Hobber playback: manual bond release, breakup visibility, retained EPS motion and compound ownership pass');
   Sys.println('Hobber native reconstruction and both continuous removal graphs pass');
 }
}
