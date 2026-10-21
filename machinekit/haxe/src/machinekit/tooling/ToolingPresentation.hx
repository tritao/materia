package machinekit.tooling;

import materia.assembly.AssemblyDefinition;
import materia.presentation.AssemblyPresentation;
import materia.presentation.PresentationGeometry;
import materia.presentation.Procedures;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.tooling.casting.InsertFixturePlanner.InsertPlatePlan;

/** Adapts planner order and world withdrawal directions; the viewer never sorts the steps. */
class ToolingPresentation {
  public static function socket(plan:InsertPlatePlan,artifact:SceneArtifactData,
      review:InsertFixtureCad.InsertFixtureReview,retainedScrewVolume:Float):AssemblyPresentation {
    var definition=artifact.assemblyDefinition;if(definition==null)throw "Tooling presentation requires an assembly";
    var result=SocketFixtureCad.presentation(plan,definition),fingerprint=PresentationGeometry.fingerprint(artifact);
    var travel=review.removal.travelMm,displacements=new Map<String,materia.assembly.AssemblyDefinition.AssemblyVector>();
    for(direction in review.directions)displacements.set(direction.id,{x:direction.x*travel,y:direction.y*travel,z:direction.z*travel});
    var offsets=ToolingExplode.presentation(definition,"swept-removal",displacements).explode;
    var depths=new Map<String,Int>(),visiting=new Map<String,Bool>();
    function depth(id:String):Int {
      var known=depths.get(id);if(known!=null)return known;
      if(visiting.exists(id))throw "Cyclic tooling withdrawal dependencies";
      visiting.set(id,true);var value=1;
      for(edge in review.removal.blocking)if(edge.movingId==id)value=Std.int(Math.max(value,1+depth(edge.blockerId)));
      visiting.remove(id);depths.set(id,value);return value;
    }
    for(offset in offsets)offset.stage=depth(offset.select.occurrence);
    result.explode=offsets;
    var release:Array<PartSelector> =[for(id in review.releaseBeforeTranslation){occurrence:id,subtree:false}];
    var steps:Array<ProcedureStep> = [
      {id:"cure",title:"Confirm cure and handling",view:"assembled",instruction:"Confirm cure and the required handling/process evidence before demoulding.",highlight:[],action:Note},
      {id:"release-threads",title:"Release the threaded fasteners",instruction:"Unscrew the highlighted fasteners before translating any tooling. Thread release is a manual operation.",highlight:release,action:Note}
    ];
    for(id in review.removal.removalOrder) {
      var matches=offsets.filter(offset->offset.select.occurrence==id);if(matches.length!=1)throw 'Missing planned withdrawal "$id"';
      var offset=matches[0];
      steps.push({id:"withdraw-"+id,title:"Withdraw "+id,instruction:"Withdraw "+id+" along the checked straight path.",
        view:"assembled",highlight:[offset.select],action:Remove(offset.select,{path:[{x:0,y:0,z:0},
          {x:offset.direction.x*travel,y:offset.direction.y*travel,z:offset.direction.z*travel}],
          evidence:Checked("swept-removal",Passed,fingerprint,haxe.Json.stringify(review.removal))})});
    }
    var disassembly:Procedure={id:"demould",label:"Demould the socket fixture",kind:Disassembly,start:Assembled,steps:steps,source:Generated("swept-removal")};
    var assembly=Procedures.reverse(disassembly,"assemble","Assemble the socket fixture",[]);
    for(step in assembly.steps)if(step.id=="release-threads") {
      step.title="Complete threaded fixings";step.instruction="Engage and tighten the threaded fixings to their approved specifications.";
    } else if(step.id=="cure") {
      step.title="Confirm tooling readiness";step.instruction="Confirm the required closure, handling and process evidence before pouring.";
    }
    result.procedures=[disassembly,assembly];
    if(retainedScrewVolume<=review.removal.contactVolumeToleranceMm3)throw "Negative control must have a measured obstruction";
    var plate=plan.assembly.members[1].panel.id,offset=offsets.filter(value->value.select.occurrence==plate)[0];
    result.procedures.push({id:"screws-left-fitted",label:"Screws left fitted (negative control)",kind:Disassembly,start:Assembled,source:Generated("swept-removal"),steps:[
      {id:"blocked-plate",title:"Plate withdrawal is blocked",instruction:"The screw heads obstruct this withdrawal. Release the screws before removing the plate.",
        view:"socket-mounting",highlight:[offset.select],action:Remove(offset.select,{path:[{x:0,y:0,z:0},
          {x:offset.direction.x*travel,y:offset.direction.y*travel,z:offset.direction.z*travel}],
          evidence:Checked("swept-removal",Failed,fingerprint,haxe.Json.stringify({retainedScrewIntersectionMm3:retainedScrewVolume,retained:review.releaseBeforeTranslation}))})}
    ]});
    materia.presentation.PresentationCodec.validate(definition,result);return result;
  }
  static function assemble(procedure:Procedure,id:String,label:String):Procedure {
    var result=Procedures.reverse(procedure,id,label,[]);
    for(step in result.steps)switch step.action {
      case Note:
        if(StringTools.startsWith(step.id,"release-")) {
          step.title="Secure tooling and protection";
          step.instruction="Secure the declared fixings, bonds and threaded protection to the approved process before pouring.";
        } else if(step.id=="flip-to-service") {
          step.title="Set the assembly orientation";
          step.instruction="Set the tooling assembly orientation using the approved handling plan.";
        } else if(step.id=="withdraw-cured-casting") {
          step.title="Confirm the casting setup";
          step.instruction="Confirm the empty cavity and permanent insert setup before assembling the surrounding tooling.";
        }
      case _:
    }
    return result;
  }

  /** Shared adapter for flat parts and rigid groups already verified together by SweptRemoval. */
  static function planned(artifact:SceneArtifactData,result:AssemblyPresentation,report:SweptRemoval.RemovalReport,
      directions:Array<{id:String,x:Float,y:Float,z:Float}>,groups:Array<{id:String,members:Array<String>}>,
      id:String,label:String,semantic:Array<ProcedureStep>):AssemblyPresentation {
    var definition=artifact.assemblyDefinition;if(definition==null)throw "Tooling presentation requires an assembly";
    var fingerprint=PresentationGeometry.fingerprint(artifact),travel=report.travelMm;
    var resolved=new Map<String,Array<String>>(),deltas=new Map<String,materia.assembly.AssemblyDefinition.AssemblyVector>();
    for(direction in directions) {
      var parts=[direction.id];
      for(group in groups)if(group.id==direction.id)parts=group.members.copy();
      // The cured body is a check solid, not an occurrence in the tooling-only export.
      if(direction.id=="cured-casting")parts=[];
      resolved.set(direction.id,parts);
      for(part in parts) {
        if(deltas.exists(part))throw "Tooling movement belongs to multiple checked groups: "+part;
        deltas.set(part,{x:direction.x*travel,y:direction.y*travel,z:direction.z*travel});
      }
    }
    result.explode=ToolingExplode.presentation(definition,"swept-removal",deltas).explode;
    var depths=new Map<String,Int>(),visiting=new Map<String,Bool>();
    function depth(member:String):Int {
      var known=depths.get(member);if(known!=null)return known;
      if(visiting.exists(member))throw "Cyclic tooling withdrawal dependencies";
      visiting.set(member,true);var value=1;
      for(edge in report.blocking)if(edge.movingId==member)value=Std.int(Math.max(value,1+depth(edge.blockerId)));
      visiting.remove(member);depths.set(member,value);return value;
    }
    var byPart=new Map<String,ExplodeOffset>();for(offset in result.explode)byPart.set(offset.select.occurrence,offset);
    var steps=semantic.copy();
    for(member in report.removalOrder) {
      var parts=resolved.get(member);if(parts==null)throw "Missing checked tooling withdrawal direction: "+member;
      var motions:Array<PartMotion> = [];
      for(part in parts) {
        var offset=byPart.get(part);if(offset==null)throw "Missing tooling withdrawal offset: "+part;
        offset.stage=depth(member);
        motions.push({select:offset.select,motion:{path:[{x:0,y:0,z:0},
          {x:offset.direction.x*offset.distance,y:offset.direction.y*offset.distance,z:offset.direction.z*offset.distance}],
          evidence:Checked("swept-removal",Passed,fingerprint,haxe.Json.stringify({member:member,parts:parts,travelMm:travel,contactVolumeToleranceMm3:report.contactVolumeToleranceMm3,blockers:[for(edge in report.blocking)if(edge.movingId==member)edge.blockerId]}))}});
      }
      var action:StepAction=motions.length==0?Note:motions.length==1?Remove(motions[0].select,motions[0].motion):RemoveGroup(motions);
      steps.push({id:"withdraw-"+member,title:"Withdraw "+member,
        instruction:motions.length==0?"Remove the cured casting after the surrounding tooling is released. The casting check solid is not included in this tooling scene.":
          "Withdraw the highlighted tooling together along the checked straight path.",
        highlight:[for(motion in motions)motion.select],action:action});
    }
    var procedure:Procedure={id:id,label:label,kind:Disassembly,start:Assembled,steps:steps,source:Generated("swept-removal")};
    result.procedures=[procedure,assemble(procedure,id+"-assembly","Assemble tooling")];
    materia.presentation.PresentationCodec.validate(definition,result);return result;
  }

  public static function box(plan:materia.tooling.casting.MoldPlan,artifact:SceneArtifactData,
      review:BoxMoldCad.BoxMoldReview):AssemblyPresentation {
    return planned(artifact,BoxMoldCad.presentation(plan,artifact.assemblyDefinition),review.demolding,
      review.demoldingMotions,review.rigidGroups,"demould","Demould the box mould",[
        {id:"release-fixings",title:"Release tooling fixings",instruction:"Release loose fasteners and the declared adhesive bonds before straight withdrawal. EPS breakup is a manual operation.",highlight:[],action:Note}
      ]);
  }

  public static function casting(plan:materia.tooling.casting.CastingMoldPlan,artifact:SceneArtifactData,
      review:CastingMoldCad.CastingMoldReview):AssemblyPresentation {
    var result=planned(artifact,CastingMoldCad.presentation(plan,artifact.assemblyDefinition),review.demolding,
      review.demoldingMotions,review.permanentGroups,"demould","Demould the casting mould",[]);
    var procedure=result.procedures[0],withdrawals=new Map<String,ProcedureStep>();
    for(step in procedure.steps)withdrawals.set(step.id,step);
    var steps:Array<ProcedureStep> = [];
    // The planner has already ordered the semantic dependencies and withdrawals.
    for(plannedStep in review.demoldingSteps) {
      var withdrawal=withdrawals.get(plannedStep.id);
      if(withdrawal!=null){steps.push(withdrawal);continue;}
      var highlights:Array<PartSelector> = [];
      for(member in plannedStep.memberIds)if(member!="cured-casting")highlights.push({occurrence:member,subtree:false});
      steps.push({id:plannedStep.id,title:plannedStep.id,
        instruction:plannedStep.kind==CastingMoldCad.DemoldingStepKind.FlipToService?
          "Flip the cured casting using the qualified handling plan. This is a manual handling operation.":
          "Release the highlighted loose hardware and threaded protection before withdrawing tooling. This is a manual operation.",
        highlight:highlights,action:Note});
    }
    procedure.steps=steps;result.procedures=[procedure,assemble(procedure,"assemble","Assemble the casting mould")];
    materia.presentation.PresentationCodec.validate(artifact.assemblyDefinition,result);return result;
  }

}
