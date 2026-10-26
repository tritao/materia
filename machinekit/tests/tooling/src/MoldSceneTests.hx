import materia.project.SceneArtifact;
import materia.project.SceneSourceMap.SceneSourceMaps;
import materia.tooling.casting.CastingMoldPlanner;
import machinekit.tooling.CastingMoldCad;
import machinekit.tooling.CastingMoldExport;

/** Real source/scene round trips; no kit export or full review in preview generation. */
class MoldSceneTests {
 static var assertions=0;
 static function check(value:Bool,message:String):Void {assertions++;if(!value)throw message;}
 static function main():Void {
  if(Sys.args().indexOf("--compact")>=0) {
   var capacity=new machinekit.milling.MillCapacityLayout();
   var frame=BenchMillFrame.create(capacity.frame,capacity.zLayout());
   CastingMoldPlanner.setGeometry(new machinekit.tooling.NativeCastingMoldKernel());
   var plan=CastingMoldPlanner.plan(MillMoldSetup.document(frame),MillMoldSetup.options(frame));
   check(plan.document.input.id=="bench-mill-base","Compact VMC uses the same source adapter");
   check(plan.panels.members.length>0,"Compact VMC plans with the shared setup");
   check(plan.document.input.envelope[0].height==frame.layout.railPlane,"Compact VMC keeps its source height");
  } else {
   var frame=MillProject.smallFrame();
   for(casting in [MillMoldSetup.MillMoldCasting.Base,MillMoldSetup.MillMoldCasting.Column]) {
    var document=MillMoldSetup.document(frame,CastingShopRecipe.BOARD_THICKNESS_MM,casting);
    var plan=CastingMoldCad.plan(document,MillMoldSetup.options(frame,casting));
    var data=SceneArtifact.decode(CastingMoldExport.artifact(plan));
    var project:materia.project.SceneArtifact.SceneArtifactProject=cast data.project;
    check(project!=null,"Mold carries source metadata");
    var map:materia.project.SceneSourceMap=cast project.sources;
    check(map!=null,"Mold carries a source map");
    var definition:materia.assembly.AssemblyDefinition=cast data.assemblyDefinition;
    check(map.members.length==definition.occurrences.length,"Exactly one provenance record per occurrence");
    for(occurrence in definition.occurrences) {
     check(occurrence.includePath!=null && occurrence.includePath!="","Every member has exactly one role group");
     var member=map.members.filter(member->member.occurrence==occurrence.id);
     check(member.length==1,"Role ownership does not duplicate physical occurrences");
     check(member[0].shared || member[0].sources.length>0,"Every member names sources or shared hardware");
     for(id in member[0].sources)
      check(SceneSourceMaps.tooling(map,id).indexOf(occurrence.id)>=0,"Source to tooling round trip: "+id);
    }
    for(feature in document.input.features) {
     var source=map.features.filter(source->source.id==feature.id);
     check(source.length==1,"Every source feature survives the scene: "+feature.id);
     check(SceneSourceMaps.tooling(map,feature.id).length>0,"Every source feature resolves its forming tooling: "+feature.id);
     check(source[0].faces.length>0,"Actual cavity faces are highlightable: "+feature.id);
    }
    for(former in plan.formers)for(id in former.featureIds)
     check(SceneSourceMaps.tooling(map,id).indexOf(former.volume.id)>=0,"Former names its source intent");
    for(unit in plan.coreUnits)for(id in unit.featureIds)
     check(SceneSourceMaps.tooling(map,id).length>0,"Every EPS source feature has forming tooling");
    var again=SceneArtifact.decode(SceneArtifact.encode(data));
    var againProject:materia.project.SceneArtifact.SceneArtifactProject=cast again.project;
    check(haxe.Json.stringify(againProject.sources)==haxe.Json.stringify(map),"Source map survives a second round trip");
   }
  }
  Sys.println("MoldSceneTests: "+assertions+" assertions passed");
 }
}
