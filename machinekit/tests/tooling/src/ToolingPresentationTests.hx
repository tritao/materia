import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyFrames as F;
import materia.presentation.PresentationEvaluator;
import machinekit.tooling.ToolingExplode;

class ToolingPresentationTests {
  public static function main():Void {
    var rotation=F.fromRotationMatrix(0,0,0,[0,-1,0,1,0,0,0,0,1]);
    var definition:AssemblyDefinition={schemaVersion:materia.assembly.AssemblyDefinitionCodec.VERSION,id:"rotated-review",definitions:[{id:"part",connectors:[{name:"seat",frame:F.identity()}]}],
      occurrences:[{id:"base",definition:"part",initialPose:rotation},{id:"cover",definition:"part",initialPose:F.translation(30,0,0)}],
      joints:[{id:"fixed",type:"fixed",role:"tree",parent:"base",parentConnector:"seat",child:"cover",childConnector:"seat",
        axis:{x:0,y:0,z:1},defaultValue:0,limits:{lower:null,upper:null,velocity:null,effort:null}}]};
    var offsets=new Map<String,materia.assembly.AssemblyDefinition.AssemblyVector>();offsets.set("cover",{x:10,y:0,z:0});
    var record=ToolingExplode.presentation(definition,"test",offsets),cursor=PresentationEvaluator.reset(record);
    var solved=new Map<String,materia.assembly.AssemblyRecord.AssemblyFrame>();
    solved.set("base",rotation);solved.set("cover",F.translation(30,0,0));cursor.explode=1;
    var moved=PresentationEvaluator.evaluate(definition,solved,record,cursor,"").poses.get("cover");
    if(moved==null||Math.abs(moved.x-40)>1e-8||Math.abs(moved.y)>1e-8)throw "Review offsets must be expressed in the rotated joint parent frame";
    var nested:AssemblyDefinition={schemaVersion:materia.assembly.AssemblyDefinitionCodec.VERSION,id:"nested-review",definitions:[],joints:[],
      occurrences:[{id:"unit",definition:"sub",assembly:"sub",initialPose:rotation}],
      assemblies:[{id:"sub",definitions:[{id:"part",connectors:[]}],occurrences:[{id:"cover",definition:"part",initialPose:F.translation(30,0,0)}],joints:[]}]};
    offsets=new Map();offsets.set("unit/cover",{x:10,y:0,z:0});
    record=ToolingExplode.presentation(nested,"test",offsets);cursor=PresentationEvaluator.reset(record);cursor.explode=1;
    solved=new Map();solved.set("unit/cover",F.compose(rotation,F.translation(30,0,0)));
    moved=PresentationEvaluator.evaluate(nested,solved,record,cursor,"").poses.get("unit/cover");
    if(moved==null||Math.abs(moved.x-10)>1e-8||Math.abs(moved.y-30)>1e-8)throw "Nested review offsets must use the structural parent frame";
    Sys.println("Tooling presentation: rotated joint and nested parent checks passed");
    PanelNativeTests.run();
    BoxMoldTests.run();
  }
}
