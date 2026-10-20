import haxe.io.Bytes;
import materia.project.SceneArtifact;
import materia.project.SceneArtifactProgram;
import app.ProjectArtifactLoader;
import app.ProjectScenePreparation;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinitionCodec;

class SheetMachiningTests {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
  static function part(id:String):materia.project.SceneArtifact.SceneArtifactPart {
    var p=Bytes.alloc(96),n=Bytes.alloc(96),ix=Bytes.alloc(48);
    p.setDouble(24,1);p.setDouble(56,1);p.setDouble(88,1);
    var indices=[0,2,1,0,1,3,0,3,2,1,2,3];for(i in 0...indices.length)ix.setInt32(i*4,indices[i]);
    return {id:id,name:id,red:0.5,green:0.5,blue:0.5,vertices:p,normals:n,indices:ix,vertexCount:4,indexCount:12,faceRanges:[]};
  }
  static function main():Void {
    var f=AssemblyFrames.identity();
    var source=Bytes.ofString("G55\nG0 X10\nM2\n");
    var job:materia.project.SceneArtifact.SceneArtifactMachining={programFile:"programs/doors.nc",programSha256:SceneArtifactProgram.digest(source),
      axes:["x","y","z"],spindle:"spindle",channelScope:"router",workOffset:[0,0,0],workOffsets:[{number:55,offset:[0.1,0.2,0.3]}],stockSpacing:0.001,
      tools:[{number:1,length:0.12,profile:[[0.0,0.0,0.0,0.0,0.003,0.0],[0.0,0.0,0.003,0.0,0.003,0.02]]}],stocks:["blank"],
      targets:[{id:"door",part:"finished",stock:"blank",position:[0.1,0.2,0.3],angle:Math.PI/2}]};
    var artifact:materia.project.SceneArtifact.SceneArtifactData={lengthUnit:"mm",metresPerUnit:0.001,
      parts:[part("raw"),part("finished")],machining:job,assemblyDefinition:{schemaVersion:AssemblyDefinitionCodec.VERSION,id:"router",lengthUnit:"mm",
        definitions:[{id:"raw",connectors:[]}],occurrences:[{id:"blank",definition:"raw",initialPose:f}],joints:[]}};
    var requested="";
    var loaded= @:privateAccess ProjectArtifactLoader.machiningJob(artifact,[],0.001,p->{requested=p;return source;});
    if(loaded==null)throw "missing loaded machining job";
    check(requested=="programs/doors.nc"&&loaded.source==source.toString(),"loader resolves the exact hashed program dependency");
    check(loaded.workOffsets[0].number==55&&loaded.workOffsets[0].offset[1]==0.2&&loaded.stockSpacing==0.001,"work offsets and stock pitch reach the CNC job");
    var target=loaded.targets[0];
    check(target.id=="door"&&target.stock=="blank","per-part identity and stock association survive loading");
    var found=false;
    for(i in 0...Std.int(target.positions.length/3))if(Math.abs(target.positions[i*3]-0.1)<1e-10&&Math.abs(target.positions[i*3+1]-0.201)<1e-10&&Math.abs(target.positions[i*3+2]-0.3)<1e-10)found=true;
    check(found,"target rotation acts after CAD-to-SI scaling and before SI translation");
    check(ProjectScenePreparation.components(artifact).length==1&&ProjectScenePreparation.components(artifact)[0].id=="raw","host and browser prepare physical components without target bodies");
    Sys.println('Sheet machining checks passed ($assertions assertions)');
  }
}
