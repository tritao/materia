import cadkit.Shape;
import haxe.io.Bytes;
import materia.project.SceneArtifact;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinitionCodec;

/** Deliberately different geometry makes variant persistence observable. */
class VariantPreview {
 public static function small():Bytes return box(10);
 public static function large():Bytes return box(20);
 static function box(size:Float):Bytes {
  var shape=Shape.box(size,size,size);
  try {
   var mesh=shape.tessellateRelative();
   var bytes=SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,
    parts:[{id:"machine",name:"Fixture machine",red:0.5,green:0.5,blue:0.6,materialId:"machined-steel",
     vertexCount:mesh.vertexCount,indexCount:mesh.indexCount,vertices:mesh.vertices,normals:mesh.normals,indices:mesh.indices,
     faceRanges:[for(range in mesh.faceRanges){faceIndex:range.faceIndex,firstIndex:range.firstIndex,indexCount:range.indexCount}]}],
    assemblyDefinition:{schemaVersion:AssemblyDefinitionCodec.VERSION,id:"variant-fixture",lengthUnit:"mm",
     definitions:[{id:"machine",connectors:[]}],occurrences:[{id:"machine",definition:"machine",grounded:true,initialPose:AssemblyFrames.identity()}],joints:[]}});
   shape.close();return bytes;
  }catch(error:Dynamic){shape.close();throw error;}
 }
}

function main():Void {}
