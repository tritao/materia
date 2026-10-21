package materia.presentation;

import materia.project.SceneArtifact.SceneArtifactData;
import materia.assembly.AssemblyDefinitionCodec;
import haxe.io.Bytes;
import haxe.io.BytesBuffer;

/** A producer and viewer share this fingerprint; presentation records are deliberately excluded. */
class PresentationGeometry {
  public static function fingerprint(artifact:SceneArtifactData):String {
    var bytes=new BytesBuffer();
    function chunk(value:Bytes):Void {bytes.addInt32(value.length);bytes.add(value);}
    chunk(Bytes.ofString(Std.string(artifact.metresPerUnit)));
    var definition=artifact.assemblyDefinition;
    chunk(Bytes.ofString(definition==null?"":AssemblyDefinitionCodec.encode(definition)));
    var parts=artifact.parts.copy();parts.sort((a,b)->Reflect.compare(a.id,b.id));
    for(part in parts) {
      chunk(Bytes.ofString(part.id));chunk(part.vertices);chunk(part.indices);
    }
    var digest=haxe.crypto.Sha256.make(bytes.getBytes()),hex=new StringBuf();
    for(index in 0...digest.length)hex.add(StringTools.hex(digest.get(index),2).toLowerCase());
    return hex.toString();
  }
}
