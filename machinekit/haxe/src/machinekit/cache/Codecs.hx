package machinekit.cache;

import haxe.io.Bytes;
import haxe.io.BytesInput;
import haxe.io.BytesOutput;
import machinekit.component.ComponentDisplayMesh.ComponentDisplayMeshData;

/** Exact (bit-for-bit) byte codecs for the plain values a persistent `Memo` keeps. */
class Codecs {
  public static function floats(list:Array<Float>):Bytes {
    var out = new BytesOutput();
    writeFloats(out, list);
    return out.getBytes();
  }

  public static function unfloats(bytes:Bytes):Array<Float> {
    var input = new BytesInput(bytes);
    var list = readFloats(input);
    done(input, bytes);
    return list;
  }

  public static function displayMesh(mesh:ComponentDisplayMeshData):Bytes {
    var out = new BytesOutput();
    writeFloats(out, mesh.positions);
    writeFloats(out, mesh.normals);
    out.writeInt32(mesh.indices.length);
    for (index in mesh.indices) out.writeInt32(index);
    return out.getBytes();
  }

  public static function undisplayMesh(bytes:Bytes):ComponentDisplayMeshData {
    var input = new BytesInput(bytes);
    var positions = readFloats(input), normals = readFloats(input);
    var indices = [for (_ in 0...input.readInt32()) input.readInt32()];
    done(input, bytes);
    return {positions: positions, normals: normals, indices: indices};
  }

  static function writeFloats(out:BytesOutput, list:Array<Float>):Void {
    out.writeInt32(list.length);
    for (value in list) out.writeDouble(value);
  }

  static function readFloats(input:BytesInput):Array<Float> return [for (_ in 0...input.readInt32()) input.readDouble()];

  /** A stored value must be consumed exactly; anything else is damage, which the memo treats as a miss. */
  static function done(input:BytesInput, bytes:Bytes):Void {
    var rest = 0;
    try {
      input.readByte();
      rest = 1;
    } catch (error:Dynamic) {}
    if (rest != 0) throw "stored value has trailing bytes";
  }
}
