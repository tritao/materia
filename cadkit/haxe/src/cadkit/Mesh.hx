package cadkit;

import CadKit;
import haxe.io.Bytes;
import haxe.io.BytesInput;
import haxe.io.BytesOutput;

/** Bulk tessellation streams owned by Haxeon. Buffers use native ABI layout. */
class Mesh {
  public final vertices:Bytes;
  public final normals:Bytes;
  public final indices:Bytes;
  /** Consecutive pairs of 3D points in the same units as vertices. */
  public final edgeSegments:Bytes;
  public final edgeIds:Bytes;
  public final vertexCount:Int;
  /** Maximum linear tessellation deflection, in the mesh's length units. */
  public final linearDeflection:Float;
  public final indexCount:Int;
  public final faceRanges:Array<cadkit.MeshFaceRange>;

  private function new(
    vertices:Bytes,
    normals:Bytes,
    indices:Bytes,
    edgeSegments:Bytes,
    edgeIds:Bytes,
    vertexCount:Int,
    indexCount:Int,
    faceRanges:Array<cadkit.MeshFaceRange>,
    linearDeflection:Float
  ) {
    this.vertices = vertices;
    this.normals = normals;
    this.indices = indices;
    this.edgeSegments = edgeSegments;
    this.edgeIds = edgeIds;
    this.vertexCount = vertexCount;
    this.linearDeflection = linearDeflection;
    this.indexCount = indexCount;
    this.faceRanges = faceRanges;
  }

  public static function fromShape(shape:CadKit.ShapeHandle, options:CadKit.MeshOptions, linearDeflection:Float):Mesh {
    var native = CadKit.shapeTessellateChecked(shape, options);
    try {
      var vertices = CadKit.meshCopyVerticesBytesChecked(native.borrow());
      var normals = CadKit.meshCopyNormalsBytesChecked(native.borrow());
      var indices = CadKit.meshCopyIndicesBytesChecked(native.borrow());
      var edgeSegments = CadKit.meshCopyEdgeSegmentsBytesChecked(native.borrow());
      var edgeIds = CadKit.meshCopyEdgeIdsBytesChecked(native.borrow());
      var vertexCount = CadKit.meshVertexCountChecked(native.borrow());
      var indexCount = CadKit.meshIndexCountChecked(native.borrow());
      var faceRangeCount = CadKit.meshFaceRangeCountChecked(native.borrow());
      var faceRanges:Array<cadkit.MeshFaceRange> = [];
      for (index in 0...faceRangeCount) {
        var nativeRange = CadKit.meshFaceRangeAtChecked(native.borrow(), index);
        faceRanges.push(new cadkit.MeshFaceRange(
          nativeRange.get_faceIndex(),
          nativeRange.get_firstIndex(),
          nativeRange.get_indexCount()
        ));
      }
      var result = new Mesh(
        vertices,
        normals,
        indices,
        edgeSegments,
        edgeIds,
        vertexCount,
        indexCount,
        faceRanges,
        linearDeflection
      );
      native.close();
      return result;
    } catch (error:Dynamic) {
      native.close();
      throw error;
    }
  }

  static inline final FORMAT = 0x4d455301;

  /**
   * The mesh as bytes, exactly: counts, deflection, the five streams as they are, the face ranges. For the persistent
   * tessellation cache; `fromBytes` gives back a mesh equal in every field.
   */
  public function toBytes():Bytes {
    var out = new BytesOutput();
    out.writeInt32(FORMAT);
    out.writeInt32(vertexCount);
    out.writeInt32(indexCount);
    out.writeDouble(linearDeflection);
    for (stream in [vertices, normals, indices, edgeSegments, edgeIds]) {
      out.writeInt32(stream.length);
      out.write(stream);
    }
    out.writeInt32(faceRanges.length);
    for (range in faceRanges) {
      out.writeInt32(range.faceIndex);
      out.writeInt32(range.firstIndex);
      out.writeInt32(range.indexCount);
    }
    return out.getBytes();
  }

  /** From `toBytes`; anything else (wrong format, short, trailing bytes) throws. */
  public static function fromBytes(bytes:Bytes):Mesh {
    var input = new BytesInput(bytes);
    if (input.readInt32() != FORMAT) throw "stored mesh has the wrong format";
    var vertexCount = input.readInt32(), indexCount = input.readInt32(), linearDeflection = input.readDouble();
    var streams:Array<Bytes> = [];
    for (_ in 0...5) {
      var length = input.readInt32();
      if (length < 0 || length > bytes.length) throw "stored mesh has a bad stream length";
      streams.push(input.read(length));
    }
    var count = input.readInt32();
    if (count < 0 || count > bytes.length) throw "stored mesh has a bad face range count";
    var ranges:Array<cadkit.MeshFaceRange> = [];
    for (_ in 0...count) ranges.push(new cadkit.MeshFaceRange(input.readInt32(), input.readInt32(), input.readInt32()));
    var extra = false;
    try {
      input.readByte();
      extra = true;
    } catch (error:Dynamic) {
    }
    if (extra) throw "stored mesh has trailing bytes";
    return new Mesh(
      streams[0],
      streams[1],
      streams[2],
      streams[3],
      streams[4],
      vertexCount,
      indexCount,
      ranges,
      linearDeflection
    );
  }

  /** Return the index range for a face from the tessellated shape snapshot. */
  public function rangeFor(face:Face):Null<cadkit.MeshFaceRange> {
    if (face.index < 0 || face.index >= faceRanges.length) return null;
    var range = faceRanges[face.index];
    if (range.faceIndex != face.index) return null;
    return range;
  }

}
