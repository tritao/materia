package cadkit;

import CadKit;
import haxe.io.Bytes;

/** Bulk tessellation streams owned by Haxeon. Buffers use native ABI layout. */
class Mesh {
	public final vertices:Bytes;
	public final normals:Bytes;
	public final indices:Bytes;
	public final vertexCount:Int;
	public final indexCount:Int;
	public final faceRanges:Array<cadkit.MeshFaceRange>;

	private function new(
		vertices:Bytes,
		normals:Bytes,
		indices:Bytes,
		vertexCount:Int,
		indexCount:Int,
		faceRanges:Array<cadkit.MeshFaceRange>) {
		this.vertices = vertices;
		this.normals = normals;
		this.indices = indices;
		this.vertexCount = vertexCount;
		this.indexCount = indexCount;
		this.faceRanges = faceRanges;
	}

	public static function fromShape(
		shape:CadKit.ShapeHandle,
		options:CadKit.MeshOptions):Mesh {
		var native = CadKit.shapeTessellateChecked(shape, options);
		try {
			var vertices = CadKit.meshCopyVerticesBytesChecked(native.borrow());
			var normals = CadKit.meshCopyNormalsBytesChecked(native.borrow());
			var indices = CadKit.meshCopyIndicesBytesChecked(native.borrow());
			var vertexCount = CadKit.meshVertexCountChecked(native.borrow());
			var indexCount = CadKit.meshIndexCountChecked(native.borrow());
			var faceRangeCount = CadKit.meshFaceRangeCountChecked(native.borrow());
			var faceRanges:Array<cadkit.MeshFaceRange> = [];
			for (index in 0...faceRangeCount) {
				var nativeRange = CadKit.meshFaceRangeAtChecked(native.borrow(), index);
				faceRanges.push(new cadkit.MeshFaceRange(
					nativeRange.get_faceIndex(),
					nativeRange.get_firstIndex(),
					nativeRange.get_indexCount()));
			}
			var result = new Mesh(
				vertices,
				normals,
				indices,
				vertexCount,
				indexCount,
				faceRanges);
			native.close();
			return result;
		} catch (error:Dynamic) {
			native.close();
			throw error;
		}
	}

	/** Return the index range for a face from the tessellated shape snapshot. */
	public function rangeFor(face:Face):Null<cadkit.MeshFaceRange> {
		if (face.index < 0 || face.index >= faceRanges.length)
			return null;
		var range = faceRanges[face.index];
		if (range.faceIndex != face.index)
			return null;
		return range;
	}

}
