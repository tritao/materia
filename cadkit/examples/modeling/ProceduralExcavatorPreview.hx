import haxe.io.Bytes;
import cadkit.Mesh;

private typedef PreviewMesh = {
	var nameBytes:Array<Int>;
	var mesh:Mesh;
	var vertices:Bytes;
	var normals:Bytes;
	var indices:Bytes;
}

/** Project entrypoint that exposes code-authored CAD parts as a binary geometry artifact. */
class ProceduralExcavatorPreview {
	public static function preview():Bytes {
		var components = ProceduralExcavator.build();
		var result:Bytes;
		try {
			var meshes:Array<PreviewMesh> = [];
			var length = 12; // Signature, version, component count.
			for (component in components) {
				var mesh = component.part.shape.tessellate(1.0, 0.7);
				var vertices:Bytes = mesh.vertices;
				var normals:Bytes = mesh.normals;
				var indices:Bytes = mesh.indices;
				if (vertices.length != mesh.vertexCount * 24 || normals.length != vertices.length ||
					indices.length != mesh.indexCount * 4)
					throw 'CadKit returned inconsistent mesh buffers for "${component.name}"';
				var nameBytes = utf8(component.name);
				length += 16 + nameBytes.length + vertices.length + normals.length + indices.length + mesh.faceRanges.length * 12;
				meshes.push({nameBytes: nameBytes, mesh: mesh, vertices: vertices, normals: normals, indices: indices});
			}
			result = Bytes.alloc(length);
			var offset = 0;
			for (byte in [77, 84, 82, 71]) result.set(offset++, byte); // MTRG
			offset = putInt32(result, offset, 1); // Artifact version.
			offset = putInt32(result, offset, components.length);
			for (item in meshes) {
				var mesh = item.mesh;
				offset = putInt32(result, offset, item.nameBytes.length);
				for (byte in item.nameBytes) result.set(offset++, byte);
				offset = putInt32(result, offset, mesh.vertexCount);
				offset = putInt32(result, offset, mesh.indexCount);
				offset = putInt32(result, offset, mesh.faceRanges.length);
				result.blit(offset, item.vertices, 0, item.vertices.length);
				offset += item.vertices.length;
				result.blit(offset, item.normals, 0, item.normals.length);
				offset += item.normals.length;
				result.blit(offset, item.indices, 0, item.indices.length);
				offset += item.indices.length;
				for (range in mesh.faceRanges) {
					offset = putInt32(result, offset, range.faceIndex);
					offset = putInt32(result, offset, range.firstIndex);
					offset = putInt32(result, offset, range.indexCount);
				}
			}
			if (offset != result.length) throw "Could not finalize the CAD geometry artifact";
		} catch (error:Dynamic) {
			for (component in components)
				component.close();
			throw error;
		}
		for (component in components)
			component.close();
		return result;
	}

	static function putInt32(output:Bytes, offset:Int, value:Int):Int {
		output.set(offset, value);
		output.set(offset + 1, value >>> 8);
		output.set(offset + 2, value >>> 16);
		output.set(offset + 3, value >>> 24);
		return offset + 4;
	}

	static function utf8(value:String):Array<Int> {
		var result:Array<Int> = [];
		var index = 0;
		while (index < value.length) {
			var code = value.charCodeAt(index++);
			if (code >= 0xd800 && code <= 0xdbff) {
				if (index >= value.length) throw "Component name contains invalid Unicode";
				var low = value.charCodeAt(index++);
				if (low < 0xdc00 || low > 0xdfff) throw "Component name contains invalid Unicode";
				code = 0x10000 + ((code - 0xd800) << 10) + (low - 0xdc00);
			} else if (code >= 0xdc00 && code <= 0xdfff) {
				throw "Component name contains invalid Unicode";
			}
			if (code <= 0x7f) result.push(code);
			else if (code <= 0x7ff) {
				result.push(0xc0 | (code >> 6));
				result.push(0x80 | (code & 0x3f));
			} else if (code <= 0xffff) {
				result.push(0xe0 | (code >> 12));
				result.push(0x80 | ((code >> 6) & 0x3f));
				result.push(0x80 | (code & 0x3f));
			} else {
				result.push(0xf0 | (code >> 18));
				result.push(0x80 | ((code >> 12) & 0x3f));
				result.push(0x80 | ((code >> 6) & 0x3f));
				result.push(0x80 | (code & 0x3f));
			}
		}
		return result;
	}
}

// Haxeon emits runtime modules with an entry function even when the host calls
// the stable preview function directly.
function main():Void {}
