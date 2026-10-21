import haxe.Json;
import haxe.io.Bytes;

/** A minimal binary glTF 2.0 writer: one mesh, one primitive, one PBR material. */
class Glb {
	/**
	 * Writes positions and normals (doubles, `vertexCount` x 3) and triangle indices (32-bit, `indexCount`) as a
	 * GLB, scaling positions by `scale` (CadKit models in millimetres; glTF is in metres). glTF is Y-up and Materia
	 * Z-up, so every vector turns -90 degrees about X: (x, y, z) -> (x, z, -y).
	 */
	public static function write(positions:Bytes, normals:Bytes, indices:Bytes, vertexCount:Int, indexCount:Int, scale:Float,
			color:Array<Float>, metallic:Float, roughness:Float):Bytes {
		var normalOffset = vertexCount * 12, indexOffset = normalOffset * 2;
		var binaryLength = indexOffset + indexCount * 4;
		var binary = Bytes.alloc(binaryLength);
		var min = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
		var max = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		for (vertex in 0...vertexCount) {
			var x = positions.getDouble(vertex * 24) * scale, y = positions.getDouble(vertex * 24 + 8) * scale,
				z = positions.getDouble(vertex * 24 + 16) * scale;
			var p = [x, z, -y];
			for (axis in 0...3) {
				binary.setFloat(vertex * 12 + axis * 4, p[axis]);
				// The bounds must be the stored 32-bit values, so read each one back.
				var stored = binary.getFloat(vertex * 12 + axis * 4);
				if (stored < min[axis]) min[axis] = stored;
				if (stored > max[axis]) max[axis] = stored;
			}
			var nx = normals.getDouble(vertex * 24), ny = normals.getDouble(vertex * 24 + 8), nz = normals.getDouble(vertex * 24 + 16);
			binary.setFloat(normalOffset + vertex * 12, nx);
			binary.setFloat(normalOffset + vertex * 12 + 4, nz);
			binary.setFloat(normalOffset + vertex * 12 + 8, -ny);
		}
		for (index in 0...indexCount)
			binary.setInt32(indexOffset + index * 4, indices.getInt32(index * 4));
		var accessors:Array<Dynamic> = [
			{bufferView: 0, componentType: 5126, count: vertexCount, type: "VEC3", min: min, max: max},
			{bufferView: 1, componentType: 5126, count: vertexCount, type: "VEC3"},
			{bufferView: 2, componentType: 5125, count: indexCount, type: "SCALAR"}
		];
		var gltf:Dynamic = {
			asset: {version: "2.0", generator: "Materia docs/components"},
			scene: 0,
			scenes: [{nodes: [0]}],
			nodes: [{mesh: 0}],
			meshes: [{primitives: [{attributes: {POSITION: 0, NORMAL: 1}, indices: 2, material: 0}]}],
			materials: [{pbrMetallicRoughness: {baseColorFactor: [color[0], color[1], color[2], 1.0], metallicFactor: metallic,
				roughnessFactor: roughness}}],
			buffers: [{byteLength: binaryLength}],
			bufferViews: [
				{buffer: 0, byteOffset: 0, byteLength: normalOffset, target: 34962},
				{buffer: 0, byteOffset: normalOffset, byteLength: normalOffset, target: 34962},
				{buffer: 0, byteOffset: indexOffset, byteLength: indexCount * 4, target: 34963}
			],
			accessors: accessors
		};
		var text = Json.stringify(gltf);
		while (Bytes.ofString(text).length % 4 != 0)
			text += " ";
		var json = Bytes.ofString(text);
		// The binary chunk is already a multiple of 4: floats and 32-bit indices.
		var total = 12 + 8 + json.length + 8 + binaryLength;
		var out = Bytes.alloc(total);
		out.setInt32(0, 0x46546C67); // "glTF"
		out.setInt32(4, 2);
		out.setInt32(8, total);
		out.setInt32(12, json.length);
		out.setInt32(16, 0x4E4F534A); // "JSON"
		out.blit(20, json, 0, json.length);
		out.setInt32(20 + json.length, binaryLength);
		out.setInt32(24 + json.length, 0x004E4942); // "BIN\0"
		out.blit(28 + json.length, binary, 0, binaryLength);
		return out;
	}
}
