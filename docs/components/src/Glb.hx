import haxe.Json;
import haxe.io.Bytes;

/** A minimal binary glTF 2.0 writer: one mesh of PBR triangles, with its feature edges as unlit lines. */
class Glb {
	/** Edge lines: a dark grey that reads on light and metallic parts alike (linear RGB). */
	static final EDGE_COLOR = [0.02, 0.025, 0.03, 1.0];

	/**
	 * Writes positions and normals (doubles, `vertexCount` x 3) and triangle indices (32-bit, `indexCount`) as a
	 * GLB, scaling positions by `scale` (CadKit models in millimetres; glTF is in metres). glTF is Y-up and Materia
	 * Z-up, so every vector turns -90 degrees about X: (x, y, z) -> (x, z, -y). `edges` holds the feature edges as
	 * pairs of points, flat, in the positions' units.
	 */
	public static function write(positions:Bytes, normals:Bytes, indices:Bytes, vertexCount:Int, indexCount:Int, edges:Array<Float>,
			scale:Float, color:Array<Float>, metallic:Float, roughness:Float):Glb {
		var normalOffset = vertexCount * 12, indexOffset = normalOffset * 2;
		var edgeOffset = indexOffset + indexCount * 4, edgeVertices = Std.int(edges.length / 3);
		var binaryLength = edgeOffset + edgeVertices * 12;
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
		var edgeMin = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
		var edgeMax = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		for (vertex in 0...edgeVertices) {
			var p = [edges[vertex * 3] * scale, edges[vertex * 3 + 2] * scale, -edges[vertex * 3 + 1] * scale];
			for (axis in 0...3) {
				binary.setFloat(edgeOffset + vertex * 12 + axis * 4, p[axis]);
				var stored = binary.getFloat(edgeOffset + vertex * 12 + axis * 4);
				if (stored < edgeMin[axis]) edgeMin[axis] = stored;
				if (stored > edgeMax[axis]) edgeMax[axis] = stored;
			}
		}
		var accessors:Array<Dynamic> = [
			{bufferView: 0, componentType: 5126, count: vertexCount, type: "VEC3", min: min, max: max},
			{bufferView: 1, componentType: 5126, count: vertexCount, type: "VEC3"},
			{bufferView: 2, componentType: 5125, count: indexCount, type: "SCALAR"}
		];
		var primitives:Array<Dynamic> = [{attributes: {POSITION: 0, NORMAL: 1}, indices: 2, material: 0}];
		var bufferViews:Array<Dynamic> = [
			{buffer: 0, byteOffset: 0, byteLength: normalOffset, target: 34962},
			{buffer: 0, byteOffset: normalOffset, byteLength: normalOffset, target: 34962},
			{buffer: 0, byteOffset: indexOffset, byteLength: indexCount * 4, target: 34963}
		];
		// Feature edges, drawn as unlit lines (mode 1) the way the editor outlines CAD parts.
		if (edgeVertices > 0) {
			bufferViews.push({buffer: 0, byteOffset: edgeOffset, byteLength: edgeVertices * 12, target: 34962});
			accessors.push({bufferView: 3, componentType: 5126, count: edgeVertices, type: "VEC3", min: edgeMin, max: edgeMax});
			primitives.push({attributes: {POSITION: 3}, mode: 1, material: 1});
		}
		var materials:Array<Dynamic> = [
			{pbrMetallicRoughness: {baseColorFactor: [color[0], color[1], color[2], 1.0], metallicFactor: metallic,
				roughnessFactor: roughness}},
			{pbrMetallicRoughness: {baseColorFactor: EDGE_COLOR}, extensions: {KHR_materials_unlit: {}}}
		];
		var gltf:Dynamic = {
			asset: {version: "2.0", generator: "Materia docs/components"},
			scene: 0,
			scenes: [{nodes: [0]}],
			nodes: [{mesh: 0}],
			meshes: [{primitives: primitives}],
			materials: materials,
			extensionsUsed: ["KHR_materials_unlit"],
			buffers: [{byteLength: binaryLength}],
			bufferViews: bufferViews,
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
		return new Glb(out, min, max);
	}

	/** The file. */
	public final bytes:Bytes;

	/** The mesh's bounds in glTF axes (Y up), in metres. */
	public final min:Array<Float>;

	public final max:Array<Float>;

	function new(bytes:Bytes, min:Array<Float>, max:Array<Float>) {
		this.bytes = bytes;
		this.min = min;
		this.max = max;
	}
}
