import haxe.Json;
import haxe.io.Bytes;

/**
 * A minimal binary glTF 2.0 writer: one mesh whose primitives are PBR triangles (optionally with texture
 * coordinates and textures) and unlit lines. Positions, normals and coordinates are 32-bit floats and indices
 * 32-bit integers; images are referenced by URI, relative to the GLB.
 */
class Glb {
	final views:Array<{bytes:Bytes, target:Int}> = [];
	final accessors:Array<Dynamic> = [];
	final primitives:Array<Dynamic> = [];
	final materials:Array<Dynamic> = [];
	final images:Array<Dynamic> = [];
	final textures:Array<Dynamic> = [];
	final imageIndex:Map<String, Int> = [];
	var unlit = false;

	/** Bounds of everything added, as stored (32-bit), in glTF axes. */
	public final min = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];

	public final max = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];

	public function new() {}

	/** A metallic-roughness material; `color` is linear RGB. Returns its index. */
	public function material(color:Array<Float>, metallic:Float, roughness:Float, ?colorImage:String, ?normalImage:String):Int {
		var pbr:Dynamic = {baseColorFactor: [color[0], color[1], color[2], 1.0], metallicFactor: metallic, roughnessFactor: roughness};
		if (colorImage != null)
			pbr.baseColorTexture = {index: texture(colorImage)};
		var material:Dynamic = {pbrMetallicRoughness: pbr};
		if (normalImage != null)
			material.normalTexture = {index: texture(normalImage)};
		materials.push(material);
		return materials.length - 1;
	}

	/** An unlit material of linear RGBA `color`, for lines. */
	public function unlitMaterial(color:Array<Float>):Int {
		unlit = true;
		materials.push({pbrMetallicRoughness: {baseColorFactor: color}, extensions: {KHR_materials_unlit: {}}});
		return materials.length - 1;
	}

	/** Triangles: flat xyz positions and normals, optional flat uv, three indices per triangle. */
	public function triangles(positions:Array<Float>, normals:Array<Float>, uvs:Null<Array<Float>>, indices:Array<Int>, material:Int):Void {
		if (indices.length == 0)
			return;
		var attributes:Dynamic = {POSITION: vectors(positions, true), NORMAL: vectors(normals, false)};
		if (uvs != null)
			attributes.TEXCOORD_0 = coordinates(uvs);
		var bytes = Bytes.alloc(indices.length * 4);
		for (index in 0...indices.length)
			bytes.setInt32(index * 4, indices[index]);
		accessors.push({bufferView: view(bytes, 34963), componentType: 5125, count: indices.length, type: "SCALAR"});
		primitives.push({attributes: attributes, indices: accessors.length - 1, material: material});
	}

	/** Line segments: flat xyz, two points per segment. */
	public function lines(points:Array<Float>, material:Int):Void {
		if (points.length == 0)
			return;
		primitives.push({attributes: {POSITION: vectors(points, true)}, mode: 1, material: material});
	}

	public function bytes():Bytes {
		var binaryLength = 0;
		var bufferViews:Array<Dynamic> = [];
		for (entry in views) {
			bufferViews.push({buffer: 0, byteOffset: binaryLength, byteLength: entry.bytes.length, target: entry.target});
			binaryLength += entry.bytes.length;
		}
		var gltf:Dynamic = {
			asset: {version: "2.0", generator: "Materia docs/components"},
			scene: 0,
			scenes: [{nodes: [0]}],
			nodes: [{mesh: 0}],
			meshes: [{primitives: primitives}],
			materials: materials,
			buffers: [{byteLength: binaryLength}],
			bufferViews: bufferViews,
			accessors: accessors
		};
		if (unlit)
			gltf.extensionsUsed = ["KHR_materials_unlit"];
		if (images.length > 0) {
			gltf.images = images;
			gltf.textures = textures;
			// Repeat in both directions with trilinear filtering: thread textures tile along their helix.
			gltf.samplers = [{magFilter: 9729, minFilter: 9987, wrapS: 10497, wrapT: 10497}];
		}
		var text = Json.stringify(gltf);
		while (Bytes.ofString(text).length % 4 != 0)
			text += " ";
		var json = Bytes.ofString(text);
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
		var offset = 28 + json.length;
		for (entry in views) {
			out.blit(offset, entry.bytes, 0, entry.bytes.length);
			offset += entry.bytes.length;
		}
		return out;
	}

	function texture(uri:String):Int {
		var image = imageIndex.get(uri);
		if (image == null) {
			images.push({uri: uri});
			textures.push({sampler: 0, source: images.length - 1});
			image = images.length - 1;
			imageIndex.set(uri, image);
		}
		return image;
	}

	function view(bytes:Bytes, target:Int):Int {
		views.push({bytes: bytes, target: target});
		return views.length - 1;
	}

	function vectors(values:Array<Float>, bounded:Bool):Int {
		var count = Std.int(values.length / 3);
		var bytes = Bytes.alloc(count * 12);
		var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
		var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		for (index in 0...values.length) {
			bytes.setFloat(index * 4, values[index]);
			// Bounds must be the stored 32-bit values, so read each one back.
			var stored = bytes.getFloat(index * 4), axis = index % 3;
			if (stored < low[axis]) low[axis] = stored;
			if (stored > high[axis]) high[axis] = stored;
		}
		var accessor:Dynamic = {bufferView: view(bytes, 34962), componentType: 5126, count: count, type: "VEC3"};
		if (bounded) {
			accessor.min = low;
			accessor.max = high;
			for (axis in 0...3) {
				if (low[axis] < min[axis]) min[axis] = low[axis];
				if (high[axis] > max[axis]) max[axis] = high[axis];
			}
		}
		accessors.push(accessor);
		return accessors.length - 1;
	}

	function coordinates(values:Array<Float>):Int {
		var bytes = Bytes.alloc(values.length * 4);
		for (index in 0...values.length)
			bytes.setFloat(index * 4, values[index]);
		accessors.push({bufferView: view(bytes, 34962), componentType: 5126, count: Std.int(values.length / 2), type: "VEC2"});
		return accessors.length - 1;
	}
}
