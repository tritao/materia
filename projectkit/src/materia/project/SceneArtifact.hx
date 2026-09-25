package materia.project;

import haxe.io.Bytes;
import materia.project.AssemblyCodec;
import materia.project.AssemblyRecord;
import materia.project.AssemblyDefinition;
import materia.project.AssemblyDefinition.AssemblyStateRecord;
import materia.project.AssemblyDefinitionCodec;

typedef SceneArtifactFaceRange = {
	var faceIndex:Int;
	var firstIndex:Int;
	var indexCount:Int;
}

typedef SceneArtifactPart = {
	var id:String;
	var name:String;
	var red:Float;
	var green:Float;
	var blue:Float;
	var vertexCount:Int;
	var indexCount:Int;
	var vertices:Bytes;
	var normals:Bytes;
	var indices:Bytes;
	@:optional var edgeSegments:Bytes;
	@:optional var edgeIds:Bytes;
	var faceRanges:Array<SceneArtifactFaceRange>;
}

typedef SceneArtifactData = {
	/** Metres represented by one coordinate in every mesh stream. */
	var metresPerUnit:Float;
	var parts:Array<SceneArtifactPart>;
	@:optional var assembly:AssemblyRecord;
	/** Geometry definitions are parts; occurrences live in this separate kinematic definition. */
	@:optional var assemblyDefinition:AssemblyDefinition;
	@:optional var assemblyState:AssemblyStateRecord;
}

/** Versioned, producer-independent scene geometry exchange format. */
class SceneArtifact {
	public static inline var VERSION:Int = 6;
	public static inline var MAX_BYTES:Int = 150000000;
	static inline var MAX_VERTICES:Int = 2000000;
	static inline var MAX_TRIANGLES:Int = 4000000;
	static inline var MAX_FACE_RANGES:Int = 100000;

	public static function encode(data:SceneArtifactData):Bytes {
		validateHeader(data);
		var names:Array<{id:Bytes, name:Bytes}> = [];
		var assembly = data.assembly == null ? Bytes.alloc(0) : Bytes.ofString(AssemblyCodec.encode(data.assembly));
		var assemblyDefinition = data.assemblyDefinition == null ? Bytes.alloc(0)
			: Bytes.ofString(AssemblyDefinitionCodec.encode(data.assemblyDefinition));
		var assemblyState = data.assemblyState == null ? Bytes.alloc(0)
			: Bytes.ofString(AssemblyDefinitionCodec.encodeState(data.assemblyDefinition, data.assemblyState));
		if (assembly.length > 2000000 || assemblyDefinition.length > 2000000 || assemblyState.length > 2000000)
			throw "Scene artifact assembly metadata is too large";
		var length = 32 + assembly.length + assemblyDefinition.length + assemblyState.length;
		for (part in data.parts) {
			validatePart(part, true);
			var id = Bytes.ofString(part.id), name = Bytes.ofString(part.name);
			if (id.length == 0 || id.length > 4096 || name.length == 0 || name.length > 4096)
				throw "Scene artifact has an invalid part ID or name";
			names.push({id: id, name: name});
			length += 36 + id.length + name.length + part.vertices.length + part.normals.length
				+ part.indices.length + (part.edgeSegments == null ? 0 : part.edgeSegments.length)
				+ (part.edgeIds == null ? 0 : part.edgeIds.length)
				+ part.faceRanges.length * 12;
			if (length > MAX_BYTES) throw "Scene artifact exceeds the 150 MB limit";
		}
		var result = Bytes.alloc(length), offset = 0;
		for (byte in [77, 84, 82, 71]) result.set(offset++, byte); // MTRG.
		offset = putInt(result, offset, VERSION);
		result.setDouble(offset, data.metresPerUnit); offset += 8;
		offset = putInt(result, offset, data.parts.length);
		for (index in 0...data.parts.length) {
			var part = data.parts[index], text = names[index];
			offset = putInt(result, offset, text.id.length);
			result.blit(offset, text.id, 0, text.id.length); offset += text.id.length;
			offset = putInt(result, offset, text.name.length);
			result.blit(offset, text.name, 0, text.name.length); offset += text.name.length;
			for (color in [part.red, part.green, part.blue]) {
				result.setFloat(offset, color); offset += 4;
			}
			offset = putInt(result, offset, part.vertexCount);
			offset = putInt(result, offset, part.indexCount);
			offset = putInt(result, offset, part.faceRanges.length);
			offset = putInt(result, offset, part.edgeSegments == null ? 0 : part.edgeSegments.length);
			for (buffer in [part.vertices, part.normals, part.indices]) {
				result.blit(offset, buffer, 0, buffer.length); offset += buffer.length;
			}
			if (part.edgeSegments != null) {
				result.blit(offset, part.edgeSegments, 0, part.edgeSegments.length);
				offset += part.edgeSegments.length;
			}
			if (part.edgeIds != null) {
				result.blit(offset, part.edgeIds, 0, part.edgeIds.length);
				offset += part.edgeIds.length;
			}
			for (range in part.faceRanges) {
				offset = putInt(result, offset, range.faceIndex);
				offset = putInt(result, offset, range.firstIndex);
				offset = putInt(result, offset, range.indexCount);
			}
		}
		offset = putInt(result, offset, assembly.length);
		result.blit(offset, assembly, 0, assembly.length); offset += assembly.length;
		offset = putInt(result, offset, assemblyDefinition.length);
		result.blit(offset, assemblyDefinition, 0, assemblyDefinition.length); offset += assemblyDefinition.length;
		offset = putInt(result, offset, assemblyState.length);
		result.blit(offset, assemblyState, 0, assemblyState.length); offset += assemblyState.length;
		if (offset != result.length) throw "Scene artifact size mismatch";
		return result;
	}

	public static function decode(source:Bytes):SceneArtifactData {
		if (source == null || source.length > MAX_BYTES) throw "Scene artifact is missing or too large";
		return new SceneArtifactReader(source).read();
	}

	static function validateHeader(data:SceneArtifactData):Void {
		if (data == null || !finite(data.metresPerUnit) || data.metresPerUnit <= 0.0 ||
			data.parts == null || data.parts.length == 0 || data.parts.length > 1000)
			throw "Scene artifact has invalid units or part count";
		var ids = new Map<String, Bool>();
		for (part in data.parts) {
			if (part.id == null || StringTools.trim(part.id).length == 0 || ids.exists(part.id))
				throw "Scene artifact has a duplicate or empty part ID";
			ids.set(part.id, true);
		}
		var assembly = data.assembly;
		if (assembly != null) {
			AssemblyCodec.validate(assembly);
			if (assembly.instances.length != data.parts.length)
				throw "Assembly must place every scene artifact part";
			for (instance in assembly.instances) if (!ids.exists(instance.id))
				throw 'Assembly instance "${instance.id}" has no geometry part';
		}
		if (data.assemblyState != null && data.assemblyDefinition == null)
			throw "Scene artifact assembly state has no definition";
		if (data.assemblyDefinition != null) {
			AssemblyDefinitionCodec.validate(data.assemblyDefinition);
			for (definition in data.assemblyDefinition.definitions) if (!ids.exists(definition.id))
				throw 'Assembly component definition "${definition.id}" has no geometry part';
			if (data.assemblyState != null)
				AssemblyDefinitionCodec.validateState(data.assemblyDefinition, data.assemblyState);
		}
	}

	static function validatePart(part:SceneArtifactPart, requireEdgeIds:Bool = false):Void {
		if (part.name == null || StringTools.trim(part.name).length == 0 ||
			part.vertexCount <= 0 || part.vertexCount > MAX_VERTICES || part.indexCount <= 0 ||
			part.indexCount % 3 != 0 || part.indexCount / 3 > MAX_TRIANGLES ||
			part.faceRanges == null || part.faceRanges.length > MAX_FACE_RANGES ||
			part.vertices == null || part.vertices.length != part.vertexCount * 24 ||
			part.normals == null || part.normals.length != part.vertexCount * 24 ||
			part.indices == null || part.indices.length != part.indexCount * 4)
			throw 'Scene artifact part "${part.id}" has invalid mesh streams';
		if (part.edgeSegments != null) {
			if (part.edgeSegments.length % 48 != 0 || part.edgeSegments.length > MAX_BYTES)
				throw 'Scene artifact part "${part.id}" has invalid edge segments';
			for (offset in 0...Std.int(part.edgeSegments.length / 8))
				if (!finite(part.edgeSegments.getDouble(offset * 8)))
					throw 'Scene artifact part "${part.id}" has a non-finite edge endpoint';
		}
		if ((requireEdgeIds && part.edgeSegments != null && part.edgeSegments.length > 0 && part.edgeIds == null) ||
			(part.edgeIds != null &&
				(part.edgeSegments == null || part.edgeIds.length != Std.int(part.edgeSegments.length / 48) * 4)))
			throw 'Scene artifact part "${part.id}" has inconsistent edge identities';
		for (color in [part.red, part.green, part.blue])
			if (!finite(color) || color < 0.0 || color > 1.0)
				throw 'Scene artifact part "${part.id}" has an invalid color';
		for (range in part.faceRanges)
			if (range.faceIndex < 0 || range.firstIndex < 0 || range.indexCount < 0 ||
				range.firstIndex % 3 != 0 || range.indexCount % 3 != 0 ||
				range.firstIndex + range.indexCount > part.indexCount)
				throw 'Scene artifact part "${part.id}" has an invalid face range';
		for (vertex in 0...part.vertexCount) for (axis in 0...3) {
			var offset = vertex * 24 + axis * 8;
			if (!finite(part.vertices.getDouble(offset)) || !finite(part.normals.getDouble(offset)))
				throw 'Scene artifact part "${part.id}" has a non-finite vertex or normal';
		}
		for (index in 0...part.indexCount) {
			var vertex = part.indices.getInt32(index * 4);
			if (vertex < 0 || vertex >= part.vertexCount)
				throw 'Scene artifact part "${part.id}" has an out-of-range index';
		}
	}

	static inline function finite(value:Float):Bool return value == value && value - value == 0.0;

	static function putInt(bytes:Bytes, offset:Int, value:Int):Int {
		bytes.set(offset, value); bytes.set(offset + 1, value >>> 8);
		bytes.set(offset + 2, value >>> 16); bytes.set(offset + 3, value >>> 24);
		return offset + 4;
	}
}

private class SceneArtifactReader {
	final source:Bytes;
	var offset:Int = 0;
	public function new(source:Bytes) this.source = source;

	public function read():SceneArtifactData {
		for (expected in [77, 84, 82, 71]) if (readByte() != expected)
			throw "Scene artifact has an invalid signature";
		var version = readInt();
		if (version != 2 && version != 3 && version != 4 && version != 5 && version != SceneArtifact.VERSION)
			throw "Unsupported scene artifact version";
		var metresPerUnit = readDouble();
		var count = readInt();
		if (!Math.isFinite(metresPerUnit) || metresPerUnit <= 0.0 || count <= 0 || count > 1000)
			throw "Scene artifact has invalid units or part count";
		var ids = new Map<String, Bool>(), parts:Array<SceneArtifactPart> = [];
		for (_ in 0...count) {
			var id = readText(), name = readText();
			if (ids.exists(id)) throw 'Scene artifact has duplicate part ID "$id"';
			ids.set(id, true);
			var red = readFloat(), green = readFloat(), blue = readFloat();
			var vertexCount = readInt(), indexCount = readInt(), rangeCount = readInt();
			var edgeByteCount = version >= 4 ? readInt() : 0;
			if (vertexCount <= 0 || vertexCount > 2000000 || indexCount <= 0 ||
				indexCount % 3 != 0 || indexCount / 3 > 4000000 || rangeCount < 0 || rangeCount > 100000 ||
				edgeByteCount < 0 || edgeByteCount % 48 != 0 || edgeByteCount > SceneArtifact.MAX_BYTES)
				throw 'Scene artifact part "$id" has invalid mesh counts';
		var vertices = readBytes(vertexCount * 24), normals = readBytes(vertexCount * 24),
				indices = readBytes(indexCount * 4), edgeSegments = readBytes(edgeByteCount);
			var edgeIds = version >= 5 ? readBytes(Std.int(edgeByteCount / 48) * 4) : null;
			var faceRanges:Array<SceneArtifactFaceRange> = [];
			for (_ in 0...rangeCount)
				faceRanges.push({faceIndex: readInt(), firstIndex: readInt(), indexCount: readInt()});
			var part:SceneArtifactPart = {id: id, name: name, red: red, green: green, blue: blue,
				vertexCount: vertexCount, indexCount: indexCount, vertices: vertices, normals: normals,
				indices: indices, edgeSegments: edgeSegments, edgeIds: edgeIds,
				faceRanges: faceRanges};
			@:privateAccess SceneArtifact.validatePart(part, version >= 5);
			parts.push(part);
		}
		var assembly:Null<AssemblyRecord> = null;
		if (version >= 3) {
			var length = readInt();
			if (length < 0 || length > 2000000) throw "Scene artifact assembly metadata is too large";
			if (length > 0) assembly = AssemblyCodec.decode(readBytes(length).getString(0, length));
		}
		var assemblyDefinition:Null<AssemblyDefinition> = null, assemblyState:Null<AssemblyStateRecord> = null;
		if (version >= 6) {
			var definitionLength = readInt();
			if (definitionLength < 0 || definitionLength > 2000000)
				throw "Scene artifact assembly definition is too large";
			if (definitionLength > 0)
				assemblyDefinition = AssemblyDefinitionCodec.decode(readBytes(definitionLength).getString(0, definitionLength));
			var stateLength = readInt();
			if (stateLength < 0 || stateLength > 2000000)
				throw "Scene artifact assembly state is too large";
			if (stateLength > 0) {
				if (assemblyDefinition == null) throw "Scene artifact assembly state has no definition";
				assemblyState = AssemblyDefinitionCodec.decodeState(assemblyDefinition,
					readBytes(stateLength).getString(0, stateLength));
			}
		}
		if (offset != source.length) throw "Scene artifact contains trailing data";
		var result:SceneArtifactData = {metresPerUnit: metresPerUnit, parts: parts, assembly: assembly,
			assemblyDefinition: assemblyDefinition, assemblyState: assemblyState};
		@:privateAccess SceneArtifact.validateHeader(result);
		return result;
	}

	function readText():String {
		var length = readInt();
		if (length <= 0 || length > 4096) throw "Scene artifact has an invalid part ID or name";
		var value = readBytes(length).getString(0, length);
		if (StringTools.trim(value).length == 0) throw "Scene artifact has an empty part ID or name";
		return value;
	}

	function readByte():Int {
		if (offset >= source.length) throw "Scene artifact ended unexpectedly";
		return source.get(offset++);
	}
	function readInt():Int {
		var a = readByte(), b = readByte(), c = readByte(), d = readByte();
		return a | (b << 8) | (c << 16) | (d << 24);
	}
	function readDouble():Float {
		var value = readBytes(8);
		return value.getDouble(0);
	}
	function readFloat():Float {
		var value = readBytes(4);
		return value.getFloat(0);
	}
	function readBytes(length:Int):Bytes {
		if (length < 0 || length > source.length - offset)
			throw "Scene artifact has an invalid buffer length";
		var result = Bytes.alloc(length);
		result.blit(0, source, offset, length);
		offset += length;
		return result;
	}
}
