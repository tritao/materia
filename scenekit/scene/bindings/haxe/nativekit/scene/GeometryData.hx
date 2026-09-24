package nativekit.scene;

import NativeKitScene;
import haxe.io.Bytes;

/** Owns the backing arrays used to describe one geometry resource. */
class GeometryData {
	final value:nkscene_geometry_data;
	final vertices:Array<nkscene_geometry_vertex> = [];
	final subelements:Array<nkscene_subelement_range> = [];
	final streams:Array<nkscene_vertex_stream> = [];
	final strokeSegments:Array<nkscene_stroke_segment> = [];
	final streamData:Array<Bytes> = [];
	final indexValues:Array<Int> = [];
	var indices:Bytes = Bytes.alloc(0);
	var packedIndices:Null<Bytes>;
	var packedIndexCount:Int = 0;
	var positionStreamCount:Null<Int>;

	public function new() {
		value = new nkscene_geometry_data();
		value.set_struct_size(nkscene_geometry_data.size());
	}

	/** Describes a box primitive; native SceneKit generates its faces, normals, and edges. */
	public static function box(width:Float, height:Float, depth:Float):GeometryData {
		if (!Math.isFinite(width) || !Math.isFinite(height) || !Math.isFinite(depth) ||
			width <= 0.0 || height <= 0.0 || depth <= 0.0)
			throw "Box dimensions must be positive finite values";
		return new GeometryData()
			.setPrimitiveType(NkscenePrimitiveType.Box)
			.setBounds(-width / 2.0, -height / 2.0, -depth / 2.0,
				width / 2.0, height / 2.0, depth / 2.0);
	}

	public function addVertex(x:Float, y:Float, z:Float):Int {
		var vertex = new nkscene_geometry_vertex();
		vertex.set_position(0, x);
		vertex.set_position(1, y);
		vertex.set_position(2, z);
		vertices.push(vertex);
		return vertices.length - 1;
	}

	public function addTriangle(first:Int, second:Int, third:Int):GeometryData {
		if (packedIndices != null)
			throw "Cannot append triangles after setting a packed index buffer";
		appendIndex(first);
		appendIndex(second);
		appendIndex(third);
		return this;
	}

	public function setPrimitiveType(primitiveType:Int):GeometryData {
		value.set_primitive_type(primitiveType);
		return this;
	}

	/** Adds one tightly packed or explicitly strided vertex attribute stream. */
	public function addStream(semantic:Int, format:Int, data:Bytes, count:Int,
			stride:Int = 0):GeometryData {
		if (count < 0 || stride < 0)
			throw "Geometry stream count and stride must be non-negative";
		var stream = new nkscene_vertex_stream();
		stream.set_struct_size(nkscene_vertex_stream.size());
		stream.set_semantic(semantic);
		stream.set_format(format);
		stream.set_stride(stride);
		stream.set_data_bytes(data);
		stream.set_count(count);
		streams.push(stream);
		streamData.push(data);
		if (semantic == 1) {
			if (positionStreamCount != null)
				throw "Geometry data can contain only one position stream";
			positionStreamCount = count;
		}
		return this;
	}

	public function setBounds(minX:Float, minY:Float, minZ:Float,
			maxX:Float, maxY:Float, maxZ:Float):GeometryData {
		var bounds = new nkscene_bounds();
		bounds.set_minimum(0, minX);
		bounds.set_minimum(1, minY);
		bounds.set_minimum(2, minZ);
		bounds.set_maximum(0, maxX);
		bounds.set_maximum(1, maxY);
		bounds.set_maximum(2, maxZ);
		bounds.set_valid(1);
		value.set_bounds(bounds);
		return this;
	}

	public function addSubelement(firstPrimitive:Int, primitiveCount:Int,
			subelement:Int):GeometryData {
		var range = new nkscene_subelement_range();
		range.set_first_primitive(firstPrimitive);
		range.set_primitive_count(primitiveCount);
		range.set_subelement(subelement);
		subelements.push(range);
		return this;
	}

	/** Adds one object-space centerline segment for the analytic GPU stroke pass. */
	public function addStrokeSegment(startX:Float, startY:Float, startZ:Float,
			endX:Float, endY:Float, endZ:Float):GeometryData {
		var segment = new nkscene_stroke_segment();
		segment.set_start(0, startX);
		segment.set_start(1, startY);
		segment.set_start(2, startZ);
		segment.set_end(0, endX);
		segment.set_end(1, endY);
		segment.set_end(2, endZ);
		strokeSegments.push(segment);
		return this;
	}

	public function vertexCount():Int {
		if (vertices.length != 0)
			return vertices.length;
		return positionStreamCount == null ? 0 : cast positionStreamCount;
	}

	public function triangleCount():Int
		return Std.int((packedIndices == null ? indexValues.length : packedIndexCount) / 3);

	/** Supplies a tightly packed uint32 index stream without per-index Haxe allocations. */
	public function setIndexBuffer(data:Bytes, count:Int):GeometryData {
		if (data == null || count < 0 || count % 3 != 0 || data.length != count * 4 || indexValues.length != 0)
			throw "Packed triangle index buffer has an invalid size";
		packedIndices = data;
		packedIndexCount = count;
		return this;
	}

	@:allow(Scene)
	function nativeValue():nkscene_geometry_data {
		if (vertices.length != 0)
			value.set_vertices(vertices);
		if (packedIndices != null) {
			indices = packedIndices;
			value.set_indices_bytes(indices);
			value.set_index_count(packedIndexCount);
		} else if (indexValues.length != 0) {
			indices = Bytes.alloc(indexValues.length * 4);
			for (index in 0...indexValues.length)
				indices.setInt32(index * 4, indexValues[index]);
			value.set_indices_bytes(indices);
			value.set_index_count(indexValues.length);
		}
		value.set_subelements(subelements);
		value.set_stroke_segment_count(strokeSegments.length);
		value.set_stroke_segments(strokeSegments);
		if (streams.length != 0) {
			value.set_stream_count(streams.length);
			value.set_streams(streams);
		}
		return value;
	}

	function appendIndex(index:Int):Void {
		if (index < 0 || index > 0x7fffffff)
			throw "Geometry index must be a non-negative 31-bit integer";
		indexValues.push(index);
	}
}
