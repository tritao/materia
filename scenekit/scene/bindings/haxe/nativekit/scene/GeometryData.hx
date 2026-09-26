package nativekit.scene;

import NativeKitScene;
import haxe.io.Bytes;

/** Owns the backing arrays used to describe one geometry resource. */
class GeometryData {
	public static inline var EDGE_SUBELEMENT_TAG:Int = 0x40000000;
	final value:nkscene_geometry_data;
	final vertices:Array<nkscene_geometry_vertex> = [];
	final subelements:Array<nkscene_subelement_range> = [];
	final streams:Array<nkscene_vertex_stream> = [];
	final strokeSegments:Array<nkscene_stroke_segment> = [];
	final streamData:Array<Bytes> = [];
	final streamSemantics:Array<Int> = [];
	final streamFormats:Array<Int> = [];
	final streamStrides:Array<Int> = [];
	final streamCounts:Array<Int> = [];
	final indexValues:Array<Int> = [];
	var indices:Bytes = Bytes.alloc(0);
	var packedIndices:Null<Bytes>;
	var packedIndexCount:Int = 0;
	var positionStreamCount:Null<Int>;
	var boundsValid:Bool = false;
	var boundsMinimum:Array<Float> = [0.0, 0.0, 0.0];
	var boundsMaximum:Array<Float> = [0.0, 0.0, 0.0];

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
		streamSemantics.push(semantic);
		streamFormats.push(format);
		streamStrides.push(stride);
		streamCounts.push(count);
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
		boundsValid = true;
		boundsMinimum = [minX, minY, minZ];
		boundsMaximum = [maxX, maxY, maxZ];
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
			endX:Float, endY:Float, endZ:Float, edgeIndex:Int = -1):GeometryData {
		if (edgeIndex >= 0x3fffffff) throw "CAD edge index exceeds stroke identity range";
		var segment = new nkscene_stroke_segment();
		segment.set_start(0, startX);
		segment.set_start(1, startY);
		segment.set_start(2, startZ);
		segment.set_end(0, endX);
		segment.set_end(1, endY);
		segment.set_end(2, endZ);
		segment.set_subelement(edgeIndex < 0 ? 0 : EDGE_SUBELEMENT_TAG | (edgeIndex + 1));
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

	/**
	 * Builds a renderable geometry containing only one subelement's triangles.
	 * This is used for face overlays while retaining the source vertex streams.
	 */
	public function subelementGeometry(subelement:Int):Null<GeometryData> {
		if (subelement < 0 || streams.length == 0)
			return null;
		var primitiveCount = 0;
		for (range in subelements)
			if (range.get_subelement() == subelement)
				primitiveCount += range.get_primitive_count();
		if (primitiveCount == 0 || (packedIndices == null && indexValues.length == 0))
			return null;

		var result = new GeometryData();
		for (index in 0...streams.length)
			result.addStream(streamSemantics[index], streamFormats[index], streamData[index],
				streamCounts[index], streamStrides[index]);
		if (boundsValid)
			result.setBounds(boundsMinimum[0], boundsMinimum[1], boundsMinimum[2],
				boundsMaximum[0], boundsMaximum[1], boundsMaximum[2]);

		var selectedIndices = Bytes.alloc(primitiveCount * 3 * 4);
		var targetIndex = 0;
		for (range in subelements) if (range.get_subelement() == subelement) {
			var firstPrimitive = range.get_first_primitive();
			var endPrimitive = firstPrimitive + range.get_primitive_count();
			for (primitive in firstPrimitive...endPrimitive) {
				for (corner in 0...3) {
					var sourceIndex = primitive * 3 + corner;
					var vertexIndex = packedIndices == null
						? indexValues[sourceIndex]
						: packedIndices.getInt32(sourceIndex * 4);
					selectedIndices.setInt32(targetIndex * 4, vertexIndex);
					targetIndex++;
				}
			}
		}
		result.setIndexBuffer(selectedIndices, targetIndex);
		result.addSubelement(0, primitiveCount, subelement);
		return result;
	}

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
