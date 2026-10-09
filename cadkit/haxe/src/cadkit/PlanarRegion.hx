package cadkit;

import CadKit;
import haxe.io.Bytes;

/** One piece of a planar boundary, in the plane's units: a line, or an arc about (cx, cy) running `sweep` radians from `startAngle`. */
class PlanarSegment {
	public final arc:Bool;
	public final x0:Float;
	public final y0:Float;
	public final x1:Float;
	public final y1:Float;
	public final cx:Float;
	public final cy:Float;
	public final radius:Float;
	public final startAngle:Float;
	/** Signed, about +z: positive runs counter-clockwise. Zero for a line. */
	public final sweep:Float;

	public function new(arc:Bool, x0:Float, y0:Float, x1:Float, y1:Float, cx:Float, cy:Float, radius:Float, startAngle:Float, sweep:Float) {
		this.arc = arc;
		this.x0 = x0; this.y0 = y0; this.x1 = x1; this.y1 = y1;
		this.cx = cx; this.cy = cy; this.radius = radius;
		this.startAngle = startAngle; this.sweep = sweep;
	}

	public function length():Float
		return arc ? radius * Math.abs(sweep) : Math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));

	/** The same piece run the other way. */
	public function reversed():PlanarSegment
		return new PlanarSegment(arc, x1, y1, x0, y0, cx, cy, radius, startAngle + sweep, -sweep);

	/** The point `fraction` (0 to 1) of the way along, by arc angle or by chord. */
	public function pointAt(fraction:Float):{x:Float, y:Float} {
		if (!arc) return {x: x0 + (x1 - x0) * fraction, y: y0 + (y1 - y0) * fraction};
		var angle = startAngle + sweep * fraction;
		return {x: cx + radius * Math.cos(angle), y: cy + radius * Math.sin(angle)};
	}

	/** The integral of (x dy - y dx) / 2 along the piece: summed round a loop it is the loop's signed area. */
	public function areaTerm():Float {
		if (!arc) return (x0 * y1 - x1 * y0) * 0.5;
		var end = startAngle + sweep;
		return (radius * radius * sweep + radius * (cx * (Math.sin(end) - Math.sin(startAngle)) - cy * (Math.cos(end) - Math.cos(startAngle)))) * 0.5;
	}
}

/** A closed boundary of a planar face: its segments end to end. */
class PlanarLoop {
	/** Which face of the region it bounds. */
	public final face:Int;
	public final outer:Bool;
	public final z:Float;
	public final segments:Array<PlanarSegment>;

	public function new(face:Int, outer:Bool, z:Float, segments:Array<PlanarSegment>) {
		this.face = face;
		this.outer = outer;
		this.z = z;
		this.segments = segments;
	}

	/** Positive when the loop runs counter-clockwise seen from +z. */
	public function signedArea():Float {
		var total = 0.0;
		for (segment in segments) total += segment.areaTerm();
		return total;
	}

	public function length():Float {
		var total = 0.0;
		for (segment in segments) total += segment.length();
		return total;
	}

	public function reversed():PlanarLoop {
		var back = [for (index in 0...segments.length) segments[segments.length - 1 - index].reversed()];
		return new PlanarLoop(face, outer, z, back);
	}

	/** Starts at segment `index`. */
	public function startingAt(index:Int):PlanarLoop
		return new PlanarLoop(face, outer, z, segments.slice(index).concat(segments.slice(0, index)));
}

/**
 * A planar region: a planar face, or several, with holes. Slicing works on these: a section of a
 * solid, its offsets, and booleans between regions all give regions. Owns its shape.
 */
class PlanarRegion {
	public final shape:Shape;

	public function new(shape:Shape) {
		this.shape = shape;
	}

	/** The region of `solid` on the plane z = `height` (the solid's own units). */
	public static function section(solid:Shape, height:Float):PlanarRegion
		return new PlanarRegion(solid.section(Geometry.vec3(0.0, 0.0, height), Geometry.vec3(0.0, 0.0, 1.0)));

	public function faceCount():Int return shape.subshapeCount(CadKit.ShapeKind.Face);

	public function isEmpty():Bool return faceCount() == 0;

	public function area():Float return isEmpty() ? 0.0 : shape.area();

	/** Positive grows the region (holes shrink); negative shrinks it. */
	public function offset(distance:Float):PlanarRegion return new PlanarRegion(shape.planarOffset(distance));

	public function cut(other:PlanarRegion):PlanarRegion return new PlanarRegion(shape.planarCut(other.shape));

	public function common(other:PlanarRegion):PlanarRegion return new PlanarRegion(shape.planarCommon(other.shape));

	public function fuse(other:PlanarRegion):PlanarRegion return new PlanarRegion(shape.planarFuse(other.shape));

	/**
	 * Every boundary loop, outer loops counter-clockwise and holes clockwise, so the material is always on
	 * the left. Lines and circular arcs are exact; other curves are cut into lines within `chordTolerance`.
	 */
	public function loops(?chordTolerance:Float = 0.00001):Array<PlanarLoop> {
		if (isEmpty()) return [];
		var bytes = shape.planarBoundary(chordTolerance);
		var count = Std.int(bytes.length / 112);
		var result:Array<PlanarLoop> = [];
		var current:Array<PlanarSegment> = [];
		var key = -1, face = 0, outer = false, z = 0.0;
		function finish():Void {
			if (current.length == 0) return;
			var loop = new PlanarLoop(face, outer, z, current);
			var area = loop.signedArea();
			result.push((outer && area < 0.0) || (!outer && area > 0.0) ? loop.reversed() : loop);
			current = [];
		}
		for (index in 0...count) {
			var at = index * 112;
			function value(slot:Int):Float return bytes.getDouble(at + slot * 8);
			var segmentFace = Std.int(value(0)), wire = Std.int(value(1));
			var wireKey = segmentFace * 100000 + wire;
			if (wireKey != key) {
				finish();
				key = wireKey; face = segmentFace; outer = value(2) != 0.0; z = value(6);
			}
			current.push(new PlanarSegment(value(3) != 0.0, value(4), value(5), value(7), value(8),
				value(9), value(10), value(11), value(12), value(13)));
		}
		finish();
		return result;
	}

	public function close():Void shape.close();
}
