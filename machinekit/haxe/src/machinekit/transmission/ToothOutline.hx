package machinekit.transmission;

import cadkit.modeling.Curve;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;

/**
 * A closed planar tooth outline (at z = 0) built from its true pieces: splines through sampled involutes,
 * arcs on tip and root circles, and straight lines. Unlike a polyline through samples, a flank is then one
 * smooth face, so shading and edge display show the tooth, not its sampling. Pieces must join end to start;
 * `close` checks that the outline returns to its first point.
 */
class ToothOutline {
	final pieces:Array<Curve> = [];
	var first:Null<Vector>;
	var last:Null<Vector>;

	public function new() {}

	public function line(start:Vector, end:Vector):ToothOutline
		return add(start, end, Curve.line(start, end));

	/** A circular arc from `start` through `middle` to `end`. */
	public function arc(start:Vector, middle:Vector, end:Vector):ToothOutline
		return add(start, end, Curve.arc(start, middle, end));

	/** An interpolating spline through `points`, in order. */
	public function spline(points:Array<Vector>):ToothOutline {
		if (points.length < 2)
			throw "A tooth spline needs at least two points";
		return add(points[0], points[points.length - 1], points.length == 2 ? Curve.line(points[0], points[1]) : Curve.spline(points));
	}

	/** The closed wire. The pieces are consumed. */
	public function close():Curve {
		if (first == null || last == null || first.subtract(last).length() > 1e-9) {
			dispose();
			throw "A tooth outline must end where it starts";
		}
		var wire:Curve;
		try
			wire = Curve.wire(pieces)
		catch (error:Dynamic) {
			dispose();
			throw error;
		}
		dispose();
		return wire;
	}

	/** The outline extruded along +Z from z0 to z1: a ruled loft between two copies, so every piece becomes one face. */
	public function extrude(z0:Float, z1:Float):Part {
		var wire = close();
		var sections:Array<Curve> = [];
		try {
			for (z in [z0, z1])
				sections.push(wire.placed(at(z, 0)));
			wire.close();
			var result = Part.loft(sections, true);
			for (section in sections)
				section.close();
			return result;
		} catch (error:Dynamic) {
			wire.close();
			for (section in sections)
				section.close();
			throw error;
		}
	}

	/** The placement of a section at height `z`, turned by `angle` radians about +Z. */
	public static function at(z:Float, angle:Float):Location
		return new Location(new Plane(new Vector(0, 0, z), new Vector(Math.cos(angle), Math.sin(angle), 0), new Vector(0, 0, 1)));

	function add(start:Vector, end:Vector, curve:Curve):ToothOutline {
		if (last != null && last.subtract(start).length() > 1e-9) {
			curve.close();
			dispose();
			throw "Tooth outline pieces must join end to start";
		}
		if (first == null)
			first = start;
		last = end;
		pieces.push(curve);
		return this;
	}

	function dispose():Void {
		for (piece in pieces)
			piece.close();
		pieces.resize(0);
	}
}
