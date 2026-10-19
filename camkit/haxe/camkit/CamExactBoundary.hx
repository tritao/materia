package camkit;

import cadkit.modeling.Curve;
import cadkit.modeling.Vector;
import cadkit.modeling.Sketch;
import cadkit.modeling.Plane;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;

/** Exact compensation belongs to CamKit. Native geometry uses mm for its tolerance.
 * The final centreline is sampled to the declared chord tolerance, in metres.
 */
class CamExactBoundary {
	static function wire(contour:CamContour):Curve {
		var source = contour.exactBoundary;
		if (source == null)
			throw "Exact CAM offset needs a boundary";
		var curves:Array<Curve> = [];
		try {
			var boundary = source.copy(), reverse = contour.signedArea < 0;
			if (reverse)
				boundary.reverse();
			for (s in boundary) {
				var a = reverse ? s.end : s.start,
					b = reverse ? s.start : s.end,
					mid = s.point(0.5);
				var start = new Vector(a.x * 1000, a.y * 1000, 0),
					end = new Vector(b.x * 1000, b.y * 1000, 0);
				curves.push(s.isLine() ? Curve.line(start, end) : Curve.arc(start, new Vector(mid.x * 1000, mid.y * 1000, 0), end));
			}
			var result = Curve.wire(curves);
			for (c in curves)
				c.close();
			return result;
		} catch (e:Dynamic) {
			for (c in curves)
				c.close();
			throw e;
		}
	}

	/** Matched-radius relief centres may be isolated points discarded by regular offsets.
	 * Supply separate vertical entries within the SAME operation/provenance, after
	 * proving that the complete cutter disc fits the source's removable region.
	 */
	public static function reliefCenters(contour:CamContour, radius:Float, outside:Bool):Array<Point3> {
		var result:Array<Point3> = [], source = contour.exactBoundary;
		if (source == null)
			return result;
		var orientation = contour.signedArea > 0 ? 1 : -1;
		for (s in source)
			if (s.center != null
				&& Math.abs(s.radius - radius) < 1e-10
				&& (outside ? s.sweep * orientation < 0 : s.sweep * orientation > 0)) {
				var p = new Point3(s.center.x, s.center.y, 0);
				if (result.filter(q -> q.distanceTo(p) < 1e-10).length == 0)
					result.push(p);
			}
		if (result.length == 0)
			return result;
		var boundary = wire(contour), face:Null<Sketch> = null;
		try {
			face = Sketch.face(boundary);
			for (p in result) {
				var disc = Sketch.circle(radius * 1000, new Plane(new Vector(p.x * 1000, p.y * 1000, 0), Vector.X(), Vector.Z()));
				var overlap:Null<cadkit.Shape> = null;
				try {
					overlap = outside ? disc.shape.common(face.shape) : disc.shape.cut(face.shape);
					var area = overlap.area();
					overlap.close();
					overlap = null;
					if (area > 1e-6)
						throw "CAM matched-radius relief would gouge the source boundary";
					disc.close();
				} catch (e:Dynamic) {
					if (overlap != null)
						overlap.close();
					disc.close();
					throw e;
				}
			}
			face.close();
			boundary.close();
			return result;
		} catch (e:Dynamic) {
			if (face != null)
				face.close();
			boundary.close();
			throw e;
		}
	}

	public static function offset(contour:CamContour, radius:Float, depth:Float, outside:Bool):Array<PathGeometry> {
		var source = contour.exactBoundary;
		if (source == null)
			throw "Exact CAM offset needs a boundary";
		// A cutter-sized circular hole has a point centreline, supplied by reliefCenters.
		if (!outside && source.length > 0 && source[0].center != null) {
			var center = source[0].center;
			if (source.filter(s -> s.center == null || s.center.distance(center) > 1e-10 || Math.abs(s.radius - radius) > 1e-10).length == 0)
				return [];
		}
		var boundary = wire(contour), shifted:Null<Curve> = null;
		var owned:Array<cadkit.Edge> = [];
		try {
			shifted = boundary.offset((outside ? 1 : -1) * radius * 1000);
			if (shifted.shape.subshapeCount(CadKit.ShapeKind.Wire) != 1)
				throw "CAM exact offset splits or exhausts the profile";
			var edges = shifted.shape.edges();
			for (i in 0...edges.count())
				owned.push(edges.at(i));
			var sampled = CamContour.fromEdges(owned, "mm", contour.chordTolerance);
			if (outside ? Math.abs(sampled.signedArea) <= Math.abs(contour.signedArea) : Math.abs(sampled.signedArea) >= Math.abs(contour.signedArea))
				throw "CAM exact offset has incorrect material side";
			var result:Array<PathGeometry> = [];
			for (i in 0...sampled.vertices.length) {
				var a = sampled.vertices[i],
					b = sampled.vertices[(i + 1) % sampled.vertices.length];
				result.push(Line(new Point3(a.x, a.y, depth), new Point3(b.x, b.y, depth)));
			}
			for (e in owned)
				e.close();
			owned = [];
			shifted.close();
			shifted = null;
			boundary.close();
			return result;
		} catch (e:Dynamic) {
			for (edge in owned)
				edge.close();
			if (shifted != null)
				shifted.close();
			boundary.close();
			throw e;
		}
	}
}
