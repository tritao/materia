package machinekit.pressbrake;

import cadkit.modeling.Curve;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.WallOutline;

/** Authored punch contact dimensions and its upper section, in millimetres and radians.
 * The upper section begins above the rounded tip's tangent points; the contact nose is derived
 * from the radius and included angle. A gooseneck's concavity belongs to this section.
 */
class PunchProfile {
	public final id:String;
	public final kind:PunchKind;
	public final tipRadius:Float;
	public final includedAngle:Float;
	public final height:Float;
	public final upperSection:ToolProfilePolygon;
	final contactEdge:Int;

	public function new(id:String, kind:PunchKind, tipRadius:Float, includedAngle:Float,
		height:Float, upperSection:ToolProfilePolygon) {
		if (id == null || id.length == 0) throw "Punch profile needs an identity";
		if (kind == null || upperSection == null) throw 'Punch profile "$id" needs a kind and an upper section';
		if (!Math.isFinite(tipRadius) || !(tipRadius > 0)) throw 'Punch profile "$id" needs a positive tip radius';
		if (!Math.isFinite(includedAngle) || !(includedAngle > 0 && includedAngle < Math.PI))
			throw 'Punch profile "$id" needs an included angle between zero and pi';
		if (!Math.isFinite(height) || !(height > tipRadius)) throw 'Punch profile "$id" needs a height above its tip';
		this.id = id;
		this.kind = kind;
		this.tipRadius = tipRadius;
		this.includedAngle = includedAngle;
		this.height = height;
		this.upperSection = upperSection;
		var maximum = 0.0;
		var minimum = height;
		for (point in upperSection.points()) {
			if (point.z < tangentHeight() - 1e-9 || point.z > height + 1e-9)
				throw 'Punch profile "$id" upper section exceeds its contact or height datum';
			maximum = Math.max(maximum, point.z);
			minimum = Math.min(minimum, point.z);
		}
		if (Math.abs(maximum - height) > 1e-9 || Math.abs(minimum - tangentHeight()) > 1e-9)
			throw 'Punch profile "$id" upper section must reach the tangent and mounting datums';
		var points = upperSection.points();
		var edge = -1;
		for (i in 0...points.length) {
			var a = points[i], b = points[(i + 1) % points.length];
			if (Math.abs(a.z - tangentHeight()) <= 1e-9 && Math.abs(b.z - tangentHeight()) <= 1e-9
				&& Math.abs(a.y + tangentWidth()) <= 1e-9 && Math.abs(b.y - tangentWidth()) <= 1e-9)
				edge = i;
		}
		if (edge < 0) throw 'Punch profile "$id" needs a chord between its two contact tangents';
		contactEdge = edge;
		var before = points[(edge + points.length - 1) % points.length];
		var after = points[(edge + 2) % points.length];
		var slope = Math.tan(includedAngle / 2);
		if (!(before.z > tangentHeight()) || !(after.z > tangentHeight())
			|| Math.abs(before.y + tangentWidth() + slope * (before.z - tangentHeight())) > 1e-8
			|| Math.abs(after.y - tangentWidth() - slope * (after.z - tangentHeight())) > 1e-8)
			throw 'Punch profile "$id" sides must meet its rounded tip tangentially';
	}

	/** A new owned extrusion with an exact circular contact nose; the caller closes it. */
	public function geometry(start:Float, end:Float):Part {
		if (!Math.isFinite(start) || !Math.isFinite(end) || !(end > start))
			throw 'Punch profile "$id" needs a finite positive bed interval';
		var plane = Plane.YZ().offset(start);
		var points = upperSection.points();
		var edges:Array<Curve> = [];
		var wire:Null<Curve> = null;
		var sketch:Null<Sketch> = null;
		try {
			for (i in 0...points.length) {
				var a = plane.toWorld(new Vector(points[i].y, points[i].z));
				var b = points[(i + 1) % points.length];
				var finish = plane.toWorld(new Vector(b.y, b.z));
				edges.push(Curve.named(i == contactEdge ? Curve.arc(a, plane.toWorld(new Vector(0, 0)), finish) : Curve.line(a, finish),
					i == contactEdge ? "punch.contact" : 'punch.section.$i'));
			}
			wire = Curve.wire(edges);
			sketch = Sketch.named(Sketch.face(wire, [], plane), "punch.section");
			var part = sketch.extrude(end - start);
			sketch.close();
			wire.close();
			for (edge in edges) edge.close();
			return part;
		} catch (error:Dynamic) {
			if (sketch != null) sketch.close();
			if (wire != null) wire.close();
			for (edge in edges) edge.close();
			throw error;
		}
	}

	/** Tangent-line cover of the contact arc. Its outward error is at most `maximumError` mm. */
	public function collisionSection(maximumError:Float):ToolProfilePolygon {
		if (!Math.isFinite(maximumError) || !(maximumError > 0))
			throw 'Punch profile "$id" needs a positive collision section error';
		var arc = contactArc();
		var result:Array<ToolProfilePoint> = [];
		var points = upperSection.points();
		for (i in 0...points.length) {
			result.push(points[i]);
			if (i == contactEdge) for (point in arc.tangentVertices(maximumError)) result.push(point);
		}
		return new ToolProfilePolygon(result);
	}
	/** Exact contact section for refining prism overlap at the rounded nose. */
	public function contactSection():WallOutline {
		var points = upperSection.points(), edges:Array<Segment> = [];
		for (i in 0...points.length) {
			var next = points[(i + 1) % points.length];
			edges.push(i == contactEdge ? contactArc().segment()
				: Segment.line(new Point2(points[i].y, points[i].z), new Point2(next.y, next.z)));
		}
		return new WallOutline(edges);
	}
	function contactArc():ToolProfileArc return new ToolProfileArc(new ToolProfilePoint(0, tipRadius), tipRadius,
		-Math.PI + includedAngle / 2, Math.PI - includedAngle);

	/** Tip at z=0; the circle centre is one radius above it. */
	public function tangentHeight():Float return tipRadius * (1 - Math.sin(includedAngle / 2));

	/** The positive-Y contact tangent; the opposite tangent is its reflection. */
	public function tangentWidth():Float return tipRadius * Math.cos(includedAngle / 2);
}
