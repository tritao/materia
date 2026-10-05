package machinekit.pressbrake;

import cadkit.modeling.Curve;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.WallOutline;

/** Single-V die section with exact circular shoulders, top at z=0 and mounting face at -height. */
class DieProfile {
	public final id:String;
	public final groove:DieGroove;
	public final width:Float;
	public final height:Float;

	public function new(id:String, groove:DieGroove, width:Float, height:Float) {
		if (id == null || id.length == 0 || groove == null) throw "Die profile needs an identity and a groove";
		if (!Math.isFinite(width) || !(width > groove.vWidth)) throw 'Die profile "$id" needs material outside its shoulders';
		if (!Math.isFinite(height) || !(height > groove.depth())) throw 'Die profile "$id" groove reaches its mounting face';
		this.id = id;
		this.groove = groove;
		this.width = width;
		this.height = height;
	}

	/** A new owned extrusion; the caller closes it. */
	public function geometry(start:Float, end:Float):Part {
		if (!Math.isFinite(start) || !Math.isFinite(end) || !(end > start))
			throw 'Die profile "$id" needs a finite positive bed interval';
		var plane = Plane.YZ().offset(start);
		var points = sectionPoints();
		var edges:Array<Curve> = [];
		var wire:Null<Curve> = null;
		var sketch:Null<Sketch> = null;
		try {
			for (i in 0...points.length) {
				var a = plane.toWorld(new Vector(points[i].y, points[i].z));
				var b = points[(i + 1) % points.length];
				var finish = plane.toWorld(new Vector(b.y, b.z));
				if (groove.shoulderRadius > 0 && (i == 3 || i == 6)) {
					var angle = i == 3 ? (Math.PI / 2 + Math.PI - groove.includedAngle / 2) / 2
						: (Math.PI / 2 + groove.includedAngle / 2) / 2;
					var centerY = i == 3 ? groove.vWidth / 2 : -groove.vWidth / 2;
					var middle = plane.toWorld(new Vector(centerY + groove.shoulderRadius * Math.cos(angle),
						-groove.shoulderRadius + groove.shoulderRadius * Math.sin(angle)));
					edges.push(Curve.named(Curve.arc(a, middle, finish), i == 3 ? "die.shoulder.right" : "die.shoulder.left"));
				} else edges.push(Curve.named(Curve.line(a, finish), 'die.section.$i'));
			}
			wire = Curve.wire(edges);
			sketch = Sketch.named(Sketch.face(wire, [], plane), "die.section");
			var result = sketch.extrude(end - start);
			sketch.close();
			wire.close();
			for (edge in edges) edge.close();
			return result;
		} catch (error:Dynamic) {
			if (sketch != null) sketch.close();
			if (wire != null) wire.close();
			for (edge in edges) edge.close();
			throw error;
		}
	}

	/** Conservative contact cover with exact straight flanks and an explicit radial error bound. */
	public function collisionSection(maximumError:Float):ToolProfilePolygon {
		if (!Math.isFinite(maximumError) || !(maximumError > 0))
			throw 'Die profile "$id" needs a positive collision section error';
		var points = sectionPoints();
		var result:Array<ToolProfilePoint> = [];
		for (i in 0...points.length) {
			result.push(points[i]);
			if (groove.shoulderRadius > 0 && (i == 3 || i == 6)) {
				var arc = contactArc(i);
				for (point in arc.tangentVertices(maximumError)) result.push(point);
			}
		}
		return new ToolProfilePolygon(result);
	}
	/** Exact straight flanks and circular shoulders, using the same authored section datums. */
	public function contactSection():WallOutline {
		var points = sectionPoints(), edges:Array<Segment> = [];
		for (i in 0...points.length) {
			var next = points[(i + 1) % points.length];
			edges.push(groove.shoulderRadius > 0 && (i == 3 || i == 6) ? contactArc(i).segment()
				: Segment.line(new Point2(points[i].y, points[i].z), new Point2(next.y, next.z)));
		}
		return new WallOutline(edges);
	}
	function contactArc(index:Int):ToolProfileArc return index == 3
		? new ToolProfileArc(new ToolProfilePoint(groove.vWidth / 2, -groove.shoulderRadius),
			groove.shoulderRadius, Math.PI / 2, (Math.PI - groove.includedAngle) / 2)
		: new ToolProfileArc(new ToolProfilePoint(-groove.vWidth / 2, -groove.shoulderRadius),
			groove.shoulderRadius, groove.includedAngle / 2, (Math.PI - groove.includedAngle) / 2);

	function sectionPoints():Array<ToolProfilePoint> {
		var result = [new ToolProfilePoint(-width / 2, -height), new ToolProfilePoint(width / 2, -height),
			new ToolProfilePoint(width / 2, 0), new ToolProfilePoint(groove.vWidth / 2, 0)];
		if (groove.shoulderRadius > 0) result.push(new ToolProfilePoint(groove.tangentY(), groove.tangentZ()));
		result.push(new ToolProfilePoint(0, -groove.depth()));
		if (groove.shoulderRadius > 0) result.push(new ToolProfilePoint(-groove.tangentY(), groove.tangentZ()));
		result.push(new ToolProfilePoint(-groove.vWidth / 2, 0));
		result.push(new ToolProfilePoint(-width / 2, 0));
		return result;
	}
}
