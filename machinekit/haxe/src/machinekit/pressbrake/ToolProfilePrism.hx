package machinekit.pressbrake;

import cadkit.modeling.Part;
import cadkit.modeling.Curve;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;

/** One tooling section extruded along the bed's X axis, with the same section for collision pieces. */
class ToolProfilePrism {
	public final section:ToolProfilePolygon;
	public final start:Float;
	public final end:Float;

	public function new(section:ToolProfilePolygon, start:Float, end:Float) {
		if (section == null) throw "Tool profile prism needs a section";
		if (!Math.isFinite(start) || !Math.isFinite(end) || !(end > start))
			throw "Tool profile prism needs a finite positive bed interval";
		this.section = section;
		this.start = start;
		this.end = end;
	}

	/** A new owned solid; the caller closes it. */
	public function geometry():Part return extrude(section.points());

	/** New owned convex solids whose union is the section extrusion; the caller closes them. */
	public function convexGeometry():Array<Part> {
		var result:Array<Part> = [];
		try {
			for (piece in section.convexPieces()) result.push(extrude(piece));
			return result;
		} catch (error:Dynamic) {
			for (part in result) part.close();
			throw error;
		}
	}

	function extrude(points:Array<ToolProfilePoint>):Part {
		var plane = Plane.YZ().offset(start);
		var edges:Array<Curve> = [];
		var wire:Null<Curve> = null;
		var sketch:Null<Sketch> = null;
		try {
			for (i in 0...points.length) {
				var next = points[(i + 1) % points.length];
				edges.push(Curve.named(Curve.line(plane.toWorld(new Vector(points[i].y, points[i].z)),
					plane.toWorld(new Vector(next.y, next.z))), 'tool-section.edge.$i'));
			}
			wire = Curve.wire(edges);
			sketch = Sketch.named(Sketch.face(wire, [], plane), "tool-section");
			var part = sketch.extrude(end - start);
			sketch.close(); wire.close();
			for (edge in edges) edge.close();
			return part;
		} catch (error:Dynamic) {
			if (sketch != null) sketch.close();
			if (wire != null) wire.close();
			for (edge in edges) edge.close();
			throw error;
		}
	}
}
