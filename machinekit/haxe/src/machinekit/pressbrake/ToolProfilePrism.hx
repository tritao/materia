package machinekit.pressbrake;

import cadkit.modeling.Part;
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
		var sketch = Sketch.polygon([for (point in points) new Vector(point.y, point.z)], Plane.YZ().offset(start));
		try {
			var part = sketch.extrude(end - start);
			sketch.close();
			return part;
		} catch (error:Dynamic) {
			sketch.close();
			throw error;
		}
	}
}
