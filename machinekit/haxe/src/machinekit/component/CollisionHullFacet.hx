package machinekit.component;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Authored convex pieces in the component's local CAD frame, preserving routed gaps. */
class CollisionHullFacet implements ComponentFacet {
	public final hulls:Array<Array<Float>>;
	public function new(hulls:Array<Array<Float>>) {
		this.hulls = [for (hull in hulls) hull.copy()];
	}
	public static function fromBoxes(boxes:Array<Array<Float>>):CollisionHullFacet {
		var hulls:Array<Array<Float>> = [];
		for (box in boxes) {
			if (box.length != 6) throw "Collision box needs six bounds";
			for (axis in 0...3) if (!(box[axis] < box[axis + 3])) throw "Collision box needs positive volume";
			var hull:Array<Float> = [];
			for (x in [box[0], box[3]]) for (y in [box[1], box[4]]) for (z in [box[2], box[5]]) {
				hull.push(x); hull.push(y); hull.push(z);
			}
			hulls.push(hull);
		}
		return new CollisionHullFacet(hulls);
	}
	/** A bounded convex profile, in CAD millimetres; no union hull fills a neighbouring gap. */
	public static function prism(points:Array<Vector>, lower:Float, upper:Float):Array<Float> {
		if (points.length < 3 || points.length > 32 || !Math.isFinite(lower) || !Math.isFinite(upper) || lower >= upper)
			throw "Collision prism needs a bounded convex profile and positive depth";
		var hull:Array<Float> = [];
		for (z in [lower,upper]) for (point in points) { hull.push(point.x); hull.push(point.y); hull.push(z); }
		return hull;
	}
	/** Inscribed cylinder proxy, retaining the exact axial limits and cardinal radii. */
	public static function cylinder(radius:Float, lower:Float, upper:Float):Array<Float> {
		if (!Math.isFinite(radius) || radius <= 0) throw "Collision cylinder needs a positive radius";
		return prism([for (i in 0...32) new Vector(radius*Math.cos(i*2*Math.PI/32),radius*Math.sin(i*2*Math.PI/32))],lower,upper);
	}
	/** Extrude an authored constant section from its actual top cap, retaining every hole. */
	public static function fromExtrusion(part:Part, lower:Float, upper:Float):CollisionHullFacet {
		var mesh = part.shape.tessellate(), hulls:Array<Array<Float>> = [];
		for (i in 0...Std.int(mesh.indices.length/12)) {
			var points:Array<Vector> = [];
			for (j in 0...3) {
				var offset = mesh.indices.getInt32((i*3+j)*4)*24;
				points.push(new Vector(mesh.vertices.getDouble(offset),mesh.vertices.getDouble(offset+8),mesh.vertices.getDouble(offset+16)));
			}
			if (Math.abs(points[0].z-upper) > 1e-6 || Math.abs(points[1].z-upper) > 1e-6 || Math.abs(points[2].z-upper) > 1e-6) continue;
			var area = (points[1].x-points[0].x)*(points[2].y-points[0].y)-(points[1].y-points[0].y)*(points[2].x-points[0].x);
			if (Math.abs(area) > 1e-10) hulls.push(prism(points,lower,upper));
		}
		if (hulls.length == 0) throw "Collision extrusion has no triangles on its authored top cap";
		return new CollisionHullFacet(hulls);
	}
	public function check(component:MachineComponent):Void {
		if (hulls.length == 0 || !component.hasGeometry()) throw "Collision pieces need component geometry";
		for (hull in hulls) {
			if (hull.length < 12 || hull.length > 192 || hull.length % 3 != 0) throw "Invalid collision hull";
			for (coordinate in hull) if (!Math.isFinite(coordinate)) throw "Collision hull must be finite";
		}
	}
	public function describe():String return '${hulls.length} convex collision pieces';
	public static function of(component:MachineComponent):Null<CollisionHullFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, CollisionHullFacet)) return cast facet;
		return null;
	}
}
