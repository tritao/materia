package machinekit.component;

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
