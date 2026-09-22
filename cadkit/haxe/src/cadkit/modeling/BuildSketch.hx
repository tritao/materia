package cadkit.modeling;

import CadKit;
import cadkit.Shape;

class BuildSketch {
	public final plane:Plane;

	private var current:Null<Sketch>;
	private var closed:Bool;

	public function new(?plane:Plane) {
		this.plane = plane == null ? Plane.XY() : plane;
		current = null;
		closed = false;
	}

	public function add(sketch:Sketch, mode:Mode = Add):Void {
		if (closed)
			throw "builder is closed";
		if (Math.abs(Math.abs(plane.normal.dot(sketch.plane.normal)) - 1) > 1e-10
			|| Math.abs(sketch.plane.origin.subtract(plane.origin).dot(plane.normal)) > 1e-7)
			throw "sketch must lie on builder plane";
		if (current == null) {
			if (mode != Add)
				throw "first sketch operation must add";
			current = new Sketch(sketch.shape.cloneShape(), plane);
			return;
		}
		var next = current.combine(sketch, mode);
		current.close();
		current = next;
	}

	private function temporary(sketch:Sketch, mode:Mode):Void {
		try {
			add(sketch, mode);
			sketch.close();
		} catch (error:Dynamic) {
			sketch.close();
			throw error;
		}
	}

	public function rectangle(width:Float, height:Float, mode:Mode = Add):Void {
		temporary(Sketch.rectangle(width, height, plane), mode);
	}

	public function circle(radius:Float, center:Vector, mode:Mode = Add):Void {
		if (Math.abs(center.z) > 1e-10)
			throw "circle center must lie in local XY";
		temporary(Sketch.circle(radius, new Plane(plane.toWorld(center), plane.xDirection, plane.normal)), mode);
	}

	public function slot(length:Float, width:Float, mode:Mode = Add):Void {
		temporary(Sketch.slot(length, width, plane), mode);
	}

	/** Placements are expressed in the builder's local coordinate frame. */
	public function circles(radius:Float, locations:Array<Location>, mode:Mode = Add):Void {
		for (location in locations) {
			var world = plane.location().compose(location).plane;
			temporary(Sketch.circle(radius, world), mode);
		}
	}

	public function finish():Sketch {
		if (closed)
			throw "builder is closed";
		if (current == null)
			return new Sketch(Shape.fromOwnedHandle(CadKit.compoundChecked([])), plane);
		var result = current;
		current = null;
		return result;
	}

	public function close():Void {
		if (closed)
			return;
		closed = true;
		if (current != null)
			current.close();
		current = null;
	}

	public static function build(callback:BuildSketch->Void, ?plane:Plane):Sketch {
		var builder = new BuildSketch(plane);
		try {
			callback(builder);
			var result = builder.finish();
			builder.close();
			return result;
		} catch (error:Dynamic) {
			builder.close();
			throw error;
		}
	}
}
