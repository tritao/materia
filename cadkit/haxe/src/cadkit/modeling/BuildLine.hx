package cadkit.modeling;

/** Explicit callback builder. Points are local to the workplane. */
class BuildLine {
	public final plane:Plane;

	private final curves:Array<Curve>;
	private var closed:Bool;

	public function new(?plane:Plane) {
		this.plane = plane == null ? Plane.XY() : plane;
		curves = [];
		closed = false;
	}

	public function add(curve:Curve):Void {
		if (closed)
			throw "builder is closed";
		curves.push(new Curve(curve.shape.cloneShape()));
	}

	public function line(start:Vector, end:Vector):Void {
		if (closed)
			throw "builder is closed";
		curves.push(Curve.line(plane.toWorld(start), plane.toWorld(end)));
	}

	public function arc(start:Vector, middle:Vector, end:Vector):Void {
		if (closed)
			throw "builder is closed";
		curves.push(Curve.arc(plane.toWorld(start), plane.toWorld(middle), plane.toWorld(end)));
	}

	public function spline(points:Array<Vector>):Void {
		if (closed)
			throw "builder is closed";
		var world:Array<Vector> = [];
		for (p in points)
			world.push(plane.toWorld(p));
		curves.push(Curve.spline(world));
	}

	public function finish():Curve {
		if (closed)
			throw "builder is closed";
		return Curve.wire(curves);
	}

	public function close():Void {
		if (closed)
			return;
		closed = true;
		for (curve in curves)
			curve.close();
		curves.resize(0);
	}

	public static function build(callback:BuildLine->Void, ?plane:Plane):Curve {
		var builder = new BuildLine(plane);
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
