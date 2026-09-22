package cadkit.parametric.features;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParametricError;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;

/** Editable world-coordinate path; point count and closure are fixed for this feature. */
class PolylineFeature extends Feature {
	private final coordinates:Array<Parameter>;

	public final closed:Bool;

	public function new(points:Array<Vector>, closed:Bool = false) {
		super();
		if (points.length < (closed ? 3 : 2))
			throw new ParametricError("polyline has too few points");
		this.closed = closed;
		coordinates = [];
		for (i in 0...points.length) {
			var p = points[i];
			coordinates.push(new Parameter(this, "polyline." + i + ".x", p.x, -1e300));
			coordinates.push(new Parameter(this, "polyline." + i + ".y", p.y, -1e300));
			coordinates.push(new Parameter(this, "polyline." + i + ".z", p.z, -1e300));
		}
	}

	public function coordinate(point:Int, axis:Int):Parameter {
		if (point < 0 || point >= pointCount() || axis < 0 || axis > 2)
			throw new ParametricError("polyline coordinate out of range");
		return coordinates[point * 3 + axis];
	}

	public function pointCount():Int {
		return Std.int(coordinates.length / 3);
	}

	public function points():Array<Vector> {
		var result:Array<Vector> = [];
		for (i in 0...pointCount())
			result.push(new Vector(coordinate(i, 0).value, coordinate(i, 1).value, coordinate(i, 2).value));
		return result;
	}

	override public function serializationType():String {
		return "polyline";
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var points:Array<CadKit.Vec3> = [];
		for (p in this.points())
			points.push(p.native());
		return EvaluationResult.fromShape(Shape.fromOwnedHandle(CadKit.polylineChecked(points, closed ? 1 : 0)));
	}
}
