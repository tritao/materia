package cadkit.parametric;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Axis;
import cadkit.modeling.Vector;
import cadkit.modeling.Selection;

/** Immutable geometric intent, evaluated afresh on each staged result.
 * This is a query, not a claim of persistent topological identity.
 */
class SelectionRecipe {
	public final kind:String;
	public final geometry:String;
	public final parallel:Null<Vector>;
	public final position:String;
	public final axis:Vector;
	public final expectedCount:Int;
	public final tolerance:Float;

	public function new(kind:String, geometry:String = "any", ?parallel:Vector, position:String = "all", ?axis:Vector, expectedCount:Int = 1,
			tolerance:Float = 0.000001) {
		if (kind != "edge" && kind != "face")
			throw new ParametricError("selection kind must be edge or face");
		if (geometry != "any"
			&& !(kind == "edge" && (geometry == "line" || geometry == "circle"))
			&& !(kind == "face" && geometry == "plane"))
			throw new ParametricError("unsupported selection geometry");
		if (position != "all" && position != "min" && position != "max" && position != "ends")
			throw new ParametricError("invalid selection position");
		if (expectedCount < 1 || !Math.isFinite(tolerance) || tolerance < 0)
			throw new ParametricError("invalid selection count or tolerance");
		this.kind = kind;
		this.geometry = geometry;
		this.parallel = parallel == null ? null : parallel.normalized();
		this.position = position;
		this.axis = axis == null ? Vector.Z() : axis.normalized();
		this.expectedCount = expectedCount;
		this.tolerance = tolerance;
	}

	/** Returns owned shapes; throws on changed cardinality, preserving ambiguity. */
	public function resolve(shape:Shape):Array<Shape> {
		var selection = Selection.from(shape, kind == "edge" ? CadKit.ShapeKind.Edge : CadKit.ShapeKind.Face);
		try {
			if (geometry == "line")
				selection.curve(CadKit.CurveKind.Line);
			if (geometry == "circle")
				selection.curve(CadKit.CurveKind.Circle);
			if (geometry == "plane")
				selection.surface(CadKit.SurfaceKind.Plane);
			if (parallel != null)
				selection.parallel(new Axis(new Vector(), parallel));
			var direction = new Axis(new Vector(), axis);
			if (position == "min" || position == "max")
				selection.extreme(direction, position == "max", tolerance);
			if (position == "ends" && selection.count() > 0) {
				selection.sortByAxis(direction);
				var first = selection.at(0);
				var last = selection.at(selection.count() - 1);
				var lo = 0.0;
				var hi = 0.0;
				try {
					lo = Selection.center(first).dot(axis);
					hi = Selection.center(last).dot(axis);
					first.close();
					last.close();
				} catch (error:Dynamic) {
					first.close();
					last.close();
					throw error;
				}
				selection.filter(function(s) {
					var value = Selection.center(s).dot(axis);
					return Math.abs(value - lo) <= tolerance || Math.abs(value - hi) <= tolerance;
				});
			}
			var count = selection.count();
			if (count != expectedCount)
				throw new ParametricError("selection expected " + expectedCount + " " + kind + " elements, matched " + count,
					count == 0 ? ReferenceState.Unresolved : ReferenceState.Ambiguous);
			var result = selection.all();
			selection.close();
			return result;
		} catch (error:Dynamic) {
			selection.close();
			throw error;
		}
	}
}
