package cadkit.modeling;

import CadKit;
import cadkit.Shape;
import cadkit.Edge;
import cadkit.Operation;
import cadkit.SelectionError;
import cadkit.SelectionErrorKind;

/** Owns its topology handles. Refinements mutate this selection and release rejects.
 * Terminal values are independent owners. Close selections even after unique()/all().
 */
class Selection {
	private var values:Array<Shape>;
	private var closed:Bool;

	public function new(values:Array<Shape>) {
		this.values = values;
		closed = false;
	}

	public static function from(shape:Shape, kind:CadKit.ShapeKind):Selection {
		var result = new Selection([]);
		try {
			for (i in 0...shape.subshapeCount(kind))
				result.appendOwned(shape.subshape(kind, i));
			return result;
		} catch (error:Dynamic) {
			result.close();
			throw error;
		}
	}

	public static function history(operation:Null<Operation>, kind:CadKit.ShapeKind, relation:CadKit.HistoryRelation):Selection {
		var result = new Selection([]);
		if (operation == null)
			return result;
		try {
			for (i in 0...operation.historyCount(relation)) {
				var target = operation.historyTargetAt(relation, i);
				try {
					if (target.kind() == kind)
						result.appendOwned(target.cloneShape());
					else
						for (j in 0...target.subshapeCount(kind))
							result.appendOwned(target.subshape(kind, j));
					target.close();
				} catch (error:Dynamic) {
					target.close();
					throw error;
				}
			}
			return result;
		} catch (error:Dynamic) {
			result.close();
			throw error;
		}
	}

	private function check():Void {
		if (closed)
			throw "selection is closed";
	}

	private function appendOwned(shape:Shape):Void {
		for (value in values)
			if (value.sameAs(shape)) {
				shape.close();
				return;
			}
		values.push(shape);
	}

	public function close():Void {
		if (closed)
			return;
		closed = true;
		for (value in values)
			value.close();
		values.resize(0);
	}

	public function count():Int {
		check();
		return values.length;
	}

	public function borrowShapes():Array<Shape> {
		check();
		return values.copy();
	}

	public function all():Array<Shape> {
		check();
		var result:Array<Shape> = [];
		try {
			for (value in values)
				result.push(value.cloneShape());
			return result;
		} catch (error:Dynamic) {
			for (value in result)
				value.close();
			throw error;
		}
	}

	public function unique():Shape {
		check();
		if (values.length == 0)
			throw new SelectionError(SelectionErrorKind.Empty, "selection matched no topology");
		if (values.length != 1)
			throw new SelectionError(SelectionErrorKind.Ambiguous, "selection matched multiple topology elements");
		return values[0].cloneShape();
	}

	public function at(index:Int):Shape {
		check();
		if (index < 0 || index >= values.length)
			throw "selection index out of bounds";
		return values[index].cloneShape();
	}

	public function filter(predicate:Shape->Bool):Selection {
		check();
		var keep:Array<Shape> = [];
		var reject:Array<Shape> = [];
		// Evaluate before releasing anything: a throwing predicate leaves selection intact.
		for (value in values) {
			if (predicate(value))
				keep.push(value);
			else
				reject.push(value);
		}
		values = keep;
		for (value in reject)
			value.close();
		return this;
	}

	public function curve(kind:CadKit.CurveKind):Selection {
		return filter(function(s) {
			return s.kind() == CadKit.ShapeKind.Edge && s.curveKind() == kind;
		});
	}

	public function surface(kind:CadKit.SurfaceKind):Selection {
		return filter(function(s) {
			return s.kind() == CadKit.ShapeKind.Face && s.surfaceKind() == kind;
		});
	}

	/** Parallel linear edges or planar face normals, never a curved shape's midpoint tangent. */
	public function parallel(axis:Axis, tolerance:Float = 0.000001):Selection {
		if (!Math.isFinite(tolerance) || tolerance < 0 || tolerance > 1)
			throw "invalid angular tolerance";
		return filter(function(s) {
			var direction:Vector;
			if (s.kind() == CadKit.ShapeKind.Edge && s.curveKind() == CadKit.CurveKind.Line)
				direction = Vector.fromNative(s.tangentAt());
			else if (s.kind() == CadKit.ShapeKind.Face && s.surfaceKind() == CadKit.SurfaceKind.Plane)
				direction = Vector.fromNative(s.faceNormal());
			else
				return false;
			return 1 - Math.abs(direction.normalized().dot(axis.direction)) <= tolerance;
		});
	}

	/** Faces use area centroid; other topology uses its bounding-box center. */
	public static function center(shape:Shape):Vector {
		if (shape.kind() == CadKit.ShapeKind.Face)
			return Vector.fromNative(shape.center());
		if (shape.kind() == CadKit.ShapeKind.Vertex)
			return Vector.fromNative(shape.position());
		var bounds = shape.bounds();
		return Vector.fromNative(bounds.get_min()).add(Vector.fromNative(bounds.get_max())).scale(0.5);
	}

	public static function measure(shape:Shape):Float {
		if (shape.kind() == CadKit.ShapeKind.Edge)
			return shape.edgeLength();
		if (shape.kind() == CadKit.ShapeKind.Face)
			return shape.faceArea();
		if (shape.kind() == CadKit.ShapeKind.Solid)
			return shape.volume();
		throw "measure requires edges, faces, or solids";
	}

	public function sortBy(key:Shape->Float):Selection {
		check();
		var records:Array<{shape:Shape, key:Float, index:Int}> = [];
		for (i in 0...values.length) {
			var k = key(values[i]);
			if (!Math.isFinite(k))
				throw "selection sort key must be finite";
			records.push({shape: values[i], key: k, index: i});
		}
		records.sort(function(a, b) {
			return a.key < b.key ? -1 : (a.key > b.key ? 1 : a.index - b.index);
		});
		values = [];
		for (record in records)
			values.push(record.shape);
		return this;
	}

	public function sortByAxis(axis:Axis):Selection {
		return sortBy(function(s) {
			return center(s).subtract(axis.origin).dot(axis.direction);
		});
	}

	public function sortBySize():Selection {
		return sortBy(measure);
	}

	/** Keep all tied extrema so unique() can report ambiguity. */
	public function extreme(axis:Axis, maximum:Bool = true, tolerance:Float = 0.000001):Selection {
		if (!Math.isFinite(tolerance) || tolerance < 0)
			throw "invalid selection tolerance";
		sortByAxis(axis);
		if (values.length == 0)
			return this;
		var target = center(values[maximum ? values.length - 1 : 0]).subtract(axis.origin).dot(axis.direction);
		return filter(function(s) {
			return Math.abs(center(s).subtract(axis.origin).dot(axis.direction) - target) <= tolerance;
		});
	}

	public function groupBy(key:Shape->Float, tolerance:Float = 0.000001):Array<Selection> {
		if (!Math.isFinite(tolerance) || tolerance < 0)
			throw "invalid grouping tolerance";
		sortBy(key);
		var groups:Array<Selection> = [];
		var anchor = 0.0;
		try {
			for (value in values) {
				var k = key(value);
				if (groups.length == 0 || Math.abs(k - anchor) > tolerance) {
					groups.push(new Selection([]));
					anchor = k;
				}
				groups[groups.length - 1].appendOwned(value.cloneShape());
			}
			return groups;
		} catch (error:Dynamic) {
			for (group in groups)
				group.close();
			throw error;
		}
	}

	public function children(kind:CadKit.ShapeKind):Selection {
		check();
		var result = new Selection([]);
		try {
			for (value in values)
				for (i in 0...value.subshapeCount(kind))
					result.appendOwned(value.subshape(kind, i));
			return result;
		} catch (error:Dynamic) {
			result.close();
			throw error;
		}
	}

	public function edgeValues():Array<Edge> {
		check();
		var result:Array<Edge> = [];
		try {
			for (value in values) {
				if (value.kind() != CadKit.ShapeKind.Edge)
					throw "finishing selection must contain edges";
				result.push(new Edge(value.cloneShape()));
			}
			return result;
		} catch (error:Dynamic) {
			for (edge in result)
				edge.close();
			throw error;
		}
	}
}
