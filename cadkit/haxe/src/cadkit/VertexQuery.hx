package cadkit;

import CadKit;
import cadkit.SelectionError;
import cadkit.SelectionErrorKind;
import cadkit.Shape;
import cadkit.Vertex;

/** Deterministic, lazy vertex selector. Returned vertices are caller-owned. */
class VertexQuery {
	private final owner:Shape;
	private var hasPoint:Bool;
	private var pointX:Float;
	private var pointY:Float;
	private var pointZ:Float;
	private var pointTolerance:Float;

	private function new(owner:Shape) {
		this.owner = owner;
		hasPoint = false;
		pointX = 0.0;
		pointY = 0.0;
		pointZ = 0.0;
		pointTolerance = 0.0;
	}

	public static function from(shape:Shape):VertexQuery {
		return new VertexQuery(shape.cloneShape());
	}

	/** Release the query owner without evaluating it. */
	public function close():Bool {
		return owner.close();
	}

	public function near(point:CadKit.Vec3, tolerance:Float):VertexQuery {
		validateTolerance(tolerance);
		hasPoint = true;
		pointX = point.get_x();
		pointY = point.get_y();
		pointZ = point.get_z();
		pointTolerance = tolerance;
		return this;
	}

	public function count():Int {
		var values = all();
		var result = values.length;
		for (value in values)
			value.close();
		return result;
	}

	public function all():Array<Vertex> {
		var values:Array<Vertex> = [];
		var collection = owner.vertices();
		for (index in 0...collection.count()) {
			var candidate = collection.at(index);
			if (matches(candidate))
				values.push(candidate);
			else
				candidate.close();
		}
		owner.close();
		return values;
	}

	public function first():Null<Vertex> {
		var collection = owner.vertices();
		for (index in 0...collection.count()) {
			var candidate = collection.at(index);
			if (matches(candidate)) {
				owner.close();
				return candidate;
			}
			candidate.close();
		}
		owner.close();
		return null;
	}

	public function unique():Vertex {
		var values = all();
		if (values.length == 1)
			return values[0];
		for (value in values)
			value.close();
		if (values.length == 0)
			throw new SelectionError(SelectionErrorKind.Empty, "vertex selector matched no vertices");
		throw new SelectionError(SelectionErrorKind.Ambiguous, "vertex selector matched multiple vertices");
	}

	private function matches(value:Vertex):Bool {
		if (!hasPoint)
			return true;
		var position = value.position();
		var dx = position.get_x() - pointX;
		var dy = position.get_y() - pointY;
		var dz = position.get_z() - pointZ;
		return dx * dx + dy * dy + dz * dz <= pointTolerance * pointTolerance;
	}

	private function validateTolerance(tolerance:Float):Void {
		if (!Math.isFinite(tolerance) || tolerance < 0.0)
			invalid("selector tolerance is invalid");
	}

	private function invalid(message:String):Void {
		owner.close();
		throw new SelectionError(SelectionErrorKind.Invalid, message);
	}
}
