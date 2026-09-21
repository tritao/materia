package cadkit;

import CadKit;
import cadkit.Edge;
import cadkit.SelectionError;
import cadkit.SelectionErrorKind;
import cadkit.Shape;

/** Deterministic, lazy edge selector. Returned edges are caller-owned. */
class EdgeQuery {
	private final owner:Shape;
	private var hasCurve:Bool;
	private var wantedCurve:CadKit.CurveKind;
	private var hasMinimumLength:Bool;
	private var minimumLength:Float;
	private var hasMaximumLength:Bool;
	private var maximumLength:Float;
	private var hasTangent:Bool;
	private var tangentX:Float;
	private var tangentY:Float;
	private var tangentZ:Float;
	private var tangentLengthSquared:Float;
	private var tangentTolerance:Float;
	private var hasPoint:Bool;
	private var pointX:Float;
	private var pointY:Float;
	private var pointZ:Float;
	private var pointTolerance:Float;

	private function new(owner:Shape) {
		this.owner = owner;
		hasCurve = false;
		wantedCurve = CadKit.CurveKind.Unknown;
		hasMinimumLength = false;
		minimumLength = 0.0;
		hasMaximumLength = false;
		maximumLength = 0.0;
		hasTangent = false;
		tangentX = 0.0;
		tangentY = 0.0;
		tangentZ = 0.0;
		tangentLengthSquared = 0.0;
		tangentTolerance = 0.01;
		hasPoint = false;
		pointX = 0.0;
		pointY = 0.0;
		pointZ = 0.0;
		pointTolerance = 0.0;
	}

	public static function from(shape:Shape):EdgeQuery {
		return new EdgeQuery(shape.cloneShape());
	}

	/** Release the query owner without evaluating it. */
	public function close():Bool {
		return owner.close();
	}

	public function curve(kind:CadKit.CurveKind):EdgeQuery {
		hasCurve = true;
		wantedCurve = kind;
		return this;
	}

	public function lengthAtLeast(minimum:Float):EdgeQuery {
		if (!Math.isFinite(minimum) || minimum < 0.0)
			invalid("edge length minimum is invalid");
		hasMinimumLength = true;
		minimumLength = minimum;
		return this;
	}

	public function lengthAtMost(maximum:Float):EdgeQuery {
		if (!Math.isFinite(maximum) || maximum < 0.0)
			invalid("edge length maximum is invalid");
		hasMaximumLength = true;
		maximumLength = maximum;
		return this;
	}

	public function tangentParallelTo(axis:CadKit.Vec3, tolerance:Float = 0.01):EdgeQuery {
		validateParallelTolerance(tolerance);
		hasTangent = true;
		var x = axis.get_x();
		var y = axis.get_y();
		var z = axis.get_z();
		var lengthSquared = x * x + y * y + z * z;
		if (!Math.isFinite(lengthSquared) || lengthSquared <= 0.0)
			invalid("edge selector axis is invalid");
		tangentX = x;
		tangentY = y;
		tangentZ = z;
		tangentLengthSquared = lengthSquared;
		tangentTolerance = tolerance;
		return this;
	}

	public function near(point:CadKit.Vec3, tolerance:Float):EdgeQuery {
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

	public function all():Array<Edge> {
		var values:Array<Edge> = [];
		var collection = owner.edges();
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

	public function first():Null<Edge> {
		var collection = owner.edges();
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

	public function unique():Edge {
		var values = all();
		if (values.length == 1)
			return values[0];
		for (value in values)
			value.close();
		if (values.length == 0)
			throw new SelectionError(SelectionErrorKind.Empty, "edge selector matched no edges");
		throw new SelectionError(SelectionErrorKind.Ambiguous, "edge selector matched multiple edges");
	}

	private function matches(value:Edge):Bool {
		if (hasCurve && value.curveKind() != wantedCurve)
			return false;
		if (hasMinimumLength && value.length() < minimumLength)
			return false;
		if (hasMaximumLength && value.length() > maximumLength)
			return false;
		if (hasTangent) {
			var tangent = value.tangentAt();
			var dot = tangent.get_x() * tangentX + tangent.get_y() * tangentY + tangent.get_z() * tangentZ;
			if (dot < 0.0)
				dot = -dot;
			var threshold = 1.0 - tangentTolerance;
			if (dot * dot < threshold * threshold * tangentLengthSquared)
				return false;
		}
		if (hasPoint) {
			var start = value.startPosition();
			var dx = start.get_x() - pointX;
			var dy = start.get_y() - pointY;
			var dz = start.get_z() - pointZ;
			if (dx * dx + dy * dy + dz * dz > pointTolerance * pointTolerance)
				return false;
		}
		return true;
	}

	private function validateTolerance(tolerance:Float):Void {
		if (!Math.isFinite(tolerance) || tolerance < 0.0)
			invalid("selector tolerance is invalid");
	}

	private function validateParallelTolerance(tolerance:Float):Void {
		if (!Math.isFinite(tolerance) || tolerance < 0.0 || tolerance > 1.0)
			invalid("parallel tolerance is invalid");
	}

	private function invalid(message:String):Void {
		owner.close();
		throw new SelectionError(SelectionErrorKind.Invalid, message);
	}
}
