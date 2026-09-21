package cadkit;

import CadKit;
import cadkit.Face;
import cadkit.SelectionError;
import cadkit.SelectionErrorKind;
import cadkit.Shape;

/** Deterministic, lazy face selector. Returned faces are caller-owned. */
class FaceQuery {
	private final owner:Shape;
	private var hasSurface:Bool;
	private var wantedSurface:CadKit.SurfaceKind;
	private var hasNormal:Bool;
	private var normalX:Float;
	private var normalY:Float;
	private var normalZ:Float;
	private var normalLengthSquared:Float;
	private var normalTolerance:Float;
	private var hasMinimumArea:Bool;
	private var minimumArea:Float;
	private var hasMaximumArea:Bool;
	private var maximumArea:Float;
	private var hasCenter:Bool;
	private var centerX:Float;
	private var centerY:Float;
	private var centerZ:Float;
	private var centerTolerance:Float;

	private function new(owner:Shape) {
		this.owner = owner;
		hasSurface = false;
		wantedSurface = CadKit.SurfaceKind.Unknown;
		hasNormal = false;
		normalX = 0.0;
		normalY = 0.0;
		normalZ = 0.0;
		normalLengthSquared = 0.0;
		normalTolerance = 0.01;
		hasMinimumArea = false;
		minimumArea = 0.0;
		hasMaximumArea = false;
		maximumArea = 0.0;
		hasCenter = false;
		centerX = 0.0;
		centerY = 0.0;
		centerZ = 0.0;
		centerTolerance = 0.0;
	}

	public static function from(shape:Shape):FaceQuery {
		return new FaceQuery(shape.cloneShape());
	}

	/** Release the query owner without evaluating it. */
	public function close():Bool {
		return owner.close();
	}

	public function surface(kind:CadKit.SurfaceKind):FaceQuery {
		hasSurface = true;
		wantedSurface = kind;
		return this;
	}

	public function normalParallelTo(axis:CadKit.Vec3, tolerance:Float = 0.01):FaceQuery {
		validateParallelTolerance(tolerance);
		hasNormal = true;
		var x = axis.get_x();
		var y = axis.get_y();
		var z = axis.get_z();
		var lengthSquared = x * x + y * y + z * z;
		if (!Math.isFinite(lengthSquared) || lengthSquared <= 0.0)
			invalid("face selector axis is invalid");
		normalX = x;
		normalY = y;
		normalZ = z;
		normalLengthSquared = lengthSquared;
		normalTolerance = tolerance;
		return this;
	}

	public function areaAtLeast(minimum:Float):FaceQuery {
		if (!Math.isFinite(minimum) || minimum < 0.0)
			invalid("face area minimum is invalid");
		hasMinimumArea = true;
		minimumArea = minimum;
		return this;
	}

	public function areaAtMost(maximum:Float):FaceQuery {
		if (!Math.isFinite(maximum) || maximum < 0.0)
			invalid("face area maximum is invalid");
		hasMaximumArea = true;
		maximumArea = maximum;
		return this;
	}

	public function centerNear(point:CadKit.Vec3, tolerance:Float):FaceQuery {
		validateTolerance(tolerance);
		hasCenter = true;
		centerX = point.get_x();
		centerY = point.get_y();
		centerZ = point.get_z();
		centerTolerance = tolerance;
		return this;
	}

	public function count():Int {
		var values = all();
		var result = values.length;
		for (value in values)
			value.close();
		return result;
	}

	public function all():Array<Face> {
		var values:Array<Face> = [];
		var collection = owner.faces();
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

	public function first():Null<Face> {
		var collection = owner.faces();
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

	public function unique():Face {
		var values = all();
		if (values.length == 1)
			return values[0];
		for (value in values)
			value.close();
		if (values.length == 0)
			throw new SelectionError(SelectionErrorKind.Empty, "face selector matched no faces");
		throw new SelectionError(SelectionErrorKind.Ambiguous, "face selector matched multiple faces");
	}

	private function matches(value:Face):Bool {
		if (hasSurface && value.surfaceKind() != wantedSurface)
			return false;
		if (hasNormal) {
			var normal = value.normal();
			var dot = normal.get_x() * normalX + normal.get_y() * normalY + normal.get_z() * normalZ;
			if (dot < 0.0)
				dot = -dot;
			var threshold = 1.0 - normalTolerance;
			if (dot * dot < threshold * threshold * normalLengthSquared)
				return false;
		}
		if (hasMinimumArea && value.area() < minimumArea)
			return false;
		if (hasMaximumArea && value.area() > maximumArea)
			return false;
		if (hasCenter) {
			var center = value.center();
			var dx = center.get_x() - centerX;
			var dy = center.get_y() - centerY;
			var dz = center.get_z() - centerZ;
			if (dx * dx + dy * dy + dz * dz > centerTolerance * centerTolerance)
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
