package cadkit.parametric;

import cadkit.parametric.ParametricError;
import cadkit.parametric.Feature;

/** Validated scalar parameter owned by one feature. */
class Parameter {
	public final name:String;
	public final minimum:Float;
	public final integer:Bool;
	public final maximum:Float;
	public final kind:String;
	public var value(default, null):Float;

	private final owner:Feature;
	private var registered:Bool;

	public function new(owner:Feature, name:String, value:Float, minimum:Float, integer:Bool = false, maximum:Float = 1e300,
		kind:String = ParameterKind.Scalar) {
		this.owner = owner;
		this.name = name;
		this.minimum = minimum;
		this.integer = integer;
		this.maximum = maximum;
		this.kind = ParameterKind.validate(kind);
		registered = true;
		validate(value);
		this.value = value;
		owner.registerParameter(this);
	}

	public function set(next:Float):Void {
		ensureRegistered();
		validate(next);
		if (owner.document != null && owner.document.setNamedParameter(this, next))
			return;
		if (next == value)
			return;

		var previous = value;
		value = next;
		owner.parameterChanged(this, previous);
	}

	/** Internal undo/redo path; does not create another history record. */
	public function restore(next:Float):Void {
		ensureRegistered();
		validate(next);
		if (next == value)
			return;
		value = next;
		owner.parameterRestored(this);
	}

	public function ownerFeature():Feature {
		return owner;
	}

	public function setRegistered(value:Bool):Void {
		registered = value;
	}

	public function isRegistered():Bool {
		return registered;
	}

	public function validateValue(value:Float):Void {
		validate(value);
	}

	private function validate(candidate:Float):Void {
		if (!Math.isFinite(candidate) || candidate <= minimum)
			throw new ParametricError(name + " must be finite and greater than " + Std.string(minimum));
		if (candidate > maximum)
			throw new ParametricError(name + " must not exceed " + Std.string(maximum));
		if (integer && candidate != Std.int(candidate))
			throw new ParametricError(name + " must be an integer");
	}

	private function ensureRegistered():Void {
		if (!registered)
			throw new ParametricError("feature parameter is no longer active: " + name);
	}
}
