package cadkit.parametric;

import cadkit.parametric.ParametricError;
import cadkit.parametric.Feature;

/** Validated scalar parameter owned by one feature. */
class Parameter {
	public final name:String;
	public final minimum:Float;
	public var value(default, null):Float;

	private final owner:Feature;

	public function new(owner:Feature, name:String, value:Float, minimum:Float) {
		this.owner = owner;
		this.name = name;
		this.minimum = minimum;
		validate(value);
		this.value = value;
	}

	public function set(next:Float):Void {
		validate(next);
		if (next == value)
			return;

		var previous = value;
		value = next;
		owner.parameterChanged(this, previous);
	}

	/** Internal undo/redo path; does not create another history record. */
	public function restore(next:Float):Void {
		validate(next);
		if (next == value)
			return;
		value = next;
		owner.parameterRestored(this);
	}

	private function validate(candidate:Float):Void {
		if (!Math.isFinite(candidate) || candidate <= minimum)
			throw new ParametricError(
				name + " must be finite and greater than " + Std.string(minimum));
	}
}
