package cadkit.parametric;

import cadkit.parametric.Parameter;

/** One coalesced parameter change in a transaction. */
class ParameterChange {
	public final parameter:Parameter;
	public final oldValue:Float;
	public var newValue:Float;

	public function new(parameter:Parameter, oldValue:Float, newValue:Float) {
		this.parameter = parameter;
		this.oldValue = oldValue;
		this.newValue = newValue;
	}
}
