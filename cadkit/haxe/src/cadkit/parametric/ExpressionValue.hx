package cadkit.parametric;

class ExpressionValue {
	public final value:Float;
	public final kind:String;

	public function new(value:Float, kind:String) {
		this.value = value;
		this.kind = kind;
	}
}
