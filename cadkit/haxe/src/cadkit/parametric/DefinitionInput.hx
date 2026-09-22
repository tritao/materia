package cadkit.parametric;

class DefinitionInput {
	public final name:String;
	public final kind:String;
	public final unit:String;
	public var defaultValue(default, null):Float;

	public function new(name:String, kind:String, unit:String, defaultValue:Float) {
		if (name == null || StringTools.trim(name) == "")
			throw new ParametricError("definition input name must not be empty");
		this.name = name;
		this.kind = ParameterKind.validate(kind);
		this.unit = UnitConversion.validateUnit(this.kind, unit);
		this.defaultValue = UnitConversion.toCanonical(defaultValue, this.kind, this.unit);
	}

	public function restore(value:Float):Void
		defaultValue = value;
}
