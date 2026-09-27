package cadkit.parametric;

/** One editable definition input, stored in canonical units when numeric. */
class DefinitionInput {
	public final name:String;
	public final kind:String;
	public final unit:String;
	public final allowedValues:Null<Array<String>>;
	public var defaultValue(default, null):Dynamic;

	public function new(name:String, kind:String, unit:String, defaultValue:Dynamic, ?allowedValues:Array<String>) {
		if (name == null || StringTools.trim(name) == "")
			throw new ParametricError("definition input name must not be empty");
		this.name = name;
		this.kind = switch kind {
			case TypedProperty.TypeBoolean, TypedProperty.TypeInteger, TypedProperty.TypeToken: kind;
			default: ParameterKind.validate(kind);
		};
		this.unit = isNumeric() ? UnitConversion.validateUnit(this.kind, unit) :
			(this.kind == TypedProperty.TypeInteger ? "count" : "1");
		if (this.kind == TypedProperty.TypeToken) {
			if (allowedValues == null || allowedValues.length == 0)
				throw new ParametricError("token input needs allowed values: " + name);
			var seen = new Map<String, Bool>();
			for (allowed in allowedValues) {
				if (allowed == null || allowed.length == 0 || seen.exists(allowed))
					throw new ParametricError("token input has invalid allowed values: " + name);
				seen.set(allowed, true);
			}
			this.allowedValues = allowedValues.copy();
		} else {
			if (allowedValues != null) throw new ParametricError("allowed values require a token input: " + name);
			this.allowedValues = null;
		}
		this.defaultValue = normalize(defaultValue, this.unit);
	}

	public static function boolean(name:String, value:Bool):DefinitionInput
		return new DefinitionInput(name, TypedProperty.TypeBoolean, "1", value);

	public static function integer(name:String, value:Int):DefinitionInput
		return new DefinitionInput(name, TypedProperty.TypeInteger, "count", value);

	public static function token(name:String, value:String, allowedValues:Array<String>):DefinitionInput
		return new DefinitionInput(name, TypedProperty.TypeToken, "1", value, allowedValues);

	public function isNumeric():Bool return kind != TypedProperty.TypeBoolean &&
		kind != TypedProperty.TypeInteger && kind != TypedProperty.TypeToken;

	public function normalize(value:Dynamic, ?sourceUnit:String):Dynamic {
		if (isNumeric()) {
			if (!Std.isOfType(value, Int) && !Std.isOfType(value, Float))
				throw new ParametricError("numeric input requires a number: " + name);
			return UnitConversion.toCanonical(cast value, kind, sourceUnit == null ? unit : sourceUnit);
		}
		if (sourceUnit != null && sourceUnit != unit)
			throw new ParametricError("unit is not valid for input: " + name);
		switch kind {
			case TypedProperty.TypeBoolean:
				if (!Std.isOfType(value, Bool)) throw new ParametricError("boolean input requires a bool: " + name);
			case TypedProperty.TypeInteger:
				if (!Std.isOfType(value, Int)) throw new ParametricError("integer input requires an integer: " + name);
			case TypedProperty.TypeToken:
				if (!Std.isOfType(value, String) || allowedValues.indexOf(cast(value, String)) < 0)
					throw new ParametricError("token is outside allowed values: " + name);
		}
		return value;
	}

	public function restore(value:Dynamic):Void
		defaultValue = value;
}
