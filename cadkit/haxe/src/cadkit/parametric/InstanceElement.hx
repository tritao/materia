package cadkit.parametric;

class InstanceElement extends Element {
	public var definitionId(default, null):DefinitionId;

	private final overrides:Map<String, Dynamic>;

	public function new(document:Document, id:ElementId, name:String, definitionId:DefinitionId) {
		super(document, id, name, ElementKind.Instance);
		this.definitionId = definitionId;
		overrides = new Map();
	}

	public function overrideValue(name:String):Null<Float>
		return cast overrides.get(name);

	public function typedOverrideValue(name:String):Dynamic return overrides.get(name);

	public function overrideNames():Array<String> {
		var result = [];
		for (name in overrides.keys())
			result.push(name);
		result.sort(Reflect.compare);
		return result;
	}

	public function resolved(name:String):Float {
		var input = document.definition(definitionId).input(name);
		if (!input.isNumeric()) throw new ParametricError("definition input is not numeric: " + name);
		return cast resolvedValue(name);
	}

	public function resolvedValue(name:String):Dynamic {
		var value = overrides.get(name);
		return value == null ? document.definition(definitionId).input(name).defaultValue : value;
	}

	public function resolvedBoolean(name:String):Bool {
		if (document.definition(definitionId).input(name).kind != TypedProperty.TypeBoolean)
			throw new ParametricError("definition input is not boolean: " + name);
		return cast resolvedValue(name);
	}

	public function resolvedInteger(name:String):Int {
		if (document.definition(definitionId).input(name).kind != TypedProperty.TypeInteger)
			throw new ParametricError("definition input is not integer: " + name);
		return cast resolvedValue(name);
	}

	public function resolvedToken(name:String):String {
		if (document.definition(definitionId).input(name).kind != TypedProperty.TypeToken)
			throw new ParametricError("definition input is not a token: " + name);
		return cast resolvedValue(name);
	}

	public function setOverride(name:String, value:Float, ?unit:String):Void
		document.setInstanceOverride(this, name, value, unit);

	public function setTypedOverride(name:String, value:Dynamic):Void
		document.setInstanceOverrideTyped(this, name, value);

	public function connector(name:String):Placement
		return document.worldPlacement(this).compose(DefinitionEvaluatorRegistry.connector(
			document.definition(definitionId), this, name));

	/** Give this instance its own editable definition while preserving its identity and placement. */
	public function makeUnique(?definitionName:String):Definition
		return document.makeInstanceUnique(this, definitionName);

	public function removeOverride(name:String):Void
		document.removeInstanceOverride(this, name);

	public function restoreOverride(name:String, value:Dynamic):Void {
		if (value == null)
			overrides.remove(name);
		else
			overrides.set(name, value);
	}

	public function restoreDefinitionId(value:DefinitionId):Void
		definitionId = value;
}
