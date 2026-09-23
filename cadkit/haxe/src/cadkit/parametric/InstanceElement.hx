package cadkit.parametric;

class InstanceElement extends Element {
	public var definitionId(default, null):DefinitionId;

	private final overrides:Map<String, Float>;

	public function new(document:Document, id:ElementId, name:String, definitionId:DefinitionId) {
		super(document, id, name, "instance");
		this.definitionId = definitionId;
		overrides = new Map();
	}

	public function overrideValue(name:String):Null<Float>
		return overrides.get(name);

	public function overrideNames():Array<String> {
		var result = [];
		for (name in overrides.keys())
			result.push(name);
		result.sort(Reflect.compare);
		return result;
	}

	public function resolved(name:String):Float {
		var value = overrides.get(name);
		return value == null ? document.definition(definitionId).input(name).defaultValue : value;
	}

	public function setOverride(name:String, value:Float, ?unit:String):Void
		document.setInstanceOverride(this, name, value, unit);

	/** Give this instance its own editable definition while preserving its identity and placement. */
	public function makeUnique(?definitionName:String):Definition
		return document.makeInstanceUnique(this, definitionName);

	public function removeOverride(name:String):Void
		document.removeInstanceOverride(this, name);

	public function restoreOverride(name:String, value:Null<Float>):Void {
		if (value == null)
			overrides.remove(name);
		else
			overrides.set(name, value);
	}

	public function restoreDefinitionId(value:DefinitionId):Void
		definitionId = value;
}
