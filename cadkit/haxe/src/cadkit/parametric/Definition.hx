package cadkit.parametric;

class Definition {
	public final document:Document;
	public final id:DefinitionId;
	public var name(default, null):String;
	public final recipe:String;
	public var revision(default, null):Int;

	private final inputsByName:Map<String, DefinitionInput>;
	private final orderedInputs:Array<DefinitionInput>;

	public function new(document:Document, id:DefinitionId, name:String, recipe:String, inputs:Array<DefinitionInput>) {
		if (name == null || StringTools.trim(name) == "")
			throw new ParametricError("definition name must not be empty");
		this.document = document;
		this.id = id;
		this.name = name;
		this.recipe = recipe;
		revision = 1;
		inputsByName = new Map();
		orderedInputs = [];
		for (input in inputs) {
			if (inputsByName.exists(input.name))
				throw new ParametricError("duplicate definition input: " + input.name);
			inputsByName.set(input.name, input);
			orderedInputs.push(input);
		}
	}

	public function input(name:String):DefinitionInput {
		var result = inputsByName.get(name);
		if (result == null)
			throw new ParametricError("unknown definition input: " + name);
		return result;
	}

	public function inputs():Array<DefinitionInput>
		return orderedInputs.copy();

	public function outputs():Array<String>
		return ["frame"];

	public function setDefault(name:String, value:Float, ?unit:String):Void
		document.setDefinitionDefault(this, name, value, unit);

	public function restoreDefault(name:String, value:Float, revision:Int):Void {
		input(name).restore(value);
		this.revision = revision;
	}

	public function restoreRevision(value:Int):Void
		revision = value;
}
