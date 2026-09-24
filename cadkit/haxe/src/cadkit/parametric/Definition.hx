package cadkit.parametric;

class Definition {
	public static inline var SubgraphRecipe:String = "cadkit.subgraph";

	public final document:Document;
	public final id:DefinitionId;
	public var name(default, null):String;
	public final recipe:String;
	public var revision(default, null):Int;
	public var subgraph(default, null):Null<DefinitionSubgraph>;

	private final inputsByName:Map<String, DefinitionInput>;
	private final orderedInputs:Array<DefinitionInput>;
	private final outputsByName:Map<String, DefinitionOutput>;
	private final orderedOutputs:Array<DefinitionOutput>;
	private final propertyValues:Map<String, TypedProperty>;

	public function new(document:Document, id:DefinitionId, name:String, recipe:String, inputs:Array<DefinitionInput>,
		outputs:Array<DefinitionOutput>, ?subgraph:DefinitionSubgraph) {
		if (name == null || StringTools.trim(name) == "")
			throw new ParametricError("definition name must not be empty");
		this.document = document;
		this.id = id;
		this.name = name;
		this.recipe = recipe;
		this.subgraph = subgraph;
		revision = 1;
		inputsByName = new Map();
		orderedInputs = [];
		outputsByName = new Map();
		orderedOutputs = [];
		propertyValues = new Map();
		for (input in inputs) {
			if (inputsByName.exists(input.name))
				throw new ParametricError("duplicate definition input: " + input.name);
			inputsByName.set(input.name, input);
			orderedInputs.push(input);
		}
		for (output in outputs) {
			if (outputsByName.exists(output.name))
				throw new ParametricError("duplicate definition output: " + output.name);
			outputsByName.set(output.name, output);
			orderedOutputs.push(output);
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

	public function output(name:String):DefinitionOutput {
		var result = outputsByName.get(name);
		if (result == null)
			throw new ParametricError("unknown definition output: " + name);
		return result;
	}

	public function outputs():Array<DefinitionOutput>
		return orderedOutputs.copy();

	public function property(name:String):Null<TypedProperty>
		return propertyValues.get(name);

	public function properties():Array<TypedProperty> {
		var names = [for (name in propertyValues.keys()) name];
		names.sort(Reflect.compare);
		return [for (name in names) propertyValues.get(name)];
	}

	/** Internal document history and codec path. */
	public function restoreProperty(name:String, value:Null<TypedProperty>):Void {
		if (value == null)
			propertyValues.remove(name);
		else
			propertyValues.set(name, value);
	}

	public function setProperty(value:TypedProperty):Void
		document.setDefinitionProperty(this, value);

	public function removeProperty(name:String):Void
		document.removeDefinitionProperty(this, name);

	public function primaryGeometryOutput():DefinitionOutput {
		for (output in orderedOutputs)
			if (output.purpose == DefinitionOutput.Geometry)
				return output;
		throw new ParametricError("definition has no geometry output: " + name);
	}

	public function setDefault(name:String, value:Float, ?unit:String):Void
		document.setDefinitionDefault(this, name, value, unit);

	public function restoreDefault(name:String, value:Float, revision:Int):Void {
		input(name).restore(value);
		this.revision = revision;
	}

	public function restoreRevision(value:Int):Void
		revision = value;
}
