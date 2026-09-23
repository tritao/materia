package cadkit.parametric;

/** Serialized authored feature graph and the input/output ports exposed by a reusable definition. */
class DefinitionSubgraph {
	public final graph:String;
	private final inputBindings:Map<String, String>;
	private final outputFeatureIds:Map<String, Int>;

	public function new(graph:String, inputBindings:Map<String, String>, outputFeatureIds:Map<String, Int>) {
		if (graph == null || StringTools.trim(graph) == "")
			throw new ParametricError("definition subgraph must contain an encoded document");
		this.graph = graph;
		this.inputBindings = new Map();
		this.outputFeatureIds = new Map();
		for (name in inputBindings.keys())
			this.inputBindings.set(name, inputBindings.get(name));
		for (name in outputFeatureIds.keys())
			this.outputFeatureIds.set(name, outputFeatureIds.get(name));
	}

	public function parameterName(input:String):String {
		var result = inputBindings.get(input);
		if (result == null)
			throw new ParametricError("definition subgraph has no binding for input: " + input);
		return result;
	}

	public function featureId(output:String):Int {
		var result = outputFeatureIds.get(output);
		if (result == null)
			throw new ParametricError("definition subgraph has no feature for output: " + output);
		return result;
	}

	public function inputNames():Array<String> {
		var result = [for (name in inputBindings.keys()) name];
		result.sort(Reflect.compare);
		return result;
	}

	public function outputNames():Array<String> {
		var result = [for (name in outputFeatureIds.keys()) name];
		result.sort(Reflect.compare);
		return result;
	}
}
