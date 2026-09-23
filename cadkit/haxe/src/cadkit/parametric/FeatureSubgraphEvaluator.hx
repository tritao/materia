package cadkit.parametric;

import cadkit.Shape;

/** Evaluates a reusable definition from its authored feature graph and named ports. */
class FeatureSubgraphEvaluator implements DefinitionEvaluator {
	public function new() {}

	public function evaluate(definition:Definition, instance:InstanceElement, output:String):Shape {
		var subgraph = definition.subgraph;
		if (subgraph == null)
			throw new ParametricError("feature subgraph definition has no graph: " + definition.name);
		definition.output(output);
		var graph = DocumentCodec.decode(subgraph.graph, true, false);
		var result:Null<Shape> = null;
		try {
			for (input in definition.inputs()) {
				var parameter = graph.parameter(subgraph.parameterName(input.name));
				if (parameter.kind != input.kind)
					throw new ParametricError("definition input type no longer matches subgraph parameter: " + input.name);
				var value = UnitConversion.fromCanonical(instance.resolved(input.name), parameter.kind, parameter.unit);
				parameter.set(value, parameter.unit);
			}
			graph.recompute();
			var feature = graph.featureById(subgraph.featureId(output));
			if (feature == null || !feature.active)
				throw new ParametricError("definition output feature is missing or inactive: " + output);
			var shape = feature.currentShape();
			if (shape == null)
				throw new ParametricError("definition output has not produced geometry: " + output);
			result = shape.cloneShape();
			graph.close();
			return cast result;
		} catch (error:Dynamic) {
			graph.close();
			if (result != null)
				result.close();
			throw error;
		}
	}
}
