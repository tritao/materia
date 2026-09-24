package cadkit.parametric.features;

import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.ElementReference;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.ParametricError;

/** Materializes one named definition output at an instance's local placement. */
class DefinitionOutputFeature extends Feature {
	public final instance:ElementReference;
	public final outputName:String;

	public function new(instance:ElementReference, outputName:String) {
		super();
		if (instance == null)
			throw new ParametricError("definition output requires an instance");
		if (outputName == null || StringTools.trim(outputName) == "")
			throw new ParametricError("definition output name must not be empty");
		this.instance = instance;
		this.outputName = outputName;
	}

	override public function serializationType():String
		return "definition-output";

	override public function elementDependencies():Array<String>
		return [instance.elementId.value];

	override public function elementReferences():Array<ElementReference>
		return [instance];

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var resolved = context.owner().resolveElement(instance);
		if (resolved.kind != "instance")
			throw new ParametricError("definition output reference is not an instance: " + instance.elementId.value);
		var typed:InstanceElement = cast resolved;
		var local = context.owner().definitionOutput(typed, outputName);
		return EvaluationResult.fromShape(typed.localPlacement.location.apply(local));
	}
}
