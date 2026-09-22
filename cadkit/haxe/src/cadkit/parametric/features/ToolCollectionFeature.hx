package cadkit.parametric.features;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Model;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.ParametricError;

/** A compound of individually identified Boolean tools. */
class ToolCollectionFeature extends Feature {
	public final tools:Array<Feature>;
	public final labels:Array<String>;

	public function new(tools:Array<Feature>, labels:Array<String>) {
		super();
		if (tools.length == 0 || tools.length != labels.length)
			throw new ParametricError("tool collection requires matching nonempty tools and labels");
		this.tools = tools.copy();
		this.labels = labels.copy();
		var seen = new Map<String, Bool>();
		for (label in labels) {
			if (label == null || StringTools.trim(label) == "" || seen.exists(label))
				throw new ParametricError("tool collection labels must be unique and nonempty");
			seen.set(label, true);
		}
	}

	override public function serializationType():String
		return "tool-collection";

	override public function dependencies():Array<FeatureId>
		return [for (tool in tools) tool.id];

	override public function dependencyFeatures():Array<Feature>
		return tools.copy();

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var copies:Array<Shape> = [];
		try {
			for (tool in tools)
				copies.push(context.shape(tool).cloneShape());
			var result = EvaluationResult.fromShape(Shape.fromOwnedHandle(CadKit.compoundChecked(Model.refs(copies))));
			for (copy in copies)
				copy.close();
			return result;
		} catch (error:Dynamic) {
			for (copy in copies)
				copy.close();
			throw error;
		}
	}
}
