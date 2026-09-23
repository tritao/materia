package bimkit;

import cadkit.Geometry;
import cadkit.Shape;
import cadkit.parametric.Definition;
import cadkit.parametric.DefinitionEvaluator;
import cadkit.parametric.DefinitionEvaluatorRegistry;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.DefinitionOutput;
import cadkit.parametric.Document;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.ParametricError;

/** BIM-owned reusable window recipe and its named geometry outputs. */
class BimWindowDefinition implements DefinitionEvaluator {
	private static var evaluatorRegistered:Bool = false;

	public static function registerEvaluator():Void {
		if (evaluatorRegistered)
			return;
		var evaluator = new BimWindowDefinition();
		DefinitionEvaluatorRegistry.register("bimkit.window", evaluator);
		DefinitionEvaluatorRegistry.register("window", evaluator);
		evaluatorRegistered = true;
	}

	public static function create(document:Document, name:String, width:Float, height:Float, frameThickness:Float, depth:Float):Definition {
		registerEvaluator();
		return document.createDefinition(name, "bimkit.window", [
			new DefinitionInput("width", ParameterKind.Length, "mm", width),
			new DefinitionInput("height", ParameterKind.Length, "mm", height),
			new DefinitionInput("frameThickness", ParameterKind.Length, "mm", frameThickness),
			new DefinitionInput("depth", ParameterKind.Length, "mm", depth)
		], [
			new DefinitionOutput("body", DefinitionOutput.Geometry),
			new DefinitionOutput("opening", DefinitionOutput.Tool)
		]);
	}

	public function new() {}

	public function evaluate(_definition:Definition, instance:InstanceElement, output:String):Shape {
		var width = instance.resolved("width");
		var height = instance.resolved("height");
		var frame = instance.resolved("frameThickness");
		var depth = instance.resolved("depth");
		if (width <= 2 * frame || height <= 2 * frame || depth <= 0 || frame <= 0)
			throw new ParametricError("window dimensions do not define a valid frame");
		if (output == "opening") {
			var opening = Shape.box(width, depth + 2, height);
			try {
				var result = opening.translate(Geometry.vec3(0, -1, 0));
				opening.close();
				return result;
			} catch (error:Dynamic) {
				opening.close();
				throw error;
			}
		}
		if (output != "body")
			throw new ParametricError("unsupported window output: " + output);

		var outer = Shape.box(width, depth, height);
		var inner = Shape.box(width - 2 * frame, depth + 2, height - 2 * frame);
		try {
			var positioned = inner.translate(Geometry.vec3(frame, -1, frame));
			inner.close();
			inner = positioned;
			var result = outer.cut(inner);
			outer.close();
			inner.close();
			return result;
		} catch (error:Dynamic) {
			outer.close();
			inner.close();
			throw error;
		}
	}
}
