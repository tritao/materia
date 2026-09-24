package bimkit;

import cadkit.parametric.Definition;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.DefinitionOutput;
import cadkit.parametric.Document;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.QuantityKind;
import cadkit.parametric.TypedProperty;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.TransformFeature;

/** Factory for an authored reusable window type built from a CadKit feature subgraph. */
class BimWindowDefinition {
	public static function create(document:Document, name:String, width:Float, height:Float, frameThickness:Float,
		depth:Float):Definition {
		var transaction = document.beginTransaction();
		var graph = new Document(null, false);
		try {
			var outer = graph.add(new BoxFeature(width, depth, height));
			var cavity = graph.add(new BoxFeature(width - 2 * frameThickness, depth + 2, height - 2 * frameThickness));
			var positionedCavity = graph.add(new TransformFeature(cavity, frameThickness, -1, frameThickness));
			var body = graph.add(new BooleanFeature(outer, positionedCavity, BooleanOperation.Cut));
			var openingBox = graph.add(new BoxFeature(width, depth + 2, height));
			var opening = graph.add(new TransformFeature(openingBox, 0, -1, 0));

			var bindings = new Map<String, String>();
			bindInput(graph, bindings, "width", width, outer.width);
			bindInput(graph, bindings, "height", height, outer.height);
			bindInput(graph, bindings, "frameThickness", frameThickness, positionedCavity.x);
			graph.parameter("frameThickness").bind(positionedCavity.z);
			bindInput(graph, bindings, "depth", depth, outer.depth);
			graph.parameter("width").bind(openingBox.width);
			graph.parameter("height").bind(openingBox.height);

			graph.defineTypedParameter("cut.overlap", 1, QuantityKind.Length, "mm");
			bindExpression(graph, "cavity.width", QuantityKind.Length, "width - 2 * frameThickness", cavity.width);
			bindExpression(graph, "cavity.height", QuantityKind.Length, "height - 2 * frameThickness", cavity.height);
			bindExpression(graph, "cavity.depth", QuantityKind.Length, "depth + 2 * cut.overlap", cavity.depth);
			bindExpression(graph, "cavity.y", QuantityKind.Length, "-cut.overlap", positionedCavity.y);
			bindExpression(graph, "opening.depth", QuantityKind.Length, "depth + 2 * cut.overlap", openingBox.depth);
			bindExpression(graph, "opening.y", QuantityKind.Length, "-cut.overlap", opening.y);

			var outputFeatures = new Map<String, Int>();
			outputFeatures.set("body", body.id.toInt());
			outputFeatures.set("opening", opening.id.toInt());
			var definition = document.createSubgraphDefinition(name, graph, [
				new DefinitionInput("width", ParameterKind.Length, "mm", width),
				new DefinitionInput("height", ParameterKind.Length, "mm", height),
				new DefinitionInput("frameThickness", ParameterKind.Length, "mm", frameThickness),
				new DefinitionInput("depth", ParameterKind.Length, "mm", depth)
			], [
				new DefinitionOutput("body", DefinitionOutput.Geometry),
				new DefinitionOutput("opening", DefinitionOutput.Tool)
			], bindings, outputFeatures);
			definition.setProperty(TypedProperty.token("bim.class", "window-type", BimSchema.DefinitionClass));
			definition.setProperty(TypedProperty.token("bim.element-class", BimSchema.Window, BimSchema.ElementClass));
			graph.close();
			transaction.commit();
			return definition;
		} catch (error:Dynamic) {
			graph.close();
			transaction.cancel();
			throw error;
		}
	}

	private static function bindInput(graph:Document, bindings:Map<String, String>, name:String, value:Float,
		parameter:cadkit.parametric.Parameter):Void {
		var named = graph.defineTypedParameter(name, value, QuantityKind.Length, "mm");
		named.bind(parameter);
		bindings.set(name, name);
	}

	private static function bindExpression(graph:Document, name:String, kind:String, expression:String,
		parameter:cadkit.parametric.Parameter):Void {
		graph.defineExpression(name, kind, "mm", expression).bind(parameter);
	}
}
