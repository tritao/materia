package bimkit;

import cadkit.parametric.Definition;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.DefinitionOutput;
import cadkit.parametric.Document;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.QuantityKind;
import cadkit.parametric.TypedProperty;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.TransformFeature;

/** Factory for an authored reusable door type with a panel body and hosted opening tool. */
class BimDoorDefinition {
	public static function create(document:Document, name:String, width:Float, height:Float, depth:Float):Definition {
		var transaction = document.beginTransaction();
		var graph = new Document(null, false);
		try {
			var body = graph.add(new BoxFeature(width, depth, height));
			var openingBox = graph.add(new BoxFeature(width, depth + 2, height));
			var opening = graph.add(new TransformFeature(openingBox, 0, -1, 0));

			var bindings = new Map<String, String>();
			bindInput(graph, bindings, "width", width, body.width);
			bindInput(graph, bindings, "height", height, body.height);
			bindInput(graph, bindings, "depth", depth, body.depth);
			graph.parameter("width").bind(openingBox.width);
			graph.parameter("height").bind(openingBox.height);
			graph.defineTypedParameter("cut.overlap", 1, QuantityKind.Length, "mm");
			graph.defineExpression("opening.depth", QuantityKind.Length, "mm", "depth + 2 * cut.overlap").bind(openingBox.depth);
			graph.defineExpression("opening.y", QuantityKind.Length, "mm", "-cut.overlap").bind(opening.y);

			var outputFeatures = new Map<String, Int>();
			outputFeatures.set("body", body.id.toInt());
			outputFeatures.set("opening", opening.id.toInt());
			var definition = document.createSubgraphDefinition(name, graph, [
				new DefinitionInput("width", ParameterKind.Length, "mm", width),
				new DefinitionInput("height", ParameterKind.Length, "mm", height),
				new DefinitionInput("depth", ParameterKind.Length, "mm", depth)
			], [
				new DefinitionOutput("body", DefinitionOutput.Geometry),
				new DefinitionOutput("opening", DefinitionOutput.Tool)
			], bindings, outputFeatures);
			definition.setProperty(TypedProperty.token("bim.class", BimSchema.DoorType, BimSchema.DefinitionClass));
			definition.setProperty(TypedProperty.token("bim.element-class", BimSchema.Door, BimSchema.ElementClass));
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
}
