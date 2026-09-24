import cadkit.Shape;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Definition;
import cadkit.parametric.DefinitionEvaluator;
import cadkit.parametric.DefinitionEvaluatorRegistry;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.DefinitionOutput;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ElementKind;
import cadkit.parametric.Element;
import cadkit.parametric.ElementReference;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.PersistentReference;
import cadkit.parametric.Placement;
import cadkit.parametric.QuantityKind;
import cadkit.parametric.TypedProperty;
import cadkit.parametric.features.BoxFeature;

private class TypedPropertyBoxEvaluator implements DefinitionEvaluator {
	public function new() {}

	public function evaluate(definition:Definition, instance:InstanceElement, output:String):Shape {
		if (output != "body")
			throw "unexpected test definition output";
		var size = instance.resolved("size");
		return Shape.box(size, size, size);
	}
}

class TypedPropertySmoke {
	static function check(value:Bool, message:String):Void {
		if (!value)
			throw message;
	}

	static function property(element:Element, name:String):TypedProperty {
		var result = element.property(name);
		if (result == null)
			throw "missing test property: " + name;
		return result;
	}

	static function definitionProperty(definition:Definition, name:String):TypedProperty {
		var result = definition.property(name);
		if (result == null)
			throw "missing test definition property: " + name;
		return result;
	}

	public static function run():Void {
		DefinitionEvaluatorRegistry.register("cadkit.test.typed-property", new TypedPropertyBoxEvaluator());
		var document = new Document();
		var target = document.createObject("Target");
		var source = document.createObject("Source");
		check(target.kind == ElementKind.Object && target.output == null, "generic persistent objects have no geometry output");
		source.setPlacement(new Placement(new Plane(new Vector(10, 20, 30), Vector.X(), Vector.Z())));
		var placed = document.worldPlacement(source).location.plane.origin;
		check(placed.x == 10 && placed.y == 20 && placed.z == 30, "generic persistent objects support placement");

		var feature = document.add(new BoxFeature(2, 3, 4));
		var definition = document.createDefinition("Typed property type", "cadkit.test.typed-property",
			[new DefinitionInput("size", ParameterKind.Length, "mm", 10)], [new DefinitionOutput("body", DefinitionOutput.Geometry)]);
		definition.setProperty(TypedProperty.text("bim.class", "window-type"));
		definition.setProperty(TypedProperty.token("bim.fireRating", "EI60", "bim.fire-rating"));

		source.setProperty(TypedProperty.boolean("bim.loadBearing", true));
		source.setProperty(TypedProperty.integer("ifc.sequence", 17));
		source.setProperty(TypedProperty.text("label", "South opening"));
		source.setProperty(TypedProperty.token("bim.class", "window", "bim.element-class"));
		source.setProperty(TypedProperty.quantity("bim.width", QuantityKind.Length, 1800, "mm"));
		source.setProperty(TypedProperty.quantity("bim.area", QuantityKind.Area, 1.62, "m2"));
		source.setProperty(TypedProperty.quantity("bim.orientation", QuantityKind.Angle, 90, "deg"));
		source.setProperty(TypedProperty.quantity("bim.volume", QuantityKind.Volume, 0.03, "m3"));
		source.setProperty(TypedProperty.quantity("bim.factor", QuantityKind.Scalar, 0.75, "1"));
		source.setProperty(TypedProperty.elementReference("domain.target", new ElementReference(document.id, target.id)));
		source.setProperty(TypedProperty.definitionReference("domain.type", PersistentReference.definition(document, definition.id)));
		source.setProperty(TypedProperty.featureReference("domain.authored", PersistentReference.feature(document, feature.id)));
		source.setProperty(TypedProperty.placement("domain.placement", source.localPlacement));
		source.setProperty(new TypedProperty("ifc.Pset_Custom.UnknownMeasure", "ifc.positive-length", {wrappedValue: 4.5}, null, null,
			{propertySet: "Pset_Custom", sourceType: "IfcPositiveLengthMeasure"}));

		var transaction = document.beginTransaction();
		source.setProperty(TypedProperty.integer("ifc.sequence", 18));
		transaction.commit();
		check(property(source, "ifc.sequence").value == 18, "property update commits");
		check(document.undo() && property(source, "ifc.sequence").value == 17, "property update undo");
		check(document.redo() && property(source, "ifc.sequence").value == 18, "property update redo");

		var failed = false;
		try document.removeElement(target.id) catch (error:Dynamic) failed = true;
		check(failed, "element deletion rejects unresolved property references");

		var instance = document.createInstance("Window instance", definition);
		var unique = instance.makeUnique();
		check(definitionProperty(unique, "bim.class").value == "window-type",
			"makeUnique copies definition properties");

		var text = DocumentCodec.encode(document);
		var clone = DocumentCodec.decode(text, true, false);
		var clonedSource = clone.element(source.id);
		var clonedRef:PersistentReference = cast property(clonedSource, "domain.target").value;
		check(clone.id.value != document.id.value && clonedRef.documentId == clone.id.value && clonedRef.targetId == target.id.value,
			"cloning remaps same-document property references");
		var clonedTypeRef:PersistentReference = cast property(clonedSource, "domain.type").value;
		var clonedFeatureRef:PersistentReference = cast property(clonedSource, "domain.authored").value;
		check(clonedTypeRef.documentId == clone.id.value && clonedTypeRef.targetId == definition.id.value
			&& clonedFeatureRef.documentId == clone.id.value && clonedFeatureRef.targetId == Std.string(feature.id.toInt()),
			"definition and feature references clone with their document");
		var opaque = property(clonedSource, "ifc.Pset_Custom.UnknownMeasure");
		check(opaque.type == "ifc.positive-length" && Reflect.field(opaque.metadata, "sourceType") == "IfcPositiveLengthMeasure"
			&& Reflect.field(opaque.value, "wrappedValue") == 4.5, "unknown property schemas round-trip with type metadata");
		var clonedDefinitionProperty = clone.definition(unique.id).property("bim.class");
		check(clonedDefinitionProperty != null, "definition properties round-trip");
		clone.close();

		var reopened = DocumentCodec.decode(text, false, false);
		check(property(reopened.element(source.id), "bim.width").unit == "mm", "quantity unit round-trips");
		reopened.close();
		document.close();
	}
}
