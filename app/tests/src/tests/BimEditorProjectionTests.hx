package tests;

import app.BimInspectorDescriptors;
import app.BimEditorDemo;
import app.BimModelEditor;
import app.BimSpatialTree;
import app.BimTypeTree;
import bimkit.BimDocument;
import bimkit.BimSchema;
import cadkit.parametric.ElementReference;
import cadkit.parametric.QuantityKind;
import cadkit.parametric.TypedProperty;
import nativekit.ui.core.CommandContext;
import nativekit.ui.core.EditorDocument;
import nativekit.ui.core.PropertyBinding;
import nativekit.ui.core.PropertyDescriptor;
import nativekit.ui.core.PropertyEditResult;
import nativekit.ui.core.PropertyType;
import nativekit.ui.core.PropertyValue;

class BimEditorProjectionTests {
	static function check(value:Bool, message:String):Void {
		if (!value)
			throw message;
	}

	static function descriptor(descriptors:Array<PropertyDescriptor>, suffix:String):PropertyDescriptor {
		for (value in descriptors)
			if (StringTools.endsWith(value.id, suffix))
				return value;
		throw "missing property inspector descriptor " + suffix;
	}

	static function propertyValue(element:cadkit.parametric.Element, name:String):Dynamic {
		var property = element.property(name);
		if (property == null)
			throw "missing property " + name;
		return property.value;
	}

	static function main():Int {
		var model = new BimDocument();
		var project = model.createProject("Project");
		var site = model.createSite("Site", project.id);
		var building = model.createBuilding("Building", site.id);
		var base = model.cad.createLevel("Base", 0);
		var top = model.cad.createLevel("Top", 3000);
		var storey = model.createStorey("Ground", building.id, new ElementReference(model.cad.id, base.id),
			new ElementReference(model.cad.id, top.id));
		var space = model.createSpace("Lobby", storey.id);
		var wall = model.createWall("Wall", 5000, 200, 3000);
		model.addToStorey(storey.id, wall.id);
		wall.setProperty(TypedProperty.boolean("bim.loadBearing", true));
		wall.setProperty(TypedProperty.quantity("bim.designThickness", QuantityKind.Length, 200, "mm"));
		wall.setProperty(TypedProperty.token("bim.material", "masonry", "bim.material"));
		var windowType = model.createWindowDefinition("Window Type", 1000, 1200, 80, 200);
		var first = model.createWindow("Window 01", windowType);
		var second = model.createWindow("Window 02", windowType);
		model.addToStorey(storey.id, first.id);
		model.addToStorey(storey.id, second.id);
		model.hostOpening(first, wall.id, 500, 900);
		model.hostOpening(second, wall.id, 3000, 900);

		var tree = new BimSpatialTree(model);
		check(tree.rootCount() == 1 && tree.childCount(tree.rootKeyAt(0)) == 1,
			"spatial tree projects Project → Site from aggregate relationships");
		var siteKey = tree.childKeyAt(tree.rootKeyAt(0), 0);
		var buildingKey = tree.childKeyAt(siteKey, 0);
		var storeyKey = tree.childKeyAt(buildingKey, 0);
		check(tree.childCount(storeyKey) == 2 && tree.childCount(BimSpatialTree.ElementPrefix + wall.id.value) == 2
			&& tree.elementIdForKey(tree.childKeyAt(BimSpatialTree.ElementPrefix + wall.id.value, 0)) != null
			&& tree.buildItem(storeyKey) != null,
			"spatial tree nests hosted openings under Walls while keeping containment in the Storey");
		var oldTreeRevision = tree.revision();
		wall.rename("Exterior Wall");
		check(tree.revision() > oldTreeRevision, "tree projection revision follows persistent element names");

		var typeTree = new BimTypeTree(model);
		check(typeTree.rootCount() == 1 && typeTree.childCount(typeTree.rootKeyAt(0)) == 2
			&& typeTree.buildItem(typeTree.rootKeyAt(0)) != null,
			"type tree projects one reusable Window Type to its CadKit instances");
		var editor = new BimModelEditor("bim-editor", model);
		check(!editor.showingTypes, "BIM editor starts with its spatial tree");
		editor.showTypeTree();
		check(editor.showingTypes, "BIM editor can switch to its type projection");
		editor.showSpatialTree();

		var properties = BimInspectorDescriptors.forElement(wall);
		var loadBearing = descriptor(properties, ":property:bim.loadBearing");
		var thickness = descriptor(properties, ":property:bim.designThickness");
		var material = descriptor(properties, ":property:bim.material");
		var levelReference = descriptor(BimInspectorDescriptors.forElement(storey), ":property:bim.baseLevel");
		check(loadBearing.type == PropertyType.Bool && thickness.type == PropertyType.Float && thickness.unit == "mm"
			&& material.type == PropertyType.Text && levelReference.readOnly,
			"generic property inspector maps typed values and persistent references to suitable editors");
		var editorDocument = new EditorDocument("bim-inspector-test");
		var binding = new PropertyBinding(loadBearing, new CommandContext(editorDocument));
		switch binding.apply(PropertyValue.Bool(false)) {
			case PropertyEditResult.Applied:
			default: throw "boolean BIM property edit was rejected";
		}
		check(propertyValue(wall, "bim.loadBearing") == false, "inspector descriptor writes a typed BIM property");
		check(model.undo() && propertyValue(wall, "bim.loadBearing") == true,
			"inspector property edits remain in CadKit's persistent undo history");
		check(model.redo() && propertyValue(wall, "bim.loadBearing") == false,
			"inspector property edits remain redoable");
		var definitionProperties = BimInspectorDescriptors.forDefinition(windowType);
		var widthInput = descriptor(definitionProperties, ":input:width");
		check(widthInput.type == PropertyType.Float && widthInput.unit == "mm",
			"type inspector exposes CAD definition inputs alongside persistent BIM properties");
		model.close();
		var demo = BimEditorDemo.create();
		check(demo.cad.allRelationships().length >= 10 && demo.cad.outputFeatureOrNull() == null,
			"app BIM panel starts with a populated relationship-based building model");
		demo.close();
		return 0;
	}
}
