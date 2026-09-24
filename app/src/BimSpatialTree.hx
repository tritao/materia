package app;

import bimkit.BimDocument;
import bimkit.BimSchema;
import cadkit.parametric.Element;
import cadkit.parametric.ElementId;
import cadkit.parametric.TypedProperty;
import nativekit.ui.core.View;
import nativekit.ui.widgets.text.Text;
import nativekit.ui.widgets.collections.TreeRootMetadata;
import nativekit.ui.widgets.collections.TreeViewModel;

/** Editor tree projection over BIM aggregate, containment, and host relationships. */
class BimSpatialTree implements TreeViewModel {
	public static inline var ElementPrefix:String = "bim:element:";

	public final model:BimDocument;
	private var knownSignature:Null<String>;
	private var viewRevision:Int;

	public function new(model:BimDocument) {
		if (model == null)
			throw "BIM spatial tree needs a document";
		this.model = model;
		knownSignature = null;
		viewRevision = 1;
	}

	public function rootCount():Int
		return projects().length;

	public function rootRange(start:Int, count:Int):Array<TreeRootMetadata> {
		var result:Array<TreeRootMetadata> = [];
		var values = projects();
		var first = start < 0 ? 0 : start;
		var last = Std.int(Math.min(values.length, first + count));
		for (index in first...last)
			result.push(new TreeRootMetadata(key(values[index].id), true));
		return result;
	}

	public function rootKeyAt(index:Int):String {
		var values = projects();
		if (index < 0 || index >= values.length)
			throw "BIM project tree root index is out of range";
		return key(values[index].id);
	}

	public function childCount(parentKey:String):Int
		return children(parentKey).length;

	public function childKeyAt(parentKey:String, index:Int):String {
		var values = children(parentKey);
		if (index < 0 || index >= values.length)
			throw "BIM spatial tree child index is out of range";
		return key(values[index].id);
	}

	public function initiallyExpanded(nodeKey:String):Bool {
		var element = elementForKey(nodeKey);
		if (element == null)
			return false;
		var classification = bimClass(element);
		if (classification == BimSchema.Project || classification == BimSchema.Site || classification == BimSchema.Building
			|| classification == BimSchema.Storey)
			return true;
		return classification == BimSchema.Wall && model.openingsForWall(element.id).length > 0;
	}

	public function estimatedExtent():Float
		return 26.0;

	public function extentIsUniform():Bool
		return true;

	public function extentAt(_key:String):Float
		return 26.0;

	public function buildItem(nodeKey:String):View {
		var element = elementForKey(nodeKey);
		if (element == null)
			return new Text(nodeKey);
		var classification = bimClass(element);
		return new Text(element.name + (classification == "" ? "" : "  ·  " + classification));
	}

	public function revision():Int {
		var next = signature();
		if (knownSignature == null)
			knownSignature = next;
		else if (knownSignature != next) {
			knownSignature = next;
			viewRevision++;
		}
		return viewRevision;
	}

	public function elementIdForKey(nodeKey:String):Null<ElementId> {
		if (nodeKey == null || !StringTools.startsWith(nodeKey, ElementPrefix))
			return null;
		return new ElementId(nodeKey.substr(ElementPrefix.length));
	}

	private function projects():Array<Element> {
		var result:Array<Element> = [];
		for (element in model.cad.allElements())
			if (bimClass(element) == BimSchema.Project)
				result.push(element);
		result.sort(function(a, b) return Reflect.compare(a.id.value, b.id.value));
		return result;
	}

	private function children(parentKey:String):Array<Element> {
		var parent = elementForKey(parentKey);
		if (parent == null)
			return [];
		var classification = bimClass(parent);
		if (classification == BimSchema.Project || classification == BimSchema.Site || classification == BimSchema.Building)
			return model.aggregateChildren(parent.id);
		if (classification == BimSchema.Storey) {
			var result:Array<Element> = [];
			for (element in model.containedElements(parent.id))
				if (!isHostedOpening(element))
					result.push(element);
			return result;
		}
		if (classification == BimSchema.Wall) {
			var result:Array<Element> = [];
			for (opening in model.openingsForWall(parent.id))
				result.push(model.cad.element(opening.openingId));
			return result;
		}
		return [];
	}

	private function isHostedOpening(element:Element):Bool {
		var classification = bimClass(element);
		if (classification != BimSchema.Window && classification != BimSchema.Door)
			return false;
		for (relationship in model.allRelationships())
			if (relationship.openingId.value == element.id.value)
				return true;
		return false;
	}

	private function elementForKey(nodeKey:String):Null<Element> {
		var id = elementIdForKey(nodeKey);
		return id == null ? null : model.cad.findElement(id);
	}

	private function bimClass(element:Element):String {
		var property = element.property("bim.class");
		if (property == null || property.type != TypedProperty.TypeToken || property.tokenDomain != BimSchema.ElementClass)
			return "";
		return cast property.value;
	}

	private function key(id:ElementId):String
		return ElementPrefix + id.value;

	private function signature():String {
		var result = new StringBuf();
		for (element in model.cad.allElements()) {
			result.add(element.id.value);
			result.add("|");
			result.add(element.name);
			result.add("|");
			result.add(bimClass(element));
			result.add(";");
		}
		for (relationship in model.cad.allRelationships()) {
			result.add(relationship.typeName);
			result.add("|");
			result.add(relationship.source.elementId.value);
			result.add("|");
			result.add(relationship.target.elementId.value);
			result.add(";");
		}
		return result.toString();
	}
}
