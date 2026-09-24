package app;

import bimkit.BimDocument;
import bimkit.BimSchema;
import cadkit.parametric.Definition;
import cadkit.parametric.Element;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.TypedProperty;
import nativekit.ui.core.View;
import nativekit.ui.widgets.text.Text;
import nativekit.ui.widgets.collections.TreeRootMetadata;
import nativekit.ui.widgets.collections.TreeViewModel;

/** Type-centric tree projection from reusable definitions to their instances. */
class BimTypeTree implements TreeViewModel {
	public static inline var DefinitionPrefix:String = "bim:type:";
	public static inline var InstancePrefix:String = "bim:type-instance:";

	public final model:BimDocument;
	private var lastSignature:Null<String>;
	private var viewRevision:Int;

	public function new(model:BimDocument) {
		if (model == null)
			throw "BIM type tree needs a document";
		this.model = model;
		lastSignature = null;
		viewRevision = 1;
	}

	public function rootCount():Int
		return definitions().length;

	public function rootRange(start:Int, count:Int):Array<TreeRootMetadata> {
		var result:Array<TreeRootMetadata> = [];
		var values = definitions();
		var first = start < 0 ? 0 : start;
		var last = Std.int(Math.min(values.length, first + count));
		for (index in first...last)
			result.push(new TreeRootMetadata(definitionKey(values[index]), instanceCount(values[index]) > 0));
		return result;
	}

	public function rootKeyAt(index:Int):String {
		var values = definitions();
		if (index < 0 || index >= values.length)
			throw "BIM type tree root index is out of range";
		return definitionKey(values[index]);
	}

	public function childCount(parentKey:String):Int
		return instances(parentKey).length;

	public function childKeyAt(parentKey:String, index:Int):String {
		var values = instances(parentKey);
		if (index < 0 || index >= values.length)
			throw "BIM type tree child index is out of range";
		return InstancePrefix + values[index].id.value;
	}

	public function initiallyExpanded(key:String):Bool
		return StringTools.startsWith(key, DefinitionPrefix) && childCount(key) > 0;

	public function estimatedExtent():Float
		return 26.0;

	public function extentIsUniform():Bool
		return true;

	public function extentAt(_key:String):Float
		return 26.0;

	public function buildItem(key:String):View {
		var definition = definitionForKey(key);
		if (definition != null)
			return new Text(definition.name + "  ·  Type");
		var instance = instanceForKey(key);
		return new Text(instance == null ? key : instance.name);
	}

	public function revision():Int {
		var next = signature();
		if (lastSignature == null)
			lastSignature = next;
		else if (lastSignature != next) {
			lastSignature = next;
			viewRevision++;
		}
		return viewRevision;
	}

	public function definitionIdForKey(key:String):Null<String> {
		if (key == null || !StringTools.startsWith(key, DefinitionPrefix))
			return null;
		return key.substr(DefinitionPrefix.length);
	}

	public function instanceIdForKey(key:String):Null<String> {
		if (key == null || !StringTools.startsWith(key, InstancePrefix))
			return null;
		return key.substr(InstancePrefix.length);
	}

	private function definitions():Array<Definition> {
		var result:Array<Definition> = [];
		for (definition in model.cad.allDefinitions()) {
			var classification = definition.property("bim.class");
			if (classification != null && classification.type == TypedProperty.TypeToken
				&& classification.tokenDomain == BimSchema.DefinitionClass)
				result.push(definition);
		}
		result.sort(function(a, b) return Reflect.compare(a.id.value, b.id.value));
		return result;
	}

	private function instances(parentKey:String):Array<InstanceElement> {
		var definition = definitionForKey(parentKey);
		if (definition == null)
			return [];
		var result:Array<InstanceElement> = [];
		for (element in model.cad.allElements())
			if (element.kind == "instance") {
				var instance:InstanceElement = cast element;
				if (instance.definitionId.value == definition.id.value)
					result.push(instance);
			}
		result.sort(function(a, b) return Reflect.compare(a.id.value, b.id.value));
		return result;
	}

	private function instanceCount(definition:Definition):Int {
		var count = 0;
		for (element in model.cad.allElements())
			if (element.kind == "instance") {
				var instance:InstanceElement = cast element;
				if (instance.definitionId.value == definition.id.value)
					count++;
			}
		return count;
	}

	private function definitionForKey(key:String):Null<Definition> {
		var id = definitionIdForKey(key);
		if (id == null)
			return null;
		for (definition in model.cad.allDefinitions())
			if (definition.id.value == id)
				return definition;
		return null;
	}

	private function instanceForKey(key:String):Null<Element> {
		var id = instanceIdForKey(key);
		if (id == null)
			return null;
		for (element in model.cad.allElements())
			if (element.id.value == id)
				return element;
		return null;
	}

	private function definitionKey(definition:Definition):String
		return DefinitionPrefix + definition.id.value;

	private function signature():String {
		var result = new StringBuf();
		for (definition in definitions()) {
			result.add(definition.id.value);
			result.add("|");
			result.add(definition.name);
			result.add("|");
			result.add(definition.revision);
			result.add("|");
			result.add(instanceCount(definition));
			result.add(";");
		}
		return result.toString();
	}
}
