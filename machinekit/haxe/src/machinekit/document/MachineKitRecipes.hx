package machinekit.document;

import haxe.Json;
import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.parametric.Definition;
import cadkit.parametric.DefinitionConnectorEvaluator;
import cadkit.parametric.DefinitionEvaluator;
import cadkit.parametric.DefinitionEvaluatorRegistry;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.Placement;
import cadkit.parametric.UnitConversion;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.MachineKitComponents;
import machinekit.component.ToolSpec;

/** CadKit evaluator registration for editable MachineKit single-part recipes. */
class MachineKitRecipes {
	static var evaluators:Map<String, MachineKitRecipeEvaluator> = [];
	static var components:Map<String, {key:String, component:MachineComponent}> = [];
	static var watchedDocuments:Map<String, Bool> = [];
	static var legacyMigrationRegistered:Bool = false;

	public static function register():Void {
		if (!legacyMigrationRegistered) {
			DocumentCodec.registerMigration("machinekit.legacy-derived-properties", migrateLegacyProperties);
			legacyMigrationRegistered = true;
		}
		for (type in MachineKitComponents.all()) {
			var evaluator = evaluators.get(type.id);
			if (evaluator == null) {
				evaluator = new MachineKitRecipeEvaluator(type);
				evaluators.set(type.id, evaluator);
			}
			DefinitionEvaluatorRegistry.register(type.id, evaluator, evaluator);
		}
	}

	public static function typeOrNull(id:String):Null<ComponentType> {
		for (type in MachineKitComponents.all()) if (type.id == id) return type;
		return null;
	}

	static function setOptional(values:ComponentValues, name:String,
			inner:ComponentParameterType, text:String):Void {
		var decoded:Dynamic = Json.parse(text);
		if (decoded == null) { values.setUnset(name); return; }
		switch inner {
			case Scalar | Length | Angle: values.setNumber(name, cast decoded);
			case Text | Choice(_) | CatalogDesignation(_): values.setToken(name, cast decoded);
			case _: throw 'Unsupported optional recipe input "$name"';
		}
	}

	public static function component(instance:InstanceElement):MachineComponent {
		var type = typeOrNull(instance.document.definition(instance.definitionId).recipe);
		if (type == null) throw 'Instance is not a MachineKit recipe: ${instance.id.value}';
		var documentIdentity = Std.string(instance.document.runtimeIdentity());
		watchDocument(instance.document, documentIdentity);
		var identity = documentIdentity + ":" + instance.id.value;
		var key = instance.document.resolvedInputKey(instance);
		var cached = components.get(identity);
		if (cached != null && cached.key == key) return cached.component;
		var values = new ComponentValues();
		for (parameter in type.parameters()) {
			var raw = instance.resolvedValue(parameter.name);
				switch parameter.type {
				case Optional(inner): setOptional(values, parameter.name, inner, cast raw);
				case Scalar | Length | Angle: values.setNumber(parameter.name, cast raw);
				case Count: values.setInteger(parameter.name, cast raw);
				case Bool: values.setBoolean(parameter.name, cast raw);
				case Text | Choice(_) | CatalogDesignation(_): values.setToken(parameter.name, cast raw);
			}
		}
		var built = type.create(values);
		components.set(identity, {key: key, component: built});
		return built;
	}

	/** Release cached recipe objects owned by a document that is being closed. */
	public static function forget(document:cadkit.parametric.Document):Void {
		if (document == null) return;
		var prefix = Std.string(document.runtimeIdentity()) + ":";
		var forgotten = [for (identity in components.keys()) if (StringTools.startsWith(identity, prefix)) identity];
		for (identity in forgotten) components.remove(identity);
	}

	public static function toolValues(definition:Definition, instance:InstanceElement,
			tool:ToolSpec):ComponentValues {
		var values = tool.defaults();
		var inputNames:Map<String, Bool> = [];
		for (input in definition.inputs()) inputNames.set(input.name, true);
		for (parameter in tool.parameters()) {
			var name = ToolSpec.inputName(tool.name, parameter.name);
			if (!inputNames.exists(name)) continue;
			var raw = instance.resolvedValue(name);
			switch parameter.type {
			case Optional(inner): setOptional(values, parameter.name, inner, cast raw);
			case Scalar | Length | Angle: values.setNumber(parameter.name, cast raw);
				case Count: values.setInteger(parameter.name, cast raw);
				case Bool: values.setBoolean(parameter.name, cast raw);
				case Text | Choice(_) | CatalogDesignation(_): values.setToken(parameter.name, cast raw);
			}
		}
		return tool.resolve(values);
	}

	/** Merge explicit saved edits onto freshly generated recipe definitions. */
	public static function reconcile(freshText:String, savedText:String,
			diagnostics:Array<String>):{text:String, changed:Bool} {
		register();
		var fresh = DocumentCodec.decode(freshText);
		try {
			var changed = reconcileDocument(fresh, savedText, diagnostics);
			var text = DocumentCodec.encode(fresh);
			fresh.close();
			return {text: text, changed: changed};
		} catch (error:Dynamic) {
			fresh.close();
			throw error;
		}
	}

	/** Reconcile in place when the caller already owns a fresh generated document. */
	public static function reconcileDocument(fresh:cadkit.parametric.Document, savedText:String,
			diagnostics:Array<String>):Bool {
		register();
		var tracePath = Sys.getEnv("MATERIA_RECONCILE_TRACE");
		if (tracePath != null && tracePath.length > 0) {
			var previous = sys.FileSystem.exists(tracePath) ? sys.io.File.getContent(tracePath) : "";
			sys.io.File.saveContent(tracePath, previous + "reconcile\n");
		}
		var savedVersion = documentVersion(savedText);
		var saved:cadkit.parametric.Document;
		try saved = DocumentCodec.decode(savedText) catch (error:Dynamic) throw error;
		try {
			var freshInstances = recipeInstances(fresh);
			var savedInstances = recipeInstances(saved);
			var originalValues:Map<String, String> = new Map();
			for (id in freshInstances.keys()) originalValues.set(id, resolvedInputSignature(freshInstances.get(id)));
			var entries:Array<RecipeReconcileEntry> = [];
			for (id in savedInstances.keys()) {
				var prior = savedInstances.get(id);
				var current = freshInstances.get(id);
				if (current == null) {
					diagnostics.push('Saved recipe occurrence "$id" no longer exists in source');
					continue;
				}
				var oldDefinition = saved.definition(prior.definitionId);
				var definition = fresh.definition(current.definitionId);
				if (oldDefinition.recipe != definition.recipe) {
					diagnostics.push('Saved recipe occurrence "$id" changed type in source');
					continue;
				}
				entries.push({id: id, prior: prior, current: current, oldDefinition: oldDefinition,
					freshDefinition: definition});
			}

			// Keep the largest saved group on the source definition. Any saved groups
			// split off with makeUnique() get their own fresh definition as well.
			var groups:Map<String, Array<RecipeReconcileEntry>> = new Map();
			var groupsByFreshDefinition:Map<String, Array<String>> = new Map();
			var freshOccurrenceCounts:Map<String, Int> = new Map();
			for (instance in freshInstances)
				freshOccurrenceCounts.set(instance.definitionId.value,
					(freshOccurrenceCounts.get(instance.definitionId.value) ?? 0) + 1);
			for (entry in entries) {
				var groupKey = entry.freshDefinition.id.value + ":" + entry.oldDefinition.id.value;
				var group = groups.get(groupKey);
				if (group == null) {
					group = [];
					groups.set(groupKey, group);
					var freshGroups = groupsByFreshDefinition.get(entry.freshDefinition.id.value);
					if (freshGroups == null) {
						freshGroups = [];
						groupsByFreshDefinition.set(entry.freshDefinition.id.value, freshGroups);
					}
					freshGroups.push(groupKey);
				}
				group.push(entry);
			}

			var targetDefinitions:Map<String, Definition> = new Map();
			for (freshDefinitionId in groupsByFreshDefinition.keys()) {
				var freshGroups = groupsByFreshDefinition.get(freshDefinitionId);
				var retainedGroup:String = null, retainedSize = -1;
				var matchedCount = 0;
				for (groupKey in freshGroups) {
					var matchedGroup = groups.get(groupKey);
					if (matchedGroup == null) continue;
					var size = matchedGroup.length;
					matchedCount += size;
					if (size > retainedSize) {
						retainedSize = size;
						retainedGroup = groupKey;
					}
				}
				if (matchedCount != freshOccurrenceCounts.get(freshDefinitionId)) retainedGroup = null;
				for (groupKey in freshGroups) {
					var group = groups.get(groupKey);
					if (group == null || group.length == 0) continue;
					var target = group[0].freshDefinition;
					if (groupKey != retainedGroup) {
						target = group[0].current.makeUnique(group[0].current.name + " saved definition");
						for (index in 1...group.length)
							fresh.restoreInstanceDefinition(group[index].current, target.id);
					}
					targetDefinitions.set(groupKey, target);
				}
			}

			var processedDefaults:Map<String, Bool> = new Map();
			var reportedLegacyDefaults = false;
			for (entry in entries) {
				var groupKey = entry.freshDefinition.id.value + ":" + entry.oldDefinition.id.value;
				var definition = targetDefinitions.get(groupKey);
				var freshInputs:Map<String, DefinitionInput> = new Map();
				for (input in definition.inputs()) freshInputs.set(input.name, input);

				if (!processedDefaults.exists(groupKey)) {
					processedDefaults.set(groupKey, true);
					var oldInputs:Map<String, DefinitionInput> = new Map();
					for (input in entry.oldDefinition.inputs()) oldInputs.set(input.name, input);
					for (input in definition.inputs()) {
						var oldInput = oldInputs.get(input.name);
						if (oldInput == null) {
							diagnostics.push('Saved recipe definition for "${entry.id}" has new input "${input.name}" in source; using its new default');
							continue;
						}
						if (oldInput.kind != input.kind) {
							diagnostics.push('Saved input "${input.name}" on "${entry.id}" changed type in source');
							continue;
						}
						var value = input.isNumeric()
							? UnitConversion.fromCanonical(oldInput.defaultValue, input.kind, input.unit)
							: oldInput.defaultValue;
						var normalized:Dynamic;
						try normalized = input.normalize(value, input.unit)
						catch (error:Dynamic) {
							diagnostics.push('Saved input "${input.name}" on "${entry.id}" is no longer valid: ${Std.string(error)}');
							continue;
						}
						var legacyEdit = savedVersion < DocumentCodec.VERSION
							&& Std.string(normalized) != Std.string(input.defaultValue);
						if (legacyEdit && !reportedLegacyDefaults) {
							diagnostics.push('Saved defaults were recovered from a pre-v9 project; re-save this project to record edited defaults explicitly');
							reportedLegacyDefaults = true;
						}
						if (oldInput.editedByUser || legacyEdit)
							definition.setUserEditedTypedDefault(input.name, value);
					}
					for (oldInput in entry.oldDefinition.inputs())
						if (!freshInputs.exists(oldInput.name))
							diagnostics.push('Saved input "${oldInput.name}" on "${entry.id}" was removed from source');
				}

				for (name in entry.prior.overrideNames()) {
					if (!freshInputs.exists(name)) {
						diagnostics.push('Saved override "$name" on "${entry.id}" was removed from source');
						continue;
					}
					var input = freshInputs.get(name);
					var oldInput = entry.oldDefinition.input(name);
					if (oldInput.kind != input.kind) {
						diagnostics.push('Saved override "$name" on "${entry.id}" changed type in source');
						continue;
					}
					var value = entry.prior.typedOverrideValue(name);
					if (input.isNumeric()) value = UnitConversion.fromCanonical(value, input.kind, input.unit);
					var normalized:Dynamic;
					try normalized = input.normalize(value, input.unit)
					catch (error:Dynamic) {
						diagnostics.push('Saved override "$name" on "${entry.id}" is no longer valid: ${Std.string(error)}');
						continue;
					}
					if (entry.current.typedOverrideValue(name) == normalized) continue;
					entry.current.setTypedOverride(name, value);
				}
			}

			var changed = false;
			for (entry in entries)
				if (originalValues.get(entry.id) != resolvedInputSignature(entry.current)) changed = true;
			saved.close();
			return changed;
		} catch (error:Dynamic) {
			saved.close();
			throw error;
		}
	}

	static function recipeInstances(document:cadkit.parametric.Document):Map<String, InstanceElement> {
		var result = new Map<String, InstanceElement>();
		for (element in document.allElements()) if (element.kind == "instance") {
			var instance:InstanceElement = cast element;
			if (typeOrNull(document.definition(instance.definitionId).recipe) == null) continue;
			var property = instance.property("machinekit.occurrence");
			var id:String = property == null ? instance.name : cast property.value;
			if (id == null || id.length == 0 || result.exists(id))
				throw 'Duplicate or empty MachineKit occurrence "$id"';
			result.set(id, instance);
		}
		return result;
	}

	static function resolvedInputSignature(instance:InstanceElement):String {
		var definition = instance.document.definition(instance.definitionId);
		var values:Array<Dynamic> = [];
		for (input in definition.inputs())
			values.push({name: input.name, value: instance.resolvedValue(input.name)});
		return Json.stringify({recipe: definition.recipe, values: values});
	}

	static function documentVersion(text:String):Int {
		var root:Dynamic = Json.parse(text), version:Dynamic = Reflect.field(root, "version");
		if (!Std.isOfType(version, Int)) throw "Saved recipe document has no integer version";
		return cast version;
	}

	static function watchDocument(document:cadkit.parametric.Document, identity:String):Void {
		if (watchedDocuments.exists(identity)) return;
		watchedDocuments.set(identity, true);
		document.onClose(function() {
			var prefix = identity + ":";
			var stale:Array<String> = [];
			for (key in components.keys()) if (StringTools.startsWith(key, prefix)) stale.push(key);
			for (key in stale) components.remove(key);
			watchedDocuments.remove(identity);
		});
	}

	static function migrateLegacyProperties(document:cadkit.parametric.Document, version:Int):Void {
		if (version != 7) return;
		var derived = ["machinekit.partNumber", "machinekit.material", "machinekit.catalog.source",
			"machinekit.catalog.designation"];
		for (definition in document.allDefinitions())
			for (name in derived) if (definition.property(name) != null) definition.restoreProperty(name, null);
		for (element in document.allElements())
			for (name in derived) if (element.property(name) != null) element.restoreProperty(name, null);
	}
}

private typedef RecipeReconcileEntry = {
	var id:String;
	var prior:InstanceElement;
	var current:InstanceElement;
	var oldDefinition:Definition;
	var freshDefinition:Definition;
}

private class MachineKitRecipeEvaluator implements DefinitionEvaluator implements DefinitionConnectorEvaluator {
	final type:ComponentType;

	public function new(type:ComponentType) this.type = type;

	public function evaluate(definition:Definition, instance:InstanceElement, output:String):Shape {
		var component = MachineKitRecipes.component(instance);
		var part:Part;
		if (output == "body") {
			var detail = instance.resolvedToken("detail") == "envelope" ? ComponentDetail.Envelope : ComponentDetail.Preview;
			part = component.geometry(detail);
		} else {
			var tool:Null<ToolSpec> = null;
			for (candidate in component.toolSpecs()) if (candidate.name == output) tool = candidate;
			if (tool == null) throw 'Unknown tool "$output" for ${type.id}';
			part = component.tool(output, MachineKitRecipes.toolValues(definition, instance, tool));
		}
		var result = part.shape.cloneShape();
		part.close();
		return result;
	}

	public function connector(definition:Definition, instance:InstanceElement, output:String):Placement {
		var component = MachineKitRecipes.component(instance);
		for (connector in component.connectors())
			if (connector.name == output)
				return MachineKitDocuments.placement(connector.frame);
		throw 'Unknown connector "$output" for ${type.id}';
	}

	public function connectorNames(definition:Definition, instance:InstanceElement):Array<String>
		return [for (connector in MachineKitRecipes.component(instance).connectors()) connector.name];

}
