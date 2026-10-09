package app;

import cadkit.parametric.InstanceElement;
import cadkit.parametric.ElementId;
import cadkit.parametric.DocumentCodec;
import machinekit.document.MachineKitRecipes;
import sys.FileSystem;
import sys.io.File;

/** Focused generated-project persistence check for editable MachineKit recipes. */
class MachineKitRecipeProjectTests {
	static function check(value:Bool, message:String):Void if (!value) throw message;

	static function occurrence(document:cadkit.parametric.Document, id:String):Null<InstanceElement> {
		for (element in document.allElements()) if (element.kind == "instance") {
			var property = element.property("machinekit.occurrence");
			if (property != null && property.value == id) return cast element;
		}
		return null;
	}

	static function json(text:String):Dynamic return haxe.Json.parse(text);
	static function jsonText(value:Dynamic):String return haxe.Json.stringify(value);

	static function occurrenceRecord(root:Dynamic, id:String):Dynamic {
		var elements:Array<Dynamic> = cast Reflect.field(root, "elements");
		for (element in elements) if (Reflect.field(element, "kind") == "instance") {
			var properties:Array<Dynamic> = cast Reflect.field(element, "properties");
			for (property in properties) if (Reflect.field(property, "name") == "machinekit.occurrence"
				&& Reflect.field(property, "value") == id) return element;
		}
		throw 'Recipe occurrence "$id" not found in saved document';
	}

	static function definitionRecord(root:Dynamic, occurrenceId:String):Dynamic {
		var definitionId:String = Reflect.field(occurrenceRecord(root, occurrenceId), "definition");
		var definitions:Array<Dynamic> = cast Reflect.field(root, "definitions");
		for (definition in definitions) if (Reflect.field(definition, "id") == definitionId) return definition;
		throw 'Definition "$definitionId" not found in saved document';
	}

	static function inputRecord(definition:Dynamic, name:String):Dynamic {
		var inputs:Array<Dynamic> = cast Reflect.field(definition, "inputs");
		for (input in inputs) if (Reflect.field(input, "name") == name) return input;
		throw 'Input "$name" not found in saved definition';
	}

	static function changedDefault(text:String, occurrenceId:String, name:String, value:Dynamic,
			editedByUser:Bool):String {
		var root = json(text);
		var input = inputRecord(definitionRecord(root, occurrenceId), name);
		Reflect.setField(input, "value", value);
		Reflect.setField(input, "editedByUser", editedByUser);
		return jsonText(root);
	}

	static function countDiagnostic(diagnostics:Array<String>, fragment:String):Int {
		var count = 0;
		for (diagnostic in diagnostics) if (diagnostic.indexOf(fragment) >= 0) count++;
		return count;
	}

	static function hasDiagnostic(diagnostics:Array<String>, fragment:String):Bool {
		for (diagnostic in diagnostics) if (diagnostic.indexOf(fragment) >= 0) return true;
		return false;
	}

	static function assertReconciledLoads(freshText:String, savedText:String, diagnostic:String):String {
		var diagnostics:Array<String> = [];
		var result = MachineKitRecipes.reconcile(freshText, savedText, diagnostics);
		check(hasDiagnostic(diagnostics, diagnostic), 'reconciliation reports "$diagnostic"');
		var loaded = DocumentCodec.decode(result.text);
		loaded.close();
		return result.text;
	}

	static function reconciliationRegressions(sourceText:String):Void {
		var changedSource = changedDefault(sourceText, "bearingB", "designation", "6000", false);
		var diagnostics:Array<String> = [];
		var result = MachineKitRecipes.reconcile(changedSource, sourceText, diagnostics);
		var loaded = DocumentCodec.decode(result.text);
		check(occurrence(loaded, "bearingB").resolvedToken("designation") == "6000",
			"source default changes replace an untouched saved default");
		loaded.close();

		var changedAgain = changedDefault(sourceText, "bearingB", "designation", "6001", false);
		var userEdit = changedDefault(sourceText, "bearingB", "designation", "6000", true);
		diagnostics = [];
		result = MachineKitRecipes.reconcile(changedAgain, userEdit, diagnostics);
		loaded = DocumentCodec.decode(result.text);
		check(occurrence(loaded, "bearingB").resolvedToken("designation") == "6000",
			"an explicitly edited default wins over a later source change");
		loaded.close();

		var savedDocument = DocumentCodec.decode(sourceText);
		var firstScrew:InstanceElement = cast occurrence(savedDocument, "screw1");
		var secondScrew:InstanceElement = cast occurrence(savedDocument, "screw2");
		var secondLength = secondScrew.resolved("length");
		var newLength = firstScrew.resolved("length") + 2;
		firstScrew.makeUnique("screw1 saved length");
		savedDocument.definition(firstScrew.definitionId).setUserEditedDefault("length", newLength);
		var uniqueSavedText = DocumentCodec.encode(savedDocument);
		savedDocument.close();
		diagnostics = [];
		result = MachineKitRecipes.reconcile(sourceText, uniqueSavedText, diagnostics);
		loaded = DocumentCodec.decode(result.text);
		firstScrew = cast occurrence(loaded, "screw1");
		secondScrew = cast occurrence(loaded, "screw2");
		check(firstScrew.resolved("length") == newLength && secondScrew.resolved("length") == secondLength
			&& firstScrew.definitionId.value != secondScrew.definitionId.value,
			"a makeUnique length edit remains isolated to screw1 after reload");
		loaded.close();

		var newInputSource = json(sourceText);
		var bearingDefinition = definitionRecord(newInputSource, "bearingB");
		var bearingInputs:Array<Dynamic> = cast Reflect.field(bearingDefinition, "inputs");
		bearingInputs.push({name: "newClearance", kind: "length", unit: "mm", value: 0.25,
			allowedValues: null, editedByUser: false});
		assertReconciledLoads(jsonText(newInputSource), sourceText, "new input \"newClearance\"");

		var removedInputSaved = json(sourceText);
		var oldBearingDefinition = definitionRecord(removedInputSaved, "bearingB");
		var oldInputs:Array<Dynamic> = cast Reflect.field(oldBearingDefinition, "inputs");
		oldInputs.push({name: "obsoleteClearance", kind: "length", unit: "mm", value: 0.25,
			allowedValues: null, editedByUser: false});
		assertReconciledLoads(sourceText, jsonText(removedInputSaved), "obsoleteClearance");

		var droppedRowSource = json(sourceText);
		var designation = inputRecord(definitionRecord(droppedRowSource, "bearingB"), "designation");
		var allowed:Array<String> = cast Reflect.field(designation, "allowedValues");
		allowed.remove("608");
		Reflect.setField(designation, "value", "6000");
		assertReconciledLoads(jsonText(droppedRowSource), sourceText, "is no longer valid");


	}

	static function removeToolInputs(projectText:String):String {
		var project:Dynamic = haxe.Json.parse(projectText);
		var saved:Dynamic = haxe.Json.parse(Reflect.field(project, "recipeDocument"));
		var definitions:Array<Dynamic> = cast Reflect.field(saved, "definitions");
		var removed = 0;
		for (definition in definitions) {
			var inputs:Array<Dynamic> = cast Reflect.field(definition, "inputs");
			var kept:Array<Dynamic> = [];
			for (input in inputs) {
				var name:String = Reflect.field(input, "name");
				if (StringTools.startsWith(name, "tool_")) removed++;
				else kept.push(input);
			}
			Reflect.setField(definition, "inputs", kept);
		}
		check(removed > 0, "saved recipe fixture contains typed tool inputs");
		Reflect.setField(project, "recipeDocument", haxe.Json.stringify(saved));
		return haxe.Json.stringify(project);
	}

	public static function main():Int {
		var manifest = tests.TestPaths.of("machinekit/examples/materia.project.json");
		var generated = MateriaProjectRunner.loadProject(manifest);
		check(generated.recipeDocument != null, "generated preview carries a recipe document");
		reconciliationRegressions(cast generated.recipeDocument);
		var session = new ProjectDocumentSession();
		var destination = "/tmp/materia-machinekit-recipe-" + Sys.getPid() + ".materia.json";
		var tracePath = destination + ".reconcile-trace";
		try {
			session.openGeneratedScene(generated.objects, manifest,
				generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
                generated.localCentersByDefinition, generated.metresPerUnit,
                generated.physical, generated.recipeDocument);
			var recipe = session.recipeDocument;
			check(recipe != null, "app opens the recipe document");
			var bearing = occurrence(recipe, "bearingB");
			check(bearing != null, "bearing instance is editable");
			var target:InstanceElement = cast bearing;
			var beforeObject = session.scene.object("project:bearingB");
			check(beforeObject != null, "bearing appears in the generated scene");
			var beforeObjectValue:EditorSceneObject = cast beforeObject;
			var before = beforeObjectValue.meshSnapshot;
			session.applyRecipeEdit("Change bearing", function() target.setTypedOverride("designation", "6000"));
			check(target.resolvedToken("designation") == "6000", "app edits the bearing designation");
			var afterObject = session.scene.object("project:bearingB");
			var afterObjectValue:EditorSceneObject = cast afterObject;
			check(afterObjectValue.meshSnapshot != before,
				"app refreshes geometry after a recipe edit");
			session.save(destination);
			check(File.getContent(destination).indexOf("recipeDocument") >= 0,
				"saved project retains the editable recipe document");
			// Simulate a project saved before typed tool inputs were added. Reconciliation
			// must preserve its existing edits and leave new definition inputs at defaults.
			File.saveContent(destination, removeToolInputs(File.getContent(destination)));
			session.open(destination);
			var reopened = session.recipeDocument;
			var persisted = occurrence(reopened, "bearingB").resolvedToken("designation") == "6000";
			check(persisted, "reopened project retains edited dimensions");
			var project:Dynamic = haxe.Json.parse(File.getContent(destination));
			var saved:Dynamic = haxe.Json.parse(Reflect.field(project, "recipeDocument"));
			var elements:Array<Dynamic> = cast Reflect.field(saved, "elements");
			var bearingA:Dynamic = null;
			var kept:Array<Dynamic> = [];
			for (record in elements) {
				var name:String = Reflect.field(record, "name");
				if (name == "ring") continue; // The current source now has an extra part.
				if (name == "bearingA") bearingA = record;
				if (name == "bearingB") Reflect.setField(record, "name", "bearingA");
				kept.push(record);
			}
			check(bearingA != null, "test bearing source exists");
			var orphan:Dynamic = haxe.Json.parse(haxe.Json.stringify(bearingA));
			Reflect.setField(orphan, "id", new ElementId().value);
			Reflect.setField(orphan, "name", "removed-from-source");
			var properties:Array<Dynamic> = cast Reflect.field(orphan, "properties");
			for (property in properties) if (Reflect.field(property, "name") == "machinekit.occurrence")
				Reflect.setField(property, "value", "removed-from-source");
			kept.push(orphan);
			Reflect.setField(saved, "elements", kept);
			Reflect.setField(project, "recipeDocument", haxe.Json.stringify(saved));
			File.saveContent(destination, haxe.Json.stringify(project));
			session.open(destination);
			var reconciled = session.recipeDocument;
			check(occurrence(reconciled, "bearingB").resolvedToken("designation") == "6000",
				"duplicate names preserve edits through occurrence IDs");
			check(occurrence(reconciled, "ring") != null,
				"new source part stays editable after loading older saved recipes");
			check(occurrence(reconciled, "removed-from-source") == null && session.staleEdits().length > 0,
				"removed source part produces a diagnostic");
			check(hasDiagnostic(session.staleEdits(), "no longer exists in source"),
				"removed from source diagnostic reaches the app");
			Reflect.setField(project, "recipeDocument", jsonText(saved));
			for (record in kept) if (Reflect.field(record, "kind") == "instance"
				&& Reflect.field(record, "name") == "bearingA") {
				var values:Array<Dynamic> = cast Reflect.field(record, "properties");
				for (property in values) if (Reflect.field(property, "name") == "machinekit.occurrence"
					&& Reflect.field(property, "value") == "bearingB")
					Reflect.setField(property, "value", "bearingA");
			}
			Reflect.setField(project, "recipeDocument", haxe.Json.stringify(saved));
			File.saveContent(destination, haxe.Json.stringify(project));
			var duplicateRejected = false;
			try session.open(destination) catch (_:Dynamic) duplicateRejected = true;
			check(duplicateRejected, "duplicate occurrence IDs are rejected on load");
		} catch (error:Dynamic) {
			session.dispose();
			if (FileSystem.exists(destination)) FileSystem.deleteFile(destination);
			if (FileSystem.exists(tracePath)) FileSystem.deleteFile(tracePath);
			throw error;
		}
		session.dispose();
		if (FileSystem.exists(destination)) FileSystem.deleteFile(destination);
		if (FileSystem.exists(tracePath)) FileSystem.deleteFile(tracePath);
		return 0;
	}
}
