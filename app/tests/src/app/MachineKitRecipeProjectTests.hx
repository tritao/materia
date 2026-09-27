package app;

import cadkit.parametric.InstanceElement;
import cadkit.parametric.ElementId;
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

	public static function main():Int {
		var root = FileSystem.fullPath(Sys.getCwd());
		while (!FileSystem.exists(root + "/machinekit/examples/materia.project.json")) {
			var parent = haxe.io.Path.directory(root);
			if (parent == root || parent.length == 0) throw "Could not locate Materia repository";
			root = parent;
		}
		var manifest = root + "/machinekit/examples/materia.project.json";
		var generated = MateriaProjectRunner.loadProject(manifest);
		check(generated.recipeDocument != null, "generated preview carries a recipe document");
		var session = new ProjectDocumentSession();
		var destination = "/tmp/materia-machinekit-recipe-" + Sys.getPid() + ".materia.json";
		try {
			session.openGeneratedScene(generated.objects, manifest, generated.assembly,
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
			throw error;
		}
		session.dispose();
		if (FileSystem.exists(destination)) FileSystem.deleteFile(destination);
		return 0;
	}
}
