package app;

import cadkit.parametric.InstanceElement;
import sys.FileSystem;
import sys.io.File;

/** Focused generated-project persistence check for editable MachineKit recipes. */
class MachineKitRecipeProjectTests {
	static function check(value:Bool, message:String):Void if (!value) throw message;

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
			var bearing:Null<InstanceElement> = null;
			for (element in recipe.allElements()) if (element.name == "bearingB") bearing = cast element;
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
			var persisted = false;
			for (element in reopened.allElements()) if (element.name == "bearingB")
				persisted = cast(element, InstanceElement).resolvedToken("designation") == "6000";
			check(persisted, "reopened project retains edited dimensions");
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
