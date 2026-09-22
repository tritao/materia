import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ElementReference;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.features.BoxFeature;
import haxe.Json;

private class FailingElementFeature extends Feature {
	public function new() super();
	override public function evaluate(context:EvaluationContext):EvaluationResult throw "intentional element output failure";
}

class ElementSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value) throw message;
	}

	public static function run():Void {
		var document = new Document();
		var box = document.add(new BoxFeature(10, 20, 30));
		var first = document.createElement("First wall", box);
		var second = document.createElement("Second wall", box);
		check(document.elementCount() == 2, "element registry count");
		check(first.id.value != second.id.value, "new elements receive distinct identities");
		check(document.element(first.id) == first && first.output == box, "element lookup preserves output references");
		var firstId = first.id.value;
		first.rename("North wall");
		check(first.name == "North wall" && document.undo(), "element rename is undoable");
		check(first.name == "First wall" && first.id.value == firstId, "rename undo preserves element identity");
		check(document.redo() && first.name == "North wall", "element rename is redoable");

		var copy = document.duplicateElement(first, "North wall copy");
		check(copy.id.value != first.id.value && copy.output == first.output, "duplicating creates a new element identity");
		check(document.undo() && document.findElement(copy.id) == null, "element creation undo removes the record");
		check(document.redo() && document.findElement(copy.id) == copy, "element creation redo restores the same identity");

		var reference = new ElementReference(document.id, second.id);
		document.removeElement(second.id);
		check(reference.state(document) == ElementReference.UnresolvedElement, "removed element references are explicitly unresolved");
		var failed = false;
		try document.installElement("Reused identity", box, second.id) catch (error:Dynamic) failed = true;
		check(failed && reference.state(document) == ElementReference.UnresolvedElement,
			"removed element identities remain reserved");
		check(document.undo() && reference.state(document) == ElementReference.Resolved
			&& document.element(reference.elementId) == second, "removal undo restores the referenced identity");

		var replacement = document.add(new BoxFeature(5, 5, 5));
		first.setOutput(replacement);
		check(first.id.value == firstId && first.output == replacement, "output replacement preserves element identity");
		check(document.undo() && first.output == box, "output replacement is undoable");

		document.setOutput(box);
		document.recompute();
		var stableVolume = first.shape().volume();
		var failing = document.add(new FailingElementFeature());
		first.setOutput(failing);
		failed = false;
		try document.recompute() catch (error:Dynamic) failed = true;
		check(failed && first.shape().volume() == stableVolume, "failed recompute preserves committed element geometry");

		var transaction = document.beginTransaction();
		var transient = document.createElement("Transient", box);
		transient.rename("Renamed transient");
		transaction.cancel();
		check(document.findElement(transient.id) == null, "transaction cancellation restores the element registry");

		var otherDocument = new Document();
		var otherBox = otherDocument.add(new BoxFeature(1, 1, 1));
		failed = false;
		try document.createElement("Foreign", otherBox) catch (error:Dynamic) failed = true;
		check(failed, "cross-document element outputs are rejected");
		otherDocument.close();
		document.close();

		var persisted = new Document();
		var persistedBox = persisted.add(new BoxFeature(12, 13, 14));
		var persistedElement = persisted.createElement("Persisted wall", persistedBox);
		persisted.setOutput(persistedBox);
		persisted.recompute();
		var encoded = DocumentCodec.encode(persisted);
		var opened = DocumentCodec.decode(encoded);
		check(opened.id.value == persisted.id.value, "opening preserves document identity");
		check(opened.elementCount() == 1 && opened.elementAt(0).id.value == persistedElement.id.value,
			"opening preserves element identity");
		check(opened.elementAt(0).shape().volume() == persistedElement.shape().volume(), "element output survives reload");
		var cloned = DocumentCodec.decode(encoded, true);
		check(cloned.id.value != persisted.id.value && cloned.elementAt(0).id.value == persistedElement.id.value,
			"cloning assigns a document identity distinct from opening");

		var unsupportedVersion:Dynamic = Json.parse(encoded);
		Reflect.setField(unsupportedVersion, "version", 99);
		failed = false;
		try DocumentCodec.decode(Json.stringify(unsupportedVersion)) catch (error:Dynamic) failed = true;
		check(failed, "unsupported document versions are rejected");
		var implicitOutput:Dynamic = Json.parse(encoded);
		Reflect.setField(implicitOutput, "version", 1);
		Reflect.setField(implicitOutput, "output", null);
		Reflect.setField(implicitOutput, "elements", null);
		var migrated = DocumentCodec.decode(Json.stringify(implicitOutput));
		check(migrated.elementCount() == 1 && migrated.elementAt(0).name == "Model"
			&& migrated.elementAt(0).output == migrated.outputFeature(),
			"implicit version-one output migrates into the element registry");

		var duplicateIds:Dynamic = Json.parse(encoded);
		var duplicateRecords:Array<Dynamic> = cast Reflect.field(duplicateIds, "elements");
		duplicateRecords.push({id: persistedElement.id.value, name: "Duplicate", output: persistedBox.id.toInt()});
		failed = false;
		try DocumentCodec.decode(Json.stringify(duplicateIds)) catch (error:Dynamic) failed = true;
		check(failed, "duplicate persisted element IDs are rejected");
		var dangling:Dynamic = Json.parse(encoded);
		var danglingRecords:Array<Dynamic> = cast Reflect.field(dangling, "elements");
		Reflect.setField(danglingRecords[0], "output", 9999);
		failed = false;
		try DocumentCodec.decode(Json.stringify(dangling)) catch (error:Dynamic) failed = true;
		check(failed, "dangling persisted element outputs are rejected");

		migrated.close();
		cloned.close();
		opened.close();
		persisted.close();

		var building = new PersistentBuildingModel();
		check(building.walls.length == 4 && building.document.elementCount() == 5, "building example exposes five independent elements");
		var wallIdentity = building.walls[0].id.value;
		building.resizeWall(0, 7500);
		check(building.walls[0].id.value == wallIdentity, "geometry regeneration preserves wall identity");
		var duplicateWall = building.duplicateWall(0, "North wall copy");
		check(duplicateWall.id.value != wallIdentity, "building wall duplication creates a new identity");
		var restoredBuilding = DocumentCodec.decode(DocumentCodec.encode(building.document));
		check(restoredBuilding.elementCount() == 6 && restoredBuilding.elementAt(0).id.value == wallIdentity,
			"building identities survive save and reload");
		restoredBuilding.close();
		building.close();
	}
}
