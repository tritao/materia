import cadkit.parametric.Document;
import cadkit.parametric.ElementReference;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.features.BoxFeature;

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
		var failed = false;
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
	}
}
