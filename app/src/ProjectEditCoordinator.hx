package app;

import nativekit.ui.core.EditOperation;
import nativekit.ui.core.EditorDocument;

/** One project history boundary for edits that may span several domain models. */
class ProjectEditCoordinator {
	public final document:EditorDocument;

	public function new(document:EditorDocument) {
		if (document == null) throw "Project edits require an owning document";
		this.document = document;
	}

	public function apply(label:String, redo:Void->Void, undo:Void->Void,
			?estimatedRetainedBytes:Int):Bool
		return document.apply(new EditOperation(label, redo, undo, null, null, null,
			estimatedRetainedBytes == null ? 256 : estimatedRetainedBytes));

	/** Runs a sequence as one history entry and compensates if any component fails. */
	public function applyCompound(label:String, steps:Array<EditOperation>):Bool {
		if (steps == null || steps.length == 0) throw "Compound project edits require at least one step";
		var captured = steps.copy();
		for (step in captured) if (step == null) throw "Compound project edit steps cannot be null";
		var estimatedBytes = 64;
		for (step in captured)
			estimatedBytes += step.estimatedRetainedBytes;
		return apply(label, function() runForward(captured), function() runBackward(captured), estimatedBytes);
	}

	function runForward(steps:Array<EditOperation>):Void {
		var completed = 0;
		try {
			for (step in steps) {
				step.apply();
				completed++;
			}
		} catch (failure:Dynamic) {
			var compensationFailure:Null<Dynamic> = null;
			while (completed > 0) {
				completed--;
				try steps[completed].undo() catch (error:Dynamic) compensationFailure = error;
			}
			if (compensationFailure != null)
				throw "Project edit failed (" + Std.string(failure) + "); compensation failed (" + Std.string(compensationFailure) + ")";
			throw failure;
		}
	}

	function runBackward(steps:Array<EditOperation>):Void {
		var undone:Array<Int> = [];
		var index = steps.length - 1;
		try {
			while (index >= 0) {
				steps[index].undo();
				undone.push(index);
				index--;
			}
		} catch (failure:Dynamic) {
			var compensationFailure:Null<Dynamic> = null;
			var compensationIndex = undone.length - 1;
			while (compensationIndex >= 0) {
				try steps[undone[compensationIndex]].apply() catch (error:Dynamic) compensationFailure = error;
				compensationIndex--;
			}
			if (compensationFailure != null)
				throw "Project undo failed (" + Std.string(failure) + "); compensation failed (" + Std.string(compensationFailure) + ")";
			throw failure;
		}
	}
}
