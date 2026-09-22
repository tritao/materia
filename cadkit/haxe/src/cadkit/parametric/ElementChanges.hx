package cadkit.parametric;

import cadkit.parametric.Document;
import cadkit.parametric.DocumentChange;
import cadkit.parametric.Element;
import cadkit.parametric.Feature;

class ElementCreateChange implements DocumentChange {
	final document:Document; final element:Element; final index:Int;
	public function new(document, element, index) { this.document=document; this.element=element; this.index=index; }
	public function undo():Void document.restoreElementRemoval(element);
	public function redo():Void document.restoreElementInsertion(element, index);
}

class ElementRemoveChange implements DocumentChange {
	final document:Document; final element:Element; final index:Int;
	public function new(document, element, index) { this.document=document; this.element=element; this.index=index; }
	public function undo():Void document.restoreElementInsertion(element, index);
	public function redo():Void document.restoreElementRemoval(element);
}

class ElementNameChange implements DocumentChange {
	final element:Element; final oldName:String; final newName:String;
	public function new(element, oldName, newName) { this.element=element; this.oldName=oldName; this.newName=newName; }
	public function undo():Void element.restoreName(oldName);
	public function redo():Void element.restoreName(newName);
}

class ElementOutputChange implements DocumentChange {
	final element:Element; final oldOutput:Feature; final newOutput:Feature;
	public function new(element, oldOutput, newOutput) { this.element=element; this.oldOutput=oldOutput; this.newOutput=newOutput; }
	public function undo():Void element.restoreOutput(oldOutput);
	public function redo():Void element.restoreOutput(newOutput);
}
