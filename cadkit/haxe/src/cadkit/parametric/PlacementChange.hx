package cadkit.parametric;

class PlacementChange implements DocumentChange {
	final document:Document;
	final element:Element;
	final before:Placement;
	final beforeParent:Null<ElementReference>;
	final after:Placement;
	final afterParent:Null<ElementReference>;

	public function new(document, element, before, beforeParent, after, afterParent) {
		this.document = document;
		this.element = element;
		this.before = before;
		this.beforeParent = beforeParent;
		this.after = after;
		this.afterParent = afterParent;
	}

	public function undo():Void document.restoreElementPlacement(element, before, beforeParent);
	public function redo():Void document.restoreElementPlacement(element, after, afterParent);
}
