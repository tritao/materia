package bimkit;

import cadkit.parametric.DocumentChange;

class HostRelationshipChange implements DocumentChange {
	private final model:BimDocument;
	private final opening:String;
	private final before:Null<HostRelationship>;
	private final after:Null<HostRelationship>;

	public function new(model:BimDocument, opening:String, before:Null<HostRelationship>, after:Null<HostRelationship>) {
		this.model = model;
		this.opening = opening;
		this.before = before;
		this.after = after;
	}

	public function undo():Void
		model.restoreRelationship(opening, before);

	public function redo():Void
		model.restoreRelationship(opening, after);
}
