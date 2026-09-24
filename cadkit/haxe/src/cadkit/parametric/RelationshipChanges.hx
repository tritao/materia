package cadkit.parametric;

import cadkit.parametric.TypedProperty;

class RelationshipCreateChange implements DocumentChange {
	final document:Document;
	final relationship:Relationship;
	final index:Int;

	public function new(document:Document, relationship:Relationship, index:Int) {
		this.document = document;
		this.relationship = relationship;
		this.index = index;
	}

	public function undo():Void document.restoreRelationshipRemoval(relationship);
	public function redo():Void document.restoreRelationshipInsertion(relationship, index);
}

class RelationshipRemoveChange implements DocumentChange {
	final document:Document;
	final relationship:Relationship;
	final index:Int;

	public function new(document:Document, relationship:Relationship, index:Int) {
		this.document = document;
		this.relationship = relationship;
		this.index = index;
	}

	public function undo():Void document.restoreRelationshipInsertion(relationship, index);
	public function redo():Void document.restoreRelationshipRemoval(relationship);
}

class RelationshipEndpointsChange implements DocumentChange {
	final document:Document;
	final relationship:Relationship;
	final beforeSource:ElementReference;
	final beforeTarget:ElementReference;
	final afterSource:ElementReference;
	final afterTarget:ElementReference;

	public function new(document:Document, relationship:Relationship, beforeSource:ElementReference, beforeTarget:ElementReference,
		afterSource:ElementReference, afterTarget:ElementReference) {
		this.document = document;
		this.relationship = relationship;
		this.beforeSource = beforeSource;
		this.beforeTarget = beforeTarget;
		this.afterSource = afterSource;
		this.afterTarget = afterTarget;
	}

	public function undo():Void document.restoreRelationshipEndpoints(relationship, beforeSource, beforeTarget);
	public function redo():Void document.restoreRelationshipEndpoints(relationship, afterSource, afterTarget);
}

class RelationshipPropertyChange implements DocumentChange {
	final document:Document;
	final relationship:Relationship;
	final name:String;
	final before:Null<TypedProperty>;
	final after:Null<TypedProperty>;

	public function new(document:Document, relationship:Relationship, name:String, before:Null<TypedProperty>, after:Null<TypedProperty>) {
		this.document = document;
		this.relationship = relationship;
		this.name = name;
		this.before = before;
		this.after = after;
	}

	public function undo():Void document.restoreRelationshipProperty(relationship, name, before);
	public function redo():Void document.restoreRelationshipProperty(relationship, name, after);
}
