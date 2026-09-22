package cadkit.parametric;

import cadkit.parametric.PersistentId;

class DocumentId {
	public final value:String;

	public function new(?value:String) {
		this.value = value == null ? PersistentId.create("document") : PersistentId.validate(value, "document ID");
	}

	public function toString():String return value;
}
