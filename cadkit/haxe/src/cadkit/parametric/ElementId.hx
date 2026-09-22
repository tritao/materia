package cadkit.parametric;

import cadkit.parametric.PersistentId;

class ElementId {
	public final value:String;

	public function new(?value:String) {
		this.value = value == null ? PersistentId.create("element") : PersistentId.validate(value, "element ID");
	}

	public function toString():String return value;
}
