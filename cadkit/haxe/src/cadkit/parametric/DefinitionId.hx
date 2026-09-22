package cadkit.parametric;

class DefinitionId {
	public final value:String;

	public function new(?value:String)
		this.value = value == null ? PersistentId.create("definition") : PersistentId.validate(value, "definition ID");
}
