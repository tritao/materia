package cadkit.parametric;

/** Stable identity for one typed document relationship. */
class RelationshipId {
	public final value:String;

	public function new(?value:String)
		this.value = value == null ? PersistentId.create("relationship") : PersistentId.validate(value, "relationship ID");

	public function toString():String return value;
}
