package cadkit.sketch;

class ProfileError extends haxe.Exception {
	public final kind:String;
	public final entityIds:Array<String>;
	public function new(kind:String, entityIds:Array<String>, message:String) {
		this.kind = kind; this.entityIds = entityIds.copy(); super(message);
	}
}
