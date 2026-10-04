package machinekit.component;

/** Component recipes by stable id; loading a saved assembly rebuilds its members through one. */
class ComponentRegistry {
	final types:Array<ComponentType> = [];
	final byKey:Map<String, ComponentType> = [];

	public function new() {}

	public function register(type:ComponentType):Void {
		if (type == null) throw "Null component type";
		if (byKey.exists(type.id)) throw 'Duplicate component type "${type.id}"';
		types.push(type);
		byKey.set(type.id, type);
	}

	public function has(id:String):Bool return byKey.exists(id);

	public function byId(id:String):ComponentType {
		var type = byKey.get(id);
		if (type == null) throw 'Unknown machine component type "$id"';
		return type;
	}

	/** Registered types in registration order. */
	public function all():Array<ComponentType> return types.copy();
}
