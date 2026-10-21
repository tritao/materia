package machinekit.component;

/** Component recipes by stable id; loading a saved assembly rebuilds its members through one. */
class ComponentRegistry {
	final types:Array<ComponentType> = [];
	final byKey:Map<String, ComponentType> = [];
	final categories:Map<String, String> = [];
	final kinds:Map<String, ComponentKind> = [];
	final categoryOrder:Array<String> = [];

	public function new() {}

	public function register(type:ComponentType):Void {
		if (type == null) throw "Null component type";
		if (byKey.exists(type.id)) throw 'Duplicate component type "${type.id}"';
		types.push(type);
		byKey.set(type.id, type);
	}

	public function registerFamily(family:ComponentFamily):Void register(family.recipe);

	public function familyForType(id:String):Null<ComponentFamily> return byId(id).manufacturingFamily;

	public function has(id:String):Bool return byKey.exists(id);

	public function byId(id:String):ComponentType {
		var type = byKey.get(id);
		if (type == null) throw 'Unknown machine component type "$id"';
		return type;
	}

	/**
	 * Files a registered type under a category (a heading within its package, such as "Linear guides") and a kind,
	 * for catalogues and part pickers. MachineKit's built-ins are all classified (`ComponentClassification`).
	 */
	public function classify(id:String, category:String, kind:ComponentKind):Void {
		byId(id);
		if (category == null || category.length == 0) throw 'Empty category for "$id"';
		categories.set(id, category);
		kinds.set(id, kind);
		if (categoryOrder.indexOf(category) < 0) categoryOrder.push(category);
	}

	/** Every category, in the order types were first filed under it (the display order). */
	public function categoryNames():Array<String> return categoryOrder.copy();

	/** The type's category, or null when it has not been classified. */
	public function category(id:String):Null<String> return categories.get(id);

	public function kind(id:String):Null<ComponentKind> return kinds.get(id);

	/** Registered types in registration order. */
	public function all():Array<ComponentType> return types.copy();
}
