package nativekit.scene;

/** Reusable set of nodes to highlight through a view material override. */
class SelectionSet {
	var entries:Array<NodeId> = [];

	public function new() {}

	public function add(node:NodeId):SelectionSet {
		if (!contains(node))
			entries.push(node);
		return this;
	}

	public function remove(node:NodeId):SelectionSet {
		var index = 0;
		while (index < entries.length) {
			if (entries[index].equals(node))
				entries.splice(index, 1);
			else
				index++;
		}
		return this;
	}

	public function clear():SelectionSet {
		entries.resize(0);
		return this;
	}

	public function contains(node:NodeId):Bool {
		for (value in entries)
			if (value.equals(node))
				return true;
		return false;
	}

	public function count():Int
		return entries.length;

	@:allow(SceneView)
	function apply(view:SceneView, material:Material):Void {
		for (node in entries)
			view.setSelectionMaterial(node, material);
	}
}
