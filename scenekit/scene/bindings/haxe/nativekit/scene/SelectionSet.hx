package nativekit.scene;

/** Reusable set of nodes to highlight through a view material override. */
class SelectionSet {
	var entries:Array<Node> = [];

	public function new() {}

	public function add(node:Node):SelectionSet {
		if (!contains(node))
			entries.push(node);
		return this;
	}

	public function remove(node:Node):SelectionSet {
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

	public function contains(node:Node):Bool {
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
