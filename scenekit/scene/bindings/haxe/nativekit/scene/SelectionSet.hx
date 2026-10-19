package nativekit.scene;

/** Reusable set of nodes to highlight through a view material override. */
class SelectionSet {
	var entries:Array<{node:NodeId, material:Null<Material>}> = [];

	public function new() {}

	public function add(node:NodeId, ?material:Material):SelectionSet {
		for (entry in entries) if (entry.node.equals(node)) {
			if (material != null) entry.material = material;
			return this;
		}
		entries.push({node:node, material:material});
		return this;
	}

	public function remove(node:NodeId):SelectionSet {
		var index = 0;
		while (index < entries.length) {
			if (entries[index].node.equals(node))
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
			if (value.node.equals(node))
				return true;
		return false;
	}

	public function count():Int
		return entries.length;

	@:allow(SceneView)
	function apply(view:SceneView, material:Material):Void {
		for (entry in entries)
			view.setSelectionMaterial(entry.node, entry.material == null ? material : entry.material);
	}
}
