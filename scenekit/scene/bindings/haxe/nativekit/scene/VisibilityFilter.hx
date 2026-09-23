package nativekit.scene;

/** Reusable per-node visibility policy for a SceneView. */
class VisibilityFilter {
	final entries:Array<{node:NodeId, visible:Bool}> = [];

	public function new() {}

	public function set(node:NodeId, visible:Bool):VisibilityFilter {
		var stable = node.stableValue();
		for (entry in entries) {
			if (entry.node.stableValue() == stable) {
				entry.visible = visible;
				return this;
			}
		}
		entries.push({node: node, visible: visible});
		return this;
	}

	public function show(node:NodeId):VisibilityFilter
		return set(node, true);

	public function hide(node:NodeId):VisibilityFilter
		return set(node, false);

	public function clear():VisibilityFilter {
		entries.resize(0);
		return this;
	}

	public function count():Int
		return entries.length;

	@:allow(SceneView)
	function apply(view:SceneView):Void {
		for (entry in entries)
			view.setVisibility(entry.node, entry.visible);
	}
}
