package nativekit.scene;

import NativeKitScene;

/** Stable identity for one positioned item in a scene hierarchy. */
class Node {
	final value:nkscene_node_id;

	@:allow(Transaction)
	private function new(value:nkscene_node_id) {
		this.value = value;
	}

	@:allow(PickResult, SceneNode, SpatialIndex, Snapshot)
	static function fromNative(value:nkscene_node_id):Node
		return new Node(value);

	/** Returns the stable 64-bit node value for logging or maps. */
	public function stableValue():haxe.Int64
		return value.get_value();

	public function equals(other:Node):Bool
		return stableValue() == other.stableValue();

	@:allow(Transaction, SceneView, VisibilityFilter, SelectionSet, SceneRenderer)
	function nativeValue():nkscene_node_id
		return value;
}
