package nativekit.scene;

import NativeKitScene;

/** Stable identity for one positioned item in a scene hierarchy. */
class NodeId {
	final value:nkscene_node_id;

	@:allow(Transaction)
	private function new(value:nkscene_node_id) {
		this.value = value;
	}

	@:allow(PickResult, SceneNode, SpatialIndex, SceneSnapshot)
	static function fromNative(value:nkscene_node_id):NodeId
		return new NodeId(value);

	/** Returns the stable 64-bit node value for logging or maps. */
	public function stableValue():haxe.Int64
		return value.get_value();

	public function equals(other:NodeId):Bool
		return stableValue() == other.stableValue();

	@:allow(Transaction, SceneView, VisibilityFilter, SelectionSet, SceneRenderer, SceneSnapshot)
	function nativeValue():nkscene_node_id
		return value;
}
