package nativekit.scene;

import NativeKitSceneRender;

/** Typed Haxe result for one renderer picking query. */
class PickResult {
	final picked:nkscene_render_pick_result;
	final pickedNode:NodeId;

	@:allow(SceneRenderer, SceneInteraction, SpatialIndex)
	private function new(picked:nkscene_render_pick_result) {
		this.picked = picked;
		pickedNode = NodeId.fromNative(picked.get_node());
	}

	/** Picked scene node. */
	public function node():NodeId
		return pickedNode;

	public function sourceValue():haxe.Int64
		return picked.get_source().get_value();

	public function subelement():Int
		return picked.get_subelement();

	public function worldX():Float
		return picked.get_world_position(0);

	public function worldY():Float
		return picked.get_world_position(1);

	public function worldZ():Float
		return picked.get_world_position(2);

	public function depth():Float
		return picked.get_depth();

	@:allow(SceneInteraction)
	function nativeValue():nkscene_render_pick_result
		return picked;
}
