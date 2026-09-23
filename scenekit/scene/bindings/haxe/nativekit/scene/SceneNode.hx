package nativekit.scene;

import NativeKitScene;

/** Immutable node state captured by a scene snapshot. */
class SceneNode {
	final nodeValue:Node;
	final parentValue:Null<Node>;
	final sourceId:haxe.Int64;
	final geometryId:haxe.Int64;
	final materialId:haxe.Int64;
	final cameraId:haxe.Int64;
	final lightId:haxe.Int64;
	final visibleValue:Bool;
	final localTransformValue:Transform;
	final worldTransformValue:Transform;
	final worldTransformRevisionValue:haxe.Int64;
	final boundsValue:Bounds;

	@:allow(Snapshot)
	private function new(value:nkscene_snapshot_node) {
		nodeValue = Node.fromNative(value.get_node());
		var parent = value.get_parent();
		parentValue = haxe.Int64.toInt(parent.get_value()) == 0 ? null : Node.fromNative(parent);
		sourceId = value.get_source().get_value();
		geometryId = value.get_geometry().get_value();
		materialId = value.get_material().get_value();
		cameraId = value.get_camera().get_value();
		lightId = value.get_light().get_value();
		visibleValue = value.get_visible() != 0;
		localTransformValue = Transform.fromNative(value.get_local_transform());
		worldTransformValue = Transform.fromNative(value.get_world_transform());
		worldTransformRevisionValue = value.get_world_transform_revision();
		boundsValue = Bounds.fromNative(value.get_bounds());
	}

	public function node():Node
		return nodeValue;

	public function parent():Null<Node>
		return parentValue;

	public function sourceValue():haxe.Int64
		return sourceId;

	public function geometryValue():haxe.Int64
		return geometryId;

	public function materialValue():haxe.Int64
		return materialId;

	public function cameraValue():haxe.Int64
		return cameraId;

	public function lightValue():haxe.Int64
		return lightId;

	public function visible():Bool
		return visibleValue;

	public function localTransform():Transform
		return localTransformValue;

	public function worldTransform():Transform
		return worldTransformValue;

	public function worldTransformRevision():haxe.Int64
		return worldTransformRevisionValue;

	public function bounds():Bounds
		return boundsValue;
}
