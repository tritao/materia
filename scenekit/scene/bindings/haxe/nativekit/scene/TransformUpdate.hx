package nativekit.scene;

import NativeKitScene;

/** One node transform in a bulk transaction update. */
class TransformUpdate {
    final value:nkscene_transform_update;
	public function new(node:Node, transform:Transform) {
		value = new nkscene_transform_update();
		value.set_node(node.nativeValue());
        value.set_transform(transform.nativeValue());
    }

    @:allow(Transaction)
    function nativeValue():nkscene_transform_update
        return value;
}
