package nativekit.scene;

import NativeKitScene;
import nativekit.scene.Image as SceneImage;

/** Value builder connecting a scene texture to an image asset. */
class TextureData {
    final value:nkscene_texture_data;

    public function new(image:SceneImage) {
        value = new nkscene_texture_data();
        value.set_struct_size(nkscene_texture_data.size());
        value.set_image(image.id());
    }

    @:allow(Scene)
    function nativeValue():nkscene_texture_data
        return value;
}
