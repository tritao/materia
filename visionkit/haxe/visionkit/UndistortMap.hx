package visionkit;

import VisionKitNative;

/** Reusable native remap. Dispose it after use. */
class UndistortMap {
  final owner:Ownedvk_undistort_map;
  public final rectified:CameraModel;
  var disposed:Bool = false;

  public function new(model:CameraModel, cropValid:Bool = false) {
    var rect = new vk_camera_model();
    rect.set_struct_size(vk_camera_model.size());
    var created = VisionKitNative.vk_undistort_map_create(model.native(),
      cropValid ? 1 : 0, rect);
    if (created.status != 0)
      throw 'Map creation failed with VisionKit error ${created.status}';
    owner = created.out_map;
    rectified = CameraModel.fromNative(rect);
  }

  public function remap(src:ImageView, dst:ImageView):Void {
    if (disposed) throw "Undistort map has been disposed";
    var result = VisionKitNative.vk_undistort_image(owner.borrow(), src.native(), dst.native());
    if (result != 0)
      throw 'Remap failed with VisionKit error $result';
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }
}
