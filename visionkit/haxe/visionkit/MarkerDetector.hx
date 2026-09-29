package visionkit;

import VisionKitNative;

/** Synchronous ArUco or AprilTag detector. Call dispose after use. */
class MarkerDetector {
  public static inline final ARUCO_4X4_50 = 1;
  public static inline final ARUCO_5X5_100 = 2;
  public static inline final APRILTAG_36H11 = 3;

  final owner:Ownedvk_marker_detector;
  var disposed:Bool = false;

  public function new(dictionary:Int, minPerimeterRate:Float = 0.03,
      refineCorners:Bool = true) {
    var params = new vk_marker_detector_params();
    params.set_struct_size(vk_marker_detector_params.size());
    params.set_min_marker_perimeter_rate(minPerimeterRate);
    params.set_corner_refinement(refineCorners ? 1 : 0);
    var created = VisionKitNative.vk_marker_detector_create(dictionary, params);
    if (created.status != 0)
      throw 'Marker detector creation failed with VisionKit error ${created.status}';
    owner = created.out_detector;
  }

  public function detect(image:ImageView, model:CameraModel,
      markerSizeMetres:Float, capacity:Int = 64):Array<MarkerObservation> {
    if (disposed) throw "Marker detector has been disposed";
    var result = VisionKitNative.vk_marker_detect(owner.borrow(), image.native(),
      model.native(), markerSizeMetres, capacity);
    if (result.status != 0)
      throw 'Marker detection failed with VisionKit error ${result.status}';
    var output:Array<MarkerObservation> = [];
    for (i in 0...result.out_count) {
      var marker = result.out_markers[i];
      var corners:Array<{x:Float,y:Float}> = [];
      for (j in 0...4) {
        var corner = marker.get_corners(j);
        corners.push({x: corner.get_x(), y: corner.get_y()});
      }
      output.push(new MarkerObservation(marker.get_id(), corners,
        Pose3.fromNative(marker.get_camera_T_marker()),
        marker.get_rms_reprojection_error(), marker.get_confidence()));
    }
    return output;
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }
}
