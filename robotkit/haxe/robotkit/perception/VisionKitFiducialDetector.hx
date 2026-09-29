package robotkit.perception;

import robotkit.mobile.Pose3;
import robotkit.world.SensorFrame;
import visionkit.CameraCalibration;
import visionkit.ImageView;
import visionkit.MarkerDetector;

/** Converts raw RGB camera frames into typed marker poses. */
class VisionKitFiducialDetector implements FiducialDetector {
  public final calibration:CameraCalibration;
  public final markerSizeMetres:Float;
  final detector:MarkerDetector;
  var disposed:Bool = false;

  public function new(calibration:CameraCalibration, dictionary:Int,
      markerSizeMetres:Float) {
    if (calibration == null || !Math.isFinite(markerSizeMetres) || markerSizeMetres <= 0)
      throw "Fiducial detector requires a calibration and positive marker size";
    this.calibration = calibration;
    this.markerSizeMetres = markerSizeMetres;
    detector = new MarkerDetector(dictionary);
  }

  public function detect(frame:SensorFrame):Array<FiducialMarkerObservation> {
    if (disposed) throw "Fiducial detector has been disposed";
    if (frame == null || frame.kind != "camera" || frame.image == null)
      throw "Fiducial detector requires a camera image frame";
    var image = frame.image;
    if (image.encoding != "rgb8" ||
        image.width != calibration.model.width ||
        image.height != calibration.model.height)
      throw "Camera image must be raw RGB8 and match its calibration size";
    var view = new ImageView(image.width, image.height, image.width*3,
      1, image.bytes());
    return [for (marker in detector.detect(view, calibration.model, markerSizeMetres)) {
      var p = marker.camera_T_marker;
      FiducialMarkerObservation.fromPose3(marker.id,
        new Pose3(p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw), marker.confidence);
    }];
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    detector.dispose();
  }
}
