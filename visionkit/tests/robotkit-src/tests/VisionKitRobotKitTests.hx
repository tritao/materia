package tests;

import haxe.Int64;
import haxe.io.Bytes;
import robotkit.perception.FiducialPerception;
import robotkit.perception.FiducialTargetConfig;
import robotkit.perception.VisionKitFiducialDetector;
import robotkit.world.CameraImage;
import robotkit.world.SensorFrame;
import visionkit.CameraCalibration;
import visionkit.CameraModel;
import visionkit.MarkerDetector;

class VisionKitRobotKitTests {
  static function main():Void {
    var width = 640, height = 480;
    var pixels = Bytes.alloc(width*height*3);
    for (i in 0...pixels.length) pixels.set(i, 255);
    // OpenCV 4.14's DICT_4X4_50 marker 7, including its one-cell black border.
    var rows = ["000000", "011000", "001000", "011110", "000100", "000000"];
    for (y in 0...120) for (x in 0...120) {
      var row = rows[Std.int(y/20)];
      if (row.charAt(Std.int(x/20)) == "0") {
        var offset = ((180+y)*width + 260+x)*3;
        pixels.set(offset, 0); pixels.set(offset+1, 0); pixels.set(offset+2, 0);
      }
    }
    var model = new CameraModel(width, height, 500, 500, 320, 240);
    var calibration = new CameraCalibration(model, 0.1,
      "2026-09-29T00:00:00Z", "synthetic marker", "test");
    var frame = new SensorFrame("cam", "camera", "cam-frame", Int64.ofInt(1),
      Int64.ofInt(100), [], Int64.ofInt(110), "", null, null,
      "camera.clock", "host.clock", new CameraImage(width, height, "rgb8", pixels));
    var detector = new VisionKitFiducialDetector(calibration,
      MarkerDetector.ARUCO_4X4_50, 0.24);
    var markers = detector.detect(frame);
    if (markers.length != 1 || markers[0].markerId != 7 || markers[0].pose3 == null)
      throw "VisionKit RobotKit adapter did not recover marker 7";
    var pose:robotkit.mobile.Pose3 = cast markers[0].pose3;
    if (pose.x < 0.9 || pose.x > 1.1) throw "Marker metric pose is wrong";
    var perception = new FiducialPerception("cam", [
      new FiducialTargetConfig(7, "pallet", 1.0, 0.8, 0.15)], 0.2);
    var snapshot = perception.observeCameraFrames([frame], detector);
    if (snapshot.pallets().length != 1 || snapshot.detections().length != 1 ||
        snapshot.detections()[0].frameId != "cam-frame")
      throw "FiducialPerception did not consume VisionKit marker";
    detector.dispose();
    trace("VisionKit RobotKit integration passed");
  }
}
