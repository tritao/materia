package tests;

import haxe.Int64;
import haxe.io.Bytes;
import haxe.Json;
import haxe.crypto.Sha256;
import robotkit.deployment.SerialDeployment;
import robotkit.perception.FiducialPerception;
import robotkit.streams.ImageDetection;
import robotkit.streams.ImageDetectionObservation;
import robotkit.perception.ImageDetectionLifter;
import robotkit.inference.ObjectDetectorPipeline;
import robotkit.mobile.Pose3;
import robotkit.perception.FiducialTargetConfig;
import robotkit.perception.VisionKitFiducialDetector;
import robotkit.streams.CameraImage;
import robotkit.core.SensorFrame;
import visionkit.CameraCalibration;
import visionkit.CameraModel;
import visionkit.ImageView;
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
    for (y in 0...120) for (x in 0...120) {
      var row = rows[Std.int(y/20)];
      if (row.charAt(Std.int(x/20)) == "0") {
        var offset = ((180+y)*width + 80+x)*3;
        pixels.set(offset, 0); pixels.set(offset+1, 0); pixels.set(offset+2, 0);
      }
    }
    var direct = new MarkerDetector(MarkerDetector.ARUCO_4X4_50);
    var crowded = direct.detect(new ImageView(width, height, width*3, 1, pixels),
      model, 0.24, 1);
    if (crowded.length != 2) throw "Marker detection did not grow its result buffer";
    direct.dispose();
    testLifting();
    testDeployment();
    trace("VisionKit RobotKit integration passed");
  }

  static function testLifting():Void {
    var camera = new CameraModel(16, 12, 8, 8, 8, 6);
    var rgb = new SensorFrame("rgb", "camera", "cam", Int64.ofInt(7),
      Int64.ofInt(100), [], Int64.ofInt(110), "", null, null,
      "source", "host", new CameraImage(16, 12, "rgb8", Bytes.alloc(16 * 12 * 3)));
    var depthBytes = Bytes.alloc(16 * 12 * 4);
    for (i in 0...16 * 12) depthBytes.setFloat(i * 4, 2.0);
    var depth = new SensorFrame("depth", "camera", "cam", Int64.ofInt(7),
      Int64.ofInt(100), [], Int64.ofInt(110), "", null, null,
      "source", "host", new CameraImage(16, 12, "depth32f", depthBytes));
    var digest = "0000000000000000000000000000000000000000000000000000000000000000";
    var box = new ImageDetection("crate", 0.9, 8, 5, 2, 2);
    var observation = new ImageDetectionObservation("test", "model", "rgb", "model",
      digest, "cam", Int64.ofInt(7), Int64.ofInt(100), Int64.ofInt(110),
      "source", "host", Int64.ofInt(120), "host", [box], 0);
    var source = function(_:SensorFrame) return observation;
    var lifter = new ImageDetectionLifter(camera, "rgb", source, "depth");
    var value = lifter.observe([rgb, depth]).detections()[0];
    if (Math.abs(value.pose.x - 2) > 1e-5 || Math.abs(value.pose.y + 0.25) > 1e-5 ||
        value.frameId != "cam" || value.sourceClockId != "source")
      throw "Depth lifting position or metadata is wrong";
    var stale = new SensorFrame("depth", "camera", "cam", Int64.ofInt(8),
      Int64.ofInt(100), [], Int64.ofInt(110), "", null, null,
      "source", "host", depth.image);
    try { lifter.observe([rgb, stale]); throw "Stale depth was accepted"; }
    catch (error:Dynamic) { if (Std.string(error) == "Stale depth was accepted") throw error; }
    var wrongTime = new SensorFrame("depth", "camera", "cam", Int64.ofInt(7),
      Int64.ofInt(101), [], Int64.ofInt(110), "", null, null,
      "source", "host", depth.image);
    try { lifter.observe([rgb, wrongTime]); throw "Mismatched timestamp was accepted"; }
    catch (error:Dynamic) { if (Std.string(error) == "Mismatched timestamp was accepted") throw error; }
    var pitch = Math.PI / 4;
    var mount = new Pose3(0, 0, 1, 0, Math.sin(pitch / 2), 0, Math.cos(pitch / 2));
    var groundBox = new ImageDetection("crate", 0.9, 7, 5, 2, 1);
    var groundObservation = new ImageDetectionObservation("test", "model", "rgb", "model",
      digest, "cam", Int64.ofInt(7), Int64.ofInt(100), Int64.ofInt(110),
      "source", "host", Int64.ofInt(120), "host", [groundBox], 0);
    var ground = new ImageDetectionLifter(camera, "rgb", function(_) return groundObservation,
      null, "body", mount);
    var floor = ground.observe([rgb]).detections()[0];
    if (Math.abs(floor.pose.x - 1) > 1e-5 || Math.abs(floor.pose.y) > 1e-5 ||
        floor.frameId != "body") throw "Ground lifting position is wrong";
    var unchanged = ObjectDetectorPipeline.undistortBoxes(camera, [box])[0];
    if (Math.abs(unchanged.x - box.x) > 1e-5 || Math.abs(unchanged.y - box.y) > 1e-5)
      throw "Pinhole box undistortion changed pixel coordinates";
  }

  static function testDeployment():Void {
    var root = "../../robotkit/tests/fixtures/device-deployment/";
    var directory = Sys.getCwd() + "build-robotkit/deployment-fixture";
    if (!sys.FileSystem.exists(directory)) sys.FileSystem.createDirectory(directory);
    var model:Dynamic = Json.parse(sys.io.File.getContent(root + "robot.json"));
    var sensors:Array<Dynamic> = cast Reflect.field(model, "sensors");
    sensors.push({id: "sensor/cam", name: "camera", kind: "camera", updateRate: 30,
      frame: null, rayCount: 8, maxRange: 10, startAngleRadians: 0,
      fieldOfViewRadians: 6.283185307179586, noiseStddev: 0, noiseSeed: 1});
    sys.io.File.saveContent(directory + "/robot.json", Json.stringify(model));
    sys.io.File.saveBytes(directory + "/layout.json", sys.io.File.getBytes(root + "layout.json"));
    sys.io.File.saveBytes(directory + "/schema.lock.json", sys.io.File.getBytes("../../robotkit/schema/device_wire6.lock.json"));
    var deployment:Dynamic = Json.parse(sys.io.File.getContent(root + "deployment.json"));
    Reflect.setField(deployment, "schemaVersion", 5);
    Reflect.setField(Reflect.field(deployment, "device"), "schema_lock", "schema.lock.json");
    var bytes = Bytes.ofString(calibrationJson());
    sys.io.File.saveBytes(directory + "/camera.json", bytes);
    var row:Dynamic = {sensorId: "sensor/cam", calibration: "camera.json",
      sha256: Sha256.encode(bytes.toString())};
    Reflect.setField(deployment, "cameras", [row]);
    var path = directory + "/deployment.json";
    sys.io.File.saveContent(path, Json.stringify(deployment));
    var parsed = new SerialDeployment(path);
    if (!parsed.cameras.exists("sensor/cam") || parsed.cameras.get("sensor/cam").model.width != 640)
      throw "Camera deployment was not loaded";
    Reflect.setField(row, "sha256", "0000000000000000000000000000000000000000000000000000000000000000");
    sys.io.File.saveContent(path, Json.stringify(deployment));
    expectInvalid(path);
    Reflect.setField(row, "sha256", Sha256.encode(bytes.toString()));
    var invalidBytes = Bytes.ofString("{}");
    sys.io.File.saveBytes(directory + "/camera.json", invalidBytes);
    Reflect.setField(row, "sha256", Sha256.encode(invalidBytes.toString()));
    sys.io.File.saveContent(path, Json.stringify(deployment));
    expectInvalid(path);
    sys.io.File.saveBytes(directory + "/camera.json", bytes);
    Reflect.setField(row, "sha256", Sha256.encode(bytes.toString()));
    Reflect.setField(row, "sensorId", "missing");
    sys.io.File.saveContent(path, Json.stringify(deployment));
    expectInvalid(path);
    Reflect.setField(row, "sensorId", "sensor/cam");
    Reflect.setField(row, "calibration", "../camera.json");
    sys.io.File.saveContent(path, Json.stringify(deployment));
    expectInvalid(path);
    Reflect.setField(row, "calibration", "camera.json");
    Reflect.setField(row, "extra", 1);
    sys.io.File.saveContent(path, Json.stringify(deployment));
    expectInvalid(path);
    Reflect.deleteField(row, "extra");
    Reflect.setField(deployment, "schemaVersion", 4);
    sys.io.File.saveContent(path, Json.stringify(deployment));
    expectInvalid(path);
    Reflect.setField(deployment, "schemaVersion", 5);
    Reflect.setField(sensors[0], "kind", "lidar");
    sys.io.File.saveContent(directory + "/robot.json", Json.stringify(model));
    sys.io.File.saveContent(path, Json.stringify(deployment));
    expectInvalid(path);
  }

  static function calibrationJson():String {
    return new CameraCalibration(new CameraModel(640, 480, 500, 500, 320, 240),
      0.1, "2026-09-29T00:00:00Z", "test", "fixture").toJson();
  }

  static function expectInvalid(path:String):Void {
    try { new SerialDeployment(path); } catch (_:Dynamic) { return; }
    throw "Invalid camera deployment was accepted";
  }
}
