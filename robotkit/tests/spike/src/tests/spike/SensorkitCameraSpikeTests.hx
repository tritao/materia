package tests.spike;

import haxe.Int64;
import haxe.io.Bytes;
import haxeon.wire.MessagePackFrame;
import materia.sensor.wire.SensorWireCodec;
import robotkit.inference.InferenceSession;
import robotkit.model.Frame;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.Sensor;
import robotkit.inference.ObjectDetectorPipeline;
import robotkit.perception.PerceptionHost;
import sys.io.File;

class SensorkitCameraSpikeTests {
  static function check(ok:Bool, message:String):Void if (!ok) throw message;
  static function reject(call:Void->Void):Void {
    var failed = false;
    try call() catch (_:Dynamic) failed = true;
    check(failed, "invalid sensorkit camera frame was accepted");
  }
  public static function run():Int {
    var root = Sys.getCwd();
    var prefix = root + "/robotkit";
    if (!sys.FileSystem.exists(prefix + "/tests/fixtures/sensorkit-camera.hmpk"))
      prefix = root + "/../..";
    var wire = File.getBytes(prefix + "/tests/fixtures/sensorkit-camera.hmpk");
    var model = new RobotModel("spike");
    var link = model.addLink(new Link("base", "link/base"));
    var mount = model.addFrame(new Frame("camera", link, "frame/front-camera"));
    var sensor = model.addSensor(new Sensor("Front camera", "camera", 0, "front_camera"));
    sensor.frame = mount;
    var ids:Map<String, String> = ["50" => "front_camera"];
    var frame = SensorkitCameraAdapter.decode(wire, model, ids);
    check(frame.sensorId == "front_camera" && frame.frameId == "frame/front-camera" &&
      frame.linkId == "link/base" && frame.kind == "camera", "model metadata lost");
    check(frame.sourceClockId == "sensorkit.sim" && frame.receivedClockId == "sensorkit.sim" &&
      Int64.compare(frame.sourceTimestampNs, Int64.fromFloat(1250000000.0)) == 0 &&
      Int64.compare(frame.receivedTimestampNs, Int64.fromFloat(1375000000.0)) == 0,
      "simulation timestamps or clocks were mixed");
    check(frame.image != null && frame.image.width == 16 && frame.image.height == 16 &&
      frame.image.encoding == "rgb8", "rendered RGBA camera conversion failed");
    var rgb = frame.image.bytes();
    var center = (8 * 16 + 8) * 3;
    check(rgb.get(center) > 200 && rgb.get(center + 1) < 40 && rgb.get(center + 2) < 40,
      "rendered triangle did not survive wire conversion");
    ids.remove("50");
    reject(function() SensorkitCameraAdapter.decode(wire, model, ids));
    ids.set("50", "front_camera");
    var packed = SensorWireCodec.decodePackedFrameMessage(MessagePackFrame.unpack(wire));
    packed.stride = Int64.ofInt(1);
    reject(function() SensorkitCameraAdapter.decode(
      MessagePackFrame.pack(SensorWireCodec.encodePackedFrameMessage(packed)), model, ids));
    var modelPath = prefix + "/inference/tests/fixtures/detector.onnx";
    var detector = new ObjectDetectorPipeline("spike", "detector", "front_camera", "fixture",
      modelPath, InferenceSession.modelDigest(modelPath));
    var host = new PerceptionHost([detector]);
    host.submit(frame);
    var found = false;
    for (_ in 0...500) {
      var observations = host.poll();
      if (observations.length > 0) {
        check(observations[0].detections.length == 1 &&
          observations[0].sourceClockId == "sensorkit.sim", "detector result lost provenance");
        found = true;
        break;
      }
      Sys.sleep(0.005);
    }
    var sourcePixels = SensorWireCodec.decodePackedFrameMessage(MessagePackFrame.unpack(wire)).data;
    var widePixels = Bytes.alloc(640 * 480 * 4);
    for (y in 0...480) for (x in 0...640) {
      var source = ((Std.int(y * 16 / 480) * 16) + Std.int(x * 16 / 640)) * 4;
      var target = (y * 640 + x) * 4;
      for (channel in 0...4) widePixels.set(target + channel, sourcePixels.get(source + channel));
    }
    packed = SensorWireCodec.decodePackedFrameMessage(MessagePackFrame.unpack(wire));
    packed.width = Int64.ofInt(640);
    packed.height = Int64.ofInt(480);
    packed.stride = Int64.ofInt(640 * 4);
    packed.sequence = Int64.ofInt(2);
    packed.data = widePixels;
    var wideWire = MessagePackFrame.pack(SensorWireCodec.encodePackedFrameMessage(packed));
    var begin = Sys.time();
    var largeFrame = SensorkitCameraAdapter.decode(wideWire, model, ids);
    for (_ in 0...9) SensorkitCameraAdapter.decode(wideWire, model, ids);
    Sys.println('sensorkit_camera_haxe_decode_640x480_mean_ms=' + (Sys.time() - begin) * 100.0);
    check(largeFrame.image != null && largeFrame.image.width == 640 &&
      largeFrame.image.height == 480, "wide frame conversion failed");
    host.submit(largeFrame);
    var wideFound = false;
    for (_ in 0...500) {
      var observations = host.poll();
      if (observations.length > 0) { wideFound = true; break; }
      Sys.sleep(0.005);
    }
    host.dispose();
    check(found, "detector did not finish on rendered camera frame");
    check(wideFound, "detector did not finish on 640x480 camera frame");
    Sys.println("Sensorkit camera spike passed");
    return 1;
  }
}
