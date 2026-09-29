package tests;

import haxe.Int64;
import haxe.io.Bytes;
import robotkit.deployment.SerialDeployment;
import robotkit.inference.InferenceSession;
import robotkit.perception.ImageDetection;
import robotkit.perception.ImageDetectionObservation;
import robotkit.perception.ObjectDetectorPipeline;
import robotkit.perception.PerceptionHost;
import robotkit.perception.PerceptionPipelineRegistry;
import robotkit.world.CameraImage;
import robotkit.world.ReplayRobot;
import robotkit.world.RobotRecording;
import robotkit.world.RobotSnapshot;
import robotkit.world.SensorFrame;

class PerceptionInferenceTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function throws(call:Void->Void, fragment:String):Void {
    var message = "";
    try call() catch (error:Dynamic) message = Std.string(error);
    check(message.indexOf(fragment) >= 0, 'Expected "$fragment", got "$message"');
  }
  static function fixture(name:String):String {
    var root = Sys.getCwd() + "/robotkit/tests/fixtures/device-deployment/";
    if (!sys.FileSystem.exists(root + name)) root = Sys.getCwd() + "/fixtures/device-deployment/";
    return root + name;
  }
  static function frame(sequence:Int):SensorFrame {
    var pixels = Bytes.alloc(8 * 4 * 3);
    for (i in 0...pixels.length) pixels.set(i, 51);
    return new SensorFrame("front_camera", "camera", "frame/front-camera", Int64.ofInt(sequence),
      Int64.ofInt(100 + sequence), [], Int64.ofInt(200 + sequence), "link/base",
      null, null, "camera.clock", "host.monotonic", new CameraImage(8, 4, "rgb8", pixels));
  }
  static function wait(host:PerceptionHost):ImageDetectionObservation {
    for (i in 0...500) {
      var found = host.poll();
      if (found.length > 0) return found[0];
      Sys.sleep(0.005);
    }
    throw "Detector did not finish";
  }
  public static function run():Int {
    var deployment = new SerialDeployment(fixture("perception-valid.json"));
    check(deployment.perception.length == 1, "v5 deployment loads perception");
    check(deployment.perception[0].host == "robotd" &&
      deployment.perception[0].consumers.length == 2, "host and consumers stay independent");
    for (name in ["unknown-key", "unknown-option", "bad-digest", "bad-input",
        "round-trip", "duplicate-id", "unknown-pipeline", "bad-consumer"])
      throws(function() new SerialDeployment(fixture('perception-$name.json')),
        switch name {
          case "unknown-key": "unknown perception key";
          case "unknown-option": "unknown perception options key";
          case "bad-digest": "SHA-256 mismatch";
          case "bad-input": "camera with an existing frame";
          case "round-trip": "network round trip";
          case "duplicate-id": "duplicate perception id";
          case "unknown-pipeline": "unknown perception pipeline";
          case _: "unsupported perception consumer";
        });
    var older = new SerialDeployment(fixture("deployment.json"));
    check(older.perception.length == 0, "v4 remains accepted without perception");
    check(older.fingerprint == deployment.fingerprint,
      "perception section stays outside the RKD6 device fingerprint");
    var config = deployment.perception[0];
    var direct = new PerceptionHost([PerceptionPipelineRegistry.create(config, "robotd/front_objects")]);
    var source = frame(7);
    direct.submit(source);
    var observation = wait(direct);
    check(observation.detections.length == 1, "NMS removes the lower-scoring overlap");
    var box = observation.detections[0];
    check(Math.abs(box.x - 2) < 0.01 && Math.abs(box.width - 4) < 0.01 &&
      Math.abs(box.y) < 0.01 && Math.abs(box.height - 4) < 0.01,
      "letterbox output maps to source-image pixels");
    check(observation.sequence == source.sequence &&
      observation.sourceTimestampNs == source.sourceTimestampNs &&
      observation.receivedTimestampNs == source.receivedTimestampNs &&
      observation.sourceClockId == source.sourceClockId &&
      observation.receivedClockId == source.receivedClockId &&
      observation.sourceFrameId == source.frameId &&
      Int64.compare(observation.completedTimestampNs, Int64.ofInt(0)) > 0,
      "observation retains provenance and separate clocks");
    check(observation.modelDigest == InferenceSession.modelDigest(config.modelPath),
      "model digest is actual file digest");
    var copy = observation.detections;
    copy.pop();
    check(observation.detections.length == 1, "observation owns its detection list");
    direct.dispose();

    var high = new ObjectDetectorPipeline("robotd/high", "high", "front_camera", "model",
      config.modelPath, config.modelSha256, 0.95);
    var highHost = new PerceptionHost([high]);
    highHost.submit(frame(8));
    check(wait(highHost).detections.length == 0, "threshold produces an empty result");
    highHost.dispose();

    var bad = new ObjectDetectorPipeline("robotd/bad", "bad", "front_camera", "model",
      config.modelPath, config.modelSha256);
    var jpeg = new SensorFrame("front_camera", "camera", "frame/front-camera", Int64.ofInt(1),
      Int64.ofInt(1), [], Int64.ofInt(2), "link/base", null, null, "camera.clock", "host.monotonic",
      new CameraImage(8, 4, "jpeg", Bytes.ofString("jpeg")));
    throws(function() bad.submit(jpeg), "rgb8");
    bad.dispose();

    var recording = new RobotRecording();
    var snapshot = new RobotSnapshot("robot-a", Int64.ofInt(7), Int64.ofInt(100),
      [], [], [], 1, 0, Int64.ofInt(200), [source], "robot.clock", "host.monotonic");
    recording.recordSnapshot(snapshot);
    var replay = new ReplayRobot("robot-a", recording);
    var replayHost = new PerceptionHost([PerceptionPipelineRegistry.create(config, "robotd/front_objects")]);
    replayHost.submit(replay.sensors()[0]);
    var replayed = wait(replayHost);
    check(replayed.producerId == observation.producerId &&
      replayed.modelDigest == observation.modelDigest &&
      replayed.sequence == observation.sequence &&
      replayed.sourceTimestampNs == observation.sourceTimestampNs &&
      replayed.receivedTimestampNs == observation.receivedTimestampNs &&
      replayed.detections.length == observation.detections.length &&
      Math.abs(replayed.detections[0].x - observation.detections[0].x) < 1e-6,
      "replay and direct frames produce the same observation except completion time");
    replayHost.dispose(); replay.close();
    Sys.println('RobotKit perception inference tests passed ($assertions assertions)');
    return assertions;
  }
}
