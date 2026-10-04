package tests.spike;

import haxe.Int64;
import haxe.io.Bytes;
import haxeon.wire.MessagePackFrame;
import materia.sensor.wire.MessageType;
import materia.sensor.wire.PixelFormat;
import materia.sensor.wire.SensorWireCodec;
import robotkit.model.RobotModel;
import robotkit.streams.CameraImage;
import robotkit.core.SensorFrame;

/** Phase 7 prototype: one explicit sensorkit wire to RobotKit conversion point. */
class SensorkitCameraAdapter {
  public static function decode(wire:Bytes, model:RobotModel,
      numericSensorIds:Map<String, String>):SensorFrame {
    var packed = SensorWireCodec.decodePackedFrameMessage(MessagePackFrame.unpack(wire));
    if (packed.messageType != MessageType.cameraFrame || packed.pixelFormat != PixelFormat.rgba8)
      throw "Expected a sensorkit RGBA8 camera frame";
    var sensorId = numericSensorIds.get(Int64.toStr(packed.sensor));
    var sensor:Null<robotkit.model.Sensor> = null;
    if (sensorId != null) for (candidate in model.sensors)
      if (candidate.id == sensorId && candidate.kind == "camera") sensor = candidate;
    if (sensor == null) throw "Sensorkit camera sensor ID is absent from robot model";
    var width = checkedDimension(packed.width), height = checkedDimension(packed.height);
    CameraImage.validateBytes(width, height, "rgb8", width * height * 3);
    if (Int64.compare(packed.stride, Int64.ofInt(width * 4)) != 0 ||
        packed.data.length != width * height * 4)
      throw "Sensorkit camera RGBA8 layout is invalid";
    var rgb = Bytes.alloc(width * height * 3);
    for (i in 0...width * height) {
      rgb.set(i * 3, packed.data.get(i * 4));
      rgb.set(i * 3 + 1, packed.data.get(i * 4 + 1));
      rgb.set(i * 3 + 2, packed.data.get(i * 4 + 2));
    }
    var frameId = sensor.frame == null ? "" : sensor.frame.id;
    var linkId = sensor.frame == null ? "" : sensor.frame.link.id;
    return new SensorFrame(sensorId, "camera", frameId, packed.sequence,
      timestampNs(packed.captureTime), [], timestampNs(packed.deliveryTime), linkId,
      sensor.frame == null ? null : sensor.frame.position,
      sensor.frame == null ? null : sensor.frame.rotation,
      "sensorkit.sim", "sensorkit.sim", new CameraImage(width, height, "rgb8", rgb));
  }

  static function checkedDimension(value:Int64):Int {
    if (Int64.compare(value, Int64.ofInt(1)) < 0 ||
        Int64.compare(value, Int64.ofInt(4096)) > 0)
      throw "Sensorkit camera dimension is unsupported";
    return Int64.toInt(value);
  }

  static function timestampNs(seconds:Float):Int64 {
    if (!Math.isFinite(seconds) || seconds < 0 || seconds > 9000000000.0)
      throw "Sensorkit camera timestamp is invalid";
    return Int64.fromFloat(Math.round(seconds * 1000000000.0));
  }
}
