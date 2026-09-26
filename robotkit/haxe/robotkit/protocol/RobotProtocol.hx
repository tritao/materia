package robotkit.protocol;

import haxe.io.Bytes;
import haxeon.wire.MessagePack;
import haxeon.wire.MessagePackError;
import robotkit.world.CameraImage;

/** Typed RobotKit message helpers; the frame envelope remains independent. */
class RobotProtocol {
  public static function hello(value:Hello, ?sessionId:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.Hello, MessagePack.encode(value), sessionId);

  public static function welcome(value:Welcome, ?sessionId:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.Welcome, MessagePack.encode(value), sessionId);

  public static function description(value:RobotDescription,
      ?sessionId:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.RobotDescription, MessagePack.encode(value), sessionId);

  public static function capabilities(value:RobotCapabilities,
      ?sessionId:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.RobotCapabilities, MessagePack.encode(value), sessionId);

  public static function state(value:RobotStateMsg, ?sessionId:haxe.Int64 = null,
      ?sequence:haxe.Int64 = null, ?timestampNs:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.RobotState, MessagePack.encode(value),
      sessionId, sequence, timestampNs);

  public static function jointTarget(value:JointTarget, ?sessionId:haxe.Int64 = null,
      ?sequence:haxe.Int64 = null, ?timestampNs:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.JointTarget, MessagePack.encode(value), sessionId,
      sequence, timestampNs);

  public static function jointTargets(value:JointTargets, ?sessionId:haxe.Int64 = null,
      ?sequence:haxe.Int64 = null, ?timestampNs:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.JointTargets, MessagePack.encode(value), sessionId,
      sequence, timestampNs);

  public static function stop(value:Stop, ?sessionId:haxe.Int64 = null,
      ?sequence:haxe.Int64 = null, ?timestampNs:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.Stop, MessagePack.encode(value), sessionId,
      sequence, timestampNs);

  public static function safetyReset(value:SafetyReset,
      ?sessionId:haxe.Int64 = null, ?sequence:haxe.Int64 = null,
      ?timestampNs:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.SafetyReset, MessagePack.encode(value),
      sessionId, sequence, timestampNs);

  public static function controlHeartbeat(value:ControlHeartbeat,
      ?sessionId:haxe.Int64 = null, ?timestampNs:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.ControlHeartbeat, MessagePack.encode(value),
      sessionId, null, timestampNs);

  public static function sensorFrame(value:SensorFrameMsg,
      ?sessionId:haxe.Int64 = null, ?sequence:haxe.Int64 = null,
      ?timestampNs:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.SensorFrame, MessagePack.encode(value),
      sessionId, sequence, timestampNs);

  /** Encodes image metadata separately from the potentially large pixel buffer. */
  public static function cameraFrame(value:CameraFrame, pixels:Bytes,
      ?sessionId:haxe.Int64 = null, ?sequence:haxe.Int64 = null,
      ?timestampNs:haxe.Int64 = null):RobotFrame {
    validateCameraMetadata(value);
    if (pixels == null)
      throw new MessagePackError("CameraFrame pixels are required");
    // CameraImage applies the same dimension and raw-format size checks used by
    // the world model before bytes cross the process boundary.
    validateCameraImage(value, pixels.length);
    var wireValue = new CameraFrame(value.robotId, value.sensorId, value.kind,
      value.frameId, value.sequence, value.sourceTimestampNs,
      value.receivedTimestampNs, value.width, value.height, value.format,
      new BufferRef(0, 0, pixels.length), value.linkId, value.mountPosition,
      value.mountRotation, value.sourceClockId, value.receivedClockId);
    return new RobotFrame(RobotMessageType.CameraFrame,
      MessagePack.encode(wireValue), 0, [copyBytes(pixels)], sessionId, sequence,
      timestampNs);
  }

  public static function decodeHello(frame:RobotFrame):Hello
    return decodeHelloPayload(frame);

  public static function decodeWelcome(frame:RobotFrame):Welcome
    return decodeWelcomePayload(frame);

  public static function decodeJointTarget(frame:RobotFrame):JointTarget
    return decodeJointTargetPayload(frame);

  public static function decodeJointTargets(frame:RobotFrame):JointTargets
    return decodeJointTargetsPayload(frame);

  public static function decodeStop(frame:RobotFrame):Stop
    return decodeStopPayload(frame);

  public static function decodeSafetyReset(frame:RobotFrame):SafetyReset
    return decodeSafetyResetPayload(frame);

  public static function decodeControlHeartbeat(frame:RobotFrame):ControlHeartbeat
    return decodeControlHeartbeatPayload(frame);

  public static function decodeSensorFrame(frame:RobotFrame):SensorFrameMsg
    return decodeSensorFramePayload(frame);

  /** Decodes metadata and resolves an owned copy of its referenced pixels. */
  public static function decodeCameraFrame(frame:RobotFrame):CameraFrameData
    return decodeCameraFramePayload(frame);

  public static function decodeState(frame:RobotFrame):RobotStateMsg
    return decodeStatePayload(frame);

  public static function decodeDescription(frame:RobotFrame):RobotDescription
    return decodeDescriptionPayload(frame);

  public static function decodeCapabilities(frame:RobotFrame):RobotCapabilities
    return decodeCapabilitiesPayload(frame);

  public static function decodeFault(frame:RobotFrame):Fault
    return decodeFaultPayload(frame);

  static function decodeHelloPayload(frame:RobotFrame):Hello {
    expect(frame, RobotMessageType.Hello);
    return MessagePack.decode(frame.payload);
  }

  static function decodeWelcomePayload(frame:RobotFrame):Welcome {
    expect(frame, RobotMessageType.Welcome);
    return MessagePack.decode(frame.payload);
  }

  static function decodeJointTargetPayload(frame:RobotFrame):JointTarget {
    expect(frame, RobotMessageType.JointTarget);
    return MessagePack.decode(frame.payload);
  }

  static function decodeJointTargetsPayload(frame:RobotFrame):JointTargets {
    expect(frame, RobotMessageType.JointTargets);
    return MessagePack.decode(frame.payload);
  }

  static function decodeStopPayload(frame:RobotFrame):Stop {
    expect(frame, RobotMessageType.Stop);
    return MessagePack.decode(frame.payload);
  }

  static function decodeSafetyResetPayload(frame:RobotFrame):SafetyReset {
    expect(frame, RobotMessageType.SafetyReset);
    return MessagePack.decode(frame.payload);
  }

  static function decodeControlHeartbeatPayload(frame:RobotFrame):ControlHeartbeat {
    expect(frame, RobotMessageType.ControlHeartbeat);
    return MessagePack.decode(frame.payload);
  }

  static function decodeSensorFramePayload(frame:RobotFrame):SensorFrameMsg {
    expect(frame, RobotMessageType.SensorFrame);
    return MessagePack.decode(frame.payload);
  }

  static function decodeCameraFramePayload(frame:RobotFrame):CameraFrameData {
    expect(frame, RobotMessageType.CameraFrame);
    var value:CameraFrame = MessagePack.decode(frame.payload);
    validateCameraMetadata(value);
    if (value.pixels == null || value.pixels.attachment < 0 ||
        value.pixels.attachment >= frame.attachments.length ||
        value.pixels.offset < 0 || value.pixels.length <= 0)
      throw new MessagePackError("CameraFrame has an invalid pixel buffer reference");
    var attachment = frame.attachments[value.pixels.attachment];
    // Subtract before adding to avoid integer overflow on untrusted offsets.
    if (value.pixels.offset > attachment.length ||
        value.pixels.length > attachment.length - value.pixels.offset)
      throw new MessagePackError("CameraFrame pixel buffer exceeds its attachment");
    validateCameraImage(value, value.pixels.length);
    return new CameraFrameData(value, attachment, value.pixels.offset,
      value.pixels.length);
  }

  static function decodeStatePayload(frame:RobotFrame):RobotStateMsg {
    expect(frame, RobotMessageType.RobotState);
    return MessagePack.decode(frame.payload);
  }

  static function decodeDescriptionPayload(frame:RobotFrame):RobotDescription {
    expect(frame, RobotMessageType.RobotDescription);
    return MessagePack.decode(frame.payload);
  }

  static function decodeCapabilitiesPayload(frame:RobotFrame):RobotCapabilities {
    expect(frame, RobotMessageType.RobotCapabilities);
    return MessagePack.decode(frame.payload);
  }

  static function decodeFaultPayload(frame:RobotFrame):Fault {
    expect(frame, RobotMessageType.Fault);
    return MessagePack.decode(frame.payload);
  }

  static function message(type:RobotMessageType, payload:Bytes,
      ?sessionId:haxe.Int64 = null, ?sequence:haxe.Int64 = null,
      ?timestampNs:haxe.Int64 = null):RobotFrame
    return new RobotFrame(type, payload, 0, null, sessionId, sequence,
      timestampNs);

  static function validateCameraMetadata(value:CameraFrame):Void {
    if (value == null || value.sensorId == null || value.sensorId.length == 0 ||
        value.kind == null || value.kind.length == 0 || value.frameId == null ||
        value.frameId.length == 0 || value.linkId == null ||
        value.sourceClockId == null || value.sourceClockId.length == 0 ||
        value.receivedClockId == null || value.receivedClockId.length == 0)
      throw new MessagePackError("CameraFrame is missing sensor, frame, mount, or clock identity");
    if (value.mountPosition == null || value.mountPosition.length != 3 ||
        value.mountRotation == null || value.mountRotation.length != 4)
      throw new MessagePackError("CameraFrame mount pose must contain three position and four rotation values");
    for (component in value.mountPosition)
      if (!Math.isFinite(component))
        throw new MessagePackError("CameraFrame mount position must be finite");
    for (component in value.mountRotation)
      if (!Math.isFinite(component))
        throw new MessagePackError("CameraFrame mount rotation must be finite");
    if (haxe.Int64.compare(value.robotId, haxe.Int64.ofInt(0)) < 0 ||
        haxe.Int64.compare(value.sequence, haxe.Int64.ofInt(0)) < 0 ||
        haxe.Int64.compare(value.sourceTimestampNs, haxe.Int64.ofInt(0)) < 0 ||
        haxe.Int64.compare(value.receivedTimestampNs, haxe.Int64.ofInt(0)) < 0)
      throw new MessagePackError("CameraFrame IDs, sequence, and timestamps must be non-negative");
  }

  static function validateCameraImage(value:CameraFrame, byteLength:Int):Void {
    var encoding = switch value.format {
      case PixelFormat.RGB8: "rgb8";
      case PixelFormat.Depth32F: "depth32f";
      case PixelFormat.JPEG: "jpeg";
      case _: throw new MessagePackError("CameraFrame has an unsupported pixel format");
    };
    try {
      CameraImage.validateBytes(value.width, value.height, encoding, byteLength);
    } catch (error:Dynamic) {
      throw new MessagePackError('Invalid CameraFrame image: ${Std.string(error)}');
    }
  }

  static function copySlice(source:Bytes, offset:Int, length:Int):Bytes {
    var result = Bytes.alloc(length);
    result.blit(0, source, offset, length);
    return result;
  }

  static function copyBytes(source:Bytes):Bytes
    return copySlice(source, 0, source.length);

  static function expect(frame:RobotFrame, expected:RobotMessageType):Void {
    if (frame.messageType != expected)
      throw new MessagePackError('Unexpected RobotKit message ${frame.messageType}, expected $expected');
  }
}
