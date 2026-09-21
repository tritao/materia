package robotkit.protocol;

import haxe.io.Bytes;
import haxeon.wire.MessagePack;
import haxeon.wire.MessagePackError;

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

  public static function stop(value:Stop, ?sessionId:haxe.Int64 = null,
      ?sequence:haxe.Int64 = null, ?timestampNs:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.Stop, MessagePack.encode(value), sessionId,
      sequence, timestampNs);

  public static function sensorFrame(value:SensorFrameMsg,
      ?sessionId:haxe.Int64 = null, ?sequence:haxe.Int64 = null,
      ?timestampNs:haxe.Int64 = null):RobotFrame
    return message(RobotMessageType.SensorFrame, MessagePack.encode(value),
      sessionId, sequence, timestampNs);

  public static function decodeHello(frame:RobotFrame):Hello
    return decodeHelloPayload(frame);

  public static function decodeWelcome(frame:RobotFrame):Welcome
    return decodeWelcomePayload(frame);

  public static function decodeJointTarget(frame:RobotFrame):JointTarget
    return decodeJointTargetPayload(frame);

  public static function decodeStop(frame:RobotFrame):Stop
    return decodeStopPayload(frame);

  public static function decodeSensorFrame(frame:RobotFrame):SensorFrameMsg
    return decodeSensorFramePayload(frame);

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

  static function decodeStopPayload(frame:RobotFrame):Stop {
    expect(frame, RobotMessageType.Stop);
    return MessagePack.decode(frame.payload);
  }

  static function decodeSensorFramePayload(frame:RobotFrame):SensorFrameMsg {
    expect(frame, RobotMessageType.SensorFrame);
    return MessagePack.decode(frame.payload);
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

  static function expect(frame:RobotFrame, expected:RobotMessageType):Void {
    if (frame.messageType != expected)
      throw new MessagePackError('Unexpected RobotKit message ${frame.messageType}, expected $expected');
  }
}
