package machinekit.operator;

import haxe.io.Bytes;
import machinekit.operator.wire.OperatorState;
import machinekit.operator.wire.PendantConfig;
import machinekit.operator.wire.PendantProfile;
import machinekit.operator.wire.PendantReasonText;

/**
 * Frame of the host to cabinet controller operator channel, beside (not inside) the RKD6 motion
 * protocol: marker "OPR1", message type, reserved zero, little-endian payload length, payload, then
 * CRC-32/IEEE over everything before it. The rules mirror pendant_wire/src/operator_frame.rs.
 */
class OperatorFrame {
  public static inline final PROFILE:Int = 1;
  public static inline final REASON_TEXT:Int = 2;
  public static inline final CONFIG:Int = 3;
  public static inline final STATE:Int = 4;
  public static inline final HEADER_SIZE:Int = 8;

  public final type:Int;
  public final payload:Bytes;
  public function new(type:Int, payload:Bytes) {
    if (payloadSize(type) != payload.length) throw "Operator payload has the wrong size for its type";
    this.type = type;
    this.payload = payload;
  }

  public static function payloadSize(type:Int):Int return switch type {
    case 1: PendantProfile.SIZE;
    case 2: PendantReasonText.SIZE;
    case 3: PendantConfig.SIZE;
    case 4: OperatorState.SIZE;
    case _: throw "Unknown operator message type";
  };

  public function encode():Bytes {
    var out = Bytes.alloc(HEADER_SIZE + payload.length + 4);
    out.set(0, 0x4F); out.set(1, 0x50); out.set(2, 0x52); out.set(3, 0x31); // "OPR1"
    out.set(4, type);
    out.set(5, 0);
    out.set(6, payload.length & 0xFF);
    out.set(7, payload.length >> 8);
    out.blit(HEADER_SIZE, payload, 0, payload.length);
    var crc = crc32(out, HEADER_SIZE + payload.length);
    for (i in 0...4) out.set(HEADER_SIZE + payload.length + i, (crc >> (i * 8)) & 0xFF);
    return out;
  }

  public static function decode(input:Bytes):OperatorFrame {
    if (input == null || input.length < HEADER_SIZE + 4) throw "Operator frame is too short";
    if (input.get(0) != 0x4F || input.get(1) != 0x50 || input.get(2) != 0x52 || input.get(3) != 0x31) throw "Bad operator frame marker";
    if (input.get(5) != 0) throw "Operator frame reserved byte is set";
    var length = input.get(6) | (input.get(7) << 8);
    if (input.length != HEADER_SIZE + length + 4) throw "Operator frame length mismatch";
    var crc = 0;
    for (i in 0...4) crc |= input.get(HEADER_SIZE + length + i) << (i * 8);
    if (crc != crc32(input, HEADER_SIZE + length)) throw "Operator frame CRC mismatch";
    var payload = Bytes.alloc(length);
    payload.blit(0, input, HEADER_SIZE, length);
    return new OperatorFrame(input.get(4), payload);
  }

  /** CRC-32/IEEE (reflected, polynomial 0xEDB88320), the same as the RKD6 frames. */
  public static function crc32(bytes:Bytes, length:Int):Int {
    var crc = -1;
    for (i in 0...length) {
      crc = crc ^ bytes.get(i);
      for (_ in 0...8) crc = (crc & 1) != 0 ? (crc >>> 1) ^ -306674912 : crc >>> 1;
    }
    return ~crc;
  }
}
