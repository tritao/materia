package machinekit.operator;

import haxe.io.Bytes;

/**
 * CAN-FD payloads for the three pendant messages. All integers are little endian. Each payload
 * is padded with zero bytes to a valid CAN-FD frame length (8, 16, 24); the decoder accepts only
 * that exact length and requires zero padding and zero reserved bits. CAN identifiers are
 * placeholders until CC1 fixes the bus plan.
 *
 * PENDANT_STATE (7 bytes, padded to 8): sequence u16, wheel delta i16, byte 4 = position (bits 0
 * to 3) | step (bits 4 to 5), buttons u8 (bits 0 to 5), flags u8 (bit 0 wheel enable, bit 1
 * E-stop chain closed).
 * PENDANT_CONFIG (14 bytes, padded to 16): kind u8, index u8, text 12 bytes NUL padded.
 * PENDANT_OUTPUT (23 bytes, padded to 24): LEDs u8, lease u8, text 21 bytes NUL padded.
 */
class PendantWire {
  public static inline final STATE_ID:Int = 0x181;
  public static inline final CONFIG_ID:Int = 0x201;
  public static inline final OUTPUT_ID:Int = 0x202;
  public static inline final STATE_LENGTH:Int = 8;
  public static inline final CONFIG_LENGTH:Int = 16;
  public static inline final OUTPUT_LENGTH:Int = 24;

  public static function encodeState(s:PendantState):Bytes {
    var b = Bytes.alloc(STATE_LENGTH);
    b.set(0, s.sequence & 0xFF);
    b.set(1, (s.sequence >> 8) & 0xFF);
    if (s.wheelDelta < -32768 || s.wheelDelta > 32767) throw "Wheel delta does not fit 16 bits";
    var d = s.wheelDelta & 0xFFFF;
    b.set(2, d & 0xFF);
    b.set(3, d >> 8);
    b.set(4, s.position | (s.step << 4));
    b.set(5, s.buttons);
    b.set(6, (s.wheelEnable ? 1 : 0) | (s.chainClosed ? 2 : 0));
    return b;
  }

  public static function decodeState(b:Bytes):PendantState {
    exactly(b, STATE_LENGTH, "PENDANT_STATE");
    var delta = b.get(2) | (b.get(3) << 8);
    if (delta >= 32768) delta -= 65536;
    var packed = b.get(4), flags = b.get(6);
    if ((packed & 0xC0) != 0 || (flags & 0xFC) != 0 || b.get(7) != 0) throw "Reserved PENDANT_STATE bits are set";
    return new PendantState(b.get(0) | (b.get(1) << 8), delta, packed & 0x0F, (packed >> 4) & 3, b.get(5),
      (flags & 1) != 0, (flags & 2) != 0);
  }

  public static function encodeConfig(e:PendantConfigEntry):Bytes {
    var b = Bytes.alloc(CONFIG_LENGTH);
    b.set(0, e.kind);
    b.set(1, e.index);
    putText(b, 2, e.text, PendantConfigEntry.TEXT_BYTES);
    return b;
  }

  public static function decodeConfig(b:Bytes):PendantConfigEntry {
    exactly(b, CONFIG_LENGTH, "PENDANT_CONFIG");
    var text = getText(b, 2, PendantConfigEntry.TEXT_BYTES);
    for (i in 14...CONFIG_LENGTH) if (b.get(i) != 0) throw "PENDANT_CONFIG padding is not zero";
    return new PendantConfigEntry(b.get(0), b.get(1), text);
  }

  public static function encodeOutput(o:PendantLedFrame):Bytes {
    var b = Bytes.alloc(OUTPUT_LENGTH);
    b.set(0, o.leds);
    b.set(1, switch o.lease { case LeaseState.NoLease: 0; case LeaseState.Granted: 1; case LeaseState.Lost: 2; });
    putText(b, 2, o.text, PendantLedFrame.TEXT_BYTES);
    return b;
  }

  public static function decodeOutput(b:Bytes):PendantLedFrame {
    exactly(b, OUTPUT_LENGTH, "PENDANT_OUTPUT");
    var lease = switch b.get(1) { case 0: LeaseState.NoLease; case 1: LeaseState.Granted; case 2: LeaseState.Lost; case _: throw "Unknown lease state"; };
    var text = getText(b, 2, PendantLedFrame.TEXT_BYTES);
    if (b.get(23) != 0) throw "PENDANT_OUTPUT padding is not zero";
    return new PendantLedFrame(b.get(0), lease, text);
  }

  static function exactly(b:Bytes, length:Int, name:String):Void {
    if (b == null || b.length != length) throw '$name must be $length bytes';
  }

  static function putText(b:Bytes, offset:Int, text:String, width:Int):Void {
    if (text.length > width) throw "Text does not fit the frame";
    for (i in 0...text.length) b.set(offset + i, text.charCodeAt(i));
  }

  /** Text is NUL terminated; every byte after the first NUL must also be NUL. */
  static function getText(b:Bytes, offset:Int, width:Int):String {
    var end = 0;
    while (end < width && b.get(offset + end) != 0) end++;
    for (i in end...width) if (b.get(offset + i) != 0) throw "Text bytes after the terminator";
    return b.getString(offset, end);
  }
}
