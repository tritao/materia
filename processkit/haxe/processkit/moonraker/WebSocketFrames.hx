package processkit.moonraker;

import haxe.crypto.Base64;
import haxe.io.Bytes;
import haxe.io.BytesBuffer;

/** One WebSocket message: text (opcode 1), binary (2), close (8), ping (9) or pong (10); fragments are already joined. */
class WebSocketMessage {
  public static inline var TEXT:Int = 1;
  public static inline var BINARY:Int = 2;
  public static inline var CLOSE:Int = 8;
  public static inline var PING:Int = 9;
  public static inline var PONG:Int = 10;

  public final opcode:Int;
  public final payload:Bytes;

  public function new(opcode:Int, payload:Bytes) { this.opcode = opcode; this.payload = payload; }

  public function text():String return payload.toString();
}

/**
 * WebSocket (RFC 6455) as far as a Moonraker conversation needs it: the opening handshake's key and answer, frames
 * written (masked, as a client must, or not, as a server does) and a reader that joins fragments and unmasks.
 */
class WebSocketFrames {
  static inline var GUID:String = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  /** Largest message accepted, bytes. */
  public static inline var MAX_MESSAGE:Int = 16 * 1024 * 1024;

  /** A fresh `Sec-WebSocket-Key`: sixteen random bytes in base64. */
  public static function newKey():String {
    var bytes = Bytes.alloc(16);
    for (index in 0...16) bytes.set(index, Std.random(256));
    return Base64.encode(bytes);
  }

  /** The `Sec-WebSocket-Accept` a server answers `key` with. */
  public static function acceptFor(key:String):String return Base64.encode(Sha1.encode(Bytes.ofString(key + GUID)));

  /** One whole frame (FIN set). A masked frame gets `mask` (four bytes) applied to its payload. */
  public static function encode(opcode:Int, payload:Bytes, ?mask:Bytes):Bytes {
    var out = new BytesBuffer();
    out.addByte(0x80 | opcode);
    var masked = mask != null ? 0x80 : 0;
    if (payload.length < 126) out.addByte(masked | payload.length);
    else if (payload.length < 65536) { out.addByte(masked | 126); out.addByte(payload.length >> 8); out.addByte(payload.length & 0xFF); }
    else {
      out.addByte(masked | 127);
      for (shift in [56, 48, 40, 32]) out.addByte(0);
      for (shift in [24, 16, 8, 0]) out.addByte((payload.length >> shift) & 0xFF);
    }
    if (mask != null) {
      if (mask.length != 4) throw "A WebSocket mask is four bytes";
      out.add(mask);
      for (index in 0...payload.length) out.addByte(payload.get(index) ^ mask.get(index & 3));
    } else out.add(payload);
    return out.getBytes();
  }

  /** A client's text frame with a fresh random mask. */
  public static function clientText(text:String):Bytes {
    var mask = Bytes.alloc(4);
    for (index in 0...4) mask.set(index, Std.random(256));
    return encode(WebSocketMessage.TEXT, Bytes.ofString(text), mask);
  }
}

/** Splits a byte stream into WebSocket messages: frames are unmasked, fragments joined, control frames passed as they come. */
class WebSocketReader {
  var buffer:Bytes = Bytes.alloc(0);
  var fragments = new BytesBuffer();
  var fragmentOpcode = 0;
  var fragmentLength = 0;

  public function new() {}

  public function feed(bytes:Bytes):Void {
    if (bytes.length == 0) return;
    var joined = Bytes.alloc(buffer.length + bytes.length);
    joined.blit(0, buffer, 0, buffer.length);
    joined.blit(buffer.length, bytes, 0, bytes.length);
    buffer = joined;
  }

  /** The next whole message, or null while there is none. */
  public function next():Null<WebSocketMessage> {
    while (true) {
      if (buffer.length < 2) return null;
      var first = buffer.get(0), second = buffer.get(1);
      var fin = (first & 0x80) != 0, opcode = first & 0x0F;
      if ((first & 0x70) != 0) throw "WebSocket frame uses a reserved bit";
      var masked = (second & 0x80) != 0;
      var length = second & 0x7F, at = 2;
      if (length == 126) {
        if (buffer.length < 4) return null;
        length = (buffer.get(2) << 8) | buffer.get(3);
        at = 4;
      } else if (length == 127) {
        if (buffer.length < 10) return null;
        for (index in 2...6) if (buffer.get(index) != 0) throw "WebSocket frame is too large";
        if ((buffer.get(6) & 0x80) != 0) throw "WebSocket frame is too large";
        length = (buffer.get(6) << 24) | (buffer.get(7) << 16) | (buffer.get(8) << 8) | buffer.get(9);
        at = 10;
      }
      if (length < 0 || length > WebSocketFrames.MAX_MESSAGE) throw "WebSocket frame is too large";
      var maskAt = at;
      if (masked) at += 4;
      if (buffer.length < at + length) return null;
      var payload = Bytes.alloc(length);
      for (index in 0...length) payload.set(index, masked ? buffer.get(at + index) ^ buffer.get(maskAt + (index & 3)) : buffer.get(at + index));
      var rest = Bytes.alloc(buffer.length - at - length);
      rest.blit(0, buffer, at + length, rest.length);
      buffer = rest;
      if (opcode >= 8) {
        if (!fin || length > 125) throw "WebSocket control frame is fragmented or too long";
        return new WebSocketMessage(opcode, payload);
      }
      if (opcode == 0) {
        if (fragmentOpcode == 0) throw "WebSocket continuation without a message";
      } else {
        if (fragmentOpcode != 0) throw "WebSocket message started inside another";
        fragmentOpcode = opcode;
      }
      fragmentLength += length;
      if (fragmentLength > WebSocketFrames.MAX_MESSAGE) throw "WebSocket message is too large";
      fragments.add(payload);
      if (fin) {
        var message = new WebSocketMessage(fragmentOpcode, fragments.getBytes());
        fragments = new BytesBuffer();
        fragmentOpcode = 0;
        fragmentLength = 0;
        return message;
      }
    }
  }
}
