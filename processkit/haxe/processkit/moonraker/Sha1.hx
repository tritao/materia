package processkit.moonraker;

import haxe.io.Bytes;

/** SHA-1 (RFC 3174), which the WebSocket handshake needs to prove the server understood the upgrade. Not for security. */
class Sha1 {
  static inline function rotate(value:Int, bits:Int):Int return (value << bits) | (value >>> (32 - bits));

  public static function encode(data:Bytes):Bytes {
    var length = data.length;
    // The message, a 1 bit, zeros up to 56 mod 64 and the length in bits as 64 bits.
    var padded = ((length + 8) >> 6) + 1;
    var words = [for (_ in 0...padded * 16) 0];
    for (index in 0...length) words[index >> 2] |= data.get(index) << (24 - (index & 3) * 8);
    words[length >> 2] |= 0x80 << (24 - (length & 3) * 8);
    words[padded * 16 - 2] = length >>> 29;
    words[padded * 16 - 1] = length << 3;
    var h0 = (0x6745 << 16) | 0x2301, h1 = (0xEFCD << 16) | 0xAB89, h2 = (0x98BA << 16) | 0xDCFE;
    var h3 = (0x1032 << 16) | 0x5476, h4 = (0xC3D2 << 16) | 0xE1F0;
    var w = [for (_ in 0...80) 0];
    for (block in 0...padded) {
      for (t in 0...16) w[t] = words[block * 16 + t];
      for (t in 16...80) w[t] = rotate(w[t - 3] ^ w[t - 8] ^ w[t - 14] ^ w[t - 16], 1);
      var a = h0, b = h1, c = h2, d = h3, e = h4;
      for (t in 0...80) {
        var f:Int, k:Int;
        if (t < 20) { f = (b & c) | (~b & d); k = (0x5A82 << 16) | 0x7999; }
        else if (t < 40) { f = b ^ c ^ d; k = (0x6ED9 << 16) | 0xEBA1; }
        else if (t < 60) { f = (b & c) | (b & d) | (c & d); k = (0x8F1B << 16) | 0xBCDC; }
        else { f = b ^ c ^ d; k = (0xCA62 << 16) | 0xC1D6; }
        var next = rotate(a, 5) + f + e + k + w[t];
        e = d; d = c; c = rotate(b, 30); b = a; a = next;
      }
      h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
    }
    var out = Bytes.alloc(20);
    for (index in 0...5) {
      var value = [h0, h1, h2, h3, h4][index];
      out.set(index * 4, (value >>> 24) & 0xFF);
      out.set(index * 4 + 1, (value >>> 16) & 0xFF);
      out.set(index * 4 + 2, (value >>> 8) & 0xFF);
      out.set(index * 4 + 3, value & 0xFF);
    }
    return out;
  }
}
