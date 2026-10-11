package processkit.screwdriving.openprotocol;

import haxe.io.Bytes;

/** Revision-1, single-station ASCII messages; length excludes the trailing NUL.
    Atlas Copco Open Protocol 2.16.0, section 2.2.2:
    https://asutp.org/Atlas_Copco/OpenProtocol_Specification_R_2.16.0.pdf */
class OpenProtocolFrame {
  public final mid:Int;
  public final bytes:Bytes;
  public function new(mid:Int, data:String = "") {
    if (mid < 1 || mid > 9999 || data == null) throw "Open Protocol needs a valid MID and data";
    var body = Bytes.ofString(data);
    if (body.length + 20 > 9999) throw "Open Protocol message is too long";
    var content = Bytes.ofString(digits(body.length + 20,4) + digits(mid,4) + "0010        " + data);
    bytes = Bytes.alloc(content.length + 1);
    bytes.blit(0,content,0,content.length); bytes.set(content.length,0);
    this.mid = mid;
    ascii(bytes,0,bytes.length - 1);
  }
  public static function digits(value:Int, width:Int):String {
    if (value < 0 || width < 1 || width > 9) throw "Invalid Open Protocol numeric field";
    var text = Std.string(value);
    if (text.length > width) throw "Open Protocol numeric field exceeds its width";
    while (text.length < width) text = "0" + text;
    return text;
  }
  public static function number(bytes:Bytes, offset:Int, width:Int):Float {
    if (offset < 0 || width < 1 || offset + width > bytes.length) throw "Truncated Open Protocol numeric field";
    var value = 0.0;
    for (i in offset...offset + width) {
      var digit = bytes.get(i) - 48;
      if (digit < 0 || digit > 9) throw 'Open Protocol needs ASCII digits at byte ${i + 1}';
      value = value * 10 + digit;
    }
    return value;
  }
  static function ascii(bytes:Bytes, offset:Int, end:Int):Void {
    for (i in offset...end) if (bytes.get(i) < 32 || bytes.get(i) > 126)
      throw 'Open Protocol needs printable ASCII at byte ${i + 1}';
  }
  public static function decode(bytes:Bytes):OpenProtocolFrame {
    if (bytes == null || bytes.length < 21 || bytes.length > 10000 || bytes.get(bytes.length - 1) != 0)
      throw "Malformed Open Protocol message boundary";
    ascii(bytes,0,bytes.length - 1);
    if (number(bytes,0,4) != bytes.length - 1) throw "Open Protocol length differs from its bytes";
    var revision = bytes.getString(8,3);
    if (revision != "001" && revision != "000" && revision != "   ") throw "Open Protocol profile requires revision 1";
    if (bytes.get(11) != 48 && bytes.get(11) != 32) throw "Open Protocol profile requires acknowledged messages";
    // This profile does not negotiate sequence numbering, linking or multiple stations.
    for (i in 12...20) if (bytes.get(i) != 32 && bytes.get(i) != 48)
      throw "Open Protocol profile does not support routing, sequencing or linked messages";
    var mid = Std.int(number(bytes,4,4));
    return new OpenProtocolFrame(mid,bytes.getString(20,bytes.length - 21));
  }
}
