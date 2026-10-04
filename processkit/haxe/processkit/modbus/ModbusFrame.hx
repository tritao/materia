package processkit.modbus;

import haxe.io.Bytes;

/** Modbus TCP MBAP framing; register words are big endian. */
class ModbusFrame {
  public final transaction:Int;
  public final unit:Int;
  public final pdu:Bytes;
  public function new(transaction:Int, unit:Int, pdu:Bytes) {
    if (transaction < 0 || transaction > 65535 || unit < 1 || unit > 247 || pdu == null || pdu.length < 1 || pdu.length > 253)
      throw "Invalid Modbus TCP frame";
    this.transaction = transaction; this.unit = unit; this.pdu = pdu;
  }
  public function encode():Bytes {
    var bytes = Bytes.alloc(7 + pdu.length);
    putWord(bytes, 0, transaction); putWord(bytes, 2, 0); putWord(bytes, 4, pdu.length + 1);
    bytes.set(6, unit); bytes.blit(7, pdu, 0, pdu.length); return bytes;
  }
  public static function decode(bytes:Bytes):ModbusFrame {
    if (bytes == null || bytes.length < 8 || bytes.length > 260 || word(bytes, 2) != 0 || word(bytes, 4) != bytes.length - 6)
      throw "Malformed Modbus TCP frame";
    return new ModbusFrame(word(bytes, 0), bytes.get(6), Bytes.view(bytes, 7, bytes.length - 7));
  }
  public static function word(bytes:Bytes, offset:Int):Int {
    if (offset < 0 || offset > bytes.length - 2) throw "Truncated Modbus word";
    return (bytes.get(offset) << 8) | bytes.get(offset + 1);
  }
  public static function putWord(bytes:Bytes, offset:Int, value:Int):Void {
    if (offset < 0 || offset > bytes.length - 2 || value < 0 || value > 65535) throw "Invalid Modbus word";
    bytes.set(offset, value >> 8); bytes.set(offset + 1, value & 255);
  }
  public static function readHolding(address:Int, count:Int):Bytes {
    if (count < 1 || count > 125 || address < 0 || address + count > 65536) throw "Invalid Modbus read range";
    var pdu = Bytes.alloc(5); pdu.set(0, 3); putWord(pdu, 1, address); putWord(pdu, 3, count); return pdu;
  }
  public static function writeHolding(address:Int, value:Int):Bytes {
    var pdu = Bytes.alloc(5); pdu.set(0, 6); putWord(pdu, 1, address); putWord(pdu, 3, value); return pdu;
  }
}
