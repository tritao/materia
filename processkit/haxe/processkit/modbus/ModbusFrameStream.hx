package processkit.modbus;
import haxe.io.Bytes;

/** Bounded TCP reassembly; a receive can split or coalesce MBAP records. */
class ModbusFrameStream {
  var buffered:Array<Int> = [];
  public function new() {}
  public function feed(bytes:Bytes):Void {
    if (buffered.length + bytes.length > 2048) throw "Modbus receive backlog exceeded";
    for (i in 0...bytes.length) buffered.push(bytes.get(i));
  }
  public function next():Null<ModbusFrame> {
    if (buffered.length < 7) return null;
    var length = (buffered[4] << 8) | buffered[5];
    if (buffered[2] != 0 || buffered[3] != 0 || length < 2 || length > 254) throw "Malformed Modbus MBAP header";
    var size = 6 + length;
    if (buffered.length < size) return null;
    var bytes = Bytes.alloc(size);
    for (i in 0...size) bytes.set(i, buffered[i]);
    buffered.splice(0, size);
    return ModbusFrame.decode(bytes);
  }
}
