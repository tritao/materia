package processkit.screwdriving.openprotocol;

import haxe.io.Bytes;

/** Bounded byte reassembly for fragmented and coalesced TCP messages. */
class OpenProtocolStream {
  var buffered:Array<Int> = [];
  public function new() {}
  public function feed(bytes:Bytes):Void {
    if (bytes == null || buffered.length + bytes.length > 20000) throw "Open Protocol receive backlog exceeded";
    for (i in 0...bytes.length) buffered.push(bytes.get(i));
  }
  public function next():Null<OpenProtocolFrame> {
    if (buffered.length < 4) return null;
    var header = Bytes.alloc(4);
    for (i in 0...4) header.set(i,buffered[i]);
    var length = Std.int(OpenProtocolFrame.number(header,0,4));
    if (length < 20) throw "Open Protocol message is shorter than its header";
    if (buffered.length <= length) return null;
    var bytes = Bytes.alloc(length + 1);
    for (i in 0...length + 1) bytes.set(i,buffered[i]);
    var frame = OpenProtocolFrame.decode(bytes);
    buffered.splice(0,length + 1);
    return frame;
  }
}
