package processkit.modbus;

import haxe.io.Bytes;
import nativekit.ffi.NativeKitTypes;
import robotkit.transport.NativeTransport;

private class Request {
  public final pdu:Bytes;
  public final complete:Bytes->Void;
  public function new(pdu:Bytes, complete:Bytes->Void) { this.pdu = pdu; this.complete = complete; }
}

/** Nonblocking, single-transaction Modbus TCP client, polled by the process owner. */
class ModbusTcpClient {
  final transport:OwnedTransportHandle;
  public final unit:Int;
  public final timeout:Float;
  public var fault(default, null):Null<String> = null;
  final stream = new ModbusFrameStream();
  final queue:Array<Request> = [];
  public var sentBytes(default, null) = 0;
  var pending:Null<Request> = null;
  var transaction = 0;
  var deadline = 0.0;
  var lastNow = -1.0;
  var closed = false;

  public function new(host:String, port:Int, unit:Int, timeout:Float = 0.5) {
    if (unit < 1 || unit > 247 || !Math.isFinite(timeout) || timeout <= 0.0) throw "Invalid Modbus TCP configuration";
    this.unit = unit; this.timeout = timeout;
    transport = NativeTransport.connect(host, port);
  }
  public function idle():Bool return pending == null && queue.length == 0;
  function enqueue(pdu:Bytes, complete:Bytes->Void):Void {
    if (closed || fault != null) throw "Modbus client is disconnected";
    if (queue.length >= 64) throw "Modbus request queue exceeded";
    queue.push(new Request(pdu, complete));
  }
  public function write(address:Int, value:Int, complete:Void->Void):Void {
    var pdu = ModbusFrame.writeHolding(address, value);
    enqueue(pdu, function(reply:Bytes) {
      if (reply.length != 5) throw "Malformed Modbus write response";
      for (i in 0...5) if (reply.get(i) != pdu.get(i)) throw "Modbus write acknowledgement differs";
      complete();
    });
  }
  public function writeMultiple(address:Int, values:Array<Int>, complete:Void->Void):Void {
    var pdu = ModbusFrame.writeMultiple(address, values);
    enqueue(pdu, function(reply:Bytes) {
      if (reply.length != 5 || ModbusFrame.word(reply, 1) != address || ModbusFrame.word(reply, 3) != values.length)
        throw "Malformed Modbus multiple-write acknowledgement";
      complete();
    });
  }
  public function read(address:Int, count:Int, complete:Array<Int>->Void):Void {
    enqueue(ModbusFrame.readHolding(address, count), function(reply:Bytes) {
      if (reply.length != 2 + count * 2 || reply.get(1) != count * 2) throw "Malformed Modbus read response";
      var values:Array<Int> = [];
      for (i in 0...count) values.push(ModbusFrame.word(reply, 2 + i * 2));
      complete(values);
    });
  }
  /** Discard unsent setpoints before a process stop; the in-flight transaction still has an acknowledgement. */
  public function discardQueued():Void queue.splice(0, queue.length);
  public function close():Void {
    if (closed) return;
    closed = true; queue.splice(0, queue.length); pending = null; transport.close();
  }
  public function poll(now:Float):Void {
    if (closed || fault != null) return;
    try {
      if (!Math.isFinite(now) || now < lastNow) throw "Modbus owner clock moved backwards";
      lastNow = now;
      var bytes = NativeTransport.receive(transport.borrow(), 2048);
      if (bytes.length > 0) stream.feed(bytes);
      while (true) {
        var frame = stream.next(); if (frame == null) break;
        var request = pending;
        if (request == null || frame.transaction != transaction || frame.unit != unit) throw "Unexpected Modbus transaction";
        if (frame.pdu.get(0) == (request.pdu.get(0) | 128)) throw "Modbus server exception";
        if (frame.pdu.get(0) != request.pdu.get(0)) throw "Unexpected Modbus function";
        pending = null; request.complete(frame.pdu);
      }
      if (pending != null && now >= deadline) throw "Modbus response timed out";
      if (pending == null && queue.length > 0) {
        var request = queue.shift();
        if (request != null) {
          transaction = (transaction + 1) & 65535; pending = request; deadline = now + timeout;
          var encoded = new ModbusFrame(transaction, unit, request.pdu).encode();
          var status = NativeTransport.sendStatus(transport.borrow(), encoded);
          if (status != 0) throw 'Modbus transport send failed: $status';
          sentBytes += encoded.length;
        }
      }
    } catch (error:Dynamic) {
      fault = Std.string(error);
      try close() catch (_:Dynamic) {}
    }
  }
}
