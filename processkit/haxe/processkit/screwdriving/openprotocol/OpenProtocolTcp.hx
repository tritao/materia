package processkit.screwdriving.openprotocol;

import haxeon.platform.interop.TransportInterop;

import haxe.io.Bytes;
import nativekit.ffi.NativeKitTypes;

/** NativeKit's asynchronous TCP transport; the process owner services the client. */
class OpenProtocolTcp implements OpenProtocolTransport {
  final transport:OwnedTransportHandle;
  var closed = false;
  public function new(host:String, port:Int) transport = TransportInterop.connect(host,port);
  public function send(bytes:Bytes):Void {
    if (closed) throw "Open Protocol TCP transport is closed";
    var status = TransportInterop.sendStatus(transport.borrow(),bytes);
    if (status != 0) throw 'Open Protocol TCP send failed: $status';
  }
  public function receive():Bytes {
    if (closed) throw "Open Protocol TCP transport is closed";
    return TransportInterop.receive(transport.borrow(),2048);
  }
  public function close():Void {
    if (closed) return;
    closed = true; transport.close();
  }
}
