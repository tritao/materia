package robotkit.transport;
import nativekit.ffi.NativeKitTypes;

import nativekit.ffi.NativeKit;
import haxe.Int64;
import haxe.io.Bytes;

/** Small Haxe façade over NativeKit's asynchronous TCP byte transport. */
class NativeTransport {
  /** nk_transport_receive uses this transport-specific status when drained. */
  static inline final RECEIVE_WOULD_BLOCK:Int = -205;

  public static function options(port:Int):TransportOptions {
    if (port <= 0 || port > 65535)
      throw "RobotKit TCP port is outside the valid range";
    var value = new TransportOptions();
    value.set_struct_size(TransportOptions.size());
    value.set_kind(TransportKind.Tcp);
    value.set_flags(TransportFlags.NoDelay);
    value.set_host("127.0.0.1");
    value.set_port(port);
    value.set_timeout_ms(5000);
    value.set_backlog(8);
    value.set_receive_buffer_size(Int64.ofInt(4 * 1024 * 1024));
    value.set_send_buffer_size(Int64.ofInt(4 * 1024 * 1024));
    return value;
  }

  public static function listen(port:Int):OwnedListenerHandle {
    var result = NativeKit.nk_transport_listen(options(port));
    if (result.status != Result.Ok)
      throw 'RobotKit TCP listen failed: ${NativeKit.nk_last_error()}';
    return result.out_listener;
  }

  public static function connect(host:String, port:Int):OwnedTransportHandle {
    var value = options(port);
    value.set_host(host);
    var result = NativeKit.nk_transport_connect(value);
    if (result.status != Result.Ok)
      throw 'RobotKit TCP connect failed: ${NativeKit.nk_last_error()}';
    return result.out_transport;
  }

  public static function send(transport:TransportHandle, bytes:Bytes):Void {
    if (bytes == null)
      throw "RobotKit TCP cannot send null bytes";
    NativeKit.nk_transport_send_checked(transport, bytes, bytes.length);
  }

  public static function receive(transport:TransportHandle,
      ?capacity:Int = 64 * 1024):Bytes {
    if (capacity <= 0)
      throw "RobotKit TCP receive capacity must be positive";
    var buffer = Bytes.alloc(capacity);
    var result = NativeKit.nk_transport_receive(transport, buffer, capacity);
    if (result.status == RECEIVE_WOULD_BLOCK)
      return Bytes.alloc(0);
    if (result.status != Result.Ok)
      throw 'RobotKit TCP receive failed: ${NativeKit.nk_last_error()}';
    var count = Int64.toInt(result.out_received);
    return count == 0 ? Bytes.alloc(0) : Bytes.view(buffer, 0, count);
  }

  public static function close(transport:TransportHandle):Void
    NativeKit.nk_transport_close_checked(transport);
}
