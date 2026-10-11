package processkit.moonraker;

import haxe.Json;
import haxe.io.Bytes;
import nativekit.ffi.NativeKitTypes;
import processkit.moonraker.HttpFrames;
import processkit.moonraker.WebSocketFrames;
import robotkit.transport.NativeTransport;

private class Call {
  public final id:Int;
  public final complete:Dynamic->Void;
  public var deadline = 0.0;
  public function new(id:Int, complete:Dynamic->Void) { this.id = id; this.complete = complete; }
}

/**
 * Moonraker's JSON-RPC WebSocket, on NativeKit's TCP transport and polled by whoever owns the clock. After the opening
 * handshake it subscribes (`printer.objects.subscribe`) to the objects `MoonrakerStatus` reads: Moonraker answers with
 * their state and then pushes `notify_status_update` with the fields that changed, each of which is merged into the state
 * kept here, so a twin is told the moment the machine reports instead of when it next asks. A handshake or a call that is
 * not answered within `timeout` seconds, a malformed frame or a close from the server is the socket's `fault`, and ends it.
 */
class MoonrakerSocket {
  final transport:OwnedTransportHandle;
  public final host:String;
  public final port:Int;
  public final timeout:Float;
  public var fault(default, null):Null<String> = null;
  /** Called with the machine's state each time the subscription answers or pushes an update. */
  public var onUpdate:Null<MoonrakerStatus->Void> = null;
  /** The latest state, once the subscription has answered. */
  public var status(default, null):Null<MoonrakerStatus> = null;
  /** How many updates have arrived (the answer to the subscription counts), and the clock of the latest. */
  public var updates(default, null) = 0;
  public var updatedAt(default, null) = Math.NEGATIVE_INFINITY;
  final http = new HttpFrameStream();
  final reader = new WebSocketReader();
  final key = WebSocketFrames.newKey();
  final waiting:Array<{method:String, params:Dynamic, call:Call}> = [];
  final calls = new Map<Int, Call>();
  final merged:Dynamic = {};
  var nextId = 1;
  var outstanding = 0;
  var open = false;
  var closed = false;
  var deadline = 0.0;
  var started = false;
  var subscribed = false;

  public function new(host:String, port:Int, timeout:Float = 2.0) {
    if (!Math.isFinite(timeout) || timeout <= 0.0) throw "Invalid Moonraker timeout";
    this.host = host; this.port = port; this.timeout = timeout;
    transport = NativeTransport.connect(host, port);
  }

  public function isOpen():Bool return open && fault == null;

  /** Asks for the objects `MoonrakerStatus` reads once the socket is open; updates follow on their own. */
  public function subscribe():Void {
    if (subscribed) return;
    subscribed = true;
    call("printer.objects.subscribe", {objects: MoonrakerStatus.subscription()}, function(result) {
      merge(Reflect.field(result, "status"));
      announce();
    });
  }

  /** Calls a JSON-RPC method (sent when the socket opens, if it has not yet). */
  public function call(method:String, params:Dynamic, complete:Dynamic->Void):Void {
    if (closed || fault != null) throw "Moonraker socket is disconnected";
    if (waiting.length + outstanding >= 64) throw "Moonraker call queue exceeded";
    waiting.push({method: method, params: params, call: new Call(nextId++, complete)});
  }

  public function close():Void {
    if (closed) return;
    closed = true;
    waiting.splice(0, waiting.length);
    transport.close();
  }

  static function isObject(value:Dynamic):Bool return value != null && Reflect.isObject(value) && !Std.isOfType(value, Array) && !Std.isOfType(value, String);

  /** The changes in `update` (objects of fields) laid over the state: a field is replaced whole, an object keeps the fields not mentioned. */
  function merge(update:Dynamic):Void {
    if (!isObject(update)) throw "Moonraker status update is not an object";
    for (name in Reflect.fields(update)) {
      var fields:Dynamic = Reflect.field(update, name);
      if (!isObject(fields)) throw 'Moonraker status for "$name" is not an object';
      var own:Dynamic = Reflect.field(merged, name);
      if (own == null) { own = {}; Reflect.setField(merged, name, own); }
      for (field in Reflect.fields(fields)) Reflect.setField(own, field, Reflect.field(fields, field));
    }
  }

  function announce():Void {
    status = MoonrakerStatus.fromObjects(merged);
    updates++;
    var listener = onUpdate;
    if (listener != null) listener(status);
  }

  function sendFrame(bytes:Bytes):Void {
    var code = NativeTransport.sendStatus(transport.borrow(), bytes);
    if (code != 0) throw 'Moonraker transport send failed: $code';
  }

  function handle(message:WebSocketMessage, now:Float):Void {
    if (message.opcode == WebSocketMessage.PING) {
      var mask = Bytes.alloc(4);
      for (index in 0...4) mask.set(index, Std.random(256));
      sendFrame(WebSocketFrames.encode(WebSocketMessage.PONG, message.payload, mask));
    } else if (message.opcode == WebSocketMessage.CLOSE) {
      throw "Moonraker closed the connection";
    } else if (message.opcode == WebSocketMessage.TEXT) {
      var body:Dynamic = Json.parse(message.text());
      var id:Dynamic = Reflect.field(body, "id");
      if (id != null) {
        var pending = calls.get(id);
        if (pending == null) throw "Unexpected Moonraker reply";
        calls.remove(id);
        outstanding--;
        var problem:Dynamic = Reflect.field(body, "error");
        if (problem != null) throw 'Moonraker error: ${Reflect.field(problem, "message")}';
        pending.complete(Reflect.field(body, "result"));
      } else if (Reflect.field(body, "method") == "notify_status_update") {
        var params:Array<Dynamic> = Reflect.field(body, "params");
        if (params == null || params.length < 1) throw "Moonraker status notification has no parameters";
        merge(params[0]);
        updatedAt = now;
        announce();
      }
    }
  }

  /** Moves the conversation on: the handshake, the calls waiting to go out, and every message that has arrived. */
  public function poll(now:Float):Void {
    if (closed || fault != null) return;
    try {
      if (!started) {
        started = true;
        deadline = now + timeout;
        var headers = new Map<String, String>();
        headers.set("Upgrade", "websocket");
        headers.set("Connection", "Upgrade");
        headers.set("Sec-WebSocket-Key", key);
        headers.set("Sec-WebSocket-Version", "13");
        sendFrame(new HttpRequest("GET", "/websocket", headers, Bytes.alloc(0)).encode(host + ":" + port));
      }
      var bytes = NativeTransport.receive(transport.borrow(), 65536);
      if (!open) {
        if (bytes.length > 0) http.feed(bytes);
        var reply = http.nextResponse();
        if (reply != null) {
          if (reply.status != 101 || reply.header("Sec-WebSocket-Accept") != WebSocketFrames.acceptFor(key))
            throw 'Moonraker refused the WebSocket upgrade: ${reply.status} ${reply.reason}';
          open = true;
          reader.feed(http.takeRest());
        } else if (now >= deadline) throw "Moonraker WebSocket handshake timed out";
      } else if (bytes.length > 0) reader.feed(bytes);
      if (open) {
        while (true) {
          var message = reader.next();
          if (message == null) break;
          handle(message, now);
        }
        while (waiting.length > 0) {
          var next = waiting.shift();
          if (next == null) break;
          next.call.deadline = now + timeout;
          calls.set(next.call.id, next.call);
          outstanding++;
          sendFrame(WebSocketFrames.clientText(Json.stringify({jsonrpc: "2.0", method: next.method, params: next.params, id: next.call.id})));
        }
        for (pending in calls) if (now >= pending.deadline) throw "Moonraker call timed out";
      }
    } catch (error:Dynamic) {
      fault = Std.string(error);
      try close() catch (_:Dynamic) {}
    }
  }
}
