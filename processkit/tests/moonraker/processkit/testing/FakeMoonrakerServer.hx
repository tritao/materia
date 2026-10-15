package processkit.testing;

import haxe.Json;
import haxe.io.Bytes;
import haxeon.platform.NativeKitEventBytes;
import haxeon.platform.NativeKitEventValue;
import haxeon.platform.NativeKitRuntime;
import nativekit.ffi.NativeKitTypes;
import processkit.moonraker.HttpFrames;
import processkit.moonraker.MoonrakerStatus.MoonrakerHeater;
import processkit.moonraker.WebSocketFrames;
import robotkit.transport.NativeTransport;

private class Link {
  public final transport:TransportHandle;
  public final http = new HttpFrameStream();
  public var socket:Null<WebSocketReader> = null;
  /** What this connection subscribed to: object to fields (empty for all of them). */
  public var subscription:Null<Map<String, Array<String>>> = null;
  /** The last value sent for each `object.field`, as JSON, so that an update carries only what changed. */
  public final sent = new Map<String, String>();
  public var dropped = false;
  public function new(transport:TransportHandle) this.transport = transport;
}

/**
 * A Moonraker and a Klipper in one process, on a real TCP socket: it keeps uploaded files, answers the queries and print
 * commands the client sends, serves the JSON-RPC WebSocket with `printer.objects.subscribe` and pushes the fields that
 * changed, and runs a fake job when asked to print: the toolhead follows a polyline at a speed and the heaters come up to
 * their targets, advanced by `advance` so that a test paces it.
 *
 * Like Klipper it keeps two positions: `position`, where the nozzle is (`motion_report.live_position`), and the end of the
 * moves already queued, which runs `lookahead` millimetres ahead along the route while printing (`toolhead.position`).
 *
 * It also models the configuration: a `printer.cfg` uploaded to the config root is parsed when the firmware is restarted
 * (`printer/firmware_restart`; `FIRMWARE_RESTART` as a script fails, as on a machine whose Klippy goes away mid-request), after which `configfile.settings` holds what Klipper would report,
 * with `deviations` replacing values the way Klipper's own rounding or limits might and `warnings` as it would warn.
 */
class FakeMoonrakerServer {
  final listener:OwnedListenerHandle;
  final subscription:Dynamic;
  final links:Array<Link> = [];
  /** Uploaded G-code by name, and the config root's files by name. */
  public final files = new Map<String, String>();
  public final configFiles = new Map<String, String>();
  public var requests(default, null) = 0;
  public var state = "standby";
  public var filename = "";
  public var message = "";
  /** The polyline the toolhead follows while printing: x, y, z, e of each corner, millimetres. */
  public var route:Array<Array<Float>> = [[0.0, 0.0, 0.0, 0.0], [100.0, 0.0, 0.2, 5.0], [100.0, 100.0, 0.2, 10.0], [0.0, 100.0, 0.2, 15.0]];
  public var speed = 50.0;
  /** How far ahead of the nozzle the queued moves end, millimetres of route, while printing. */
  public var lookahead = 8.0;
  public var position = [0.0, 0.0, 0.0, 0.0];
  public var extruder:MoonrakerHeater = {temperature: 22.0, target: 0.0};
  public var bed:MoonrakerHeater = {temperature: 22.0, target: 0.0};
  /** Klippy's state (`startup`, `ready`, `error`) and its message. */
  public var klippyState = "ready";
  public var stateMessage = "";
  /** How many `server/info` answers say `startup` after a restart before Klippy is ready. */
  public var startupAnswers = 2;
  /** Values the running configuration reports instead of the file's: `section/key` to text. */
  public final deviations = new Map<String, String>();
  public final warnings:Array<String> = [];
  public final scripts:Array<String> = [];
  public var restarts(default, null) = 0;
  var settings:Dynamic = {};
  var startupLeft = 0;
  var travelled = 0.0;

  function new(listener:OwnedListenerHandle, subscription:Dynamic) {
    this.listener = listener;
    this.subscription = subscription;
  }

  /** Listens on `port`; connections are accepted as the runtime's events are polled. */
  public static function start(runtime:NativeKitRuntime, port:Int):FakeMoonrakerServer {
    var listener = NativeTransport.listen(port);
    var holder:Array<FakeMoonrakerServer> = [];
    var token = runtime.events.listen(function(event:NativeKitEventValue) {
      switch event {
        case Raw(kind, source, _, _, _, _, data):
          if (kind == EventKind.TransportAccepted && source.rawValue() == listener.borrow().rawValue() && holder.length == 1)
            holder[0].links.push(new Link(new TransportHandle(NativeKitEventBytes.readU32(data, 4))));
        case _: return;
      }
    });
    var server = new FakeMoonrakerServer(listener, token);
    holder.push(server);
    return server;
  }

  /** The number of connections made so far. */
  public function connections():Int return links.length;

  /** Closes every connection and stops listening. */
  public function stop():Void {
    drop();
    subscription.dispose();
    listener.close();
  }

  public function drop():Void {
    for (link in links) if (!link.dropped) { NativeTransport.close(link.transport); link.dropped = true; }
  }

  function length():Float {
    var total = 0.0;
    for (index in 1...route.length) total += distance(route[index - 1], route[index]);
    return total;
  }

  static function distance(a:Array<Float>, b:Array<Float>):Float
    return Math.sqrt(Math.pow(b[0] - a[0], 2) + Math.pow(b[1] - a[1], 2) + Math.pow(b[2] - a[2], 2));

  function progress():Float {
    var total = length();
    return total <= 0.0 ? 0.0 : Math.min(1.0, travelled / total);
  }

  /** The route's point `along` millimetres from its start, and its extruder travel per millimetre there. */
  function routeAt(along:Float):{point:Array<Float>, feed:Float} {
    var left = Math.max(0.0, along);
    for (index in 1...route.length) {
      var span = distance(route[index - 1], route[index]);
      if (left <= span) {
        var fraction = span <= 0.0 ? 0.0 : left / span;
        return {point: [for (axis in 0...4) route[index - 1][axis] + fraction * (route[index][axis] - route[index - 1][axis])],
          feed: span <= 0.0 ? 0.0 : (route[index][3] - route[index - 1][3]) / span};
      }
      left -= span;
    }
    return {point: route[route.length - 1].copy(), feed: 0.0};
  }

  /** Lets `seconds` pass: heaters move 5 degrees a second towards their targets, and a printing toolhead moves along the route; subscribers are told what changed. */
  public function advance(seconds:Float):Void {
    function approach(heater:MoonrakerHeater):Void {
      var goal = heater.target > 0.0 ? heater.target : 22.0;
      var step = 5.0 * seconds;
      heater.temperature = Math.abs(goal - heater.temperature) <= step ? goal : heater.temperature + (goal > heater.temperature ? step : -step);
    }
    approach(extruder);
    approach(bed);
    if (state == "printing") {
      travelled += speed * seconds;
      position = routeAt(travelled).point;
      if (travelled >= length()) state = "complete";
    }
    push();
  }

  function printing():Bool return state == "printing";

  /** Every object Klipper reports, as the fake models them. */
  function objects():Map<String, Map<String, Dynamic>> {
    var here = routeAt(travelled);
    var queued = printing() ? routeAt(travelled + lookahead).point : position.copy();
    var table = new Map<String, Map<String, Dynamic>>();
    function put(name:String, fields:Array<Array<Dynamic>>):Void {
      var map = new Map<String, Dynamic>();
      for (field in fields) map.set(field[0], field[1]);
      table.set(name, map);
    }
    put("motion_report", [["live_position", position.copy()], ["live_velocity", printing() ? speed : 0.0],
      ["live_extruder_velocity", printing() ? here.feed * speed : 0.0]]);
    put("toolhead", [["position", queued]]);
    put("extruder", [["temperature", extruder.temperature], ["target", extruder.target]]);
    put("heater_bed", [["temperature", bed.temperature], ["target", bed.target]]);
    put("print_stats", [["state", state], ["filename", filename], ["message", message]]);
    put("virtual_sdcard", [["progress", progress()]]);
    put("configfile", [["settings", settings], ["warnings", [for (text in warnings) {type: "warning", message: text}]]]);
    return table;
  }

  /** The objects and fields asked for (all fields of an object when its list is empty), as the JSON object Moonraker sends. */
  function select(asked:Map<String, Array<String>>):Dynamic {
    var all = objects(), result:Dynamic = {};
    for (name in asked.keys()) {
      var own = all.get(name);
      if (own == null) continue;
      var fields:Dynamic = {};
      var wanted = asked.get(name);
      for (field in own.keys()) if (wanted.length == 0 || wanted.indexOf(field) >= 0) Reflect.setField(fields, field, own.get(field));
      Reflect.setField(result, name, fields);
    }
    return result;
  }

  function sendHttp(link:Link, status:Int, reason:String, value:Dynamic):Void
    NativeTransport.send(link.transport, HttpResponse.encode(status, reason, "application/json", Bytes.ofString(Json.stringify(value))));

  function sendSocket(link:Link, value:Dynamic):Void
    NativeTransport.send(link.transport, WebSocketFrames.encode(WebSocketMessage.TEXT, Bytes.ofString(Json.stringify(value))));

  /** Tells each subscriber the fields that changed since it was last told. */
  function push():Void {
    for (link in links) {
      var asked = link.subscription;
      if (link.dropped || asked == null) continue;
      var current:Dynamic = select(asked), changes:Dynamic = {}, any = false;
      for (name in Reflect.fields(current)) {
        var fields:Dynamic = Reflect.field(current, name), delta:Dynamic = {};
        for (field in Reflect.fields(fields)) {
          var encoded = Json.stringify(Reflect.field(fields, field));
          if (link.sent.get(name + "." + field) == encoded) continue;
          link.sent.set(name + "." + field, encoded);
          Reflect.setField(delta, field, Reflect.field(fields, field));
          any = true;
        }
        if (Reflect.fields(delta).length > 0) Reflect.setField(changes, name, delta);
      }
      if (any) sendSocket(link, {jsonrpc: "2.0", method: "notify_status_update", params: [changes, 0.0]});
    }
  }

  function begin(name:String):Bool {
    if (!files.exists(name)) return false;
    filename = name; state = "printing"; travelled = 0.0; message = "";
    // The toolhead stands at the start of its route when the print begins, not where it was before.
    if (route.length > 0) position = route[0].copy();
    extruder.target = 210.0; bed.target = 60.0;
    return true;
  }

  /** Answers what has arrived on every connection; call from the test's loop. */
  public function poll():Void {
    for (link in links.copy()) {
      if (link.dropped) continue;
      var bytes = NativeTransport.receive(link.transport, 65536);
      var socket = link.socket;
      if (socket != null) {
        socket.feed(bytes);
        while (true) {
          var message = socket.next();
          if (message == null) break;
          if (message.opcode == WebSocketMessage.TEXT) rpc(link, Json.parse(message.text()));
          else if (message.opcode == WebSocketMessage.PING) NativeTransport.send(link.transport, WebSocketFrames.encode(WebSocketMessage.PONG, message.payload));
          else if (message.opcode == WebSocketMessage.CLOSE) { NativeTransport.close(link.transport); link.dropped = true; }
        }
        continue;
      }
      if (bytes.length > 0) link.http.feed(bytes);
      while (link.socket == null && !link.dropped) {
        var request = link.http.nextRequest();
        if (request == null) break;
        requests++;
        handle(link, request);
      }
      if (link.socket != null) link.socket.feed(link.http.takeRest());
    }
  }

  function rpc(link:Link, call:Dynamic):Void {
    var id:Dynamic = Reflect.field(call, "id"), method:String = Reflect.field(call, "method");
    var params:Dynamic = Reflect.field(call, "params");
    if (method == "printer.objects.subscribe") {
      var asked = new Map<String, Array<String>>();
      var wanted:Dynamic = Reflect.field(params, "objects");
      for (name in Reflect.fields(wanted)) {
        var fields:Dynamic = Reflect.field(wanted, name);
        asked.set(name, fields == null ? [] : cast fields);
      }
      link.subscription = asked;
      link.sent.clear();
      var status = select(asked);
      for (name in Reflect.fields(status)) {
        var fields:Dynamic = Reflect.field(status, name);
        for (field in Reflect.fields(fields)) link.sent.set(name + "." + field, Json.stringify(Reflect.field(fields, field)));
      }
      sendSocket(link, {jsonrpc: "2.0", result: {eventtime: 0.0, status: status}, id: id});
    } else sendSocket(link, {jsonrpc: "2.0", error: {code: -32601, message: "Method not found"}, id: id});
  }

  function handle(link:Link, request:HttpRequest):Void {
    var question = request.target.indexOf("?");
    var path = question < 0 ? request.target : request.target.substr(0, question);
    var query = new Map<String, String>();
    if (question >= 0) for (pair in request.target.substr(question + 1).split("&")) {
      var equals = pair.indexOf("=");
      query.set(equals < 0 ? pair : pair.substr(0, equals), equals < 0 ? "" : percentDecode(pair.substr(equals + 1)));
    }
    switch [request.method, path] {
      case ["GET", "/websocket"]: upgrade(link, request);
      case ["GET", "/server/info"]:
        var answer = klippyState;
        if (startupLeft > 0) { startupLeft--; answer = "startup"; }
        else if (klippyState == "startup") klippyState = answer = "ready";
        sendHttp(link, 200, "OK", {result: {klippy_state: answer, klippy_connected: answer != "error", state_message: stateMessage,
          moonraker_version: "fake"}});
      case ["GET", "/printer/objects/query"]:
        var asked = new Map<String, Array<String>>();
        for (name in query.keys()) asked.set(name, query.get(name).length == 0 ? [] : query.get(name).split(","));
        sendHttp(link, 200, "OK", {result: {eventtime: 0.0, status: select(asked)}});
      case ["POST", "/server/files/upload"]: upload(link, request);
      case ["POST", "/printer/gcode/script"]:
        var script = query.get("script");
        scripts.push(script);
        if (script == "FIRMWARE_RESTART") {
          restart();
          sendHttp(link, 503, "Service Unavailable", {error: {message: "Klippy Disconnected"}});
        } else sendHttp(link, 200, "OK", {result: "ok"});
      case ["POST", "/printer/firmware_restart"]:
        restart();
        sendHttp(link, 200, "OK", {result: "ok"});
      case ["POST", "/printer/print/start"]:
        if (begin(query.get("filename"))) { sendHttp(link, 200, "OK", {result: "ok"}); push(); }
        else sendHttp(link, 404, "Not Found", {error: {message: "file not found"}});
      case ["POST", "/printer/print/pause"]:
        if (state == "printing") state = "paused";
        sendHttp(link, 200, "OK", {result: "ok"});
        push();
      case ["POST", "/printer/print/resume"]:
        if (state == "paused") state = "printing";
        sendHttp(link, 200, "OK", {result: "ok"});
        push();
      case ["POST", "/printer/print/cancel"]:
        if (state == "printing" || state == "paused") { state = "cancelled"; extruder.target = 0.0; bed.target = 0.0; }
        sendHttp(link, 200, "OK", {result: "ok"});
        push();
      case _:
        if (request.method == "GET" && StringTools.startsWith(path,"/server/files/")) {
          var parts=path.substr(14).split("/"), root=parts.shift();
          var name=[for (part in parts) percentDecode(part)].join("/");
          var content=root == "config" ? configFiles.get(name) : root == "gcodes" ? files.get(name) : null;
          if (content == null) sendHttp(link,404,"Not Found",{error:{message:"file not found"}});
          else NativeTransport.send(link.transport,HttpResponse.encode(200,"OK","text/csv",Bytes.ofString(content)));
        } else sendHttp(link, 404, "Not Found", {error: {message: "no such endpoint"}});
    }
  }

  function upgrade(link:Link, request:HttpRequest):Void {
    var key = request.header("Sec-WebSocket-Key");
    if (key == null || request.header("Upgrade") == null) { sendHttp(link, 400, "Bad Request", {error: {message: "not an upgrade"}}); return; }
    NativeTransport.send(link.transport, Bytes.ofString("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n" +
      "Sec-WebSocket-Accept: " + WebSocketFrames.acceptFor(key) + "\r\n\r\n"));
    link.socket = new WebSocketReader();
  }

  /** Klipper's restart: the config root's `printer.cfg` is read, and Klippy is back after `startupAnswers` answers of `startup`. */
  function restart():Void {
    restarts++;
    var text = configFiles.get("printer.cfg");
    if (text == null) { klippyState = "error"; stateMessage = "Unable to open config file printer.cfg"; settings = {}; return; }
    settings = typed(text);
    klippyState = "ready";
    stateMessage = "";
    startupLeft = startupAnswers;
  }

  static function number(text:String):Null<Float> {
    if (text.length == 0) return null;
    for (index in 0...text.length) {
      var code = text.charCodeAt(index);
      if (!((code >= 48 && code <= 57) || code == 43 || code == 45 || code == 46 || code == 101 || code == 69)) return null;
    }
    var value = Std.parseFloat(text);
    return Math.isFinite(value) ? value : null;
  }

  /** `configfile.settings` for a config's text: sections and keys lower-cased, numbers and booleans as such, `deviations` applied. */
  function typed(text:String):Dynamic {
    var result:Dynamic = {};
    var section:Null<Dynamic> = null;
    var sectionName = "";
    for (raw in text.split("\n")) {
      var line = StringTools.trim(raw);
      if (line.length == 0 || line.charAt(0) == "#" || line.charAt(0) == ";") continue;
      if (line.charAt(0) == "[") {
        sectionName = line.substr(1, line.length - 2);
        var existing:Dynamic = Reflect.field(result, sectionName.toLowerCase());
        if (existing == null) { existing = {}; Reflect.setField(result, sectionName.toLowerCase(), existing); }
        section = existing;
        continue;
      }
      var colon = line.indexOf(":"), equals = line.indexOf("=");
      var at = colon < 0 ? equals : (equals < 0 ? colon : Std.int(Math.min(colon, equals)));
      var own = section;
      if (own == null || at <= 0) continue;
      var key = StringTools.trim(line.substr(0, at)).toLowerCase();
      var value = StringTools.trim(line.substr(at + 1));
      var deviation = deviations.get(sectionName + "/" + key);
      if (deviation != null) value = deviation;
      var parsed = number(value);
      var typedValue:Dynamic = value;
      // Klipper parses a ratio list (`gear_ratio: 54:18`) into lists of numbers.
      var ratios = [for (pair in value.split(",")) [for (part in pair.split(":")) number(StringTools.trim(part))]];
      var isList = value.indexOf(":") >= 0 && [for (pair in ratios) for (part in pair) if (part == null) part].length == 0;
      if (isList) typedValue = [for (pair in ratios) [for (part in pair) (part:Float)]];
      else if (parsed != null) typedValue = parsed;
      else if (value.toLowerCase() == "true") typedValue = true;
      else if (value.toLowerCase() == "false") typedValue = false;
      Reflect.setField(own, key, typedValue);
    }
    return result;
  }

  /** A query value with its %XX escapes undone (UTF-8). */
  static function percentDecode(text:String):String {
    var out = new haxe.io.BytesBuffer(), index = 0;
    var bytes = Bytes.ofString(text);
    while (index < bytes.length) {
      var code = bytes.get(index);
      if (code == 37 && index + 2 < bytes.length + 0) {
        out.addByte(Std.parseInt("0x" + text.substr(index + 1, 2)));
        index += 3;
      } else {
        out.addByte(code);
        index++;
      }
    }
    return out.getBytes().toString();
  }

  /** A multipart form: the fields `root` and `print`, and the file part. */
  function upload(link:Link, request:HttpRequest):Void {
    var type = request.header("Content-Type");
    var marker = type == null ? -1 : type.indexOf("boundary=");
    if (marker < 0) { sendHttp(link, 400, "Bad Request", {error: {message: "not multipart"}}); return; }
    var boundary = "--" + type.substr(marker + 9);
    var fields = new Map<String, String>();
    var name = "", content = "";
    for (part in request.body.toString().split(boundary)) {
      var split = part.indexOf("\r\n\r\n");
      if (split < 0) continue;
      var head = part.substr(0, split), value = part.substr(split + 4);
      if (StringTools.endsWith(value, "\r\n")) value = value.substr(0, value.length - 2);
      var nameAt = head.indexOf('name="');
      if (nameAt < 0) continue;
      var field = head.substring(nameAt + 6, head.indexOf('"', nameAt + 6));
      var fileAt = head.indexOf('filename="');
      if (fileAt >= 0) { name = head.substring(fileAt + 10, head.indexOf('"', fileAt + 10)); content = value; }
      else fields.set(field, value);
    }
    var root = fields.get("root");
    if (name.length == 0 || (root != "gcodes" && root != "config")) { sendHttp(link, 400, "Bad Request", {error: {message: "no file"}}); return; }
    var started = false;
    if (root == "config") configFiles.set(name, content);
    else {
      files.set(name, content);
      started = fields.get("print") == "true" && begin(name);
    }
    sendHttp(link, 201, "Created", {item: {path: name, root: root}, print_queued: false, print_started: started});
    push();
  }
}
