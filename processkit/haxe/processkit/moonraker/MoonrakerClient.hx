package processkit.moonraker;

import haxe.Json;
import haxe.io.Bytes;
import haxe.io.BytesBuffer;
import nativekit.ffi.NativeKitTypes;
import processkit.moonraker.HttpFrames;
import robotkit.transport.NativeTransport;

private class Pending {
  public final request:HttpRequest;
  public final complete:HttpResponse->Void;
  public function new(request:HttpRequest, complete:HttpResponse->Void) { this.request = request; this.complete = complete; }
}

/**
 * A nonblocking Moonraker client over NativeKit's TCP transport, polled by whoever owns the clock (the application's
 * frame, a test's loop). One request is in flight at a time on a kept-alive HTTP/1.1 connection; the rest wait their
 * turn. A transport failure, a reply that does not come within `timeout` seconds, an error status or a body that is
 * not what the request asked for is the client's `fault`, and ends it: a machine that answers wrongly is to be
 * reconnected to, not argued with.
 */
class MoonrakerClient {
  final transport:OwnedTransportHandle;
  public final host:String;
  public final port:Int;
  public final timeout:Float;
  public var fault(default, null):Null<String> = null;
  final stream = new HttpFrameStream();
  final queue:Array<Pending> = [];
  var pending:Null<Pending> = null;
  var deadline = 0.0;
  var closed = false;
  public var sentBytes(default, null) = 0;

  public function new(host:String, port:Int, timeout:Float = 2.0) {
    if (!Math.isFinite(timeout) || timeout <= 0.0) throw "Invalid Moonraker timeout";
    this.host = host; this.port = port; this.timeout = timeout;
    transport = NativeTransport.connect(host, port);
  }

  /** Nothing is in flight or waiting. */
  public function idle():Bool return pending == null && queue.length == 0;

  /** Sends a request with no body when the ones before it have been answered. */
  public function send(method:String, target:String, complete:HttpResponse->Void):Void
    sendBody(method, target, null, null, complete);

  /** Sends a request with a body of `contentType` (none when null) when the ones before it have been answered. */
  public function sendBody(method:String, target:String, contentType:Null<String>, body:Null<Bytes>, complete:HttpResponse->Void, accept:String = "application/json"):Void {
    if (closed || fault != null) throw "Moonraker client is disconnected";
    if (queue.length >= 64) throw "Moonraker request queue exceeded";
    var headers = new Map<String, String>();
    headers.set("Accept", accept);
    if (contentType != null) headers.set("Content-Type", contentType);
    queue.push(new Pending(new HttpRequest(method, target, headers, body == null ? Bytes.alloc(0) : body), complete));
  }

  /** The reply's JSON, or a thrown description of why it is not one (an error status, a body that is not JSON). */
  static function json(reply:HttpResponse):Dynamic {
    if (reply.status < 200 || reply.status >= 300) throw 'Moonraker answered ${reply.status} ${reply.reason}: ${reply.text()}';
    return Json.parse(reply.text());
  }

  /** The latest status read, whose values an absent field of the next reply keeps. */
  var lastStatus:Null<MoonrakerStatus> = null;

  /** What the machine is doing now. */
  public function status(complete:MoonrakerStatus->Void):Void
    send("GET", MoonrakerStatus.QUERY, function(reply) {
      var read = MoonrakerStatus.parse(json(reply), lastStatus);
      lastStatus = read;
      complete(read);
    });

  /** The server's own information (Klippy's state among it), as Moonraker's `result`. */
  public function serverInfo(complete:Dynamic->Void):Void
    send("GET", "/server/info", function(reply) complete(Reflect.field(json(reply), "result")));

  /** Puts `content` in the gcodes as `filename`, and starts printing it when `print` is set. Calls back with the stored path. */
  public function upload(filename:String, content:String, print:Bool, complete:String->Void):Void
    uploadTo("gcodes", filename, content, print, complete);

  /** Puts `content` in one of Moonraker's file roots (`gcodes`, `config`) as `filename`. Calls back with the stored path. */
  public function uploadTo(root:String, filename:String, content:String, print:Bool, complete:String->Void):Void {
    var boundary = "----materia" + StringTools.hex(Std.random(0x7fffffff), 8);
    var body = new BytesBuffer();
    function field(name:String, value:String):Void
      body.addString('--$boundary\r\nContent-Disposition: form-data; name="$name"\r\n\r\n$value\r\n');
    field("root", root);
    field("print", print ? "true" : "false");
    body.addString('--$boundary\r\nContent-Disposition: form-data; name="file"; filename="$filename"\r\n' +
      'Content-Type: application/octet-stream\r\n\r\n');
    body.addString(content);
    body.addString('\r\n--$boundary--\r\n');
    sendBody("POST", "/server/files/upload", 'multipart/form-data; boundary=$boundary', body.getBytes(), function(reply) {
      var item:Dynamic = Reflect.field(json(reply), "item");
      complete(item == null ? filename : Std.string(Reflect.field(item, "path")));
    });
  }

  /** A query value with every byte outside the unreserved set as %XX (the name is UTF-8). */
  static function percentEncode(text:String):String {
    var bytes = Bytes.ofString(text), out = new StringBuf();
    for (index in 0...bytes.length) {
      var code = bytes.get(index);
      var plain = (code >= 48 && code <= 57) || (code >= 65 && code <= 90) || (code >= 97 && code <= 122) || code == 45 || code == 46 || code == 95 || code == 126;
      if (plain) out.addChar(code);
      else out.add("%" + StringTools.hex(code, 2));
    }
    return out.toString();
  }

  /** Read an explicitly published artifact. Klipper's /tmp is not a Moonraker file root. */
  public function downloadFile(root:String,path:String,complete:String->Void):Void {
    if (root!="config" && root!="gcodes") throw "Download requires an explicit config or gcodes root";
    if (path == null || path.length == 0) throw "Download needs a relative file path";
    var segments=path.split("/");
    for (segment in segments) if (segment.length == 0 || segment == "." || segment == ".." || segment.indexOf("\\")>=0)
      throw "Download path must stay within its named file root";
    sendBody("GET","/server/files/"+root+"/"+[for (segment in segments) percentEncode(segment)].join("/"),null,null,reply -> {
      if (reply.status<200 || reply.status>=300) throw 'Moonraker artifact download failed: ${reply.status} ${reply.reason}';
      complete(reply.text());
    },"*/*");
  }

  /** Runs G-code or a Klipper command on the machine. */
  public function script(gcode:String, complete:Void->Void):Void
    send("POST", "/printer/gcode/script?script=" + percentEncode(gcode), function(reply) { json(reply); complete(); });

  /**
    Restarts the firmware so it reads its configuration again. Moonraker's own endpoint, not `FIRMWARE_RESTART` as a
    script: Klippy goes away while the script runs, so the script's request ends in an error even when the restart works.
  **/
  public function firmwareRestart(complete:Void->Void):Void
    send("POST", "/printer/firmware_restart", function(reply) { json(reply); complete(); });

  /** The configuration Klipper is running: `settings` (section to key to value, as it parsed them) and its `warnings`. */
  public function runningConfig(complete:{settings:Dynamic, warnings:Array<Dynamic>}->Void):Void
    send("GET", "/printer/objects/query?configfile=settings,warnings", function(reply) {
      var result:Dynamic = Reflect.field(json(reply), "result");
      var status:Dynamic = result == null ? null : Reflect.field(result, "status");
      var file:Dynamic = status == null ? null : Reflect.field(status, "configfile");
      if (file == null) throw "Moonraker reply has no configfile";
      var warnings:Dynamic = Reflect.field(file, "warnings");
      complete({settings: Reflect.field(file, "settings"), warnings: warnings == null ? [] : warnings});
    });

  /** Starts printing a file already in the gcodes. */
  public function start(filename:String, complete:Void->Void):Void
    send("POST", "/printer/print/start?filename=" + percentEncode(filename), function(reply) { json(reply); complete(); });

  public function pause(complete:Void->Void):Void
    send("POST", "/printer/print/pause", function(reply) { json(reply); complete(); });

  public function resume(complete:Void->Void):Void
    send("POST", "/printer/print/resume", function(reply) { json(reply); complete(); });

  public function cancel(complete:Void->Void):Void
    send("POST", "/printer/print/cancel", function(reply) { json(reply); complete(); });

  public function close():Void {
    if (closed) return;
    closed = true;
    queue.splice(0, queue.length);
    pending = null;
    transport.close();
  }

  /** Moves the conversation on: reads what has arrived, answers the request in flight and sends the next. */
  public function poll(now:Float):Void {
    if (closed || fault != null) return;
    try {
      var bytes = NativeTransport.receive(transport.borrow(), 65536);
      if (bytes.length > 0) stream.feed(bytes);
      while (true) {
        var reply = stream.nextResponse();
        if (reply == null) break;
        var request = pending;
        if (request == null) throw "Unexpected Moonraker reply";
        pending = null;
        request.complete(reply);
      }
      if (pending != null && now >= deadline) throw "Moonraker reply timed out";
      if (pending == null && queue.length > 0) {
        var next = queue.shift();
        if (next != null) {
          pending = next;
          deadline = now + timeout;
          var encoded = next.request.encode(HttpRequest.authority(host, port));
          var status = NativeTransport.sendStatus(transport.borrow(), encoded);
          if (status != 0) throw 'Moonraker transport send failed: $status';
          sentBytes += encoded.length;
        }
      }
    } catch (error:Dynamic) {
      fault = Std.string(error);
      try close() catch (_:Dynamic) {}
    }
  }
}
