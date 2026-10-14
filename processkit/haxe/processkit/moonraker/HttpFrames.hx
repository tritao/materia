package processkit.moonraker;

import haxe.io.Bytes;
import haxe.io.BytesBuffer;

/** A request as it goes on the wire: HTTP/1.1, the body (if any) sized by Content-Length. */
class HttpRequest {
  /** The `Host` header's authority for a host and port: an IPv6 literal (it has a colon) goes in brackets. */
  public static function authority(host:String, port:Int):String
    return (host.indexOf(":") >= 0 ? "[" + host + "]" : host) + ":" + port;

  public final method:String;
  public final target:String;
  public final headers:Map<String, String>;
  public final body:Bytes;

  public function new(method:String, target:String, headers:Map<String, String>, body:Bytes) {
    this.method = method; this.target = target; this.headers = headers; this.body = body;
  }

  /** Header names are case-insensitive: `header("content-type")`. */
  public function header(name:String):Null<String> {
    var wanted = name.toLowerCase();
    for (key in headers.keys()) if (key.toLowerCase() == wanted) return headers.get(key);
    return null;
  }

  public function encode(host:String):Bytes {
    var out = new BytesBuffer();
    out.addString('$method $target HTTP/1.1\r\nHost: $host\r\n');
    for (key in headers.keys()) if (key.toLowerCase() != "content-length" && key.toLowerCase() != "host")
      out.addString('$key: ${headers.get(key)}\r\n');
    if (body.length > 0 || method == "POST" || method == "PUT") out.addString('Content-Length: ${body.length}\r\n');
    out.addString("\r\n");
    out.add(body);
    return out.getBytes();
  }
}

/** A response: status, headers and body. */
class HttpResponse {
  public final status:Int;
  public final reason:String;
  public final headers:Map<String, String>;
  public final body:Bytes;

  public function new(status:Int, reason:String, headers:Map<String, String>, body:Bytes) {
    this.status = status; this.reason = reason; this.headers = headers; this.body = body;
  }

  public function header(name:String):Null<String> {
    var wanted = name.toLowerCase();
    for (key in headers.keys()) if (key.toLowerCase() == wanted) return headers.get(key);
    return null;
  }

  public function text():String return body.toString();

  public static function encode(status:Int, reason:String, contentType:String, body:Bytes):Bytes {
    var out = new BytesBuffer();
    out.addString('HTTP/1.1 $status $reason\r\nContent-Type: $contentType\r\nContent-Length: ${body.length}\r\n\r\n');
    out.add(body);
    return out.getBytes();
  }
}

/**
 * Splits a byte stream into HTTP messages. A message is its start line, its headers up to a blank line and a body of
 * Content-Length bytes (none without the header). Chunked transfer coding is refused: Moonraker sends lengths.
 */
class HttpFrameStream {
  /** Largest header block and body accepted, in bytes. */
  public static inline var MAX_HEADERS:Int = 16384;
  public static inline var MAX_BODY:Int = 64 * 1024 * 1024;

  var buffer:Bytes = Bytes.alloc(0);

  public function new() {}

  public function feed(bytes:Bytes):Void {
    if (bytes.length == 0) return;
    var joined = Bytes.alloc(buffer.length + bytes.length);
    joined.blit(0, buffer, 0, buffer.length);
    joined.blit(buffer.length, bytes, 0, bytes.length);
    buffer = joined;
  }

  /** The bytes buffered and not yet part of a message. */
  public function pending():Int return buffer.length;

  /** Takes what is buffered: after an upgrade to WebSocket, the bytes that followed the handshake belong to the new protocol. */
  public function takeRest():Bytes {
    var rest = buffer;
    buffer = Bytes.alloc(0);
    return rest;
  }

  /** The header block's end (the index after the blank line), or -1 while it is incomplete. */
  function headerEnd():Int {
    var limit = Std.int(Math.min(buffer.length, MAX_HEADERS + 4));
    for (i in 3...limit)
      if (buffer.get(i - 3) == 13 && buffer.get(i - 2) == 10 && buffer.get(i - 1) == 13 && buffer.get(i) == 10) return i + 1;
    if (buffer.length > MAX_HEADERS) throw "HTTP header block is too large";
    return -1;
  }

  function parseHeaders(lines:Array<String>):Map<String, String> {
    var headers = new Map<String, String>();
    for (index in 1...lines.length) {
      var line = lines[index];
      if (line.length == 0) continue;
      var colon = line.indexOf(":");
      if (colon <= 0) throw 'HTTP header line "$line" has no name';
      headers.set(StringTools.trim(line.substr(0, colon)), StringTools.trim(line.substr(colon + 1)));
    }
    return headers;
  }

  /** The body's length: Content-Length, or none. */
  function bodyLength(headers:Map<String, String>):Int {
    var length = 0;
    for (key in headers.keys()) {
      var name = key.toLowerCase();
      if (name == "transfer-encoding" && headers.get(key).toLowerCase().indexOf("chunked") >= 0)
        throw "HTTP chunked transfer coding is not supported";
      if (name == "content-length") {
        var parsed = Std.parseInt(headers.get(key));
        if (parsed == null || parsed < 0 || parsed > MAX_BODY) throw 'HTTP Content-Length "${headers.get(key)}" is invalid';
        length = parsed;
      }
    }
    return length;
  }

  /** The next whole message as `{start, headers, body}`, or null while there is none. */
  function next():Null<{start:String, headers:Map<String, String>, body:Bytes}> {
    var end = headerEnd();
    if (end < 0) return null;
    var lines = buffer.getString(0, end - 4).split("\r\n");
    var headers = parseHeaders(lines);
    var length = bodyLength(headers);
    if (buffer.length < end + length) return null;
    var body = Bytes.alloc(length);
    body.blit(0, buffer, end, length);
    var rest = Bytes.alloc(buffer.length - end - length);
    rest.blit(0, buffer, end + length, rest.length);
    buffer = rest;
    return {start: lines[0], headers: headers, body: body};
  }

  /** The next whole response, or null. */
  public function nextResponse():Null<HttpResponse> {
    var message = next();
    if (message == null) return null;
    var parts = message.start.split(" ");
    if (parts.length < 2 || !StringTools.startsWith(parts[0], "HTTP/")) throw 'HTTP status line "${message.start}" is malformed';
    var status = Std.parseInt(parts[1]);
    if (status == null) throw 'HTTP status "${parts[1]}" is not a number';
    return new HttpResponse(status, parts.slice(2).join(" "), message.headers, message.body);
  }

  /** The next whole request, or null. */
  public function nextRequest():Null<HttpRequest> {
    var message = next();
    if (message == null) return null;
    var parts = message.start.split(" ");
    if (parts.length != 3 || !StringTools.startsWith(parts[2], "HTTP/")) throw 'HTTP request line "${message.start}" is malformed';
    return new HttpRequest(parts[0], parts[1], message.headers, message.body);
  }
}
