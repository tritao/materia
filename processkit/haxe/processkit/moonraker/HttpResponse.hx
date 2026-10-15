package processkit.moonraker;

import haxe.io.Bytes;

/** A response: status, headers and body. */
class HttpResponse {
  public final status:Int;
  /** Empty when the HTTP backend does not expose a reason phrase (for example HTTP/2). */
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
}
