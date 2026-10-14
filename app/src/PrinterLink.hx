package app;

import processkit.moonraker.ConfigInstaller;
import processkit.moonraker.MoonrakerClient;
import processkit.moonraker.MoonrakerSocket;
import processkit.moonraker.MoonrakerStatus;

/**
 * The editor's link to a real printer running Klipper behind Moonraker. The machine's state arrives over Moonraker's
 * WebSocket subscription: each update is given to `onStatus`, which the editor uses to mirror the machine in the printer
 * assembly (the digital twin follows the nozzle) and to show its temperatures. A second, HTTP connection sends the print
 * as a file, relays pause, resume and cancel, and installs a Klipper config (upload, firmware restart, read back what
 * Klipper runs). Everything is nonblocking and driven by `poll`; a connection that fails is reported once in `error` and
 * ends until the next `connect`.
 */
class PrinterLink {
  /** The port Moonraker listens on unless the endpoint names another. */
  public static inline var DEFAULT_PORT:Int = 7125;

  public var onStatus:Null<MoonrakerStatus->Void> = null;
  public var endpoint(default, null):String = "";
  /** The latest state the machine reported, and when (the clock `poll` is given). */
  public var status(default, null):Null<MoonrakerStatus> = null;
  public var statusAt(default, null):Float = Math.NEGATIVE_INFINITY;
  public var error(default, null):Null<String> = null;
  /** What the link last did, for the log. */
  public var note(default, null):String = "";
  /** What differed between the config last installed and the one Klipper runs; null until an install has finished. */
  public var configMismatches(default, null):Null<Array<String>> = null;
  var client:Null<MoonrakerClient> = null;
  var socket:Null<MoonrakerSocket> = null;
  var installer:Null<ConfigInstaller> = null;
  var clock = 0.0;

  public function new() {}

  /**
   * The host and port of an endpoint, `host` or `host:port`. An IPv6 literal is written in brackets with an optional port
   * (`[fe80::1]:7125`), or bare when it has no port (`::1`); the host comes back without its brackets.
   */
  public static function parse(endpoint:String):{host:String, port:Int} {
    var text = StringTools.trim(endpoint);
    if (text.length == 0) throw "The printer's address is empty";
    if (text.charAt(0) == "[") {
      var close = text.indexOf("]");
      var rest = close < 0 ? "" : text.substr(close + 1);
      if (close < 2 || (rest != "" && rest.charAt(0) != ":")) throw 'The printer address "$text" is not [ipv6] or [ipv6]:port';
      var number = rest == "" ? DEFAULT_PORT : Std.parseInt(rest.substr(1));
      if (number == null || number <= 0 || number > 65535) throw 'The printer address "$text" has no valid port';
      return {host: text.substr(1, close - 1), port: number};
    }
    if (text.indexOf(":") != text.lastIndexOf(":")) return {host: text, port: DEFAULT_PORT};
    var colon = text.lastIndexOf(":");
    if (colon < 0) return {host: text, port: DEFAULT_PORT};
    var port = Std.parseInt(text.substr(colon + 1));
    if (colon == 0 || port == null || port <= 0 || port > 65535) throw 'The printer address "$text" is not host or host:port';
    return {host: text.substr(0, colon), port: port};
  }

  public function connected():Bool return client != null && client.fault == null && socket != null && socket.fault == null;

  /** Opens the connections to `endpoint` (closing any other) and subscribes to the machine's state. */
  public function connect(endpoint:String):Void {
    disconnect();
    var where = parse(endpoint);
    this.endpoint = endpoint;
    client = new MoonrakerClient(where.host, where.port);
    var opened = new MoonrakerSocket(where.host, where.port);
    opened.onUpdate = function(reply) {
      status = reply;
      statusAt = clock;
      var listener = onStatus;
      if (listener != null) listener(reply);
    };
    opened.subscribe();
    socket = opened;
    error = null;
    note = 'Connecting to ${where.host}:${where.port}';
  }

  public function disconnect():Void {
    if (client != null) client.close();
    if (socket != null) socket.close();
    client = null;
    socket = null;
    installer = null;
    status = null;
    statusAt = Math.NEGATIVE_INFINITY;
  }

  function live():MoonrakerClient {
    var current = client;
    if (current == null || current.fault != null) throw "Not connected to a printer";
    return current;
  }

  /** Uploads `program` as `name` and, with `print`, starts it. */
  public function send(name:String, program:String, print:Bool):Void {
    var current = live();
    note = 'Sending $name (${program.length} bytes)';
    current.upload(name, program, print, function(path) note = print ? 'Printing $path' : 'Sent $path');
  }

  /**
   * Installs a Klipper config: `text` is uploaded as `name` to the config root, the firmware restarts, and what Klipper
   * runs is compared with `expected` (the same text as sections and keys). `configMismatches` holds the differences once
   * the install is done (empty when Klipper runs what was sent).
   */
  public function installConfig(name:String, text:String, expected:Map<String, Map<String, String>>):Void {
    var current = live();
    if (installer != null && !installer.done) throw "A config install is already running";
    configMismatches = null;
    var made = new ConfigInstaller(current, name, text, expected);
    installer = made;
    made.start(clock);
    note = 'Installing $name (${text.length} bytes)';
  }

  public function pause():Void live().pause(function() note = "Print paused");

  public function resume():Void live().resume(function() note = "Print resumed");

  public function cancel():Void live().cancel(function() note = "Print cancelled");

  /** The state in a line, for the log and the panel. */
  public function summary():String {
    var seen = status;
    if (error != null) return "Printer link failed: " + error;
    var mismatches = configMismatches;
    if (mismatches != null) return mismatches.length == 0 ? "Klipper runs the config that was sent" :
      'Klipper differs from the config that was sent in ${mismatches.length} places: ${mismatches[0]}';
    if (seen == null) return note;
    return '${seen.state}${seen.filename == "" ? "" : " " + seen.filename} ${Math.round(seen.progress * 100)} %, hotend ' +
      '${Math.round(seen.extruder.temperature)}/${Math.round(seen.extruder.target)} C, bed ${Math.round(seen.bed.temperature)}/${Math.round(seen.bed.target)} C';
  }

  /** Moves both conversations on; a fault of either ends the link. */
  public function poll(now:Float):Void {
    clock = now;
    var web = client, push = socket;
    if (web == null || push == null) return;
    web.poll(now);
    push.poll(now);
    var current = installer;
    if (current != null) {
      current.poll(now);
      if (current.done) {
        configMismatches = current.mismatches;
        installer = null;
      }
    }
    var fault = web.fault != null ? web.fault : push.fault;
    if (fault != null) {
      error = fault;
      disconnect();
    }
  }
}
