package processkit.moonraker;

/**
 * Puts a `printer.cfg` on a machine and finds out what Klipper made of it: uploads it to Moonraker's config root,
 * restarts the firmware (`printer/firmware_restart`), waits for Klippy to come back, reads the configuration it is running and
 * compares that with the file (see `ConfigReadback`). Polled with the client by whoever owns the clock; `mismatches` is
 * set when `done`.
 */
class ConfigInstaller {
  public var done(default, null) = false;
  /** What differs between the file and the running configuration, or why Klippy did not come back; empty when they agree. */
  public var mismatches(default, null):Array<String> = [];
  /** Seconds between asks after the restart, and how long Klippy has to be ready again. */
  public final interval:Float;
  public final readyTimeout:Float;
  final client:MoonrakerClient;
  final filename:String;
  final text:String;
  final expected:Map<String, Map<String, String>>;
  var phase = 0;
  var asked = Math.NEGATIVE_INFINITY;
  var restartedAt = 0.0;
  var inFlight = false;

  /** `expected` is the file as sections and keys (the caller's own parser reads it). */
  public function new(client:MoonrakerClient, filename:String, text:String, expected:Map<String, Map<String, String>>, interval:Float = 0.25,
      readyTimeout:Float = 30.0) {
    this.client = client; this.filename = filename; this.text = text; this.expected = expected;
    this.interval = interval; this.readyTimeout = readyTimeout;
  }

  function finish(problems:Array<String>):Void {
    mismatches = problems;
    done = true;
  }

  /** Begins: the upload goes out with the client's next turn. */
  public function start(now:Float):Void {
    if (phase != 0) throw "The config install has started";
    phase = 1;
    inFlight = true;
    client.uploadTo("config", filename, text, false, function(_) {
      client.firmwareRestart(function() {
        inFlight = false;
        phase = 2;
        restartedAt = Math.NEGATIVE_INFINITY;
      });
    });
  }

  public function poll(now:Float):Void {
    if (done || phase == 0) return;
    // A failed upload or restart ends the install too, not only a failure while waiting for Klippy.
    if (client.fault != null) { finish(['Moonraker connection failed: ${client.fault}']); return; }
    if (phase < 2) return;
    if (restartedAt == Math.NEGATIVE_INFINITY) restartedAt = now;
    if (inFlight || now - asked < interval) return;
    if (now - restartedAt > readyTimeout) { finish(["Klippy did not come back after the restart"]); return; }
    asked = now;
    inFlight = true;
    if (phase == 2) client.serverInfo(function(info) {
      inFlight = false;
      var state = Std.string(Reflect.field(info, "klippy_state"));
      if (state == "ready") phase = 3;
      else if (state == "error" || state == "shutdown") {
        var why:Dynamic = Reflect.field(info, "state_message");
        finish(['Klippy did not start ($state): ${why == null ? "no message" : Std.string(why)}']);
      }
    });
    else client.runningConfig(function(running) {
      inFlight = false;
      finish(ConfigReadback.compare(expected, running.settings, running.warnings));
    });
  }
}
