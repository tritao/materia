import haxe.Json;
import haxe.io.Bytes;
import haxeon.platform.NativeKitEventBytes;
import haxeon.platform.NativeKitEventValue;
import haxeon.platform.NativeKitRuntime;
import nativekit.ffi.NativeKitTypes;
import processkit.moonraker.HttpFrames;
import processkit.moonraker.ConfigInstaller;
import processkit.moonraker.ConfigReadback;
import processkit.moonraker.MoonrakerClient;
import processkit.moonraker.MoonrakerSocket;
import processkit.moonraker.MoonrakerStatus;
import processkit.moonraker.Sha1;
import processkit.moonraker.WebSocketFrames;
import processkit.testing.FakeMoonrakerServer;
import haxeon.platform.NativeTransport;

/**
 * The Moonraker client against a fake server on a real socket: the HTTP and WebSocket framing, the status it reads
 * (where the nozzle is, not where the queued moves end), an upload that starts a print, the print commands, the
 * subscription that pushes the machine's state as it changes, a printer.cfg installed and read back, and a server that
 * goes quiet.
 */
class MoonrakerTests {
  static var assertions = 0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }

  public static function main():Void {
    ResonanceTests.run();
    framing();
    webSocketFraming();
    status();
    readback();
    conversation();
    Sys.println('Moonraker tests passed ($assertions assertions)');
  }

  static function framing():Void {
    // A response split anywhere is one response; two in one read are two.
    var text = 'HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 11\r\n\r\n{"ok":true}' +
      'HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n';
    var bytes = Bytes.ofString(text);
    var firstEnd = text.indexOf("HTTP/1.1 404");
    for (cut in [1, 17, 60, firstEnd - 1, firstEnd, firstEnd + 1, bytes.length - 1]) {
      var stream = new HttpFrameStream();
      stream.feed(bytes.sub(0, cut));
      var first = stream.nextResponse();
      if (cut < firstEnd) check(first == null, 'a response cut at $cut is not yet one');
      stream.feed(bytes.sub(cut, bytes.length - cut));
      if (first == null) first = stream.nextResponse();
      check(first != null && first.status == 200 && first.text() == '{"ok":true}' && first.header("content-type") == "application/json",
        'a response cut at $cut comes whole');
      var second = stream.nextResponse();
      check(second != null && second.status == 404 && second.body.length == 0 && stream.pending() == 0, 'and the next follows ($cut)');
    }
    // Requests: the body follows Content-Length, and a POST with no body says so.
    var request = new HttpRequest("POST", "/printer/print/pause", new Map<String, String>(), Bytes.alloc(0)).encode("host:7125");
    var read = new HttpFrameStream();
    read.feed(request);
    var parsed = read.nextRequest();
    check(parsed != null && parsed.method == "POST" && parsed.target == "/printer/print/pause" && parsed.header("Content-Length") == "0" &&
      parsed.header("host") == "host:7125", "a request reads back as it was written");
    // What it cannot do, it refuses.
    var chunked = new HttpFrameStream();
    chunked.feed(Bytes.ofString("HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n"));
    var refused = false;
    try chunked.nextResponse() catch (_:Dynamic) refused = true;
    check(refused, "chunked transfer coding is refused");
    var huge = new HttpFrameStream();
    huge.feed(Bytes.ofString("HTTP/1.1 200 OK\r\nContent-Length: 999999999999\r\n\r\n"));
    refused = false;
    try huge.nextResponse() catch (_:Dynamic) refused = true;
    check(refused, "a length beyond the limit is refused");
    var garbled = new HttpFrameStream();
    garbled.feed(Bytes.ofString("NOT HTTP\r\n\r\n"));
    refused = false;
    try garbled.nextResponse() catch (_:Dynamic) refused = true;
    check(refused, "a status line that is not HTTP is refused");
  }

  static function hex(bytes:Bytes):String {
    var digits = "0123456789abcdef", out = new StringBuf();
    for (index in 0...bytes.length) { out.add(digits.charAt(bytes.get(index) >> 4)); out.add(digits.charAt(bytes.get(index) & 15)); }
    return out.toString();
  }

  static function fromHex(text:String):Bytes {
    var out = Bytes.alloc(text.length >> 1);
    for (index in 0...out.length) out.set(index, Std.parseInt("0x" + text.substr(index * 2, 2)));
    return out;
  }

  static function same(a:Bytes, b:Bytes):Bool {
    if (a.length != b.length) return false;
    for (index in 0...a.length) if (a.get(index) != b.get(index)) return false;
    return true;
  }

  static function webSocketFraming():Void {
    // SHA-1: the standard's own vectors, and a message of several blocks.
    check(hex(Sha1.encode(Bytes.ofString("abc"))) == "a9993e364706816aba3e25717850c26c9cd0d89d", "SHA-1 of abc");
    check(hex(Sha1.encode(Bytes.ofString(""))) == "da39a3ee5e6b4b0d3255bfef95601890afd80709", "SHA-1 of nothing");
    check(hex(Sha1.encode(Bytes.ofString("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"))) == "84983e441c3bd26ebaae4aa1f95129e5e54670f1",
      "SHA-1 of a message that needs a second block for its length");
    check(hex(Sha1.encode(Bytes.ofString([for (_ in 0...1000) "a"].join("")))) == "291e9a6c66994949b57ba5e650361e98fc36b1ba", "SHA-1 of a thousand a's");
    // The handshake's example from RFC 6455.
    check(WebSocketFrames.acceptFor("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=", "the accept answer for the RFC's key");
    check(WebSocketFrames.newKey() != WebSocketFrames.newKey(), "each handshake has its own key");
    // The RFC's frames: unmasked and masked "Hello", fragmented "Hello", a ping, 16- and 64-bit lengths.
    var reader = new WebSocketReader();
    reader.feed(fromHex("810548656c6c6f"));
    var hello = reader.next();
    check(hello != null && hello.opcode == WebSocketMessage.TEXT && hello.text() == "Hello", "an unmasked text frame");
    reader.feed(fromHex("818537fa213d7f9f4d5158"));
    hello = reader.next();
    check(hello != null && hello.text() == "Hello", "a masked text frame");
    var mask = fromHex("37fa213d");
    check(hex(WebSocketFrames.encode(WebSocketMessage.TEXT, Bytes.ofString("Hello"), mask)) == "818537fa213d7f9f4d5158", "a masked frame is written as the RFC has it");
    check(hex(WebSocketFrames.encode(WebSocketMessage.TEXT, Bytes.ofString("Hello"))) == "810548656c6c6f", "and an unmasked one");
    reader.feed(fromHex("010348656c"));
    check(reader.next() == null, "half a fragmented message is not one");
    reader.feed(fromHex("8002"));
    check(reader.next() == null, "a continuation cut short is not one");
    reader.feed(fromHex("6c6f"));
    hello = reader.next();
    check(hello != null && hello.text() == "Hello", "fragments join");
    reader.feed(fromHex("890548656c6c6f"));
    var ping = reader.next();
    check(ping != null && ping.opcode == WebSocketMessage.PING && ping.text() == "Hello", "a ping");
    for (size in [125, 126, 300, 65535, 65536, 70000]) {
      var payload = Bytes.alloc(size);
      for (index in 0...size) payload.set(index, (index * 7) & 0xFF);
      var wire = WebSocketFrames.encode(WebSocketMessage.BINARY, payload, fromHex("01020304"));
      var each = new WebSocketReader();
      // Fed a byte at a time at the start and then whole: any split is one message.
      each.feed(wire.sub(0, 3));
      check(each.next() == null, 'a frame of $size bytes cut at its header is not yet one');
      each.feed(wire.sub(3, wire.length - 3));
      var message = each.next();
      check(message != null && message.opcode == WebSocketMessage.BINARY && same(message.payload, payload), 'a frame of $size bytes round trips');
    }
    // What it refuses.
    var refused = false;
    var reserved = new WebSocketReader();
    reserved.feed(fromHex("c10548656c6c6f"));
    try reserved.next() catch (_:Dynamic) refused = true;
    check(refused, "a reserved bit is refused");
    refused = false;
    var orphan = new WebSocketReader();
    orphan.feed(fromHex("800148"));
    try orphan.next() catch (_:Dynamic) refused = true;
    check(refused, "a continuation with nothing to continue is refused");
    refused = false;
    var huge = new WebSocketReader();
    huge.feed(fromHex("827f0000000100000000"));
    try huge.next() catch (_:Dynamic) refused = true;
    check(refused, "a frame beyond the limit is refused");
  }

  static function status():Void {
    var reply = Json.parse('{"result":{"status":{"motion_report":{"live_position":[10.5,20,0.3,4.25],"live_velocity":42.5,"live_extruder_velocity":1.5},' +
      '"toolhead":{"position":[14.5,20,0.3,4.75]},"extruder":{"temperature":209.8,"target":210},' +
      '"heater_bed":{"temperature":59.5,"target":60},"print_stats":{"state":"printing","filename":"tab.gcode","message":""},' +
      '"virtual_sdcard":{"progress":0.42}}}}');
    var status = MoonrakerStatus.parse(reply);
    check(status.state == "printing" && status.filename == "tab.gcode" && status.progress == 0.42, "state, file and progress");
    check(status.position.join(",") == "10.5,20,0.3,4.25" && status.velocity == 42.5 && status.extruderVelocity == 1.5,
      "the position is the motion report's live one, with its speeds");
    check(status.queuedPosition.join(",") == "14.5,20,0.3,4.75", "and the toolhead's queued position is kept apart");
    check(status.extruder.temperature == 209.8 && status.extruder.target == 210 && status.bed.temperature == 59.5 && status.bed.target == 60, "heaters");
    check(MoonrakerStatus.QUERY.indexOf("motion_report=live_position,live_velocity,live_extruder_velocity") >= 0, "the query asks for the live position");
    var fields:Array<String> = Reflect.field(MoonrakerStatus.subscription(), "motion_report");
    check(fields.indexOf("live_position") >= 0, "and so does the subscription");
    var sparse = MoonrakerStatus.parse(Json.parse('{"result":{"status":{"print_stats":{"state":"standby"}}}}'));
    check(sparse.state == "standby" && sparse.position.join(",") == "0,0,0,0" && sparse.bed.target == 0.0, "what the machine leaves out reads as zero");
    var kept = MoonrakerStatus.parse(Json.parse('{"result":{"status":{"print_stats":{"state":"paused"}}}}'), status);
    check(kept.state == "paused" && kept.position.join(",") == "10.5,20,0.3,4.25" && kept.bed.target == 60 && kept.progress == 0.42 &&
      kept.filename == "tab.gcode", "what a later reply leaves out keeps the last value, not zero");
    var refused = false;
    try MoonrakerStatus.parse(Json.parse('{"error":{"message":"no"}}')) catch (_:Dynamic) refused = true;
    check(refused, "a reply with no status is refused");
    refused = false;
    try MoonrakerStatus.parse(Json.parse('{"result":{"status":{"motion_report":{"live_position":"here"}}}}')) catch (_:Dynamic) refused = true;
    check(refused, "a position that is not a list is refused");
  }

  static function readback():Void {
    var expected = new Map<String, Map<String, String>>();
    var stepper = new Map<String, String>();
    stepper.set("rotation_distance", "40");
    stepper.set("pid_Kp", "12.5");
    stepper.set("homing_positive_dir", "true");
    stepper.set("endstop_pin", "!PA1");
    expected.set("stepper_x", stepper);
    var settings = Json.parse('{"stepper_x":{"rotation_distance":40.0000001,"pid_kp":12.5,"homing_positive_dir":true,"endstop_pin":"!pa1","extra":1}}');
    check(ConfigReadback.compare(expected, settings, []).length == 0, "a running configuration that agrees (keys lower-cased, defaults extra) has no differences");
    var off = ConfigReadback.compare(expected, Json.parse('{"stepper_x":{"rotation_distance":39.9,"pid_kp":12.5,"homing_positive_dir":false}}'), [{message: "pin is odd"}]);
    check(off.length == 4, 'a value, a flag, a missing key and a warning are four differences, got $off');
    check(off.join("|").indexOf("rotation_distance is 39.9, the file says 40") >= 0 && off.join("|").indexOf("endstop_pin is not in the running") >= 0 &&
      off.join("|").indexOf("Klipper warns: pin is odd") >= 0, "and each says what differs");
    check(ConfigReadback.compare(expected, Json.parse("{}"), [])[0] == "[stepper_x] is not in the running configuration", "a missing section");
  }

  static function conversation():Void {
    var runtime = NativeKitRuntime.start();
    var fake = FakeMoonrakerServer.start(runtime, 36291);
    var client = new MoonrakerClient("127.0.0.1", 36291, runtime.events, 0.5);
    var socket = new MoonrakerSocket("127.0.0.1", 36291, runtime.events, 0.5);
    var installer:Null<ConfigInstaller> = null;
    function live():MoonrakerStatus {
      var current = socket.status;
      if (current == null) throw "the socket has no status yet";
      return current;
    }
    function turn():Void {
      while (runtime.events.poll()) {} fake.poll(); client.poll(Sys.time()); socket.poll(Sys.time());
      var current = installer;
      if (current != null) current.poll(Sys.time());
      runtime.events.wait(0.001);
    }
    function run(until:Void->Bool, what:String):Void {
      var limit = Sys.time() + 3.0;
      while (!until() && Sys.time() < limit && client.fault == null && socket.fault == null) turn();
      check(client.fault == null, '$what: ${client.fault}');
      check(socket.fault == null, '$what: ${socket.fault}');
      check(until(), '$what timed out');
    }
    // Server information.
    var info:Dynamic = null;
    client.serverInfo(function(result) info = result);
    run(() -> info != null, "server info");
    check(Reflect.field(info, "klippy_state") == "ready", "Klippy is ready");

    var parallel = new MoonrakerClient("127.0.0.1",36291,runtime.events,0.5);
    var mainReply = false, otherReply = false;
    client.serverInfo(_ -> mainReply = true);
    parallel.status(_ -> otherReply = true);
    var parallelLimit = Sys.time() + 3;
    while ((!mainReply || !otherReply) && Sys.time() < parallelLimit) {
      turn(); parallel.poll(Sys.time());
    }
    check(mainReply && otherReply && parallel.fault == null,"HTTP completions route by globally unique request id across owners");
    parallel.close();

    // Recorded artifacts are explicitly published in a Moonraker root, not read from Klipper's /tmp.
    fake.configFiles.set("resonances/spectrum.csv",ResonanceTests.spectrum(57));
    fake.configFiles.set("resonances/decay x.csv",ResonanceTests.decay(57,0.085));
    var measured:Null<materia.assembly.BeltCalibration.ResonanceMeasurement> = null;
    processkit.moonraker.ResonanceImport.read(client,"config","resonances/spectrum.csv","resonances/decay x.csv","x",0,0.2,value->measured=value);
    run(()->measured!=null,"recorded resonance import");
    if (measured == null) throw "No imported measurement";
    check(Math.abs(measured.frequencyHz-57)<0.001 && Math.abs(measured.dampingRatio-0.085)<0.0001,
      "downloaded Klipper artifacts yield measured frequency and damping");
    check(fake.scripts.length==0 && fake.restarts==0,"import sends no movement, shaper changes or firmware writes");
    var unsafe=false;
    try client.downloadFile("config","../outside.csv",_->{}) catch (_:Dynamic) unsafe=true;
    check(unsafe,"artifact path cannot escape the named Moonraker root");

    // An upload that starts a print: a file large enough to cross several reads.
    var program = [for (index in 0...6000) 'G1 X${index % 100} Y${index % 90} E0.01'].join("\n");
    var stored = "";
    client.upload("tab.gcode", program, true, function(path) stored = path);
    run(() -> stored != "", "upload");
    check(stored == "tab.gcode" && fake.files.get("tab.gcode") == program && fake.state == "printing", "the file arrives whole and the print starts");

    // The HTTP status gives where the nozzle is, not where the queued moves end.
    function ask():MoonrakerStatus {
      var got:Null<MoonrakerStatus> = null;
      client.status(function(value) got = value);
      run(() -> got != null, "status");
      if (got == null) throw "no status";
      return got;
    }
    var before = ask();
    check(before.state == "printing" && before.filename == "tab.gcode" && before.progress == 0.0, "printing from the start");
    fake.advance(1.0);
    var later = ask();
    check(later.position[0] > 40.0 && later.position[0] < 60.0 && Math.abs(later.position[0] - fake.position[0]) < 1e-9,
      "after a second the nozzle is fifty millimetres along, and that is what the status reports");
    check(later.queuedPosition[0] > later.position[0] + 5.0, 'the queued moves end ahead of the nozzle (${later.queuedPosition[0]} against ${later.position[0]})');
    check(later.velocity == fake.speed && later.extruder.temperature > 22.0 && later.extruder.target == 210.0 && later.bed.target == 60.0, "speed, and the heaters on their way");

    // The subscription: the answer carries the state, and every change after it is pushed as one update.
    socket.subscribe();
    run(() -> socket.status != null, "subscription");
    check(socket.updates == 1 && live().position[0] == fake.position[0], "the subscription answers with the state");
    var mirrored:Array<MoonrakerStatus> = [];
    socket.onUpdate = function(value) mirrored.push(value);
    fake.fragmentedSocketMessages = true;
    for (step in 0...5) {
      var count = socket.updates;
      fake.advance(0.1);
      run(() -> socket.updates > count, "update " + step);
      check(socket.updates == count + 1, "one advance of the machine is one update");
      var seen = live();
      check(Math.abs(seen.position[0] - fake.position[0]) < 1e-9 && Math.abs(seen.position[1] - fake.position[1]) < 1e-9,
        "the update carries where the nozzle is now");
      check(seen.queuedPosition[0] > seen.position[0] || seen.queuedPosition[1] > seen.position[1], "and not the queued end, which is ahead of it");
      check(seen.extruder.temperature == fake.extruder.temperature, "the heater changes with it");
    }
    check(mirrored.length == 5, "the listener was told each time through fragmented messages");
    run(() -> fake.pongs >= 5, "NativeKit answers interleaved control pings");
    var large = [for (_ in 0...70000) "x"].join("");
    var largeCount = socket.updates;
    fake.message = large; fake.advance(0);
    run(() -> socket.updates > largeCount,"message larger than the initial receive buffer");
    check(live().message == large,"a large fragmented message is resized and consumed whole");
    fake.message = ""; fake.advance(0);
    run(() -> live().message == "","clearing the large message");
    // An update leaves out what did not change, and the socket keeps it.
    var calm = socket.updates;
    fake.bed.target = 60.0;
    fake.advance(0.0);
    for (_ in 0...20) turn();
    check(socket.updates == calm && live().bed.target == 60.0 && live().filename == "tab.gcode", "nothing changed, nothing was pushed, and the state is kept");

    // Pause, resume and cancel over HTTP, seen over the socket.
    var done = 0;
    client.pause(function() done++);
    run(() -> done == 1 && live().state == "paused", "pause");
    var heldAt = fake.position[0];
    fake.advance(1.0);
    check(fake.position[0] == heldAt && live().velocity == 0.0, "paused, the toolhead stands");
    client.resume(function() done++);
    run(() -> done == 2 && live().state == "printing", "resume");
    client.cancel(function() done++);
    run(() -> done == 3 && live().state == "cancelled", "cancel");
    check(fake.extruder.target == 0.0 && fake.bed.target == 0.0, "cancelled, with the heaters off");

    // A print of a file the machine has is started by name.
    client.start("tab.gcode", function() done++);
    run(() -> done == 4, "start");
    check(fake.state == "printing", "started by name");
    fake.advance(1000.0);
    run(() -> live().state == "complete", "complete");
    check(ask().state == "complete", "the job completes");

    // A printer.cfg installed: uploaded to the config root, the firmware restarted, the running configuration read back.
    var text = "[stepper_x]\nrotation_distance: 40\nmicrosteps: 16\nhoming_positive_dir: true\nendstop_pin: !PIN_X_ENDSTOP\n\n[extruder]\nPID_Kp: 22.5\nrotation_distance: 22.93\ngear_ratio: 54:18\n";
    var expected = new Map<String, Map<String, String>>();
    var section:Null<Map<String, String>> = null;
    for (line in text.split("\n")) {
      if (line.length == 0) continue;
      if (line.charAt(0) == "[") { section = new Map<String, String>(); expected.set(line.substr(1, line.length - 2), section); continue; }
      var colon = line.indexOf(":");
      var own = section;
      if (own != null) own.set(line.substr(0, colon), StringTools.trim(line.substr(colon + 1)));
    }
    function install(what:String):ConfigInstaller {
      var made = new ConfigInstaller(client, "printer.cfg", text, expected, 0.01, 3.0);
      installer = made;
      made.start(Sys.time());
      run(() -> made.done, what);
      return made;
    }
    var agreed = install("install");
    check(fake.configFiles.get("printer.cfg") == text && fake.restarts == 1, "the file is in the config root and the firmware restarted");
    // The gear ratio comes back parsed, as Klipper reports a list ([[54, 18]]): the same numbers are not a difference.
    check(agreed.mismatches.length == 0, 'Klipper runs what was generated: ${agreed.mismatches}');
    check(ConfigReadback.compare(["extruder" => ["gear_ratio" => "54:18"]], {extruder: {gear_ratio: [[54.0, 17.0]]}}, []).length == 1,
      "a list that differs is a difference");
    // Klipper reports something else than the file says, and a warning.
    fake.deviations.set("stepper_x/rotation_distance", "39.5");
    fake.warnings.push("Option 'microsteps' is deprecated");
    var differs = install("install with a deviation");
    check(fake.restarts == 2 && differs.mismatches.length == 2, 'a value that differs and a warning are reported: ${differs.mismatches}');
    check(differs.mismatches.join("|").indexOf("[stepper_x] rotation_distance is 39.5, the file says 40") >= 0 &&
      differs.mismatches.join("|").indexOf("deprecated") >= 0, "with what differs");
    // The restart did not wait for nothing: Klippy answered startup before it was ready.
    check(fake.klippyState == "ready", "Klippy ended ready");
    // A config Klippy cannot start with.
    fake.klippyState = "ready";
    fake.configFiles.remove("printer.cfg");
    var failed = new ConfigInstaller(client, "other.cfg", "[x]\n", new Map<String, Map<String, String>>(), 0.01, 3.0);
    installer = failed;
    failed.start(Sys.time());
    run(() -> failed.done, "install without printer.cfg");
    check(failed.mismatches.length == 1 && failed.mismatches[0].indexOf("Klippy did not start") >= 0 && failed.mismatches[0].indexOf("printer.cfg") >= 0,
      'Klippy refusing the config is reported: ${failed.mismatches}');
    installer = null;

    // An error status is the client's fault; a silent server times out; a closed one ends the socket.
    var missing = new MoonrakerClient("127.0.0.1", 36291, runtime.events, 0.5);
    fake.klippyState = "ready";
    var answered = false;
    missing.start("nothing.gcode", function() answered = true);
    var limit = Sys.time() + 3.0;
    while (missing.fault == null && Sys.time() < limit) { while (runtime.events.poll()) {} fake.poll(); missing.poll(Sys.time()); runtime.events.wait(0.001); }
    check(missing.fault != null && missing.fault.indexOf("404") >= 0 && !answered, "an error status is the client's fault");
    // An install whose upload cannot go out ends with the client's fault; it does not wait for ever.
    var nowhere = new MoonrakerClient("127.0.0.1", 1, runtime.events, 0.3);
    var unsent = new ConfigInstaller(nowhere, "printer.cfg", "[x]\n", new Map<String, Map<String, String>>(), 0.01, 3.0);
    unsent.start(Sys.time());
    limit = Sys.time() + 3.0;
    while (!unsent.done && Sys.time() < limit) { while (runtime.events.poll()) {} nowhere.poll(Sys.time()); unsent.poll(Sys.time()); runtime.events.wait(0.001); }
    check(unsent.done && unsent.mismatches.length == 1 && unsent.mismatches[0].indexOf("Moonraker connection failed") >= 0,
      'a failed upload ends the install: ${unsent.mismatches}');
    nowhere.close();
    var quiet = false;
    client.status(function(_) quiet = true);
    limit = Sys.time() + 3.0;
    while (client.fault == null && Sys.time() < limit) { while (runtime.events.poll()) {} client.poll(Sys.time()); runtime.events.wait(0.001); }
    check(client.fault != null && client.fault.indexOf("timed out") >= 0 && !quiet, "a silent server times out");
    fake.drop();
    limit = Sys.time() + 3.0;
    while (socket.fault == null && Sys.time() < limit) turn();
    check(socket.fault != null, "a closed connection ends the socket");
    // New owners reconnect after a drop; stale terminal events must not poison them.
    var reconnected = new MoonrakerSocket("127.0.0.1",36291,runtime.events,0.5);
    reconnected.onUpdate = _ -> reconnected.close();
    reconnected.subscribe();
    limit = Sys.time() + 3;
    while (reconnected.status == null && reconnected.fault == null && Sys.time() < limit) {
      while (runtime.events.poll()) {} fake.poll(); reconnected.poll(Sys.time()); runtime.events.wait(0.001);
    }
    check(reconnected.fault == null && reconnected.status != null,"a fresh socket reconnects and subscribes after a drop");
    reconnected.close();
    // Closing an in-flight HTTP owner must suppress callbacks from queued completions.
    var abandoned = new MoonrakerClient("127.0.0.1",36291,runtime.events,0.5);
    var calledAfterClose = false;
    abandoned.status(_ -> calledAfterClose = true);
    abandoned.poll(Sys.time()); abandoned.close();
    for (_ in 0...10) { while (runtime.events.poll()) {} fake.poll(); abandoned.poll(Sys.time()); runtime.events.wait(0.001); }
    check(!calledAfterClose,"closed HTTP owners do not receive late callbacks");
    var late = new MoonrakerSocket("127.0.0.1", 1, runtime.events, 0.3);
    limit = Sys.time() + 3.0;
    while (late.fault == null && Sys.time() < limit) { while (runtime.events.poll()) {} late.poll(Sys.time()); runtime.events.wait(0.001); }
    check(late.fault != null, "a socket to nothing faults");
    late.close();
    missing.close();
    client.close();
    socket.close();
    fake.stop();
    runtime.dispose();
  }
}
