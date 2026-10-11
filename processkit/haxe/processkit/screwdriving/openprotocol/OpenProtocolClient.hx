package processkit.screwdriving.openprotocol;

private class Command {
  public final frame:OpenProtocolFrame;
  public final complete:Void->Void;
  public function new(frame:OpenProtocolFrame, complete:Void->Void) {
    this.frame = frame; this.complete = complete;
  }
}

/** Polled revision-1 client with one acknowledged command in flight.
    Specification 2.16.0: 5.2 (0001/0002, 0004/0005), 5.3.9 (0018),
    5.6 (0042/0043), 5.8 (0060/0061/0062), 5.28.1 (9999).
    https://asutp.org/Atlas_Copco/OpenProtocol_Specification_R_2.16.0.pdf
    This is a commissioned single-station profile, not automatic revision negotiation.
    For example MTFocus6000 requires communication-start revision 6, outside this profile.
    Enable only unlocks the tool; start/reverse belong to commissioned machine I/O. */
class OpenProtocolClient {
  final transport:OpenProtocolTransport;
  final timeout:Float;
  final stream = new OpenProtocolStream();
  final queue:Array<Command> = [];
  final results:Array<OpenProtocolResult> = [];
  public var initialized(default,null) = false;
  public var selectedProgram(default,null) = 0;
  public var enabled(default,null) = false;
  public var fault(default,null):Null<String> = null;
  var pending:Null<Command> = null;
  var lastResult:Null<OpenProtocolResult> = null;
  var deadline = 0.0;
  var lastNow = -1.0;
  var lastTraffic = 0.0;
  var heartbeatDeadline = 0.0;
  var heartbeatPending = false;
  var disabledAcknowledged = false;
  var closed = false;

  public function new(transport:OpenProtocolTransport, timeout:Float = 0.5) {
    if (transport == null || !Math.isFinite(timeout) || timeout <= 0 || timeout >= 10)
      throw "Open Protocol needs a transport and a response timeout below ten seconds";
    this.transport = transport; this.timeout = timeout;
    enqueue(1,"",function() {
      enqueue(42,"",function() {
        disabledAcknowledged = true;
        enqueue(60,"",function() initialized = true);
      });
    });
  }
  public static function connect(host:String, port:Int, timeout:Float = 0.5):OpenProtocolClient
    return new OpenProtocolClient(new OpenProtocolTcp(host,port),timeout);
  function enqueue(mid:Int, data:String, complete:Void->Void):Void {
    if (closed || fault != null) throw "Open Protocol client is disconnected";
    if (queue.length >= 64) throw "Open Protocol command backlog exceeded";
    queue.push(new Command(new OpenProtocolFrame(mid,data),complete));
  }
  public function idle():Bool return pending == null && queue.length == 0;
  public function ready():Bool return initialized && enabled && selectedProgram > 0 && idle() && fault == null && !closed;
  public function selectProgram(slot:Int):Void {
    if (!initialized || !idle() || enabled) throw "Open Protocol selection requires an initialized, idle, disabled tool";
    if (slot < 1 || slot > 999) throw "Open Protocol parameter set must be in 1..999";
    selectedProgram = 0;
    enqueue(18,OpenProtocolFrame.digits(slot,3),function() selectedProgram = slot);
  }
  public function enable():Void {
    if (!initialized || !idle() || selectedProgram == 0) throw "Open Protocol enable requires an acknowledged parameter set";
    disabledAcknowledged = false;
    enqueue(43,"",function() enabled = true);
  }
  /** Discard unsent work; acknowledge any in-flight command before disabling. */
  public function safe():Void {
    if (closed || fault != null) return;
    if (!initialized) return;
    queue.splice(0,queue.length); disabledAcknowledged = false;
    enqueue(42,"",function() { enabled = false; disabledAcknowledged = true; });
    // During initialization the communication-start callback always disables first.
  }
  public function safeAcknowledged():Bool return initialized && disabledAcknowledged && !enabled && idle() && fault == null && !closed;
  public function nextResult():Null<OpenProtocolResult> return results.length == 0 ? null : results.shift();
  public function close():Void {
    if (closed) return;
    closed = true; initialized = false; enabled = false;
    pending = null; queue.splice(0,queue.length); transport.close();
  }
  function send(frame:OpenProtocolFrame, now:Float):Void {
    transport.send(frame.bytes); lastTraffic = now;
  }
  function accept(frame:OpenProtocolFrame, now:Float):Void {
    var bytes = frame.bytes;
    if (frame.mid == 61) {
      if (!initialized) throw "Open Protocol result arrived before subscription acknowledgement";
      var result = new OpenProtocolResult(frame);
      var previous = lastResult;
      if (previous != null && result.id < previous.id) throw "Open Protocol tightening ID moved backwards";
      if (previous != null && result.id == previous.id) {
        if (result.fingerprint != previous.fingerprint) throw "Open Protocol repeated tightening ID changed its result";
      } else {
        if (results.length >= 64) throw "Open Protocol result backlog exceeded";
        results.push(result); lastResult = result;
      }
      send(new OpenProtocolFrame(62),now); return;
    }
    if (frame.mid == 9999) {
      if (!heartbeatPending || bytes.length != 21) throw "Unexpected Open Protocol keep-alive";
      heartbeatPending = false; return;
    }
    var command = pending;
    if (command == null) throw 'Unexpected Open Protocol MID ${frame.mid}';
    if (frame.mid == 4) {
      if (bytes.length != 27 || OpenProtocolFrame.number(bytes,20,4) != command.frame.mid)
        throw "Open Protocol command error does not match its request";
      var code = OpenProtocolFrame.number(bytes,24,2);
      throw 'Open Protocol rejected MID ${command.frame.mid}: error $code';
    }
    if (command.frame.mid == 1) {
      if (frame.mid != 2 || bytes.length != 58 || OpenProtocolFrame.number(bytes,20,2) != 1
          || OpenProtocolFrame.number(bytes,26,2) != 2 || OpenProtocolFrame.number(bytes,30,2) != 3)
        throw "Open Protocol communication acknowledgement differs from revision 1";
      var cell = OpenProtocolFrame.number(bytes,22,4);
      var channel = OpenProtocolFrame.number(bytes,28,2);
      if (channel > 20) throw "Open Protocol acknowledgement has invalid channel";
    } else if (frame.mid != 5 || bytes.length != 25 || OpenProtocolFrame.number(bytes,20,4) != command.frame.mid)
      throw "Open Protocol command acknowledgement does not match its request";
    pending = null; command.complete();
  }
  public function poll(now:Float):Void {
    if (closed || fault != null) return;
    try {
      if (!Math.isFinite(now) || now < 0 || now < lastNow) throw "Open Protocol owner clock moved backwards";
      lastNow = now;
      var received = transport.receive();
      if (received.length > 0) stream.feed(received);
      while (true) {
        var frame = stream.next(); if (frame == null) break;
        lastTraffic = now; accept(frame,now);
      }
      if (pending != null && now >= deadline) throw "Open Protocol command acknowledgement timed out";
      if (heartbeatPending && now >= heartbeatDeadline) throw "Open Protocol keep-alive timed out";
      if (pending == null && queue.length > 0) {
        var command = queue.shift();
        if (command != null) {
          pending = command; deadline = now + timeout; send(command.frame,now);
        }
      }
      if (initialized && !heartbeatPending && now - lastTraffic >= 10) {
        heartbeatPending = true; heartbeatDeadline = now + timeout;
        send(new OpenProtocolFrame(9999),now);
      }
    } catch (error:Dynamic) {
      fault = Std.string(error);
      try close() catch (_:Dynamic) {}
    }
  }
}
