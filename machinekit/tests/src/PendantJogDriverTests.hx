import haxe.io.Bytes;
import machinekit.operator.*;
import machinekit.operator.wire.OperatorState;
import machinekit.operator.wire.PendantEvent;
import machinekit.operator.wire.PendantRefusal;

private class Recorder implements JogSink {
  public var jogs:Array<String> = [];
  public function new() {}
  public function jog(axisId:String, delta:Float):Void jogs.push(axisId + ":" + Math.round(delta * 1e6) / 1e6);
}

class PendantJogDriverTests {
  static function rejects(message:String, body:() -> Void):Void {
    var failed = false;
    try body() catch (_:Dynamic) failed = true;
    if (!failed) throw 'Expected rejection: $message';
  }
  /** An event as the controller would send it: through the generated codec, constraints and all. */
  static function event(sequence:Int, axis:Int, distance:Int, authority = true, refusal = 0, reason = 0, pressed = 0):PendantEvent {
    var e = new PendantEvent();
    e.sequence = sequence; e.axis = axis; e.distance = distance; e.flags = authority ? 1 : 0;
    e.refusal = refusal; e.reason = reason; e.pressed = pressed;
    return PendantEvent.decode(e.encode());
  }
  static function driver(profile:OperatorProfile, texts:Array<String>, sink:JogSink):PendantJogDriver
    return new PendantJogDriver(profile, texts, sink);

  public static function run():Void {
    var drill = ReferenceOperatorProfiles.drillTap();
    var sink = new Recorder();
    var d = driver(drill, [], sink);

    // A move becomes a signed distance in the axis' user unit: 50 units are 0.05.
    if (!d.accept(event(1, 2, 50), 0.0)) throw "First event";
    if (!d.accept(event(2, 0, -5), 0.01)) throw "Second event";
    if (sink.jogs.join(",") != "z:0.05,x:-0.005") throw "Jogs " + sink.jogs;
    if (d.refusal != "") throw "No refusal";

    // Stale and duplicate events change nothing; the sequence wraps.
    if (d.accept(event(2, 2, 50), 0.02) || d.accept(event(1, 2, 50), 0.02)) throw "Stale or duplicate accepted";
    for (jump in [30000, 60000, 65535, 0]) if (!d.accept(event(jump, 255, 0), 0.03)) throw 'Sequence $jump';
    if (sink.jogs.length != 2) throw "Events without a move must not jog";

    // Authority: held while events keep coming, lapsing with them, never claimed by an OFF event.
    var t = 1.0;
    d.accept(event(1, 2, 0, true), t);
    if (!d.pendantHoldsAuthority(t + 0.2) || d.hmiMayJog(t + 0.2)) throw "Pendant holds authority";
    if (d.pendantHoldsAuthority(t + 0.3) || !d.hmiMayJog(t + 0.3)) throw "Authority lapses without events";
    d.accept(event(2, 255, 0, false), t + 0.01);
    if (d.pendantHoldsAuthority(t + 0.02) || !d.hmiMayJog(t + 0.02)) throw "OFF hands jogging back";

    // Refusals become status text; rule refusals use the compiled texts.
    var hobber = ReferenceOperatorProfiles.hobber();
    var compiled = PendantCompiler.compile(hobber, [1, 10, 100], 50, 250);
    if (compiled.reasonTexts.join("|") != "Gearbox engaged") throw "Compiled texts";
    var h = driver(hobber, compiled.reasonTexts, sink = new Recorder());
    h.accept(event(1, 1, 0, true, PendantRefusal.rule, 0), 0.0);
    if (h.refusal != "Gearbox engaged" || sink.jogs.length != 0) throw "Rule refusal";
    h.accept(event(2, 2, 0, true, PendantRefusal.busy), 0.01);
    if (h.refusal != "Program running") throw "Busy text";
    h.accept(event(3, 2, 30, true), 0.02);
    if (h.refusal != "" || sink.jogs.join(",") != "x:0.03") throw "Refusal clears: " + sink.jogs; // hobber axis 2 is x

    // Keys are reported as pressed.
    var keys:Array<Int> = [];
    h.onKeys = pressed -> keys.push(pressed);
    h.accept(event(4, 255, 0, true, 0, 0, 8), 0.03);
    if (keys.join(",") != "8") throw "Keys";

    // Malformed events are protocol errors, not silently dropped.
    rejects("unknown axis", () -> d.accept(event(9, 40, 0), 5.0));
    rejects("move without axis", () -> {
      var e = new PendantEvent(); e.sequence = 10; e.axis = 255; e.distance = 5;
      d.accept(PendantEvent.decode(e.encode()), 5.0);
    });
    rejects("move with a refusal", () -> d.accept(event(11, 2, 5, true, PendantRefusal.busy), 5.0));
    rejects("rule text out of range", () -> d.accept(event(12, 2, 0, true, PendantRefusal.rule, 3), 5.0));
    rejects("no sink", () -> new PendantJogDriver(drill, [], null));

    // OperatorState frames: framed, numbered, intent only.
    var out = driver(drill, [], new Recorder());
    var first = OperatorFrame.decode(out.operatorStateFrame(true));
    var second = OperatorFrame.decode(out.operatorStateFrame(false));
    if (first.type != OperatorFrame.STATE) throw "State frame type";
    var a = OperatorState.decode(first.payload), b = OperatorState.decode(second.payload);
    if (a.sequence != 1 || a.jog_allowed != 1 || b.sequence != 2 || b.jog_allowed != 0) throw "State frames";
    trace("pendant jog driver tests passed");
  }
  public static function main():Void run();
}
