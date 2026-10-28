import machinekit.operator.*;

class PendantAuthorityTests {
  static var seq = 0;
  static var t = 0.0;
  static function nx(dt = 0.01):Float { t = t + dt; return t; }
  static function frame(position = 3, delta = 0, enable = true, buttons = 0, step = 1, closed = true):PendantState {
    seq = (seq + 1) % PendantState.SEQUENCE_MODULUS;
    return new PendantState(seq, delta, position, step, buttons, enable, closed);
  }
  static function authority(?p:OperatorProfile):PendantAuthority
    return new PendantAuthority(p != null ? p : ReferenceOperatorProfiles.drillTap(), [0.001, 0.01, 0.1], 0.05);
  static function got(o:Null<PendantOutput>):PendantOutput {
    if (o == null) throw "Frame was dropped";
    return o;
  }
  static function near(a:Float, b:Float, m:String):Void { if (Math.abs(a - b) > 1e-9) throw '$m: $a != $b'; }

  public static function run():Void {
    seq = 0;
    var a = authority(), none:Array<OperatorCondition> = [];
    // The first enabled frame never jogs, the next one does: no jump from counts before enabling.
    var out = got(a.accept(frame(3, 7), t, Jog, none));
    if (out.move != null || !out.authority) throw "First frame must not move";
    nx();
    out = got(a.accept(frame(3, 5), t, Jog, none));
    var move = out.move;
    if (move == null || move.axis != "z") throw "Position 3 is Z";
    near(move.distance, 0.05, "5 counts x10");
    // Wheel not enabled: counts discarded.
    nx();
    out = got(a.accept(frame(3, 9, false), t, Jog, none));
    if (out.move != null || out.refusal != "") throw "Disabled wheel must be silent";
    // Duplicate and stale frames are dropped.
    var last = new PendantState(seq, 4, 3, 1, 0, true, true);
    if (a.accept(last, nx(), Jog, none) != null) throw "Duplicate accepted";
    if (a.accept(new PendantState(seq - 1, 4, 3, 1, 0, true, true), nx(), Jog, none) != null) throw "Stale accepted";
    // Sequence wraps.
    for (jump in [30000, 60000, 65534]) { seq = jump - 1; a.accept(frame(), nx(), Jog, none); }
    if (a.accept(frame(), nx(), Jog, none) == null) throw "Wrap rejected";

    // Mode, unmapped position, E-stop.
    out = got(a.accept(frame(), nx(), Auto, none));
    if (out.move != null || out.refusal != "Not in jog mode") throw "Mode gating";
    out = got(a.accept(frame(9, 1), nx(), Jog, none));
    if (out.refusal != "No axis here") throw "Unmapped position";
    out = got(a.accept(frame(3, 1, true, 0, 1, false), nx(), Jog, none));
    if (out.move != null || out.refusal != "E-stop open") throw "E-stop read-back";

    // Hobber jog rule.
    var h = authority(ReferenceOperatorProfiles.hobber()); t = 0.0; seq = 0;
    h.accept(frame(5), t, Jog, [GearboxEngaged]);
    out = got(h.accept(frame(5, 3), nx(), Jog, [GearboxEngaged]));
    if (out.move != null || out.refusal != "Gearbox engaged") throw "Gearbox rule";
    out = got(h.accept(frame(3, 3), nx(), Jog, [GearboxEngaged]));
    var z = out.move;
    if (z == null || z.axis != "z") throw "Z stays jogable";

    // Hold-to-run spindle jog on F2 (drill-tap key 2) and its release.
    a = authority(); t = 0.0; seq = 0;
    a.accept(frame(5, 0, false), t, Jog, none);
    out = got(a.accept(frame(5, 0, false, PendantState.F2), nx(), Jog, none));
    if (!out.spindleJog || (out.pressed & PendantState.F2) == 0) throw "Spindle jog while held";
    out = got(a.accept(frame(5, 0, false, 0), nx(), Jog, none));
    if (out.spindleJog) throw "Spindle jog must stop on release";

    // Link loss ends authority; recovery needs released keys.
    if (!a.holdsAuthority(t + 0.04)) throw "Authority within timeout";
    if (a.holdsAuthority(t + 0.06)) throw "Authority must lapse";
    a.tick(t + 0.06);
    out = got(a.accept(frame(5, 0, true, PendantState.F2), nx(0.07), Jog, none));
    if (out.spindleJog || out.refusal != "Release keys") throw "Held key must not resume";
    out = got(a.accept(frame(5, 0, false), nx(0.01), Jog, none));
    if (out.refusal != "" || !out.authority) throw "Release recovers";

    // HMI arbitration: OFF position hands jogging back to the HMI.
    a.accept(frame(0), nx(0.01), Jog, none);
    if (a.holdsAuthority(nx(0.01))) throw "OFF position gives the HMI authority";

    // A running program blocks the pendant whatever the profile says.
    a = authority(); t = 0.0; seq = 0;
    a.accept(frame(3), t, Jog, none);
    out = got(a.accept(frame(3, 4), nx(), Jog, [ProgramRunning]));
    if (out.move != null || out.refusal != "Program running" || out.spindleJog) throw "Busy";
    if (got(a.accept(frame(3, 4), nx(), Jog, none)).move == null) throw "Busy clears";

    // Host link fails safe when stale, withdrawn, duplicated or never received.
    var host = new HostLink(0.25);
    if (host.jogAllowed(0.0)) throw "No state yet";
    if (!host.accept(1, true, 0.0) || !host.jogAllowed(0.2) || host.jogAllowed(0.3)) throw "Stale host state";
    if (host.accept(1, true, 0.31)) throw "Duplicate host state";
    if (!host.accept(2, true, 0.31) || !host.jogAllowed(0.32)) throw "Fresh host state";
    if (!host.accept(3, false, 0.33) || host.jogAllowed(0.34)) throw "Withdrawn permission";
    for (jump in [30000, 60000, 65535, 0]) if (!host.accept(jump, true, 0.4)) throw 'Host sequence $jump';

    // Construction checks.
    var failed = false;
    try new PendantState(0, 0, 12, 0, 0, false, true) catch (_:Dynamic) failed = true;
    if (!failed) throw "Position 12 accepted";
    trace("pendant authority tests passed");
  }
  public static function main():Void run();
}
