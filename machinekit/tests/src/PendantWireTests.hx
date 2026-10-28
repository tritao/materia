import haxe.io.Bytes;
import machinekit.operator.*;

class PendantWireTests {
  static function rejects(message:String, body:() -> Void):Void {
    var failed = false;
    try body() catch (_:Dynamic) failed = true;
    if (!failed) throw 'Expected rejection: $message';
  }
  public static function run():Void {
    // PENDANT_STATE: exact bytes, then round trips including sign and sequence limits.
    var s = new PendantState(0x1234, -2, 11, 2, 0x2A, true, false);
    var b = PendantWire.encodeState(s);
    var expected = [0x34, 0x12, 0xFE, 0xFF, 0x2B, 0x2A, 0x01, 0x00];
    for (i in 0...8) if (b.get(i) != expected[i]) throw 'State layout byte $i is ${b.get(i)}';
    for (d in [-32768, -1, 0, 1, 32767]) for (seq in [0, 1, 65535]) {
      var back = PendantWire.decodeState(PendantWire.encodeState(new PendantState(seq, d, 7, 1, 63, false, true)));
      if (back.sequence != seq || back.wheelDelta != d || back.position != 7 || back.step != 1 || back.buttons != 63 ||
          back.wheelEnable || !back.chainClosed) throw 'State round trip $seq $d';
    }
    rejects("delta too large", () -> PendantWire.encodeState(new PendantState(0, 40000, 0, 0, 0, false, true)));
    rejects("short state", () -> PendantWire.decodeState(Bytes.alloc(7)));
    var bad = PendantWire.encodeState(s); bad.set(6, 0x05);
    rejects("reserved flag bit", () -> PendantWire.decodeState(bad));
    var bad2 = PendantWire.encodeState(s); bad2.set(4, bad2.get(4) | 0x40);
    rejects("reserved packed bit", () -> PendantWire.decodeState(bad2));
    var bad3 = PendantWire.encodeState(s); bad3.set(7, 1);
    rejects("nonzero padding", () -> PendantWire.decodeState(bad3));
    var bad4 = PendantWire.encodeState(s); bad4.set(4, 12);
    rejects("position 12", () -> PendantWire.decodeState(bad4));

    // PENDANT_CONFIG for every profile label.
    var hobber = ReferenceOperatorProfiles.hobber();
    for (i in 1...12) {
      var entry = new PendantConfigEntry(PendantConfigEntry.AXIS, i, hobber.pendantLabel(i));
      var wire = PendantWire.encodeConfig(entry);
      if (wire.length != 16) throw "Config length";
      var back = PendantWire.decodeConfig(wire);
      if (back.kind != 0 || back.index != i || back.text != entry.text) throw 'Config round trip $i';
    }
    var key = PendantWire.decodeConfig(PendantWire.encodeConfig(new PendantConfigEntry(PendantConfigEntry.KEY, 2, "Light")));
    if (key.kind != 1 || key.index != 2 || key.text != "Light") throw "Key config";
    rejects("config kind", () -> new PendantConfigEntry(2, 0, "x"));
    rejects("axis position 0", () -> new PendantConfigEntry(PendantConfigEntry.AXIS, 0, "X"));
    var junk = PendantWire.encodeConfig(new PendantConfigEntry(PendantConfigEntry.AXIS, 1, "X")); junk.set(5, 65);
    rejects("text after terminator", () -> PendantWire.decodeConfig(junk));
    var pad = PendantWire.encodeConfig(new PendantConfigEntry(PendantConfigEntry.AXIS, 1, "X")); pad.set(15, 1);
    rejects("config padding", () -> PendantWire.decodeConfig(pad));

    // PENDANT_OUTPUT, including a full-width status line.
    for (lease in [LeaseState.NoLease, LeaseState.Granted, LeaseState.Lost]) {
      var o = PendantWire.decodeOutput(PendantWire.encodeOutput(new PendantLedFrame(0x15, lease, "Gearbox engaged")));
      if (o.leds != 0x15 || o.lease != lease || o.text != "Gearbox engaged") throw "Output round trip";
    }
    var full = "123456789012345678901";
    if (PendantWire.decodeOutput(PendantWire.encodeOutput(new PendantLedFrame(0, LeaseState.Granted, full))).text != full) throw "Full text";
    rejects("text too long", () -> new PendantLedFrame(0, LeaseState.Granted, full + "2"));
    var lease = PendantWire.encodeOutput(new PendantLedFrame(0, LeaseState.Granted, "")); lease.set(1, 9);
    rejects("unknown lease", () -> PendantWire.decodeOutput(lease));
    rejects("wrong output length", () -> PendantWire.decodeOutput(Bytes.alloc(23)));

    // The wire carries what the authority consumes: encode, decode, then accept.
    var authority = new PendantAuthority(ReferenceOperatorProfiles.drillTap(), [0.001, 0.01, 0.1], 0.05);
    var none:Array<String> = [];
    authority.accept(PendantWire.decodeState(PendantWire.encodeState(new PendantState(1, 0, 3, 1, 0, true, true))), 0.0, Jog, none);
    var out = authority.accept(PendantWire.decodeState(PendantWire.encodeState(new PendantState(2, 4, 3, 1, 0, true, true))), 0.01, Jog, none);
    var move = out == null ? null : out.move;
    if (move == null || move.axis != "z" || Math.abs(move.distance - 0.04) > 1e-9) throw "Wire to authority";
    trace("pendant wire tests passed");
  }
  public static function main():Void run();
}
