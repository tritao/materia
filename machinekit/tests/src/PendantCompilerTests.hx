import haxe.io.Bytes;
import machinekit.operator.*;
import machinekit.operator.wire.PendantConfig;
import machinekit.operator.wire.OperatorState;
import machinekit.operator.wire.PendantEvent;
import machinekit.operator.wire.PendantProfile;
import machinekit.operator.wire.PendantState;
import sys.FileSystem;
import sys.io.File;

class PendantCompilerTests {
  static function rejects(message:String, body:() -> Void):Void {
    var failed = false;
    try body() catch (_:Dynamic) failed = true;
    if (!failed) throw 'Expected rejection: $message';
  }
  static function hex(b:Bytes):String {
    var digits = "0123456789abcdef", out = "";
    for (i in 0...b.length) out += digits.charAt(b.get(i) >> 4) + digits.charAt(b.get(i) & 15);
    return out;
  }
  static function join(list:Array<Bytes>):String return list.length == 0 ? "-" : [for (b in list) hex(b)].join(";");
  static function digestHex(d:Int):String {
    var digits = "0123456789abcdef", out = "";
    for (i in 0...8) out += digits.charAt((d >> (28 - i * 4)) & 15);
    return out;
  }
  /** The shared vector file is read by the Rust tests too; walk up from the working directory to find it. */
  static function vectorPath(name:String = "profiles"):String {
    var dir = Sys.getCwd();
    for (_ in 0...8) {
      var path = dir + "/machinekit/operator_link/vectors/" + name + ".txt";
      if (FileSystem.exists(dir + "/machinekit/operator_link")) return path;
      dir = dir + "/..";
    }
    throw "Cannot find machinekit/operator_link";
  }
  static function line(name:String, p:OperatorProfile):String {
    var c = PendantCompiler.compile(p, [1, 10, 100], 50, 250);
    return [name, digestHex(c.digest), hex(c.profile), join(c.reasons), join(c.labels)].join(" ");
  }

  static function check(name:String, content:String):Void {
    var path = vectorPath(name);
    if (Sys.getEnv("PENDANT_VECTORS") == "write") File.saveContent(path, content);
    else if (!FileSystem.exists(path) || File.getContent(path)
      != content) throw 'Compiled output differs from operator_link/vectors/$name.txt; run with PENDANT_VECTORS=write after reviewing:\n'
      + content;
  }

  public static function run():Void {
    // The generated wire class pins the same bytes as the Rust codec test.
    var s = new PendantState();
    s.sequence = 0x1234;
    s.wheel_delta = -2;
    s.position = 11;
    s.step = 2;
    s.buttons = 0x2A;
    s.flags = 1;
    var b = s.encode(), expected = [0x34, 0x12, 0xFE, 0xFF, 0x0B, 0x02, 0x2A, 0x01];
    for (i in 0...8) if (b.get(i) != expected[i]) throw 'State byte $i is ${b.get(i)}';
    var back = PendantState.decode(b);
    if (back.sequence != 0x1234 || back.wheel_delta != -2 || back.position != 11) throw "State round trip";
    s.position = 12;
    rejects("position constraint on encode", () -> s.encode());
    var bad = Bytes.alloc(8);
    bad.blit(0, b, 0, 8);
    bad.set(7, 4);
    rejects("flags mask on decode", () -> PendantState.decode(bad));
    var config = new PendantConfig();
    config.kind = 2;
    rejects("unknown config kind", () -> config.encode());
    var negative = new PendantProfile();
    negative.axes = [for (_ in 0...11) 255];
    negative.timeout_ms = 50;
    negative.host_timeout_ms = 250;
    negative.step_units = [-1, 10, 100];
    var decoded = PendantProfile.decode(negative.encode());
    if (decoded.step_units[0] != -1 || decoded.step_units[2] != 100) throw "Signed 32-bit round trip";

    // Compilation: axes by index, conditions by bit, reasons by id.
    var hobber = PendantCompiler.compile(ReferenceOperatorProfiles.hobber(), [1, 10, 100], 50, 250);
    var record = PendantProfile.decode(hobber.profile);
    if (record.axes[0] != 2
      || record.axes[4] != 1 || record.axes[5] != 0 || record.axes[6] != 255) throw "Axis indices";
    if (record.rule_count != 2 || record.rule_axis[0] != 1 || record.rule_axis[1] != 0 || record.rule_condition[0] != 0
      || record.rule_reason[0] != 0 || record.rule_reason[1] != 0 || record.rule_axis[2] != 0) throw "Rules";
    if (hobber.reasons.length != 1 || hobber.labels.length != 14) throw "Reasons and labels";
    if (record.spindle_key != 1 || record.has_spindle != 1 || record.timeout_ms != 50) throw "Spindle key and timeout";
    var plain = new OperatorProfile(
      "plain",
      [new OperatorAxis("x", "X")],
      [
        "x",
        null,
        null,
        null,
        null,
        null,
        null,
        null,
        null,
        null,
        null
      ],
      [
        new OperatorKey(
          Unused,
          ""
        ),
        new OperatorKey(
          Unused,
          ""
        ),
        new OperatorKey(
          Unused,
          ""
        )
      ],
      [
        Auto,
        Jog
      ],
      false,
      false,
      []
    );
    var none = PendantCompiler.compile(plain, [1, 10, 100], 50, 250);
    if (PendantProfile.decode(none.profile).spindle_key != 255) throw "No spindle key";
    var same = PendantCompiler.compile(ReferenceOperatorProfiles.hobber(), [1, 10, 100], 50, 250);
    var other = PendantCompiler.compile(ReferenceOperatorProfiles.hobber(), [1, 10, 100], 60, 250);
    if (same.digest != hobber.digest || other.digest == hobber.digest) throw "Digest follows content";
    rejects("zero step", () -> PendantCompiler.compile(ReferenceOperatorProfiles.hobber(), [0, 10, 100], 50, 250));
    rejects("host timeout", () -> PendantCompiler.compile(ReferenceOperatorProfiles.hobber(), [1, 10, 100], 50, 0));
    if (record.host_timeout_ms != 250) throw "Host timeout";
    rejects("timeout", () -> PendantCompiler.compile(ReferenceOperatorProfiles.hobber(), [1, 10, 100], 0, 250));

    // Operator frames: layout, CRC and damage detection.
    var state = new OperatorState();
    state.sequence = 7;
    state.jog_allowed = 1;
    var framed = new OperatorFrame(OperatorFrame.STATE, state.encode()).encode();
    if (framed.length != 16 || framed.get(0) != 0x4F || framed.get(4) != 4 || framed.get(6) != 4) throw "Frame layout";
    if (OperatorFrame.crc32(Bytes.ofString("123456789"), 9) != -873187034) throw "CRC check value"; // 0xCBF43926
    var parsed = OperatorFrame.decode(framed);
    if (parsed.type != OperatorFrame.STATE
      || OperatorState.decode(parsed.payload).sequence != 7) throw "Frame round trip";
    var damaged = Bytes.alloc(framed.length);
    damaged.blit(0, framed, 0, framed.length);
    damaged.set(8, 9);
    rejects("frame CRC", () -> OperatorFrame.decode(damaged));
    rejects("wrong payload size", () -> new OperatorFrame(OperatorFrame.STATE, Bytes.alloc(3)));
    rejects("unknown type", () -> new OperatorFrame(9, Bytes.alloc(4)));

    // Shared vectors: the Rust tests decode these same bytes. Set PENDANT_VECTORS=write to refresh.
    var lines = [
      line("drill-tap", ReferenceOperatorProfiles.drillTap()),
      line(
        "hobber",
        ReferenceOperatorProfiles.hobber()
      )
    ].join("\n") + "\n";
    var h = PendantCompiler.compile(ReferenceOperatorProfiles.hobber(), [1, 10, 100], 50, 250);
    var event = new PendantEvent();
    event.sequence = 9;
    event.axis = 2;
    event.flags = 1;
    event.distance = -50;
    event.pressed = 8;
    event.refusal = 6;
    event.reason = 0;
    var frames = [
      "state-allowed " + hex(framed),
      "event-move " + hex(new OperatorFrame(
        OperatorFrame.EVENT,
        event.encode()
      ).encode()),
      "profile-hobber " + hex(new OperatorFrame(
        OperatorFrame.PROFILE,
        h.profile
      ).encode()),
      "reason-hobber-0 " + hex(new OperatorFrame(
        OperatorFrame.REASON_TEXT,
        h.reasons[0]
      ).encode()),
      "config-hobber-axis-5 " + hex(new OperatorFrame(
        OperatorFrame.CONFIG,
        h.labels[4]
      ).encode())
    ].join("\n") + "\n";
    check("profiles", lines);
    check("frames", frames);
    trace("pendant compiler tests passed");
  }
  public static function main():Void run();
}
