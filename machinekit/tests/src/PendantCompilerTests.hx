import haxe.io.Bytes;
import machinekit.operator.*;
import machinekit.operator.wire.PendantConfig;
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
  static function vectorPath():String {
    var dir = Sys.getCwd();
    for (_ in 0...8) {
      var path = dir + "/machinekit/pendant_wire/vectors/profiles.txt";
      if (FileSystem.exists(dir + "/machinekit/pendant_wire")) return path;
      dir = dir + "/..";
    }
    throw "Cannot find machinekit/pendant_wire";
  }
  static function line(name:String, p:OperatorProfile):String {
    var c = PendantCompiler.compile(p, [1, 10, 100], 50);
    return [name, digestHex(c.digest), hex(c.profile), join(c.reasons), join(c.labels)].join(" ");
  }

  public static function run():Void {
    // The generated wire class pins the same bytes as the Rust codec test.
    var s = new PendantState();
    s.sequence = 0x1234; s.wheel_delta = -2; s.position = 11; s.step = 2; s.buttons = 0x2A; s.flags = 1;
    var b = s.encode(), expected = [0x34, 0x12, 0xFE, 0xFF, 0x0B, 0x02, 0x2A, 0x01];
    for (i in 0...8) if (b.get(i) != expected[i]) throw 'State byte $i is ${b.get(i)}';
    var back = PendantState.decode(b);
    if (back.sequence != 0x1234 || back.wheel_delta != -2 || back.position != 11) throw "State round trip";
    s.position = 12;
    rejects("position constraint on encode", () -> s.encode());
    var bad = Bytes.alloc(8); bad.blit(0, b, 0, 8); bad.set(7, 4);
    rejects("flags mask on decode", () -> PendantState.decode(bad));
    var config = new PendantConfig(); config.kind = 2;
    rejects("unknown config kind", () -> config.encode());
    var negative = new PendantProfile(); negative.axes = [for (_ in 0...11) 255]; negative.timeout_ms = 50;
    negative.step_units = [-1, 10, 100];
    var decoded = PendantProfile.decode(negative.encode());
    if (decoded.step_units[0] != -1 || decoded.step_units[2] != 100) throw "Signed 32-bit round trip";

    // Compilation: axes by index, conditions by bit, reasons by id.
    var hobber = PendantCompiler.compile(ReferenceOperatorProfiles.hobber(), [1, 10, 100], 50);
    var record = PendantProfile.decode(hobber.profile);
    if (record.axes[0] != 2 || record.axes[4] != 1 || record.axes[5] != 0 || record.axes[6] != 255) throw "Axis indices";
    if (record.rule_count != 2 || record.rule_axis[0] != 1 || record.rule_axis[1] != 0 || record.rule_condition[0] != 0 ||
        record.rule_reason[0] != 0 || record.rule_reason[1] != 0 || record.rule_axis[2] != 0) throw "Rules";
    if (hobber.reasons.length != 1 || hobber.labels.length != 14) throw "Reasons and labels";
    if (record.spindle_key != 1 || record.has_spindle != 1 || record.timeout_ms != 50) throw "Spindle key and timeout";
    var none = PendantCompiler.compile(ReferenceOperatorProfiles.arm(), [1, 10, 100], 50);
    if (PendantProfile.decode(none.profile).spindle_key != 255) throw "No spindle key";
    var same = PendantCompiler.compile(ReferenceOperatorProfiles.hobber(), [1, 10, 100], 50);
    var other = PendantCompiler.compile(ReferenceOperatorProfiles.hobber(), [1, 10, 100], 60);
    if (same.digest != hobber.digest || other.digest == hobber.digest) throw "Digest follows content";
    rejects("zero step", () -> PendantCompiler.compile(ReferenceOperatorProfiles.hobber(), [0, 10, 100], 50));
    rejects("timeout", () -> PendantCompiler.compile(ReferenceOperatorProfiles.hobber(), [1, 10, 100], 0));

    // Shared vectors: the Rust tests decode these same bytes. Set PENDANT_VECTORS=write to refresh.
    var lines = [line("drill-tap", ReferenceOperatorProfiles.drillTap()), line("hobber", ReferenceOperatorProfiles.hobber()),
      line("arm", ReferenceOperatorProfiles.arm())].join("\n") + "\n";
    var path = vectorPath();
    if (Sys.getEnv("PENDANT_VECTORS") == "write") File.saveContent(path, lines);
    else if (!FileSystem.exists(path) || File.getContent(path) != lines)
      throw "Compiled profiles differ from pendant_wire/vectors/profiles.txt; run with PENDANT_VECTORS=write after reviewing:\n" + lines;
    trace("pendant compiler tests passed");
  }
  public static function main():Void run();
}
