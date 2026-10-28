import haxe.io.Bytes;
import machinekit.operator.*;

class OperatorProfileTests {
  static function rejects(message:String, body:() -> Void):Void {
    var failed = false;
    try body() catch (_:Dynamic) failed = true;
    if (!failed) throw 'Expected rejection: $message';
  }
  static function profile(?pendant:Array<Null<String>>, ?keys:Array<OperatorKey>, ?rules:Array<JogRule>,
      guard:Null<OperatorGuard> = null, spindle = true):OperatorProfile
    return new OperatorProfile("test", "machinekit.control.cnc-controller",
      [new OperatorAxis("x", "X"), new OperatorAxis("z", "Z")],
      pendant != null ? pendant : ["x", "z", null, null, null, null, null, null, null, null, null],
      keys != null ? keys : [new OperatorKey(Unused, ""), new OperatorKey(Unused, ""), new OperatorKey(Unused, "")],
      [Auto, Jog], [Dro], spindle, guard, rules != null ? rules : []);

  public static function run():Void {
    for (p in [ReferenceOperatorProfiles.drillTap(), ReferenceOperatorProfiles.hobber(), ReferenceOperatorProfiles.arm()]) {
      var bytes = OperatorProfileCodec.encode(p);
      var back = OperatorProfileCodec.decode(bytes);
      if (Std.string(OperatorProfileCodec.toRecord(back)) != Std.string(OperatorProfileCodec.toRecord(p)))
        throw 'Round trip changed ${p.name}';
      if (haxe.Json.stringify(OperatorProfileCodec.toRecord(back)) != bytes.toString()) throw "Encoding is not stable";
    }
    var hobber = ReferenceOperatorProfiles.hobber();
    if (hobber.pendantLabel(5) != "B hob" || hobber.pendantLabel(7) != "") throw "Pendant labels";
    if (hobber.jogRefusal("b", [GearboxEngaged]) == null || hobber.jogRefusal("x", [GearboxEngaged]) != null ||
        hobber.jogRefusal("b", []) != null) throw "Jog rules";
    var guard = ReferenceOperatorProfiles.drillTap().guard;
    if (guard == null || guard.setupSpeedLimit != 0.25) throw "Guard parameters";

    rejects("unknown pendant axis", () -> profile(["q", null, null, null, null, null, null, null, null, null, null]));
    rejects("axis twice", () -> profile(["x", "x", null, null, null, null, null, null, null, null, null]));
    rejects("short pendant map", () -> profile(["x", "z"]));
    rejects("door key without guard", () -> profile(null, [new OperatorKey(DoorToggle, "Door"), new OperatorKey(Unused, ""), new OperatorKey(Unused, "")]));
    rejects("spindle key without spindle", () -> profile(null, [new OperatorKey(SpindleJog, "Spin"), new OperatorKey(Unused, ""), new OperatorKey(Unused, "")], null, null, false));
    rejects("rule with unknown axis", () -> profile(null, null, [new JogRule("q", DoorOpen, "Blocked")]));
    rejects("long axis label", () -> new OperatorAxis("x", "ABCDEFGHIJKLM"));
    rejects("long key label", () -> new OperatorKey(Mist, "Mist on"));
    rejects("labelled unused key", () -> new OperatorKey(Unused, "No"));
    rejects("non-ASCII label", () -> new OperatorAxis("x", "Ä"));
    rejects("no jog mode", () -> new OperatorProfile("t", "c", [new OperatorAxis("x", "X")],
      ["x", null, null, null, null, null, null, null, null, null, null],
      [new OperatorKey(Unused, ""), new OperatorKey(Unused, ""), new OperatorKey(Unused, "")], [Auto], [Dro], false, null, []));

    var record = OperatorProfileCodec.toRecord(hobber);
    record.schemaVersion = 2;
    rejects("other schema version", () -> OperatorProfileCodec.fromRecord(record));
    var extra = OperatorProfileCodec.toRecord(hobber);
    Reflect.setField(extra, "legacy", true);
    rejects("unknown field", () -> OperatorProfileCodec.fromRecord(extra));
    var broken = OperatorProfileCodec.toRecord(hobber);
    broken.pendantAxes = ["x"];
    rejects("decode validates", () -> OperatorProfileCodec.fromRecord(broken));
    var condition = OperatorProfileCodec.toRecord(hobber);
    var ruleRecords:Array<Dynamic> = Reflect.field(condition, "jogRules");
    Reflect.setField(ruleRecords[0], "blockedWhen", "unknown");
    rejects("unknown condition name", () -> OperatorProfileCodec.fromRecord(condition));
    trace("operator profile tests passed");
  }
  public static function main():Void run();
}
