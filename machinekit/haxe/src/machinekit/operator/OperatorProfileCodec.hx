package machinekit.operator;

import haxe.Json;
import haxe.io.Bytes;

/** The single saved operator profile format. Other schema versions are rejected. */
class OperatorProfileCodec {
  static final FIELDS:Array<String> = [
    "schemaVersion",
    "name",
    "axes",
    "pendantAxes",
    "keys",
    "modes",
    "hasSpindle",
    "hasDoor",
    "jogRules"
  ];

  public static function encode(profile:OperatorProfile):Bytes return Bytes.ofString(Json.stringify(toRecord(profile)));
  public static function decode(bytes:Bytes):OperatorProfile return fromRecord(Json.parse(bytes.toString()));

  public static function toRecord(p:OperatorProfile):Dynamic {
    if (p == null) throw "Operator profile is required";
    return {schemaVersion: OperatorProfile.CURRENT_VERSION, name: p.name, axes: [for (a in p.axes) {
      id:a.id, label:a.label
    }
    ], pendantAxes:p.pendantAxes.copy(), keys:[for (k in p.keys) {
      action:actionName(k.action), label:k.label
    }
    ], modes:[for (m in p.modes) modeName(m)],
      hasSpindle:p.hasSpindle, hasDoor:p.hasDoor, jogRules:[for (r in p.jogRules) {
      axis:r.axis, blockedWhen:conditionName(r.blockedWhen), reason:r.reason
    }
    ]};
  }

  public static function fromRecord(value:Dynamic):OperatorProfile {
    var version:Dynamic = value == null ? null : Reflect.field(value, "schemaVersion");
    if (!Std.isOfType(version,
      Int) || version != OperatorProfile.CURRENT_VERSION) throw "Unsupported OperatorProfile schema version";
    for (key in Reflect.fields(value)) if (!FIELDS.contains(key)) throw 'Invalid OperatorProfile field $key';
    var pendant:Array<Null<String>> = [];
    for (item in list(value, "pendantAxes")) {
      if (item != null && !Std.isOfType(item, String)) throw "Invalid OperatorProfile pendant axis";
      pendant.push(item);
    }
    return new OperatorProfile(
      text(value, "name"),
      [for (a in list(value, "axes")) new OperatorAxis(text(a, "id"), text(a, "label"))],
      pendant,
      [for (k in list(value, "keys")) new OperatorKey(readAction(text2(k, "action")), text2(k, "label"))],
      [for (m in list(value, "modes")) readMode(name(m))],
      flag(value, "hasSpindle"),
      flag(value, "hasDoor"),
      [for (r in list(value, "jogRules")) new JogRule(
        text(r, "axis"),
        readCondition(text(r, "blockedWhen")),
        text(r, "reason")
      )]
    );
  }

  static function actionName(a:KeyAction):String return switch a {
    case Unused:
      "unused";
    case DoorToggle:
      "door-toggle";
    case SpindleJog:
      "spindle-jog";
    case WorkLight:
      "work-light";
    case Mist:
      "mist";
    case Vacuum:
      "vacuum";
  };
  static function readAction(s:String):KeyAction return switch s {
    case "unused":
      Unused;
    case "door-toggle":
      DoorToggle;
    case "spindle-jog":
      SpindleJog;
    case "work-light":
      WorkLight;
    case "mist":
      Mist;
    case "vacuum":
      Vacuum;
    case _:
      throw "Invalid OperatorProfile key action";
  };
  static function conditionName(c:OperatorCondition):String return switch c {
    case GearboxEngaged:
      "gearbox-engaged";
    case DoorOpen:
      "door-open";
    case SpindleRunning:
      "spindle-running";
    case ProgramRunning:
      "program-running";
  };
  static function readCondition(s:String):OperatorCondition return switch s {
    case "gearbox-engaged":
      GearboxEngaged;
    case "door-open":
      DoorOpen;
    case "spindle-running":
      SpindleRunning;
    case "program-running":
      ProgramRunning;
    case _:
      throw "Invalid OperatorProfile condition";
  };
  static function modeName(m:OperatorMode):String return switch m {
    case Auto:
      "auto";
    case Mdi:
      "mdi";
    case Jog:
      "jog";
  };
  static function readMode(s:String):OperatorMode return switch s {
    case "auto":
      Auto;
    case "mdi":
      Mdi;
    case "jog":
      Jog;
    case _:
      throw "Invalid OperatorProfile mode";
  };

  static function name(value:Dynamic):String {
    if (!Std.isOfType(value, String)) throw "Invalid OperatorProfile name";
    return value;
  }
  static function required(value:Dynamic, name:String):Dynamic {
    if (value == null || !Reflect.hasField(value, name)) throw 'Missing OperatorProfile field $name';
    return Reflect.field(value, name);
  }
  static function list(value:Dynamic, name:String):Array<Dynamic> {
    var result:Dynamic = required(value, name);
    if (!Std.isOfType(result, Array)) throw 'Invalid OperatorProfile field $name';
    return result;
  }
  static function text(value:Dynamic, name:String):String {
    var result:Dynamic = required(value, name);
    if (!Std.isOfType(result,
      String) || StringTools.trim(result).length == 0) throw 'Invalid OperatorProfile field $name';
    return result;
  }
  /** Like `text` but allows the empty string, which an unused key's label is. */
  static function text2(value:Dynamic, name:String):String {
    var result:Dynamic = required(value, name);
    if (!Std.isOfType(result, String)) throw 'Invalid OperatorProfile field $name';
    return result;
  }
  static function number(value:Dynamic, name:String):Float {
    var result:Dynamic = required(value, name);
    if (!Std.isOfType(result, Int) && !Std.isOfType(result, Float)) throw 'Invalid OperatorProfile field $name';
    return result;
  }
  static function flag(value:Dynamic, name:String):Bool {
    var result:Dynamic = required(value, name);
    if (!Std.isOfType(result, Bool)) throw 'Invalid OperatorProfile field $name';
    return result;
  }
}
