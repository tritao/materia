package machinekit.operator;

/** Operator profiles for the planned machines. The arm uses its own teach pendant, so it has none here. */
class ReferenceOperatorProfiles {
  static function axes(pairs:Array<Array<String>>):Array<OperatorAxis> return [for (p in pairs) new OperatorAxis(
    p[0],
    p[1]
  )];

  /** The profile called `name` ("drill-tap" or "hobber"), or null. */
  public static function named(name:String):Null<OperatorProfile> return switch name {
    case "drill-tap":
      drillTap();
    case "hobber":
      hobber();
    case _:
      null;
  };

  public static function drillTap():OperatorProfile return new OperatorProfile(
    "drill-tap",
    axes([["x", "X"], ["y", "Y"], ["z", "Z"], ["a", "A"], ["spindle", "Spindle"]]),
    ["x", "y", "z", "a", "spindle", null, null, null, null, null, null],
    [new OperatorKey(DoorToggle, "Door"), new OperatorKey(SpindleJog, "Spin"), new OperatorKey(Mist, "MQL")],
    [Auto, Mdi, Jog],
    true,
    true,
    []
  );

  public static function hobber():OperatorProfile return new OperatorProfile(
    "hobber",
    axes([["c", "C table"], ["b", "B hob"], ["x", "X"], ["y", "Y"], ["z", "Z"], ["a", "A"]]),
    ["x", "y", "z", "a", "b", "c", null, null, null, null, null],
    [new OperatorKey(DoorToggle, "Door"), new OperatorKey(SpindleJog, "Hob"), new OperatorKey(WorkLight, "Light")],
    [Auto, Mdi, Jog],
    true,
    true,
    [new JogRule("b", GearboxEngaged, "Gearbox engaged"), new JogRule("c", GearboxEngaged, "Gearbox engaged")]
  );
}
