package machinekit.operator;

/** Operator profiles for the planned machines. Cabinet ids name recipes in machinekit.control. */
class ReferenceOperatorProfiles {
  static final CNC_PAGES:Array<OperatorPage> = [Dro, Program, Tools, Offsets, Services, Diagnostics];

  static function axes(pairs:Array<Array<String>>):Array<OperatorAxis> return [for (p in pairs) new OperatorAxis(p[0], p[1])];

  public static function drillTap():OperatorProfile return new OperatorProfile("drill-tap", "machinekit.control.cnc-controller",
    axes([["x", "X"], ["y", "Y"], ["z", "Z"], ["a", "A"], ["spindle", "Spindle"]]),
    ["x", "y", "z", "a", "spindle", null, null, null, null, null, null],
    [new OperatorKey(DoorToggle, "Door"), new OperatorKey(SpindleJog, "Spin"), new OperatorKey(Mist, "MQL")],
    [Auto, Mdi, Jog], CNC_PAGES, true, new OperatorGuard(0.25, 5.0), [], []);

  public static function hobber():OperatorProfile return new OperatorProfile("hobber", "machinekit.control.cnc-controller",
    axes([["c", "C table"], ["b", "B hob"], ["x", "X"], ["y", "Y"], ["z", "Z"], ["a", "A"]]),
    ["x", "y", "z", "a", "b", "c", null, null, null, null, null],
    [new OperatorKey(DoorToggle, "Door"), new OperatorKey(SpindleJog, "Hob"), new OperatorKey(WorkLight, "Light")],
    [Auto, Mdi, Jog], CNC_PAGES, true, new OperatorGuard(0.25, 5.0), ["gearbox-engaged"],
    [new JogRule("b", "gearbox-engaged", "Gearbox engaged"), new JogRule("c", "gearbox-engaged", "Gearbox engaged")]);

  /** The arm uses a teach puck; this profile serves its fixed HMI and the enable/state keys. */
  public static function arm():OperatorProfile return new OperatorProfile("arm-cell", "machinekit.control.cnc-controller",
    axes([["j1", "J1"], ["j2", "J2"], ["j3", "J3"], ["j4", "J4"], ["j5", "J5"], ["j6", "J6"]]),
    ["j1", "j2", "j3", "j4", "j5", "j6", null, null, null, null, null],
    [new OperatorKey(Unused, ""), new OperatorKey(Unused, ""), new OperatorKey(Unused, "")],
    [Auto, Jog], [Dro, Program, Services, Diagnostics], false, null, [], []);
}
