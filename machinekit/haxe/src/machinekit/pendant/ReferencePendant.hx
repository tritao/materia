package machinekit.pendant;

import machinekit.assembly.MachineAssembly;
import cadkit.threads.SurfaceDetail.DisplayBinding;
import machinekit.hmi.*;
import machinekit.operator.PendantDisplaySignals;

/**
 * The first layout of the one-SKU pendant, a one-hand handheld like commercial CNC pendants: a 114 x 210 mm face, held
 * with the palm behind it. From the top: the axis and step switches beside the E-stop, the display, F1 to F3 and the
 * wheel-enable key, the Start, Hold and Stop keys, and the wheel at the bottom under the thumb and fingers. The
 * enabling switch is a side key, planned by `PendantCase`. The cheap AliExpress E-stop is 43 mm deep, so the case is 48 mm. Start is green, Hold yellow, Stop red and the wheel-enable key blue; the other keys are white. Tuned with the layout
 * checker; every assumed part is to be replaced by the measured one.
 */
class ReferencePendant {
  /** The reference case's dimensions. */
  public static function size():PendantSize return {width: 114.0,
    height: 210.0, thickness: 3.0, wall: 3.0, depth: 48.0};

  /** The reference layout, or the same controls on a case of another size; `leftHanded` mirrors it. */
  public static function layout(?size:PendantSize, leftHanded:Bool = false, ?machine:PendantMachine):PanelLayout {
    var owner = machine == null ? PendantMachine.reference() : machine;
    var dimensions = size == null ? ReferencePendant.size() : size;
    // The pendant is held in the left hand, with the enabling switch on the left wall and the Start, Hold and Stop keys under the
    // thumb; the free right hand turns the selectors and the wheel, so they sit on the right and the E-stop at the top left, where
    // the holding hand can reach it. A left-handed pendant is the same layout turned left to right.
    var side = leftHanded ? -1.0 : 1.0;
    var controls = [
      new PlacedPart("axis", PartPresets.rotarySwitch(12), side * 39.0, 80.0),
      new PlacedPart("step", PartPresets.rotarySwitch(3), side * 8.0, 80.0),
      new PlacedPart("estop", PartPresets.xb2Es542(), side * -32.0, 80.0),
      new PlacedPart(
        "display",
        PartPresets.oled242().driven(PendantDisplaySignals.SOURCE, owner.profile, displayBindings(owner)),
        0.0,
        37.0
      ),
      new PlacedPart("f1", lit(PartPresets.pushButton16(), "f1", owner), side * -40.5, 1.5),
      new PlacedPart("f2", lit(PartPresets.pushButton16(), "f2", owner), side * -13.5, 1.5),
      new PlacedPart("f3", lit(PartPresets.pushButton16(), "f3", owner), side * 13.5, 1.5),
      new PlacedPart("wheel-enable", PartPresets.pushButton16("blue"), side * 40.5, 1.5),
      new PlacedPart("start", lit(PartPresets.pushButton16("green"), "start", owner), side * -40.5, -25.5),
      new PlacedPart("hold", lit(PartPresets.pushButton16("yellow"), "hold", owner), side * -13.5, -25.5),
      new PlacedPart("stop", lit(PartPresets.pushButton16("red"), "stop", owner), side * 13.5, -25.5),
      new PlacedPart("wheel", PartPresets.yumoIsm6045(), 0.0, -70.0)
    ];
    return new PanelLayout(
      dimensions.width,
      dimensions.height,
      dimensions.thickness,
      dimensions.wall,
      dimensions.depth,
      controls
    );
  }

  /** The joint of `joint` on the part `part`, as the assembly names it: its group, the part, then the joint. */
  public static function jointId(part:String, joint:String, prefix:String = ""):String {
    for (group in groups()) if (group.members.indexOf(part) >= 0) return prefix + group.id + "/" + part + "/" + joint;
    return prefix + part + "/" + joint;
  }

  /** A key whose LED the simulated pendant lights: the cap shows the output of the driver that is this key's LED. */
  static function lit(button:machinekit.hmi.PushButton, key:String, owner:PendantMachine):machinekit.hmi.PushButton {
    return button.lit(PendantDisplaySignals.LED_SOURCE, owner.profile, displayBindings(owner), key);
  }

  /** Which joint feeds each signal of the display's driver, and which joint each of the machine's axes moves. */
  public static function displayBindings(?machine:PendantMachine):Array<DisplayBinding> {
    var prefix = machine == null ? "" : machine.prefix;
    var bindings = [
      new DisplayBinding(PendantDisplaySignals.AXIS, jointId("axis", "position", prefix)),
      new DisplayBinding(PendantDisplaySignals.STEP, jointId("step", "position", prefix)),
      new DisplayBinding(PendantDisplaySignals.WHEEL, jointId("wheel", "encoder", prefix)),
      new DisplayBinding(PendantDisplaySignals.WHEEL_ENABLE, jointId("wheel-enable", "press", prefix)),
      new DisplayBinding(PendantDisplaySignals.ESTOP, jointId("estop", "press", prefix))
    ];
    for (key in PendantDisplaySignals.keys()) bindings.push(new DisplayBinding(key, jointId(key, "press", prefix)));
    if (machine != null) for (jog in machine.jogs) bindings.push(jog);
    return bindings;
  }

  /** How the controls are grouped in the assembly's hierarchy: by what the operator does with them. */
  public static function groups():Array<PanelGroup> return [
    new PanelGroup("jog", ["wheel", "wheel-enable"]),
    new PanelGroup("selectors", ["axis", "step"]),
    new PanelGroup("function-keys", ["f1", "f2", "f3"]),
    new PanelGroup("cycle-keys", ["start", "hold", "stop"])
  ];

  /**
   * The whole pendant as an assembly: plate, shell and board in the case group, each recording its geometry (so the scene carries
   * their histories), the controls by group, and the fasteners.
   * Every wired control is wired to the board (`PendantBoard.wire`): the wheel's 0 V and VCC to the board's supply and A and
   * B to its inputs, the keys' contacts and LEDs, the selectors' contacts and the display's header.
   */
  public static function assembly(?pendantCase:PendantCase):MachineAssembly {
    if (pendantCase == null) pendantCase = ReferencePendant.pendantCase();
    var made:PendantCase = pendantCase;
    var assembly = PanelAssemblyBuilder.build(
      made.face,
      Recorded(b -> PendantCaseDocument.recordPlate(b, made)),
      Recorded(b -> PendantCaseDocument.recordShell(b, made)),
      [new PanelExtra("board", new PendantBoard(made.face, b -> PendantCaseDocument.recordBoard(b, made)))],
      groups(),
      made.hardware()
    );
    PendantBoard.wire(assembly, pendantCase.face, groups(), "case/board");
    return assembly;
  }

  /** The case for the reference layout, or for the same controls on a case of another size. */
  public static function pendantCase(?size:PendantSize, leftHanded:Bool = false,
    ?machine:PendantMachine):PendantCase return PendantCase.plan(
    layout(size, leftHanded, machine),
    leftHanded
  );
}
