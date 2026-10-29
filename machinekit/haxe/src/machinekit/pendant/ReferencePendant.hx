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
  public static function layout(?size:PendantSize, leftHanded:Bool = false):PanelLayout {
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
        PartPresets.oled242().driven(PendantDisplaySignals.SOURCE, PROFILE, displayBindings()),
        0.0,
        37.0
      ),
      new PlacedPart("f1", PartPresets.pushButton16(), side * -40.5, 1.5),
      new PlacedPart("f2", PartPresets.pushButton16(), side * -13.5, 1.5),
      new PlacedPart("f3", PartPresets.pushButton16(), side * 13.5, 1.5),
      new PlacedPart("wheel-enable", PartPresets.pushButton16("blue"), side * 40.5, 1.5),
      new PlacedPart("start", PartPresets.pushButton16("green"), side * -40.5, -25.5),
      new PlacedPart("hold", PartPresets.pushButton16("yellow"), side * -13.5, -25.5),
      new PlacedPart("stop", PartPresets.pushButton16("red"), side * 13.5, -25.5),
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

  /**
   * The operator profile the simulated display serves. A pendant belongs to a machine and should take that machine's profile; until a
   * layout is built for one, the hobber's stands in.
   */
  public static inline var PROFILE:String = "hobber";

  /** The joint of `joint` on the part `part`, as the assembly names it: its group, the part, then the joint. */
  public static function jointId(part:String, joint:String):String {
    for (group in groups()) if (group.members.indexOf(part) >= 0) return group.id + "/" + part + "/" + joint;
    return part + "/" + joint;
  }

  /** Which joint feeds each signal of the display's driver. */
  public static function displayBindings():Array<DisplayBinding> {
    var bindings = [
      new DisplayBinding(PendantDisplaySignals.AXIS, jointId("axis", "position")),
      new DisplayBinding(PendantDisplaySignals.STEP, jointId("step", "position")),
      new DisplayBinding(PendantDisplaySignals.WHEEL, jointId("wheel", "encoder")),
      new DisplayBinding(PendantDisplaySignals.WHEEL_ENABLE, jointId("wheel-enable", "press")),
      new DisplayBinding(PendantDisplaySignals.ESTOP, jointId("estop", "press"))
    ];
    for (key in PendantDisplaySignals.keys()) bindings.push(new DisplayBinding(key, jointId(key, "press")));
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
   * The whole pendant as an assembly: plate, shell and board in the case group (from `documents` when given, so their
   * faces match the recorded steps), the controls by group, and the fasteners.
   * Every wired control is wired to the board (`PendantBoard.wire`): the wheel's 0 V and VCC to the board's supply and A and
   * B to its inputs, the keys' contacts and LEDs, the selectors' contacts and the display's header.
   */
  public static function assembly(?documents:PendantDocuments):MachineAssembly {
    var pendantCase = documents == null ? pendantCase() : documents.pendantCase;
    var assembly = PanelAssemblyBuilder.build(
      pendantCase.face,
      () -> documents == null ? pendantCase.plate() : documents.plate.part(),
      () -> documents == null ? pendantCase.shell() : documents.shell.part(),
      [new PanelExtra(
        "board",
        new PendantBoard(pendantCase.face, () -> documents == null ? pendantCase.board() : documents.board.part())
      )],
      groups(),
      pendantCase.hardware()
    );
    PendantBoard.wire(assembly, pendantCase.face, groups(), "case/board");
    return assembly;
  }

  /** The case for the reference layout, or for the same controls on a case of another size. */
  public static function pendantCase(?size:PendantSize, leftHanded:Bool = false):PendantCase return PendantCase.plan(
    layout(size, leftHanded),
    leftHanded
  );
}
