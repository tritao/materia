package machinekit.pendant;

import machinekit.hmi.*;

/**
 * The first layout of the one-SKU pendant, a one-hand handheld like commercial CNC pendants: a 114 x 210 mm face, held
 * with the palm behind it. From the top: the axis and step switches beside the E-stop, the display, F1 to F3 and the
 * wheel-enable key, the Start, Hold and Stop keys, and the wheel at the bottom under the thumb and fingers. The
 * enabling switch is a side key, planned by `PendantCase`. The cheap AliExpress E-stop is 43 mm deep, so the case is 48 mm. Start is green, Hold yellow, Stop red and the wheel-enable key blue; the other keys are white. Tuned with the layout
 * checker; every assumed part is to be replaced by the measured one.
 */
class ReferencePendant {
  public static function layout():PanelLayout {
    var controls = [
      new PlacedPart("axis", PartPresets.rotarySwitch(12), -39.0, 80.0),
      new PlacedPart("step", PartPresets.rotarySwitch(3), -8.0, 80.0),
      new PlacedPart("estop", PartPresets.xb2Es542(), 32.0, 80.0),
      new PlacedPart("display", PartPresets.oled242(), 0.0, 38.0),
      new PlacedPart("f1", PartPresets.pushButton16(), -40.5, 4.0),
      new PlacedPart("f2", PartPresets.pushButton16(), -13.5, 4.0),
      new PlacedPart("f3", PartPresets.pushButton16(), 13.5, 4.0),
      new PlacedPart("wheel-enable", PartPresets.pushButton16("blue"), 40.5, 4.0),
      new PlacedPart("start", PartPresets.pushButton16("green"), -40.5, -23.0),
      new PlacedPart("hold", PartPresets.pushButton16("yellow"), -13.5, -23.0),
      new PlacedPart("stop", PartPresets.pushButton16("red"), 13.5, -23.0),
      new PlacedPart("wheel", PartPresets.yumoIsm6045(), 0.0, -70.0)
    ];
    return new PanelLayout(114.0, 210.0, 3.0, 3.0, 48.0, controls);
  }

  /** How the controls are grouped in the assembly's hierarchy: by what the operator does with them. */
  public static function groups():Array<PanelGroup> return [
    new PanelGroup("jog", ["wheel", "wheel-enable"]),
    new PanelGroup("selectors", ["axis", "step"]),
    new PanelGroup("function-keys", ["f1", "f2", "f3"]),
    new PanelGroup("cycle-keys", ["start", "hold", "stop"])
  ];

  /** The case for the reference layout. */
  public static function pendantCase():PendantCase return PendantCase.plan(layout());
}
