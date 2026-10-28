package machinekit.pendant;

import machinekit.hmi.*;

/**
 * The first layout of the one-SKU pendant: a 190 x 110 mm face with the wheel and the wheel-enable key in
 * the right-hand column, the E-stop, display, the two rotary switches, F1 and F2 on the left, and F3 with
 * the fixed Start, Hold and Stop/Reset keys along the bottom. The cheap AliExpress E-stop is 43 mm deep,
 * so the case is 48 mm. Tuned with the layout checker; every assumed part is to be replaced by the measured
 * one.
 */
class ReferencePendant {
  public static function layout():PanelLayout {
    var controls = [
      new PlacedPart("wheel", PartPresets.yumoIsm6045(), 60.0, 10.0),
      new PlacedPart("wheel-enable", PartPresets.pushButton16(), 60.0, -38.0),
      new PlacedPart("estop", PartPresets.xb2Es542(), -70.0, 30.0),
      new PlacedPart("display", PartPresets.oled242(), -14.0, 31.0),
      new PlacedPart("axis", PartPresets.rotarySwitch(12), -76.5, -8.0),
      new PlacedPart("step", PartPresets.rotarySwitch(3), -46.5, -8.0),
      new PlacedPart("f1", PartPresets.pushButton16(), -19.0, -8.0),
      new PlacedPart("f2", PartPresets.pushButton16(), 6.0, -8.0),
      new PlacedPart("f3", PartPresets.pushButton16(), -78.5, -38.0),
      new PlacedPart("start", PartPresets.pushButton16(), -53.5, -38.0),
      new PlacedPart("hold", PartPresets.pushButton16(), -28.5, -38.0),
      new PlacedPart("stop", PartPresets.pushButton16(), -3.5, -38.0)
    ];
    return new PanelLayout(190.0, 110.0, 3.0, 3.0, 48.0, controls);
  }

  /** The case for the reference layout. */
  public static function pendantCase():PendantCase return PendantCase.plan(layout());
}
