package machinekit.pendant;

import machinekit.component.ComponentValues;

/**
 * The bought parts as controls for the layout. Each is a component (see PendantHandWheel and the other
 * Pendant* recipes) that carries its own geometry, mass and provenance; the control takes every footprint
 * and height from it. Values marked as a maker's drawing or a listing are in the component's recipe; the
 * rest are placeholders to be replaced by the measured part (see PENDANT_BENCH_PARTS.md).
 */
class PendantParts {
  /** YUMO ISM6045 100 PPR hand wheel: the recipe's defaults, from the maker's drawing. */
  public static function yumoWheel(x:Float, y:Float, withHandle:Bool = true):PendantControl {
    var v = new ComponentValues().setBoolean("handle", withHandle);
    return PendantControl.fromPart("wheel", Wheel, new PendantHandWheel(v), x, y);
  }

  /**
   * Vallder 100 PPR hand wheel (SKU 1165001): 80 mm flange, 59.5 mm grip, 27 mm above the panel, three M4
   * studs on a 72 mm circle, 45 mm opening. The depth behind the panel is not on the drawing (studs about
   * 6.5, terminal strip assumed 12).
   */
  public static function vallderWheel(x:Float, y:Float, withHandle:Bool = true):PendantControl {
    var v = new ComponentValues().setBoolean("handle", withHandle).setNumber("outerDiameter", 80).setNumber("flangeThickness", 7)
      .setNumber("coneHeight", 6).setNumber("ringDiameter", 59.5).setNumber("ringHeight", 14).setNumber("dialDiameter", 50)
      .setNumber("handleProtrusion", 14).setNumber("handleRadius", 18).setNumber("bossDiameter", 44).setNumber("panelClearance", 1)
      .setNumber("studCircleDiameter", 72).setNumber("studDiameter", 4).setNumber("studHoleDiameter", 4.8).setNumber("studLength", 12).setNumber("terminalDepth", 12)
      .setNumber("terminalLength", 30).setNumber("terminalWidth", 12).setNumber("massGrams", 276);
    return PendantControl.fromPart("wheel", Wheel, new PendantHandWheel(v), x, y);
  }

  /**
   * The AliExpress E-stop (item 1005002447999037, 2.55 EUR for the 1NC variant): a Schneider XB2-ES542 style.
   * A seller's size drawing for the family gives 31 mm above the panel and 43 behind it, a 29 mm wide block and
   * a 42 mm wide base, 52.4 g; the linked listing gives the block as 29.4 x 29.2 x 14.8 mm. Measure on arrival.
   */
  public static function eStopGeneric(x:Float, y:Float, blocks:Int = 1):PendantControl
    return PendantControl.fromPart("estop", EStop, new PendantEmergencyStop(new ComponentValues().setInteger("blocks", blocks)), x, y);

  /**
   * A compact 22 mm emergency stop (EAO Series 84 or APEM EC Compact): 13.5 to 18.7 mm behind the panel on the
   * makers' pages. The head size and base are not on those pages, so they are assumed.
   */
  public static function eStopCompact(x:Float, y:Float):PendantControl {
    var v = new ComponentValues().setToken("basis", "assumed").setNumber("headHeight", 16).setNumber("nutThickness", 3)
      .setNumber("baseLength", 4.7).setNumber("blockLength", 8).setNumber("baseWidth", 30).setNumber("baseDepth", 30)
      .setNumber("blockWidth", 22).setNumber("blockThickness", 12).setNumber("massGrams", 40);
    return PendantControl.fromPart("estop", EStop, new PendantEmergencyStop(v), x, y);
  }

  /** Panel rotary switch with a knob; `positions` is 12 for the axis switch and 3 for the step switch. */
  public static function rotary(id:String, x:Float, y:Float, positions:Int = 12):PendantControl
    return PendantControl.fromPart(id, Rotary, new PendantRotarySwitch(new ComponentValues().setInteger("positions", positions)), x, y);

  /** 16 mm illuminated momentary button. */
  public static function button(id:String, x:Float, y:Float):PendantControl
    return PendantControl.fromPart(id, Button, new PendantPushButton(), x, y);

  /** 2.42 in 128x64 OLED module. */
  public static function display242(x:Float, y:Float):PendantControl
    return PendantControl.fromPart("display", Display, new PendantOledModule(), x, y);
}
