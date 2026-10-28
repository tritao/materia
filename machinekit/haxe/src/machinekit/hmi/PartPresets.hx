package machinekit.hmi;

import machinekit.component.ComponentValues;

/**
 * The real parts we buy, as component values: a name for each set of numbers, so a panel layout says
 * "the YUMO wheel" and the BOM carries what that means, and so a measured part replaces one preset
 * rather than edits scattered numbers. Values marked as a maker's drawing or a listing are in the recipe
 * or the values below; the rest are placeholders to be replaced by the measured part (see
 * PENDANT_BENCH_PARTS.md).
 */
class PartPresets {
  /** YUMO ISM6045 100 PPR hand wheel: the recipe's defaults, from the maker's drawing. */
  public static function yumoIsm6045(withHandle:Bool = true):HandWheel
    return new HandWheel(new ComponentValues().setBoolean("handle", withHandle));

  /**
   * Vallder 100 PPR hand wheel (SKU 1165001): 80 mm flange, 59.5 mm grip, 27 mm above the panel, three M4
   * studs on a 72 mm circle, 45 mm opening. The depth behind the panel is not on the drawing (studs about
   * 6.5, terminal strip assumed 12); the panel limit of 3 mm is a reading of two small dimensions.
   */
  public static function vallderMpg(withHandle:Bool = true):HandWheel
    return new HandWheel(new ComponentValues().setBoolean("handle", withHandle).setNumber("outerDiameter", 80).setNumber("flangeThickness", 7)
      .setNumber("coneHeight", 6).setNumber("ringDiameter", 59.5).setNumber("ringHeight", 14).setNumber("dialDiameter", 50)
      .setNumber("handleProtrusion", 14).setNumber("handleRadius", 18).setNumber("bossDiameter", 44).setNumber("panelClearance", 1)
      .setNumber("studCircleDiameter", 72).setNumber("studDiameter", 4).setNumber("studHoleDiameter", 4.8).setNumber("studLength", 12)
      .setNumber("terminalDepth", 12).setNumber("terminalLength", 30).setNumber("terminalWidth", 12).setNumber("maxPanelThickness", 3)
      .setNumber("massGrams", 276));

  /**
   * The AliExpress E-stop (item 1005002447999037, 2.55 EUR for the 1NC variant): a Schneider XB2-ES542 style.
   * A seller's size drawing for the family gives 31 mm above the panel and 43 behind it, a 29 mm wide block and
   * a 42 mm wide base, 52.4 g; the linked listing gives the block as 29.4 x 29.2 x 14.8 mm. Measure on arrival.
   */
  public static function xb2Es542(blocks:Int = 1):EmergencyStop
    return new EmergencyStop(new ComponentValues().setInteger("blocks", blocks));

  /**
   * A compact 22 mm emergency stop (EAO Series 84 or APEM EC Compact): 13.5 to 18.7 mm behind the panel on the
   * makers' pages. The head size and base are not on those pages, so they are assumed.
   */
  public static function compactEStop():EmergencyStop
    return new EmergencyStop(new ComponentValues().setToken("basis", "assumed").setNumber("headHeight", 16).setNumber("nutThickness", 3)
      .setNumber("baseLength", 4.7).setNumber("blockLength", 8).setNumber("baseWidth", 30).setNumber("baseDepth", 30)
      .setNumber("blockWidth", 22).setNumber("blockThickness", 12).setNumber("massGrams", 40));

  /** Panel rotary switch with a knob; 12 positions for an axis switch, 3 for a step switch. */
  public static function rotarySwitch(positions:Int = 12):RotarySwitch
    return new RotarySwitch(new ComponentValues().setInteger("positions", positions));

  /** 16 mm illuminated momentary button. */
  public static function pushButton16():PushButton return new PushButton();

  /** 2.42 in 128x64 OLED module. */
  public static function oled242():OledModule return new OledModule();
}
