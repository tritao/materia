package machinekit.hmi;

import machinekit.component.ComponentValues;

/**
 * The real parts we buy, by name. Wheels and E-stops select a row of ControlCatalogs, which holds the numbers
 * and where they came from; the rotary switch, button and display are placeholders to be replaced by the
 * measured part (see PENDANT_BENCH_PARTS.md).
 */
class PartPresets {
  /** YUMO ISM6045 100 PPR hand wheel, from the maker's drawing. */
  public static function yumoIsm6045(withHandle:Bool = true):HandWheel
    return new HandWheel(new ComponentValues().setToken("model", ControlCatalogs.YUMO).setBoolean("handle", withHandle));

  /** Vallder 100 PPR hand wheel (SKU 1165001), from its dimension drawing. */
  public static function vallderMpg(withHandle:Bool = true):HandWheel
    return new HandWheel(new ComponentValues().setToken("model", ControlCatalogs.VALLDER).setBoolean("handle", withHandle));

  /** The AliExpress XB2-ES542 style E-stop; `blocks` NC contact blocks (1 for the 1NC variant). */
  public static function xb2Es542(blocks:Int = 1):EmergencyStop
    return new EmergencyStop(new ComponentValues().setToken("model", ControlCatalogs.XB2).setInteger("blocks", blocks));

  /** A compact 22 mm emergency stop (EAO Series 84, APEM EC Compact): depth from the makers' pages, the rest assumed. */
  public static function compactEStop():EmergencyStop
    return new EmergencyStop(new ComponentValues().setToken("model", ControlCatalogs.EAO));

  /** Panel rotary switch with a knob; 12 positions for an axis switch, 3 for a step switch. */
  public static function rotarySwitch(positions:Int = 12):RotarySwitch
    return new RotarySwitch(new ComponentValues().setInteger("positions", positions));

  /** 16 mm illuminated momentary button. */
  public static function pushButton16():PushButton return new PushButton();

  /** 2.42 in 128x64 OLED module. */
  public static function oled242():OledModule return new OledModule();
}
