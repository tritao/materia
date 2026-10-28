package machinekit.pendant;

/**
 * The bought parts, as controls. Numbers marked "vendor-drawing" come from the supplier's drawing;
 * "assumed" ones are placeholders to be replaced by the measured part (see PENDANT_BENCH_PARTS.md).
 */
class PendantParts {
  /**
   * Vallder 100 PPR hand wheel (SKU 1165001). Drawing: flange 80, panel template a 45 mm centre hole
   * and three 4.8 mm holes on a 72 mm circle at 120 degrees, 27 mm above the panel (41 with the 10 mm
   * spinner handle). The depth behind the panel is not on the drawing (two small dimensions, 3 and
   * 3.5, read as the boss and stud); the terminal strip height is assumed.
   */
  public static function vallderWheel(x:Float, y:Float, withHandle:Bool = true):PendantControl {
    var holes = [for (i in 0...3) {
      var a = Math.PI / 2 + i * 2 * Math.PI / 3;
      {x: 36.0 * Math.cos(a), y: 36.0 * Math.sin(a), diameter: 4.8};
    }];
    return new PendantControl("wheel", Wheel, x, y, Circle(45.0), Circle(80.0), withHandle ? 41.0 : 27.0, 12.0, 276.0, "vendor-drawing", holes);
  }

  /**
   * Compact 22 mm emergency stop. Depth behind the panel is from the candidates' pages (EAO Series 84
   * 13.5 mm with cable connection and 18 mm plug-in, APEM EC Compact 18.7 mm); the head diameter and
   * height are not on those pages, so the 40 mm head and 25 mm height are assumed, and the mass.
   * A standard Harmony XB5AS8445 is 82 mm deep per its datasheet, which no handheld case can take.
   */
  public static function eStopCompact(x:Float, y:Float):PendantControl
    return new PendantControl("estop", EStop, x, y, Circle(22.3), Circle(40.0), 25.0, 18.7, 40.0, "assumed");

  /**
   * The AliExpress E-stop (item 1005002447999037, 2.55 EUR for the 1NC variant): a Schneider XB2-ES542
   * style, 40 mm twist-release mushroom, snap-on ZB2-BE102C contact block. The listing gives the block
   * as 29.4 x 29.2 x 14.8 mm and the weight as 60 g. The depth behind the panel and the height above it
   * are my estimates from the product photos scaled to the block (about 46 and 35 mm); the listing's
   * "29 x 30 x 68.5 mm" does not say what it measures. Measure on arrival.
   */
  public static function eStopGeneric(x:Float, y:Float):PendantControl
    return new PendantControl("estop", EStop, x, y, Circle(22.3), Circle(40.0), 35.0, 46.0, 60.0, "assumed");

  /** Panel rotary switch with a knob (assumed: 10 mm bush hole, 25 mm knob, 25 mm body). */
  public static function rotary(id:String, x:Float, y:Float):PendantControl
    return new PendantControl(id, Rotary, x, y, Circle(10.2), Circle(25.0), 18.0, 25.0, 25.0, "assumed");

  /** 16 mm illuminated momentary button (assumed: 16.2 mm hole, 20 mm for the cap and its edge, 25 mm body). */
  public static function button(id:String, x:Float, y:Float):PendantControl
    return new PendantControl(id, Button, x, y, Circle(16.2), Circle(20.0), 6.0, 25.0, 8.0, "assumed");

  /** 2.42 in 128x64 OLED module (assumed: viewing window 56x28 with a 3 mm bezel margin, 12 mm deep, 30 g). */
  public static function display242(x:Float, y:Float):PendantControl
    return new PendantControl("display", Display, x, y, Rect(56.0, 28.0), Rect(62.0, 34.0), 1.0, 12.0, 30.0, "assumed");
}
