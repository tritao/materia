import machinekit.gantry.Gantry;
import machinekit.gantry.GantrySpec;
import machinekit.gantry.GantrySpec.GantryDrive;
import machinekit.motion.LeadScrewThread;

/**
 * A small table-top XYZ stage: three lead-screw axes with profile rails and NEMA 17 steppers on an aluminium-profile frame. It is a
 * Gantry, so it has the real joints (`x`, `y`, `z`), drives and limits of one, in a size a bench can hold. Travel is 200 x 150 x 100 mm;
 * the screws are 12 mm trapezoidal with 3 mm lead. The numbers are the sizes the gantry checks already cover; the layout is not
 * tuned for any process.
 */
class XyzStage extends Gantry {
  public static inline var TRAVEL_X:Float = 200;
  public static inline var TRAVEL_Y:Float = 150;
  public static inline var TRAVEL_Z:Float = 100;

  public function new() {
    var screw:GantryDrive = Screw(new LeadScrewThread(MetricTrapezoidal, 12, 3));
    super(new GantrySpec(TRAVEL_X, TRAVEL_Y, TRAVEL_Z, screw, screw, screw, false, "MGN12C", 17, "HFS5-4040", "HFS5-4040", true), "XYZ");
  }
}
