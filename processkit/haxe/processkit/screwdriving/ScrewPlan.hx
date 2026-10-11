package processkit.screwdriving;

import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** One qualified fastening in the arm's reference frame; distances are metres. */
class ScrewPlan {
  public final fastening:String;
  public final slot:Int;
  public final program:TighteningProgram;
  public final pickup:Transform3;
  /** Socket frame at thread entry, with +Z into the threaded member. */
  public final entry:Transform3;
  public final leadMm:Float;
  public final runDownMm:Float;
  public final approach:Float;
  public final strokeMm:Float;

  public function new(fastening:String, slot:Int, program:TighteningProgram, pickup:Transform3,
      entry:Transform3, leadMm:Float, runDownMm:Float, approach:Float = 0.04) {
    if (fastening == null || fastening.length == 0 || slot <= 0 || program == null || pickup == null || entry == null)
      throw "Screw plan needs a named fastening, qualified slot, pickup and entry frames";
    for (value in [leadMm,approach]) if (!Math.isFinite(value) || value <= 0)
      throw 'Fastening "$fastening" needs positive finite lead and approach';
    if (!Math.isFinite(runDownMm) || runDownMm < 0) throw 'Fastening "$fastening" has invalid run-down';
    this.fastening = fastening; this.slot = slot; this.program = program; this.pickup = pickup; this.entry = entry;
    this.leadMm = leadMm; this.runDownMm = runDownMm; this.approach = approach;
    if (program.settings.minimumStroke >= program.settings.maximumStroke)
      throw 'Fastening "$fastening" needs a feed window with room to follow';
    strokeMm = (program.settings.minimumStroke+program.settings.maximumStroke)/2;
    if (strokeMm <= 0) throw 'Fastening "$fastening" needs compliant feed compression';
  }
  public function transformed(reference:Transform3):ScrewPlan
    return new ScrewPlan(fastening,slot,program,reference.compose(pickup),reference.compose(entry),leadMm,runDownMm,approach);
  /** Offset a socket frame along its +Z without changing its orientation. */
  public static function along(frame:Transform3, distance:Float):Transform3
    return new Transform3(frame.transformPoint(new Vec3(0,0,distance)),frame.rotation);
}
