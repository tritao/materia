package gearkit;

/** One flexspline tooth touching the circular spline at a flank-contact position of the fixed mesh. `region` is the
    region of the tooth's closest point (`ToothRegion.Flank`: working flank; `Tip`: tip edge contact). */
class ToothContact {
  public final tooth:Int;
  public final region:ToothRegion;
  /** Clearance of that closest point (mm), with the report's contact offset included. */
  public final clearance:Float;
  /** Section angle of the tooth from the major axis, folded to (-pi/2, pi/2]. */
  public final section:Float;

  public function new(tooth:Int, region:ToothRegion, clearance:Float, section:Float) {
    this.tooth = tooth;
    this.region = region;
    this.clearance = clearance;
    this.section = section;
  }
}
