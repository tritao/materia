package gearkit;

/** Unloaded kinematic meshing of a flexspline in its derived circular spline over one wave-generator
    period (TOOTH_DESIGN_PLAN.md TD-D4). Lengths in mm, angles in radians. */
class MeshingReport {
  public var name:String = "";
  /** Circular-spline root radius (nominal, assumed full depth) minus the deepest flexspline reach; negative = bottoming. */
  public var rootClearance:Float = 0;
  /** Circular-spline tip radius minus the flexspline tip radius at the minor axis; negative = tips overlap radially (tooth meets tooth). */
  public var minorTipClearance:Float = 0;
  /** Thinnest circular-spline tooth between its nominal tip and the flexspline reach; zero = saturated (swept space fills the pitch). */
  public var minToothThickness:Float = 0;
  public var minToothThicknessRadius:Float = 0;
  /** Smallest radius at which the derived circular-spline tooth is at least `minThickness` thick: the tip the circular spline can have.
      The scan treats everything below it as open. Compare with the nominal tip to see the addendum given up. */
  public var derivedTipRadius:Float = 0;
  public var minThickness:Float = 0;
  /** Largest penetration of any flexspline point into the circular spline over the scan (0 if none). */
  public var penetration:Float = 0;
  public var penetrationTooth:Int = -1;
  public var penetrationAngle:Float = 0;
  public var penetrationSection:Float = 0;
  /** Smallest clearance seen over the scan (negative = penetration). */
  public var minClearance:Float = 0;
  public var contactThreshold:Float = 0;
  /** Mean over the scan of (teeth with clearance <= threshold) / flexspline teeth. */
  public var engagedFraction:Float = 0;
  public var meanEngaged:Float = 0;
  public var maxEngaged:Int = 0;
  /** Largest section angle from the major axis at which a tooth is in contact. */
  public var contactZone:Float = 0;
  public var fitBound:Float = 0;
  /** Each entry names the parameter that causes the interference. Empty = none. */
  public final interference:Array<String> = [];
  public function new() {}
}
