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
  /** Smallest clearance seen over the scan, any region (negative = penetration). */
  public var minClearance:Float = 0;
  /** Smallest clearance seen over the scan by region of the tooth: the working flank, the tip (blend and land) and the
      root blend. Infinite when no point of the region was in range. */
  public var minFlankClearance:Float = Math.POSITIVE_INFINITY;
  public var minTipClearance:Float = Math.POSITIVE_INFINITY;
  public var minRootBlendClearance:Float = Math.POSITIVE_INFINITY;
  public var contactThreshold:Float = 0;
  /** Flank engagement: mean over the scan of (teeth within the threshold whose closest point is on the flank) / flexspline teeth. */
  public var flankEngagedFraction:Float = 0;
  public var meanFlankEngaged:Float = 0;
  public var maxFlankEngaged:Int = 0;
  /** Tip edge contact: the same count for teeth whose closest point is on the tip blend or land. This is the tooth tip touching the circular
      spline, not flank engagement, and carries no load path of its own. */
  public var tipContactFraction:Float = 0;
  public var meanTipContact:Float = 0;
  public var maxTipContact:Int = 0;
  /** Largest section angle from the major axis at which a tooth is flank-engaged. */
  public var contactZone:Float = 0;
  public var fitBound:Float = 0;
  /** Each entry names the parameter that causes the interference. Empty = none. */
  public final interference:Array<String> = [];
  public function new() {}
}
