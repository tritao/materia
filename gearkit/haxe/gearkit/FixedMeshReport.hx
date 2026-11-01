package gearkit;

/** Result of meshing a flexspline against a fixed circular spline (`FixedMesh`), unloaded and rigid. Angles are the
    flexspline's rigid rotation about the axis, radians, positive in the drive direction. Lengths in mm. */
class FixedMeshReport {
  public static final ARCSEC = 180 * 3600 / Math.PI;

  public var name:String = "";
  public var spline:String = "";
  public final thetas:Array<Float> = [];
  /** Per wave-generator angle: the rotation at which the first drive-flank contact occurs. NaN when not bracketed. */
  public final drive:Array<Float> = [];
  /** The rotation at which the first reverse-flank contact occurs (the other end of the free range). */
  public final reverse:Array<Float> = [];
  /** Per angle: drive minus reverse; negative = the teeth cannot be placed without overlapping (interference). */
  public final backlash:Array<Float> = [];
  public final driveContacts:Array<Array<ToothContact>> = [];
  public final reverseContacts:Array<Array<ToothContact>> = [];
  public var meanDrive:Float = 0;
  /** Transmission error: drive rotation about its mean over the scan, per angle (radians). */
  public final transmissionError:Array<Float> = [];
  public var tePeakToPeak:Float = 0;
  public var teRms:Float = 0;
  public var backlashMean:Float = 0;
  public var backlashMin:Float = 0;
  public var backlashMax:Float = 0;
  /** Angles where a contact angle could not be bracketed within the search window. */
  public var unresolved:Int = 0;
  public var contactTolerance:Float = 0;
  public var contactOffset:Float = 0;
  /** Flank engagement and tip edge contact at the drive-flank position: teeth within the tolerance, by region of the closest point. */
  public var meanFlankContacts:Float = 0;
  public var maxFlankContacts:Int = 0;
  public var meanTipContacts:Float = 0;
  public var maxTipContacts:Int = 0;
  /** Fraction of angles whose closest tooth at the drive-flank position touches with its flank / its tip edge. */
  public var firstContactFlank:Float = 0;
  public var firstContactTip:Float = 0;
  /** Deepest flexspline reach past the circular-spline root circle over the scan (mm, 0 = none). */
  public var rootBottoming:Float = 0;
  /** Deepest overlap of a point whose contact normal is nearly radial, which a rotation cannot remove (mm, 0 = none). */
  public var radialOverlap:Float = 0;
  public final interference:Array<String> = [];

  public function new() {}

  public function tePeakToPeakArcsec():Float return tePeakToPeak * ARCSEC;
  public function backlashMeanArcsec():Float return backlashMean * ARCSEC;
}
