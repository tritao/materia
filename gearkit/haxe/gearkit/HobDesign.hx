package gearkit;

/** Right-hand hob from a numerically conjugated normal rack. The outside diameter
    includes the recovered cutter-tip height. Rake faces are planes parallel to Z;
    positive rake tilts the face at the outside circle. No relief or sharpening allowance. */
class HobDesign {
  public final rack:ConjugateRack;
  public final starts:Int;
  public final outsideDiameter:Float;
  public final gashes:Int;
  public final rake:Float;
  public final pitchDiameter:Float;
  public final leadAngle:Float;
  public final axialPitch:Float;
  public final lead:Float;
  public final rootHeight:Float;
  final normalRegion:Array<Point2>;

  public function new(
    rack:ConjugateRack,
    starts:Int,
    outsideDiameter:Float,
    gashes:Int,
    rake:Float = 0,
    ?rootHeight:Float
  ) {
    if (starts < 1 || gashes < 2) throw "hob needs positive starts and at least two gashes";
    if (!Math.isFinite(outsideDiameter)
      || outsideDiameter <= 0) throw "hob outside diameter must be finite and positive";
    if (!Math.isFinite(rake)
      || Math.abs(rake) >= Math.PI / 4) throw "hob rake must be finite and less than pi/4 in magnitude";
    this.rack = rack;
    this.starts = starts;
    this.outsideDiameter = outsideDiameter;
    this.gashes = gashes;
    this.rake = rake;
    // The complete recovered rake region includes the cutter-root blend.
    var upper = rack.rootSamples();
    if (upper.length == 0) upper = rack.flankSamples();
    this.rootHeight = rootHeight == null ? upper[upper.length - 1].y : rootHeight;
    normalRegion = rack.cutterRegion(this.rootHeight);
    var tipHeight = -rack.cornerSamples()[0].y;
    pitchDiameter = outsideDiameter - 2 * tipHeight;
    if (pitchDiameter <= starts * rack.module) throw "hob pitch diameter must exceed starts times normal module";
    leadAngle = Math.asin(starts * rack.module / pitchDiameter);
    axialPitch = Math.PI * rack.module / Math.cos(leadAngle);
    lead = starts * axialPitch;
    var minimumRadius = pitchDiameter / 2 - this.rootHeight;
    if (minimumRadius <= outsideDiameter / 2 * Math.abs(Math.sin(rake))) throw "rake plane misses the hob tooth root";
  }

  /** One closed rake-face tooth region. thread=0..starts-1, turn is any axial turn
      of that thread. Consecutive thread starts are one axial pitch apart. */
  public function rakeFaceRegion(gash:Int, thread:Int = 0, turn:Int = 0):Array<Point3> {
    if (gash < 0 || gash >= gashes || thread < 0 || thread >= starts) throw "hob gash or thread index is out of range";
    var phase = 2 * Math.PI * gash / gashes;
    var result:Array<Point3> = [];
    for (p in normalRegion) {
      var radius = pitchDiameter / 2 - p.y;
      var faceAngle = Math.asin(outsideDiameter / 2 * Math.sin(rake) / radius) - rake;
      var angle = phase + faceAngle;
      var z = p.x / Math.cos(leadAngle) + lead * angle / (2 * Math.PI) + thread * axialPitch + turn * lead;
      result.push(new Point3(radius * Math.cos(angle), radius * Math.sin(angle), z));
    }
    return result;
  }
}
