package gearkit;

/** Meshing of a flexspline in its derived circular spline (TOOTH_DESIGN_PLAN.md TD-D4), unloaded.
    The state of the whole ring repeats, up to relabelling teeth, every circular pitch of
    wave-generator angle, so one pitch is scanned with every flexspline tooth. */
class MeshingAnalysis {
  /** `contactThreshold` mm: a tooth is engaged when some point is within it of the circular spline.
      `minThicknessFraction` x module is the thinnest acceptable circular-spline tooth (assumed 0.05). */
  public static function run(
    pair:StrainWavePair,
    contactThreshold:Float,
    thetaSamples:Int = 48,
    minThicknessFraction:Float = 0.05
  ):MeshingReport {
    var input = pair.input, space = pair.space(), kin = pair.kinematics, report = new MeshingReport();
    var m = input.module, zf = input.flexTeeth, zc = input.circularTeeth;
    var rTip = pair.circularTipRadius(), rRoot = pair.circularRootRadius();
    report.name = input.name;
    report.contactThreshold = contactThreshold;
    report.fitBound = space.deviationBound;
    report.minThickness = minThicknessFraction * m;
    var points = pair.boundary(64);
    var major = kin.poseAt(0, 0), minor = kin.poseAt(0, Math.PI / 2);
    var reach = 0.0, minorTip = 0.0;
    for (p in points) {
      reach = Math.max(reach, major.apply(p).radius());
      minorTip = Math.max(minorTip, minor.apply(p).radius());
    }
    report.rootClearance = rRoot - reach;
    report.minorTipClearance = rTip - minorTip;
    // Circular-spline tooth thickness from the derived space.
    var thin = Math.POSITIVE_INFINITY, thinAt = rTip, derivedTip = space.maxRadius();
    var n = 400;
    for (i in 0...n + 1) {
      var r = space.minRadius() + (space.maxRadius() - space.minRadius()) * i / n;
      var t = space.toothThickness(r);
      if (t < thin) {
        thin = t;
        thinAt = r;
      }
    }
    for (i in 0...n + 1) {
      var r = space.maxRadius() - (space.maxRadius() - space.minRadius()) * i / n;
      if (space.toothThickness(r) >= report.minThickness) derivedTip = r;
      else break;
    }
    report.minToothThickness = Math.max(0, thin);
    report.minToothThicknessRadius = thinAt;
    report.derivedTipRadius = derivedTip;
    // Scan one circular pitch of wave-generator angle with every tooth.
    var pitch = 2 * Math.PI / zc, minClear = Math.POSITIVE_INFINITY, worst = 0.0;
    var engagedSum = 0, engagedMax = 0;
    for (s in 0...thetaSamples) {
      var theta = pitch * s / thetaSamples, engaged = 0;
      for (j in 0...zf) {
        var delta = kin.sectionAngle(j, theta), pose = kin.pose(j, theta);
        var folded = delta - Math.PI * Math.round(delta / Math.PI);
        var toothMin = Math.POSITIVE_INFINITY;
        for (p in points) {
          var q = pose.apply(p), r = q.radius();
          var c:Float;
          if (r > rRoot) c = rRoot - r;
          else if (r < derivedTip || r > space.maxRadius()) continue;
          else c = space.clearance(q);
          if (c < toothMin) toothMin = c;
        }
        if (toothMin <= contactThreshold) {
          engaged++;
          report.contactZone = Math.max(report.contactZone, Math.abs(folded));
        }
        if (toothMin < minClear) minClear = toothMin;
        if (-toothMin > worst) {
          worst = -toothMin;
          report.penetrationTooth = j;
          report.penetrationAngle = theta;
          report.penetrationSection = folded;
        }
      }
      engagedSum += engaged;
      engagedMax = Std.int(Math.max(engagedMax, engaged));
    }
    report.minClearance = minClear;
    report.penetration = worst;
    report.meanEngaged = engagedSum / thetaSamples;
    report.engagedFraction = report.meanEngaged / zf;
    report.maxEngaged = engagedMax;
    // Interference, each naming the parameter responsible.
    if (report.rootClearance < 0) report.interference.push('kappa too large: flexspline tips reach ${-report.rootClearance} mm past the circular-spline root (tooth ${report.penetrationTooth}, wave generator ${report.penetrationAngle} rad)');
    if (report.minorTipClearance < -space.deviationBound) report.interference.push(
      'kappa too small (or tooth too tall): at the minor axis the flexspline tip is ${-report.minorTipClearance} mm inside the circular-spline tip circle, tooth meets tooth (circular-spline thickness there ${report.minToothThickness} mm)');
    if (report.penetration > space.deviationBound && report.rootClearance >= 0) report.interference.push(
      'penetration ${report.penetration} mm exceeds the fit bound at tooth ${report.penetrationTooth}, wave generator ${report.penetrationAngle} rad');
    return report;
  }
}
