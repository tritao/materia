package gearkit;

/** Meshing of a flexspline in its derived circular spline (TOOTH_DESIGN_PLAN.md TD-D4), unloaded.
    The state of the whole ring repeats, up to relabelling teeth, every circular pitch of
    wave-generator angle, so one pitch is scanned with every flexspline tooth. */
class MeshingAnalysis {
  /** `contactThreshold` mm: a tooth is in contact when some point is within it of the circular spline. It is
      flank-engaged when its closest point lies on the working flank, and tip edge contact when its closest point
      lies on the tip (tip blend and land), which is not engagement. A closest point on the root blend counts as
      neither. Per-region minimum clearances are reported over all points of each region.
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
    // Scan one circular pitch of wave-generator angle with every tooth. Each sample point is classified
    // by its region on the tooth: flank (the working involute or arc between the root join and the tip
    // blend start), tip (tip blend and land: edge contact) or root blend.
    var profile = pair.profile;
    var regions = [for (p in points) profile.regionAt(p.radius())];
    var pitch = 2 * Math.PI / zc, worst = 0.0;
    var flankSum = 0, flankMax = 0, tipSum = 0, tipMax = 0;
    for (s in 0...thetaSamples) {
      var theta = pitch * s / thetaSamples, flankEngaged = 0, tipContact = 0;
      for (j in 0...zf) {
        var delta = kin.sectionAngle(j, theta), pose = kin.pose(j, theta);
        var folded = delta - Math.PI * Math.round(delta / Math.PI);
        var flankMin = Math.POSITIVE_INFINITY, tipMin = Math.POSITIVE_INFINITY, rootMin = Math.POSITIVE_INFINITY;
        var closest = Math.POSITIVE_INFINITY, closestRegion = ToothRegion.Root;
        for (i in 0...points.length) {
          var q = pose.apply(points[i]), r = q.radius();
          var c:Float;
          if (r > rRoot) c = rRoot - r;
          else if (r < derivedTip || r > space.maxRadius()) continue;
          else c = space.clearance(q);
          if (c < closest) {
            closest = c;
            closestRegion = regions[i];
          }
          switch regions[i] {
            case Flank: if (c < flankMin) flankMin = c;
            case Tip: if (c < tipMin) tipMin = c;
            case Root: if (c < rootMin) rootMin = c;
          }
        }
        // A tooth within the threshold is attributed to the region of its closest point: the working flank,
        // or the tip (edge contact). Counting every flank point near the tip edge as flank engagement would
        // report the edge's own neighbourhood as engagement.
        if (closest <= contactThreshold) switch closestRegion {
          case Flank:
            flankEngaged++;
            report.contactZone = Math.max(report.contactZone, Math.abs(folded));
          case Tip: tipContact++;
          case Root:
        }
        report.minFlankClearance = Math.min(report.minFlankClearance, flankMin);
        report.minTipClearance = Math.min(report.minTipClearance, tipMin);
        report.minRootBlendClearance = Math.min(report.minRootBlendClearance, rootMin);
        var toothMin = Math.min(flankMin, Math.min(tipMin, rootMin));
        if (-toothMin > worst) {
          worst = -toothMin;
          report.penetrationTooth = j;
          report.penetrationAngle = theta;
          report.penetrationSection = folded;
        }
      }
      flankSum += flankEngaged;
      flankMax = Std.int(Math.max(flankMax, flankEngaged));
      tipSum += tipContact;
      tipMax = Std.int(Math.max(tipMax, tipContact));
    }
    report.minClearance = Math.min(report.minFlankClearance, Math.min(report.minTipClearance, report.minRootBlendClearance));
    report.penetration = worst;
    report.meanFlankEngaged = flankSum / thetaSamples;
    report.flankEngagedFraction = report.meanFlankEngaged / zf;
    report.maxFlankEngaged = flankMax;
    report.meanTipContact = tipSum / thetaSamples;
    report.tipContactFraction = report.meanTipContact / zf;
    report.maxTipContact = tipMax;
    // Interference, each naming the parameter responsible.
    if (report.rootClearance < 0) report.interference.push('kappa too large: flexspline tips reach ${-report.rootClearance} mm past the circular-spline root (tooth ${report.penetrationTooth}, wave generator ${report.penetrationAngle} rad)');
    if (report.minorTipClearance < -space.deviationBound) report.interference.push(
      'kappa too small (or tooth too tall): at the minor axis the flexspline tip is ${-report.minorTipClearance} mm inside the circular-spline tip circle, tooth meets tooth (circular-spline thickness there ${report.minToothThickness} mm)');
    if (report.penetration > space.deviationBound && report.rootClearance >= 0) report.interference.push(
      'penetration ${report.penetration} mm exceeds the fit bound at tooth ${report.penetrationTooth}, wave generator ${report.penetrationAngle} rad');
    return report;
  }
}
