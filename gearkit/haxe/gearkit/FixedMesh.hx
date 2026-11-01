package gearkit;

/** A flexspline meshing with a fixed circular spline (TOOTH_DESIGN_PLAN.md TD11): unloaded, rigid teeth, planar.
    For each wave-generator angle theta the flexspline teeth sit where `StrainWaveKinematics` puts them (plus the
    `MeshErrors`), then the whole tooth ring is turned rigidly about the axis by psi, positive in the drive direction
    (the direction the flexspline turns, opposite to the wave generator). The free range of psi is the interval where no
    point of any tooth is inside circular-spline material; its two ends are the first drive-flank contact (`drive`) and the
    first reverse-flank contact (`reverse`), and its length is the backlash.
    **Root finding.** Each end is the root of a minimum signed clearance against psi. Points whose clearance falls as psi grows
    form the drive set (rotation closes their gap), the others the reverse set; each set's minimum clearance is monotone in psi
    over the search window, so the root is bracketed by doubling steps from psi = 0 and then bisected: no optimiser. Points
    that a rotation cannot move towards or away from the surface (contact normal nearly radial, e.g. tip land on tip land)
    are in neither set; their overlap is reported as `radialOverlap`. Points past the root circle are `rootBottoming`.
    The search window (a clearance band, mm) starts at 5 um, follows the previous angle's result, triples on a miss and is
    capped at 0.2 mm, after which an end is `unresolved`.
    **Periodicity.** For errors that are the same for every tooth the ring state repeats, up to relabelling the teeth and a
    turn by one circular-spline pitch, every `2 pi / circularTeeth` of wave-generator angle, so that span is one full period
    and is the default. Errors tied to a tooth (`OffsetFrame.Flexspline` runout) need the span given explicitly. */
class FixedMesh {
  static final RADIAL = 0.1;
  static final MINBAND = 0.005;
  static final MAXBAND = 0.2;

  /** `contactOffset` mm is added to every clearance: contact is declared at clearance `-contactOffset`. Pass the circular
      spline's `deviationBound` to mesh a spline against the flexspline it was derived from. A tooth counts as in contact
      when its closest point is within `contactTolerance` mm of contact. `sectionLimit` keeps only teeth within that section
      angle of the major axis (default: all). It exists for the rigid planetary limit, where a 200/202 involute pair
      interferes away from the mesh zone, as every full-depth involute pair with a tooth difference of two does. */
  public static function run(
    spline:CircularSpline,
    pair:StrainWavePair,
    errors:MeshErrors,
    thetaSamples:Int = 24,
    thetaFrom:Float = 0.0,
    thetaSpan:Float = -1.0,
    pointSamples:Int = 64,
    contactTolerance:Float = 0.001,
    contactOffset:Float = 0.0,
    sectionLimit:Float = 2.0
  ):FixedMeshReport {
    var input = pair.input, zf = input.flexTeeth, zc = input.circularTeeth;
    if (spline.teeth != zc) throw "circular spline tooth count differs from the pair's";
    if (thetaSamples < 1) throw "need at least one wave-generator angle";
    var report = new FixedMeshReport();
    report.name = input.name;
    report.spline = spline.label;
    report.contactTolerance = contactTolerance;
    report.contactOffset = contactOffset;
    var kin = pair.kinematics;
    if (errors.camAmplitude != 0) {
      var rn = kin.deformation.neutralRadius;
      kin = new StrainWaveKinematics(zf, zc, new WaveDeformation(Cams.make(input.cam, rn, input.w0() + errors.camAmplitude), rn));
    }
    var pts = pair.boundary(pointSamples), n = pts.length;
    var regions = [for (p in pts) pair.profile.regionAt(p.radius())];
    var span = thetaSpan > 0 ? thetaSpan : 2 * Math.PI / zc;
    var rootR = spline.rootRadius, tipR = spline.tipRadius;
    var qx = [for (_ in 0...zf * n) 0.0], qy = [for (_ in 0...zf * n) 0.0], qr = [for (_ in 0...zf * n) 0.0];
    var active = [for (_ in 0...zf) true];
    var nextBand = MINBAND;
    var flankSum = 0, tipSum = 0, firstFlank = 0, firstTip = 0, resolved = 0;
    for (s in 0...thetaSamples) {
      var theta = thetaFrom + span * s / thetaSamples;
      // Whole-ring translation errors at this angle.
      var ox = 0.0, oy = 0.0;
      for (o in errors.offsets) {
        var a = o.phase + switch o.frame {
          case Fixed: 0.0;
          case Carrier: theta;
          case Flexspline: kin.meanRotation(theta);
        };
        ox -= o.magnitude * Math.sin(a);
        oy += o.magnitude * Math.cos(a);
      }
      var rc = Math.cos(-errors.toothRotation), rs = Math.sin(-errors.toothRotation), over = 0.0;
      for (j in 0...zf) {
        var delta = kin.sectionAngle(j, theta);
        active[j] = Math.abs(delta - Math.PI * Math.round(delta / Math.PI)) <= sectionLimit;
        if (!active[j]) continue;
        var pose = kin.pose(j, theta), c = Math.cos(pose.angle), sn = Math.sin(pose.angle);
        for (i in 0...n) {
          var x = c * pts[i].x - sn * pts[i].y + pose.tx + ox, y = sn * pts[i].x + c * pts[i].y + pose.ty + oy;
          var k = j * n + i, x2 = rc * x - rs * y, y2 = rs * x + rc * y;
          qx[k] = x2;
          qy[k] = y2;
          qr[k] = Math.sqrt(x2 * x2 + y2 * y2);
          if (qr[k] - rootR > over) over = qr[k] - rootR;
        }
      }
      report.rootBottoming = Math.max(report.rootBottoming, over);
      var found = false, band = nextBand;
      while (true) {
        var drive = new PointSet();
        var reverse = new PointSet();
        var radial = 0.0;
        for (k in 0...zf * n) {
          if (!active[Std.int(k / n)]) continue;
          if (qr[k] > rootR || qr[k] < tipR - band) continue;
          var p = new Point2(qx[k], qy[k]);
          var c0 = spline.clearance(p) + contactOffset;
          if (c0 > band) continue;
          var eps = 2e-7, lever = RADIAL * qr[k];
          var cd = at(spline, qx[k], qy[k], 0, 1) + contactOffset, cl = at(spline, qx[k], qy[k], 0, -1) + contactOffset;
          var sd = (at(spline, qx[k], qy[k], eps, 1) - at(spline, qx[k], qy[k], -eps, 1)) / (2 * eps);
          var sl = (at(spline, qx[k], qy[k], eps, -1) - at(spline, qx[k], qy[k], -eps, -1)) / (2 * eps);
          if (c0 <= 0 && (cd <= cl ? -sd < lever : sl < lever)) radial = Math.max(radial, -c0);
          if (cd <= band && -sd >= lever) drive.add(qx[k], qy[k], Std.int(k / n), k % n, 1);
          if (cl <= band && sl >= lever) reverse.add(qx[k], qy[k], Std.int(k / n), k % n, -1);
        }
        report.radialOverlap = Math.max(report.radialOverlap, radial);
        var window = band / (1.01 * rootR);
        var d = root(drive, spline, contactOffset, true, window, rootR);
        var r = root(reverse, spline, contactOffset, false, window, rootR);
        if (Math.isNaN(d) || Math.isNaN(r)) {
          if (band >= MAXBAND) break;
          band = Math.min(MAXBAND, 3 * band);
          continue;
        }
        found = true;
        nextBand = Math.max(MINBAND, 1.3 * Math.max(Math.abs(d), Math.abs(r)) * rootR + 0.001);
        report.thetas.push(theta);
        report.drive.push(d);
        report.reverse.push(r);
        report.backlash.push(d - r);
        var dc = contacts(drive, spline, d, contactOffset, contactTolerance, regions, kin, theta, zf);
        report.driveContacts.push(dc);
        report.reverseContacts.push(contacts(reverse, spline, r, contactOffset, contactTolerance, regions, kin, theta, zf));
        var fl = 0, tp = 0, best = Math.POSITIVE_INFINITY, bestRegion = ToothRegion.Root;
        for (t in dc) {
          switch t.region {
            case Flank: fl++;
            case Tip: tp++;
            case Root:
          }
          if (t.clearance < best) {
            best = t.clearance;
            bestRegion = t.region;
          }
        }
        flankSum += fl;
        tipSum += tp;
        report.maxFlankContacts = Std.int(Math.max(report.maxFlankContacts, fl));
        report.maxTipContacts = Std.int(Math.max(report.maxTipContacts, tp));
        switch bestRegion {
          case Flank: firstFlank++;
          case Tip: firstTip++;
          case Root:
        }
        resolved++;
        break;
      }
      if (!found) report.unresolved++;
    }
    if (resolved > 0) {
      var sum = 0.0;
      for (d in report.drive) sum += d;
      report.meanDrive = sum / resolved;
      var lo = Math.POSITIVE_INFINITY, hi = Math.NEGATIVE_INFINITY, sq = 0.0;
      for (d in report.drive) {
        var e = d - report.meanDrive;
        report.transmissionError.push(e);
        lo = Math.min(lo, e);
        hi = Math.max(hi, e);
        sq += e * e;
      }
      report.tePeakToPeak = hi - lo;
      report.teRms = Math.sqrt(sq / resolved);
      var bs = 0.0;
      report.backlashMin = Math.POSITIVE_INFINITY;
      report.backlashMax = Math.NEGATIVE_INFINITY;
      for (b in report.backlash) {
        bs += b;
        report.backlashMin = Math.min(report.backlashMin, b);
        report.backlashMax = Math.max(report.backlashMax, b);
      }
      report.backlashMean = bs / resolved;
      report.meanFlankContacts = flankSum / resolved;
      report.meanTipContacts = tipSum / resolved;
      report.firstContactFlank = firstFlank / resolved;
      report.firstContactTip = firstTip / resolved;
    }
    var pitchRadius = input.module * zc / 2;
    if (report.rootBottoming > 1e-6) report.interference.push(
      'kappa or cam amplitude too large for this circular spline: flexspline tips reach ${report.rootBottoming} mm past its root circle');
    if (report.radialOverlap > 1e-6) report.interference.push(
      'tooth tips overlap by ${report.radialOverlap} mm where no rotation can clear them (kappa or cam amplitude too small for this circular spline)');
    if (resolved > 0 && report.backlashMin * pitchRadius < -(2 * spline.deviationBound + 1e-6)) report.interference.push(
      'the free rotation range is negative: ${-report.backlashMin * pitchRadius} mm at the circular-spline pitch circle of overlap at its worst');
    if (report.unresolved > 0) report.interference.push('${report.unresolved} of $thetaSamples angles have no contact angle within the search window');
    return report;
  }

  /** Clearance of the point (x, y) turned by -psi, i.e. the flexspline turned by psi in the drive direction, to the tooth
      on `side` (see `CircularSpline.sideClearance`). */
  static function at(spline:CircularSpline, x:Float, y:Float, psi:Float, side:Int):Float {
    var c = Math.cos(psi), s = Math.sin(psi);
    return spline.sideClearance(new Point2(x * c + y * s, -x * s + y * c), side);
  }

  /** Root of the monotone minimum clearance of `set` against psi, given only its sign (clear or not), starting from psi = 0:
      bracket by growing steps, then bisect. `decreasing` says which way the sign changes. NaN if there is no sign change
      within +-window. While bisecting, points that cannot reach the surface anywhere in the bracket (clearance above the
      bracket half-width times `radius`, plus 10 um) are dropped from the set; they cannot change the sign. */
  static function root(set:PointSet, spline:CircularSpline, offset:Float, decreasing:Bool, window:Float, radius:Float):Float {
    var pa = set.clear(spline, 0, offset, -1);
    var dir = pa == decreasing ? 1.0 : -1.0, step = 1e-6, a = 0.0, b = 0.0;
    while (true) {
      b = a + dir * step;
      var last = Math.abs(b) >= window;
      if (last) b = dir * window;
      if (set.clear(spline, b, offset, -1) != pa) break;
      if (last) return Math.NaN;
      a = b;
      step *= 4;
    }
    for (_ in 0...80) {
      var m = 0.5 * (a + b);
      if (set.clear(spline, m, offset, 0.5 * Math.abs(b - a) * radius * 1.01 + 0.01) == pa) a = m;
      else b = m;
      if (Math.abs(b - a) < 1e-10) break;
    }
    return 0.5 * (a + b);
  }

  static function contacts(
    set:PointSet,
    spline:CircularSpline,
    psi:Float,
    offset:Float,
    tolerance:Float,
    regions:Array<ToothRegion>,
    kin:StrainWaveKinematics,
    theta:Float,
    zf:Int
  ):Array<ToothContact> {
    var best = [for (_ in 0...zf) Math.POSITIVE_INFINITY], where = [for (_ in 0...zf) -1];
    for (k in 0...set.length()) {
      var c = at(spline, set.x[k], set.y[k], psi, set.side[k]) + offset, t = set.tooth[k];
      if (c < best[t]) {
        best[t] = c;
        where[t] = set.index[k];
      }
    }
    var out:Array<ToothContact> = [];
    for (t in 0...zf) {
      if (where[t] < 0 || best[t] > tolerance) continue;
      var delta = kin.sectionAngle(t, theta);
      out.push(new ToothContact(t, regions[where[t]], best[t], delta - Math.PI * Math.round(delta / Math.PI)));
    }
    out.sort((a, b) -> a.tooth - b.tooth);
    return out;
  }
}

private class PointSet {
  public var x:Array<Float>;
  public var y:Array<Float>;
  public var tooth:Array<Int>;
  public var index:Array<Int>;
  public var side:Array<Int>;
  var offender:Int = 0;

  public function new() {
    x = [];
    y = [];
    tooth = [];
    index = [];
    side = [];
  }

  public function add(px:Float, py:Float, t:Int, i:Int, sd:Int):Void {
    x.push(px);
    y.push(py);
    tooth.push(t);
    index.push(i);
    side.push(sd);
  }

  public function length():Int return x.length;

  /** True when every point is in clear space (clearance plus offset above zero) with the ring turned by psi. The point
      that failed last time is tried first, so a failing test usually costs one clearance. With `cull` >= 0, a passing test
      also drops the points whose clearance exceeds `cull`. */
  public function clear(spline:CircularSpline, psi:Float, offset:Float, cull:Float):Bool {
    if (x.length == 0) return true;
    if (offender < x.length && FixedMesh.at(spline, x[offender], y[offender], psi, side[offender]) + offset <= 0) return false;
    var keep:Array<Int> = [];
    for (k in 0...x.length) {
      var c = FixedMesh.at(spline, x[k], y[k], psi, side[k]) + offset;
      if (c <= 0) {
        offender = k;
        return false;
      }
      if (cull < 0 || c <= cull) keep.push(k);
    }
    if (keep.length < x.length) {
      x = [for (k in keep) x[k]];
      y = [for (k in keep) y[k]];
      tooth = [for (k in keep) tooth[k]];
      index = [for (k in keep) index[k]];
      side = [for (k in keep) side[k]];
      offender = 0;
    }
    return true;
  }
}
