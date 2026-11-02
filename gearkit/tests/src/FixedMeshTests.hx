import gearkit.CircularSpline;
import gearkit.FixedMesh;
import gearkit.FixedMeshReport;
import gearkit.FlexOffset;
import gearkit.GearSpec;
import gearkit.Involute;
import gearkit.MeshErrors;
import gearkit.StrainWaveInput;
import gearkit.StrainWavePair;

/** TD11: a flexspline meshing a fixed circular spline. Validation by independent checks: the rigid involute limit against the
    analytic backlash, a known rotation, a known centre-distance change and a known runout, and (labelled, and the only place
    zero is expected by construction) a circular spline derived from the case it is run against. */
class FixedMeshTests {
  static var assertions = 0;
  static final ALPHA = 20 * Math.PI / 180;
  static final M = 0.3;
  static final ARC = FixedMeshReport.ARCSEC;
  static var rigidTe = 0.0;
  static var rigidBacklash = 0.0;
  static var selfBound = 0.0;
  static var selfTe = 0.0;
  static var selfBacklash = 0.0;
  static var runoutRatio = 0.0;
  static var radialRatio = 0.0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }
  static function near(a:Float, b:Float, tolerance:Float, message:String):Void check(
    Math.isFinite(a) && Math.abs(a - b) <= tolerance, '$message: $a versus $b (tol $tolerance)');

  static function f(x:Float, digits:Int = 3):String {
    var k = Math.pow(10, digits);
    return Std.string(Math.round(x * k) / k);
  }

  static function d1(kappa:Float = 1.0, cam:String = "cosine"):StrainWavePair return new StrainWavePair(
    new StrainWaveInput("D1", "involute", M, 200, 202, kappa, cam, ALPHA));

  public static function run():Void {
    rigidLimit();
    selfConsistency();
    knownRotation();
    periodicity();
    runout();
    centreDistance();
    interference();
    Sys.println('FixedMeshTests: $assertions assertions passed; rigid limit TE ${f(rigidTe * ARC, 4)}" and backlash error ${f(rigidBacklash * ARC, 4)}", '
      + 'self-consistency (by construction) bound ${f(selfBound * 1000, 2)} um, TE ${f(selfTe * ARC, 2)}", backlash ${f(selfBacklash * ARC, 2)}", '
      + 'runout TE / (2 e / r_b) ${f(runoutRatio, 4)}, centre-distance shift / (e sin a / r_b) ${f(radialRatio, 4)}');
  }

  /** (a) Rigid limit. A 200/202 involute pair at centre distance m (a rigid planet on a carrier: w0 -> 0, the flexspline
      centre orbits with the wave generator). Transmission error must vanish, and the backlash must be the analytic involute
      value: the circular play on the pitch circles, pi m - s_c - s_f, over the circular-spline pitch radius (a rotation of the
      flexspline about the axis moves the pitch point by R_c; the normal play j_t cos a acts through the line of action, at
      distance R_c cos a from the axis). Only the teeth near the mesh zone are used: every full-depth involute pair with a
      tooth difference of two interferes elsewhere, which is why the real gear deforms. */
  static function rigidLimit():Void {
    var rc = M * 202 / 2, sf = new GearSpec(M, 200, 8, new Involute(ALPHA)).pitchToothThickness();
    for (shift in [0.0, 0.1, 0.25]) {
      var cs = CircularSpline.involute("involute 202 shift " + shift, M, 202, ALPHA, shift);
      var pair = new StrainWavePair(new StrainWaveInput("rigid", "involute", M, 200, 202, 1e-7, "cosine", ALPHA));
      var r = FixedMesh.run(cs, pair, new MeshErrors(0, 0, [new FlexOffset(Carrier, M, 0)]), 6, 0.0, -1.0, 64, 0.001, 0.0, 0.3);
      var sc = cs.profile.gearSpec().pitchToothThickness();
      var analytic = (Math.PI * M - sc - sf) / rc;
      check(r.unresolved == 0, 'rigid shift $shift: every angle resolved');
      near(r.tePeakToPeak * ARC, 0, 0.01, 'rigid shift $shift: transmission error vanishes (arcsec)');
      near(r.backlashMean, analytic, 0.005 / ARC, 'rigid shift $shift: backlash equals the analytic involute value');
      near(r.backlashMax - r.backlashMin, 0, 0.01 / ARC, 'rigid shift $shift: backlash does not vary with the angle');
      check(r.maxFlankContacts > 0, 'rigid shift $shift: flank contact at the drive position');
      rigidTe = Math.max(rigidTe, r.tePeakToPeak);
      rigidBacklash = Math.max(rigidBacklash, Math.abs(r.backlashMean - analytic));
      if (shift == 0.1) Sys.println('    rigid limit, circular-spline shift 0.1: backlash ${f(r.backlashMeanArcsec(), 3)}" against analytic ${f(analytic * ARC, 3)}", TE ${f(r.tePeakToPeakArcsec(), 4)}"');
    }
  }

  /** (b) Self-consistency. The circular spline derived from a case, run with that same case: transmission error and backlash
      are zero up to the sweep's deviation bound. **Zero by construction**: the spline is the envelope of this very motion,
      so this checks the solver and the sampled spline, not the gear. The bound is a length; the angle it allows is the bound
      over the smallest contact lever the solver keeps (a tenth of the radius). */
  static function selfConsistency():Void {
    var pair = d1();
    var cs = CircularSpline.derived("D1 cosine kappa 1", pair);
    var r = FixedMesh.run(cs, pair, MeshErrors.none(), 6);
    var lever = 0.1 * 30.3, allowed = cs.deviationBound / lever;
    check(r.unresolved == 0, "self-consistency: every angle resolved");
    check(r.tePeakToPeak <= allowed, 'self-consistency: TE ${r.tePeakToPeakArcsec()}" within the sweep bound ${allowed * ARC}"');
    for (k in 0...r.drive.length) {
      check(Math.abs(r.drive[k]) <= allowed && Math.abs(r.reverse[k]) <= allowed, "self-consistency: both contact angles within the sweep bound");
    }
    check(Math.abs(r.backlashMean) <= 2 * allowed, 'self-consistency: backlash ${r.backlashMeanArcsec()}" within the sweep bound');
    // Independent of the solver: no flexspline point at rest (rotation 0) is inside the spline by more than the bound.
    var kin = pair.kinematics, pts = pair.boundary(64), worst = 0.0;
    for (s in 0...6) for (j in 0...200) {
      var pose = kin.pose(j, Math.PI / 101 * s / 6);
      for (q in pts) {
        var x = pose.apply(q);
        if (x.radius() > cs.tipRadius && x.radius() < cs.rootRadius) worst = Math.min(worst, cs.clearance(x));
      }
    }
    check(worst >= -cs.deviationBound, 'self-consistency: deepest overlap ${-worst} mm within the sweep bound ${cs.deviationBound}');
    check(r.meanFlankContacts + r.meanTipContacts > 0, "self-consistency: contacts are recorded");
    selfBound = cs.deviationBound;
    selfTe = r.tePeakToPeak;
    selfBacklash = Math.abs(r.backlashMean);
  }

  /** (c) A rigid rotation of the teeth by d0 shifts the contact angles by exactly -d0 (the correction that puts the teeth back),
      and leaves the backlash alone. With a cam amplitude error so that the case is not the trivial one. */
  static function knownRotation():Void {
    var pair = d1(1.05);
    var cs = CircularSpline.derived("D1 cosine kappa 1", d1());
    var base = FixedMesh.run(cs, pair, new MeshErrors(0.001), 4);
    for (d0 in [3e-4, -2e-4]) {
      var turned = FixedMesh.run(cs, pair, new MeshErrors(0.001, d0), 4);
      check(base.unresolved == 0 && turned.unresolved == 0, "rotation: every angle resolved");
      for (k in 0...base.drive.length) {
        near(turned.drive[k], base.drive[k] - d0, 1e-8, 'rotation $d0: drive contact angle shifts by -d0');
        near(turned.reverse[k], base.reverse[k] - d0, 1e-8, 'rotation $d0: reverse contact angle shifts by -d0');
      }
      near(turned.tePeakToPeak, base.tePeakToPeak, 1e-8, 'rotation $d0: transmission error unchanged');
      near(turned.backlashMean, base.backlashMean, 1e-8, 'rotation $d0: backlash unchanged');
    }
  }

  /** The ring state repeats, up to relabelling the teeth, every circular pitch of wave-generator angle: the result at theta
      and at theta + pitch agree, and so do angles spread over a whole output pitch (theta in [0, pi]) with their
      representative in the first circular pitch. */
  static function periodicity():Void {
    var pair = d1(1.05), cs = CircularSpline.derived("D1 cosine kappa 1", d1());
    var pitch = 2 * Math.PI / 202;
    var base = FixedMesh.run(cs, pair, MeshErrors.none(), 3, 0.0003);
    var later = FixedMesh.run(cs, pair, MeshErrors.none(), 3, 0.0003 + pitch);
    var far = FixedMesh.run(cs, pair, MeshErrors.none(), 3, 0.0003 + 57 * pitch);
    for (k in 0...3) {
      near(later.drive[k], base.drive[k], 1e-8, "drive angle repeats one circular pitch later");
      near(far.drive[k], base.drive[k], 1e-8, "drive angle repeats 57 circular pitches later (inside one output pitch)");
      near(far.reverse[k], base.reverse[k], 1e-8, "reverse angle repeats 57 circular pitches later");
    }
  }

  static function planet(offsets:Array<FlexOffset>, samples:Int, span:Float, cs:CircularSpline):FixedMeshReport {
    var pair = new StrainWavePair(new StrainWaveInput("rigid", "involute", M, 200, 202, 1e-7, "cosine", ALPHA));
    return FixedMesh.run(cs, pair, new MeshErrors(0, 0, [new FlexOffset(Carrier, M, 0)].concat(offsets)), samples, 0.0, span, 64, 0.001, 0.0, 0.3);
  }

  /** (c) Runout. An eccentricity e of the flexspline tooth ring about its own axis (rotating with it) moves the contact along
      the line of action by e cos(phase), the phase turning once per revolution of the flexspline relative to the wave
      generator (theta period 2 pi Zf / Zc). Through the lever r_b (the distance of the line of action from the axis) that is a
      sinusoidal transmission error of amplitude e / r_b, peak to peak 2 e / r_b. The radial component alone, e sin a, is
      the centre-distance case below. */
  static function runout():Void {
    var cs = CircularSpline.involute("involute 202", M, 202, ALPHA, 0.05), e = 0.005, rb = M * 202 / 2 * Math.cos(ALPHA);
    var samples = 24, r = planet([new FlexOffset(Flexspline, e, 0.7)], samples, 2 * Math.PI * 200 / 202, cs);
    check(r.unresolved == 0, "runout: every angle resolved");
    runoutRatio = r.tePeakToPeak * rb / (2 * e);
    near(runoutRatio, 1, 0.02, "runout: TE peak to peak is 2 e / r_b");
    // One cycle over the span: the first harmonic carries the signal.
    var a = 0.0, b = 0.0;
    for (k in 0...samples) {
      a += r.transmissionError[k] * Math.cos(2 * Math.PI * k / samples) * 2 / samples;
      b += r.transmissionError[k] * Math.sin(2 * Math.PI * k / samples) * 2 / samples;
    }
    near(Math.sqrt(a * a + b * b) * rb, e, 0.02 * e, "runout: first-harmonic amplitude is e / r_b");
    var resid = 0.0;
    for (k in 0...samples) {
      var model = a * Math.cos(2 * Math.PI * k / samples) + b * Math.sin(2 * Math.PI * k / samples);
      resid = Math.max(resid, Math.abs(r.transmissionError[k] - model));
    }
    check(resid * rb < 0.05 * e, "runout: the error is a single sinusoid once per relative revolution");
    // The radial part of the runout, e sin a per side, opens and closes the backlash by 4 e sin a / r_b over a revolution.
    near((r.backlashMax - r.backlashMin) * rb, 4 * e * Math.sin(ALPHA), 0.1 * 4 * e * Math.sin(ALPHA), "runout: backlash swings by 4 e sin a / r_b");
  }

  /** (c) Centre-distance error. The flexspline centre moved out by e along the wave generator's major axis (a radial
      eccentricity, fixed in the carrier) shifts the drive contact by e sin(a') / r_b, with a' the operating pressure angle at
      the mean centre distance (cos a' = (r_bc - r_bf) / a), and the reverse contact the opposite way. The first-order
      e sin(20 deg) / r_b is 6 % low at e = 5 um because the base circles are close and the pressure angle moves quickly. */
  static function centreDistance():Void {
    var cs = CircularSpline.involute("involute 202", M, 202, ALPHA, 0.05), e = 0.002;
    var rbc = M * 202 / 2 * Math.cos(ALPHA), rbf = M * 200 / 2 * Math.cos(ALPHA);
    var base = planet([], 3, -1.0, cs), out = planet([new FlexOffset(Carrier, e, 0)], 3, -1.0, cs);
    var aMid = M + e / 2, alpha = Math.acos((rbc - rbf) / aMid);
    var expected = e * Math.sin(alpha) / rbc;
    radialRatio = (base.meanDrive - out.meanDrive) / expected;
    near(radialRatio, 1, 0.015, "centre distance: drive contact moves by e sin(a') / r_b");
    check(out.reverse[0] > base.reverse[0], "centre distance: the reverse contact moves the other way");
    check(out.backlashMean < base.backlashMean, "centre distance: moving the flexspline into the space reduces the backlash");
    near((out.backlashMean - base.backlashMean) / (-2 * expected), 1, 0.015, "centre distance: backlash change is twice the drive shift");
  }

  /** Interference is reported, with where: a flexspline run into a spline derived for a smaller (kappa 1) deflection is deeper than it
      allows, and one with a smaller deflection leaves tooth tips overlapping. Both give a negative free range. */
  static function interference():Void {
    var cs = CircularSpline.derived("D1 cosine kappa 1", d1());
    var deep = FixedMesh.run(cs, d1(1.1), MeshErrors.none(), 4);
    check(deep.unresolved == 0 && deep.backlashMax < 0, "kappa 1.1 into the kappa 1 spline: negative free range");
    check(deep.interference.length > 0 && deep.interference[0].indexOf("free rotation range is negative") >= 0, "kappa 1.1 into the kappa 1 spline is reported");
    check(deep.meanFlankContacts > deep.meanTipContacts, "kappa 1.1 into the kappa 1 spline touches with flanks, not tips");
    var shallow = FixedMesh.run(cs, d1(0.9), MeshErrors.none(), 3);
    check(shallow.unresolved == 0 && shallow.backlashMax < 0, "kappa 0.9 into the kappa 1 spline: negative free range");
    check(shallow.interference.length > 0 && shallow.interference.join("|").indexOf("overlap") >= 0, "kappa 0.9: the overlap is reported");
  }
}
