package tests;

import haxe.Int64;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.work.WorkSurface;
import robotkit.work.Polygon2;
import robotkit.work.Point2;
import robotkit.work.DeviationMap;
import robotkit.perception.PlaneFit;
import robotkit.perception.PlaneEstimate;
import robotkit.perception.SeededRandom;
import robotkit.perception.SimulatedSurfaceScanner;
import robotkit.perception.SurfaceRegistration;

/** M7 acceptance tests for robotkit.perception + robotkit.work: PlaneFit, SurfaceRegistration, DeviationMap. */
class PerceptionTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testRegistrationRecoversOffsetAndYaw();
    testRansacIgnoresOutliers();
    testOversizeCorrectionRejected();
    testDeviationMapReflectsBow();
    Sys.println('RobotKit perception tests passed ($assertions assertions)');
    return assertions;
  }

  static function testRegistrationRecoversOffsetAndYaw():Void {
    var design = buildWallSurface();
    var injectedOffset = 0.014;
    var injectedYaw = degToRad(0.3);
    var trueTransform = new Transform3(new Vec3(0.0, 0.0, injectedOffset),
      Quat.fromAxisAngle(new Vec3(0.0, 1.0, 0.0), injectedYaw));

    // A modest bow (a much larger one is exercised in testDeviationMapReflectsBow)
    // keeps this test's 1mm/0.05deg recovery tolerance meaningful: the
    // scanner demeans the bow over every scanned point, but RANSAC's
    // consensus set can still drop the bow's most extreme corners, which
    // would otherwise re-bias the mean by more than 1mm.
    var cloud = SimulatedSurfaceScanner.scan(design, trueTransform, 0.002, 0.08, 0.0005, 42, Int64.ofInt(0));
    var result = SurfaceRegistration.register(design, cloud, 7, 500, 0.02);

    check(result.accepted, "Registration accepts a small, in-limit correction");
    check(approx(result.translationCorrection, injectedOffset, 0.001),
      'Registration recovers the injected offset within 1mm (got ${result.translationCorrection})');
    check(approx(result.rotationCorrectionRadians, injectedYaw, degToRad(0.05)),
      'Registration recovers the injected yaw within 0.05deg (got ${radToDeg(result.rotationCorrectionRadians)}deg)');
    check(result.registered != null, "An accepted registration returns a corrected work surface");
    var registered:WorkSurface = cast result.registered;
    check(registered.provenance.sourceKind == robotkit.work.SourceKind.Work,
      "The registered work surface's provenance is 'work'");
    check(registered.provenance.designElementId == design.provenance.designElementId,
      "The registered work surface retains the design element id");
  }

  static function testRansacIgnoresOutliers():Void {
    var rng = new SeededRandom(99);
    var points:Array<Vec3> = [];
    var trueOffset = 0.02;
    for (i in 0...80) {
      var x = -1.0 + (i % 10) * (2.0 / 9.0);
      var y = -1.0 + Std.int(i / 10) * (2.0 / 4.0);
      var z = trueOffset + (rng.next() - 0.5) * 0.002;
      points.push(new Vec3(x, y, z));
    }
    // 20 outliers among 100 total points == 20%.
    for (_ in 0...20) {
      points.push(new Vec3((rng.next() - 0.5) * 2.0, (rng.next() - 0.5) * 2.0, (rng.next() - 0.5) * 4.0 + 2.0));
    }

    var estimate = PlaneFit.fitRansac(points, 5, 500, 0.01, new Vec3(0.0, 0.0, 1.0));
    check(angleBetween(estimate.normal, new Vec3(0.0, 0.0, 1.0)) < 0.02,
      "RANSAC recovers the inlier plane's normal despite 20% outliers");
    check(approx(estimate.offset, trueOffset, 0.003),
      'RANSAC recovers the inlier plane offset despite 20% outliers (got ${estimate.offset})');
    check(estimate.inlierCount >= 76, "RANSAC consensus set covers nearly all 80 inliers, not the 20 outliers");
  }

  static function testOversizeCorrectionRejected():Void {
    var design = buildWallSurface();
    var trueTransform = new Transform3(new Vec3(0.0, 0.0, 0.5), Quat.identity());
    var cloud = SimulatedSurfaceScanner.scan(design, trueTransform, 0.0, 0.1, 0.001, 11, Int64.ofInt(0));
    var result = SurfaceRegistration.register(design, cloud, 3);

    check(!result.accepted, "Registration rejects a correction that exceeds the configured translation limit");
    check(result.registered == null, "A rejected registration returns no corrected work surface");
    check(result.rejectionReason != null, "A rejected registration explains why");
  }

  static function testDeviationMapReflectsBow():Void {
    var design = buildWallSurface();
    var cloud = SimulatedSurfaceScanner.scan(design, Transform3.identity(), 0.008, 0.08, 0.0005, 21, Int64.ofInt(0));
    var map = new DeviationMap(design, 0.2);
    map.addPoints(cloud.points);

    var bounds = design.boundary.bounds();
    var centerX = (bounds.minX + bounds.maxX) * 0.5;
    var centerY = (bounds.minY + bounds.maxY) * 0.5;
    var edgeX = bounds.minX + 0.1;
    var edgeY = centerY;

    check(map.hasSample(centerX, centerY), "Deviation map has a sample near the bow's center");
    check(map.deviationAt(centerX, centerY) > map.deviationAt(edgeX, edgeY),
      "Deviation map shows a larger deviation at the bow's center than near the boundary edge");
    check(map.maxAbsDeviation() > 0.002, "Deviation map's peak deviation reflects the injected bow amplitude");
  }

  // -- fixtures and helpers ---------------------------------------------

  static function buildWallSurface():WorkSurface {
    var boundary = new Polygon2([
      new Point2(0.0, 0.0), new Point2(2.0, 0.0), new Point2(2.0, 2.0), new Point2(0.0, 2.0)
    ]);
    return new WorkSurface("wall-face", "wall", Transform3.identity(), boundary);
  }

  static function degToRad(degrees:Float):Float return degrees * Math.PI / 180.0;
  static function radToDeg(radians:Float):Float return radians * 180.0 / Math.PI;

  static function angleBetween(a:Vec3, b:Vec3):Float {
    var cosAngle = a.normalized().dot(b.normalized());
    if (cosAngle > 1.0) cosAngle = 1.0;
    if (cosAngle < -1.0) cosAngle = -1.0;
    return Math.acos(cosAngle);
  }

  static function approx(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
