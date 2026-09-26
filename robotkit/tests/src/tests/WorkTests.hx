package tests;

import robotkit.spatial.Transform3;
import robotkit.spatial.FrameTree3;
import robotkit.work.Point2;
import robotkit.work.Polygon2;
import robotkit.work.WorkSurface;
import robotkit.work.RasterToolpathGenerator;
import robotkit.work.CoverageMap;
import robotkit.process.CartesianTrajectory;

/** M5 acceptance tests for robotkit.work: WorkSurface, RasterToolpathGenerator, CoverageMap. */
class WorkTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testPolygonScanlineAndContains();
    testWorkSurfaceFrameRegistration();
    testRasterCoversWallWithDoorExclusion();
    testRasterOffsetsRotatedAndConcaveExclusions();
    testCoverageFromSimulatedExecutionMatchesPlanned();
    Sys.println('RobotKit work tests passed ($assertions assertions)');
    return assertions;
  }

  static function testPolygonScanlineAndContains():Void {
    var rectangle = rect(0.0, 0.0, 2.0, 1.0);
    var intervals = rectangle.scanlineIntervals(0.5);
    check(intervals.length == 1, "Scanline through a rectangle yields one interval");
    check(approx(intervals[0][0], 0.0, 1e-9) && approx(intervals[0][1], 2.0, 1e-9),
      "Scanline interval matches the rectangle's horizontal extent");
    check(rectangle.contains(new Point2(1.0, 0.5)), "Point inside the rectangle is contained");
    check(!rectangle.contains(new Point2(3.0, 0.5)), "Point outside the rectangle is not contained");
    check(approx(rectangle.area(), 2.0, 1e-9), "Rectangle area matches width times height");
  }

  static function testWorkSurfaceFrameRegistration():Void {
    var surface = new WorkSurface("wall-1", "wall-1/panel", Transform3.identity(), rect(0.0, 0.0, 3.0, 2.5));
    var tree = new FrameTree3();
    tree.add(surface.frameEdge());
    var lookup = tree.lookup("wall-1/panel", surface.surfaceFrameId);
    check(approx(lookup.translation.norm(), 0.0, 1e-9), "Identity frame_T_surface registers cleanly into a FrameTree3");
    check(surface.provenance.designElementId == "wall-1", "Default provenance uses the surface id as the design element id");
  }

  static function testRasterCoversWallWithDoorExclusion():Void {
    var boundary = rect(0.0, 0.0, 3.0, 2.5);
    var door = rect(1.0, 0.0, 0.9, 2.1);
    var surface = new WorkSurface("wall-2", "wall-2/panel", Transform3.identity(), boundary, [door]);

    var toolWidth = 0.1, overlap = 0.4, standoff = 0.05, feedRate = 0.1, leadInOut = 0.05;
    var toolpath = RasterToolpathGenerator.generate(surface, toolWidth, overlap, standoff, feedRate, leadInOut);
    check(toolpath.frameId == surface.surfaceFrameId, "Generated toolpath is expressed in the surface's own local frame");

    var coverage = new CoverageMap(surface, 0.02);
    sweepProcessMoves(coverage, toolpath, toolWidth * 0.5, 0.01);

    check(coverage.coverageFraction() >= 0.99, 'Raster covers at least 99% of the allowed wall area (got ${coverage.coverageFraction()})');
    check(coverage.exclusionCoverageFraction() <= 0.0001,
      'Raster covers none of the door exclusion (got ${coverage.exclusionCoverageFraction()})');
  }

  static function testCoverageFromSimulatedExecutionMatchesPlanned():Void {
    var boundary = rect(0.0, 0.0, 1.0, 0.6);
    var notch = rect(0.4, 0.0, 0.2, 0.2);
    var surface = new WorkSurface("wall-3", "wall-3/panel", Transform3.identity(), boundary, [notch]);

    var toolWidth = 0.06, overlap = 0.4, standoff = 0.05, feedRate = 0.2, leadInOut = 0.03;
    var toolpath = RasterToolpathGenerator.generate(surface, toolWidth, overlap, standoff, feedRate, leadInOut);

    var planned = new CoverageMap(surface, 0.01);
    sweepProcessMoves(planned, toolpath, toolWidth * 0.5, 0.005);

    var simulated = new CoverageMap(surface, 0.01);
    var trajectory = CartesianTrajectory.build(toolpath, 0.5, 0.05);
    var i = 0;
    while (i < trajectory.samples.length - 1) {
      var sample = trajectory.samples[i];
      var next = trajectory.samples[i + 1];
      // Only sweep within one segment's own samples: a segment-boundary pair
      // carries the departing segment's processOn flag but the arriving
      // segment's (possibly very different) position.
      if (sample.processOn && sample.segmentIndex == next.segmentIndex) {
        var from = new Point2(sample.work_T_tcp.translation.x, sample.work_T_tcp.translation.y);
        var to = new Point2(next.work_T_tcp.translation.x, next.work_T_tcp.translation.y);
        simulated.markSweep(from, to, toolWidth * 0.5, 0.005);
      }
      i++;
    }

    check(planned.coverageFraction() >= 0.99, 'Planned coverage over the small surface is at least 99% (got ${planned.coverageFraction()})');
    check(Math.abs(simulated.coverageFraction() - planned.coverageFraction()) < 0.01,
      'Simulated-execution coverage (${simulated.coverageFraction()}) matches planned coverage (${planned.coverageFraction()})');
    check(planned.exclusionCoverageFraction() <= 0.0001, "Planned coverage avoids the exclusion");
    check(simulated.exclusionCoverageFraction() <= 0.0001, "Simulated execution also avoids the exclusion");
  }

  static function testRasterOffsetsRotatedAndConcaveExclusions():Void {
    var diamond = new Polygon2([
      new Point2(1.5, 0.85), new Point2(2.15, 1.5),
      new Point2(1.5, 2.15), new Point2(0.85, 1.5)
    ]);
    var diamondSurface = new WorkSurface("rotated-exclusion", "panel", Transform3.identity(),
      rect(0.0, 0.0, 3.0, 3.0), [diamond]);
    assertRasterAvoidsExclusion(diamondSurface,
      "Raster offsets a 45-degree exclusion without process-on contact");

    var concave = new Polygon2([
      new Point2(0.8, 0.8), new Point2(2.2, 0.8), new Point2(2.2, 1.3),
      new Point2(1.3, 1.3), new Point2(1.3, 2.2), new Point2(0.8, 2.2)
    ]);
    var concaveSurface = new WorkSurface("concave-exclusion", "panel", Transform3.identity(),
      rect(0.0, 0.0, 3.0, 3.0), [concave]);
    assertRasterAvoidsExclusion(concaveSurface,
      "Raster offsets an L-shaped exclusion without process-on contact");
  }

  static function assertRasterAvoidsExclusion(surface:WorkSurface, label:String):Void {
    var toolWidth = 0.2;
    var toolpath = RasterToolpathGenerator.generate(surface, toolWidth, 0.4, 0.05, 0.2, 0.03);
    var coverage = new CoverageMap(surface, 0.025);
    sweepProcessMoves(coverage, toolpath, toolWidth * 0.5, 0.005);
    check(coverage.coverageFraction() >= 0.98,
      '$label while covering the allowed area (got ${coverage.coverageFraction()})');
    check(coverage.exclusionCoverageFraction() <= 0.0001,
      '$label (got ${coverage.exclusionCoverageFraction()})');
  }

  // -- helpers -----------------------------------------------------------

  static function sweepProcessMoves(coverage:CoverageMap, toolpath:robotkit.process.Toolpath, radius:Float, stepSize:Float):Void {
    for (i in 0...(toolpath.points.length - 1)) if (toolpath.points[i].processOn) {
      var from = toolpath.points[i].work_T_tcp.translation;
      var to = toolpath.points[i + 1].work_T_tcp.translation;
      coverage.markSweep(new Point2(from.x, from.y), new Point2(to.x, to.y), radius, stepSize);
    }
  }

  static function rect(x:Float, y:Float, width:Float, height:Float):Polygon2 return new Polygon2([
    new Point2(x, y), new Point2(x + width, y), new Point2(x + width, y + height), new Point2(x, y + height)
  ]);

  static function approx(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
