package tests;

import robotkit.work.HeightMap;
import robotkit.work.VolumeResult;
import robotkit.work.BucketSweep;
import robotkit.work.BucketSweepResult;
import robotkit.work.EarthworkRegion;
import robotkit.work.Point2;
import robotkit.work.Polygon2;

/** M11 acceptance tests for robotkit.work: HeightMap, EarthworkRegion, BucketSweep. */
class TerrainTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testBilinearSampleMatchesPlane();
    testVolumeOfKnownTrench();
    testBucketSweepRemovesExpectedVolume();
    testCellsAtGradeReported();
    Sys.println('RobotKit terrain tests passed ($assertions assertions)');
    return assertions;
  }

  static function testBilinearSampleMatchesPlane():Void {
    var cellSize = 0.25;
    var columns = 9, rows = 7;
    var elevation:Array<Float> = [];
    for (row in 0...rows) for (col in 0...columns) {
      var x = col * cellSize, y = row * cellSize;
      elevation.push(0.3 * x - 0.2 * y + 1.0);
    }
    var map = new HeightMap("ground", 0.0, 0.0, cellSize, columns, rows, elevation);
    var samples = [[0.4, 0.6], [1.9, 0.1], [0.05, 1.4], [1.99, 1.49]];
    for (sample in samples) {
      var expected = 0.3 * sample[0] - 0.2 * sample[1] + 1.0;
      var actual = map.bilinearSample(sample[0], sample[1]);
      check(approx(actual, expected, 1e-9), 'Bilinear sample at (${sample[0]}, ${sample[1]}) matches the planar field');
    }
  }

  static function testVolumeOfKnownTrench():Void {
    // A trench spans the full Y-extent of the grid, so there is no partial-cell
    // effect in Y; picking its X-boundaries exactly on grid columns makes the
    // grid's trapezoidal cell average exactly reproduce the geometric volume
    // of a trench whose true edges sit at the half-cell mark outside the last
    // full-depth column on each side (see ARCHITECTURE.md's M11 section) --
    // no discretization tolerance is needed.
    var cellSize = 0.1;
    var columns = 61, rows = 31; // 6m x 3m extent
    var existing = new HeightMap("ground", 0.0, 0.0, cellSize, columns, rows);
    var design = new HeightMap("ground", 0.0, 0.0, cellSize, columns, rows);
    var depth = 0.4;
    var c0 = 20, c1 = 39; // trench occupies columns [20, 39]
    for (row in 0...rows) for (col in c0...(c1 + 1)) design.setElevation(col, row, -depth);

    var volume = HeightMap.volumeBetween(existing, design);
    var expectedLength = (c1 - c0 + 1) * cellSize;
    var expectedWidth = (rows - 1) * cellSize;
    var expectedVolume = expectedLength * expectedWidth * depth;
    check(approx(volume.cut, expectedVolume, 1e-6), 'Trench cut volume matches the known geometric volume ($expectedVolume m^3)');
    check(approx(volume.fill, 0.0, 1e-9), "A pure cut trench reports zero fill volume");
    check(approx(volume.net(), expectedVolume, 1e-6), "Net volume equals cut minus (zero) fill");
  }

  static function testBucketSweepRemovesExpectedVolume():Void {
    var cellSize = 0.05;
    var columns = 41, rows = 21; // 2m x 1m extent
    var map = new HeightMap("ground", 0.0, 0.0, cellSize, columns, rows);
    var from = new Point2(0.5, 0.5);
    var to = new Point2(1.5, 0.5);
    var halfWidth = 0.3;
    var depth = 0.3;
    var result = BucketSweep.apply(map, from, to, halfWidth, -depth);

    check(result.verticesLowered > 0, "Bucket sweep lowers at least one vertex");
    check(result.removedVolume > 0.0, "Bucket sweep reports positive removed volume");

    var length = to.x - from.x;
    var capsuleArea = length * (2.0 * halfWidth) + Math.PI * halfWidth * halfWidth;
    var expectedVolume = capsuleArea * depth;
    var relativeError = Math.abs(result.removedVolume - expectedVolume) / expectedVolume;
    check(relativeError < 0.1,
      'Bucket sweep removed volume (${result.removedVolume}) approximates the swept capsule volume (${expectedVolume}) within 10%');

    // Sweeping again with the same edge height removes nothing further.
    var second = BucketSweep.apply(map, from, to, halfWidth, -depth);
    check(second.removedVolume == 0.0, "A repeated sweep at the same edge height removes no further material");
    check(second.verticesLowered == 0, "A repeated sweep at the same edge height lowers no further vertices");

    // Cells far from the sweep are untouched.
    check(map.elevationAt(0, 0) == 0.0, "A vertex far outside the swept capsule is untouched");
  }

  static function testCellsAtGradeReported():Void {
    var cellSize = 0.2;
    var columns = 10, rows = 10;
    var design = new HeightMap("ground", 0.0, 0.0, cellSize, columns, rows);
    var existingElevation:Array<Float> = [];
    for (row in 0...rows) for (col in 0...columns)
      existingElevation.push(((col + row) % 2 == 0) ? 0.001 : 0.05);
    var existing = new HeightMap("ground", 0.0, 0.0, cellSize, columns, rows, existingElevation);
    var region = new EarthworkRegion("pad", existing, design, [], 0.005);

    check(approx(region.gradeFraction(), 0.5, 1e-9), "Half the vertices are within grade tolerance");
    check(region.isAtGrade(0, 0), "An in-tolerance vertex is reported at grade");
    check(!region.isAtGrade(1, 0), "An out-of-tolerance vertex is not reported at grade");

    var worst = region.worstVertex();
    check(worst != null, "worstVertex reports a vertex still needing work");
    if (worst == null) throw "assertion failed: worstVertex unexpectedly null";
    check(approx(Math.abs(worst.delta), 0.05, 1e-9), "worstVertex reports the largest outstanding delta");

    // Excluding the one out-of-tolerance vertex's cell makes the whole region read as at grade.
    var wx = existing.worldX(1), wy = existing.worldY(0);
    var pad = cellSize * 0.5;
    var exclusion = new Polygon2([
      new Point2(wx - pad, wy - pad), new Point2(wx + pad, wy - pad),
      new Point2(wx + pad, wy + pad), new Point2(wx - pad, wy + pad)
    ]);
    var excludedRegion = new EarthworkRegion("pad-excluded", existing, design, [exclusion], 0.005);
    check(excludedRegion.isAtGrade(1, 0), "A vertex inside an exclusion polygon is reported at grade regardless of its delta");
  }

  static function approx(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
