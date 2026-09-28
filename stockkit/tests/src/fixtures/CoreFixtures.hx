package fixtures;

import camkit.CamContour;
import camkit.CamJob;
import cadkit.modeling.Part;
import cnckit.CncTool;
import cnckit.ir.CncGeometry;
import cnckit.ir.CncPoint;
import cnckit.parse.CncSpan;
import cnckit.tool.CutterProfile;
import oracle.ExactOracle;
import stockkit.CutMove;
import stockkit.CutMoves;
import stockkit.Stock;
import stockkit.StockGrid;

/**
  StockKit core through its Haxe API: moves the exact oracle refuses (ramps,
  helices, CamKit's ramped pocket entry) against the sampled reference, stock
  from CadKit meshes, and provenance. Oracle-backed comparisons run inside
  `OracleFixtures` and `ChainFixtures`, which already hold exact stock.
**/
class CoreFixtures {
  static inline final STOCK_X = 0.04;
  static inline final STOCK_Y = 0.02;
  static inline final STOCK_Z = 0.01;
  static inline final SAMPLES = 1500;

  public static function run():Void {
    ramps();
    helices();
    rampedPocket();
    meshStock();
    provenance();
  }

  static function ramps():Void {
    var r = 0.003;
    var ramp:CncGeometry = Line(new CncPoint(0.008, 0.007, 0.001),
      new CncPoint(0.032, 0.013, -0.004));
    for (tool in [
      {name: "flat", profile: CutterProfile.flat(2 * r, 0.02)},
      {name: "ball", profile: CutterProfile.ball(2 * r, 0.02)},
      {name: "bull-nose", profile: CutterProfile.bullNose(2 * r, 0.001, 0.02)},
      {name: "V-bit", profile: CutterProfile.vee(2 * r, Math.PI / 2, 0.02)},
      {name: "tapered ball", profile: CutterProfile.taperedBall(0.002, 10 * Math.PI / 180, 2 * r, 0.02)}
    ])
      sampled(tool.profile, [ramp], '${tool.name} ramp');
    // A ramp whose floor rises: the tool climbs out of the stock.
    sampled(CutterProfile.ball(2 * r, 0.02), [Line(new CncPoint(0.03, 0.012, -0.005),
      new CncPoint(0.012, 0.008, 0.002))], "ball climbing ramp");
  }

  static function helices():Void {
    var r = 0.003;
    function helix(radius:Float, turns:Float):CncGeometry
      return Circular(new CncPoint(0.02, 0.01, 0.001), radius, 0.3, -2 * Math.PI * turns, XY, -0.004);
    sampled(CutterProfile.flat(2 * r, 0.02), [helix(0.002, 2)], "flat helix inside its radius");
    sampled(CutterProfile.ball(2 * r, 0.02), [helix(0.004, 1.5)], "ball helix wider than its radius");
    sampled(CutterProfile.bullNose(2 * r, 0.001, 0.02), [helix(0.0025, 3)], "bull-nose helix");
  }

  /**
    CamKit's pocket at a depth its ramp fits: the entry moves in XY and Z
    together, which the OCCT oracle cannot build.
  **/
  static function rampedPocket():Void {
    var contour = new CamContour([
      new CncPoint(0.01, 0.005, 0), new CncPoint(0.03, 0.005, 0),
      new CncPoint(0.03, 0.015, 0), new CncPoint(0.01, 0.015, 0)
    ]);
    var tool = CncTool.shaped(2, 0.0, CutterProfile.flat(0.002, 0.02));
    var program = new CamJob(0.005, 12000, new CncPoint(0, 0, 0.005))
      .pocket(contour, tool, -0.001, 0.01, 0.0015, 0.002)
      .finish();
    var moves = CutMoves.fromOps(program.ops, program.tool);
    var ramped = false;
    for (move in moves) switch move.motion {
      case Path(Line(start, end)):
        if (start.z != end.z && (start.x != end.x || start.y != end.y)) ramped = true;
      case _:
    }
    Assert.check(ramped, "CamKit ramps into this pocket");
    CoreComparison.againstSampled(moves, 0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0, SAMPLES,
      "ramped CAM pocket");
  }

  /** Stock cast from CadKit tessellations matches the analytic solids. */
  static function meshStock():Void {
    var grid = StockGrid.covering(-0.0198, -0.0098, 0.0198, 0.0098, 0.0004);
    var part = Part.box(0.03, 0.015, 0.01, Center, Center, Min);
    var mesh = part.shape.tessellate(1e-5, 0.1);
    part.close();
    var fromMesh = Stock.fromMesh(grid, mesh);
    var box = Stock.box(grid, -0.015, -0.0075, 0, 0.015, 0.0075, 0.01);
    Assert.near(fromMesh.intervalCount(), box.intervalCount(), "box mesh covers the box's rays", 0);
    Assert.near(fromMesh.volume(), box.volume(), "box mesh volume matches the box", 1e-15);
    var inside = fromMesh.ray(40, 20);
    Assert.check(inside.length == 1 && inside[0].lo == 0 && inside[0].hi == 0.01
      && inside[0].hiNormal[2] == 1 && inside[0].loSource == Stock.ORIGINAL,
      "box mesh ray keeps exact faces, normals and source");
    fromMesh.dispose();
    box.dispose();

    // A cylinder: rays inside the tessellated circle span its full height.
    var radius = 0.008;
    var cylinder = Part.cylinder(radius, 0.012);
    var cylinderMesh = cylinder.shape.tessellate(1e-6, 0.05);
    cylinder.close();
    var stock = Stock.fromMesh(grid, cylinderMesh);
    var rays = stock.rays(0, 0, grid.countX, grid.countY);
    var wrong = 0;
    for (j in 0...grid.countY)
      for (i in 0...grid.countX) {
        var d = Math.sqrt(grid.x(i) * grid.x(i) + grid.y(j) * grid.y(j));
        var ray = rays[j * grid.countX + i];
        // Chords of the tessellation lie at most the deflection inside the circle.
        if (d < radius - 2e-6) {
          if (ray.length != 1 || ray[0].lo != 0 || ray[0].hi != 0.012) wrong++;
        } else if (d > radius) {
          if (ray.length != 0) wrong++;
        }
      }
    Assert.check(wrong == 0, 'cylinder mesh rays ($wrong wrong)');
    stock.dispose();
  }

  /** Surfaces remember the move that made them, and rapids through stock are reported. */
  static function provenance():Void {
    var tool = CncTool.shaped(1, 0.0, CutterProfile.flat(0.006, 0.02));
    var span = new CncSpan(1, 1, 0);
    var moves = [
      new CutMove(tool, Path(Line(new CncPoint(0.01, 0.01, -0.002), new CncPoint(0.03, 0.01, -0.002))),
        false, 0, span),
      new CutMove(tool, Path(Line(new CncPoint(0.02, 0.002, -0.001), new CncPoint(0.02, 0.018, -0.001))),
        true, 1, span)
    ];
    var grid = StockGrid.covering(0, 0, STOCK_X, STOCK_Y, 0.0005);
    var stock = Stock.box(grid, 0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0);
    var report = stock.cut(moves);
    Assert.check(report.removed.length == 2 && report.removed[0] > 0 && report.removed[1] > 0,
      "both moves remove material");
    Assert.check(report.rapidContacts.length == 1 && report.rapidContacts[0] == 1,
      "the rapid through stock is reported");
    var floor = stock.ray(40, 20); // (0.02, 0.01): the slot, then the rapid above it
    Assert.check(floor.length == 1 && floor[0].hi == -0.002 && floor[0].hiSource == 0
      && stock.history[floor[0].hiSource] == moves[0], "slot floor points back at its move");
    var crossing = stock.ray(40, 8); // (0.02, 0.004): only the rapid
    Assert.check(crossing.length == 1 && crossing[0].hi == -0.001 && crossing[0].hiSource == 1,
      "rapid's cut points back at the rapid");
    var untouched = stock.ray(2, 2);
    Assert.check(untouched[0].hiSource == Stock.ORIGINAL, "untouched stock keeps its original surface");
    // The same moves on one thread give the same stock.
    var single = Stock.box(grid, 0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0);
    single.setThreads(1);
    var again = single.cut(moves);
    Assert.check(again.removed[0] == report.removed[0] && again.removed[1] == report.removed[1],
      "one thread removes the same volumes");
    Assert.check(single.volume() == stock.volume(), "one thread leaves the same stock");
    single.dispose();
    // Cutting again appends to the history.
    stock.cut([moves[0]]);
    Assert.check(stock.history.length == 3, "history grows with every cut");
    stock.dispose();
  }

  static function sampled(profile:CutterProfile, geometry:Array<CncGeometry>, label:String):Void
    CoreComparison.againstSampled(ExactOracle.pathMoves(profile, geometry), 0, 0, -STOCK_Z,
      STOCK_X, STOCK_Y, 0, SAMPLES, label);
}
