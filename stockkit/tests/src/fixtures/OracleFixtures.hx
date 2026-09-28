package fixtures;

import camkit.CamContour;
import camkit.CamJob;
import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import toolpathkit.tool.Tool;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Point3;
import oracle.ExactOracle;
import stockkit.CutMove;
import stockkit.CutMoves;
import toolpathkit.tool.CutterProfile;

/**
  Reference cases for the simulator: each builds exact stock with the OCCT
  oracle and checks it against closed-form volumes and ray depths, so the
  oracle itself is trusted before simulated stock is compared with it.
**/
class OracleFixtures {
  static inline final STOCK_X = 0.04;
  static inline final STOCK_Y = 0.02;
  static inline final STOCK_Z = 0.01;
  static inline final VOLUME_TOLERANCE = 1e-7; // relative
  static inline final DEPTH_TOLERANCE = 1e-9; // metres

  public static function run():Void {
    var r = 0.003, length = 0.02;
    var a = new Point3(0.01, 0.01, 0), b = new Point3(0.03, 0.01, 0);
    function slot(depth:Float):PathGeometry
      return Line(new Point3(a.x, a.y, -depth), new Point3(b.x, b.y, -depth));

    // Flat end mill slot: stadium footprint times depth.
    var d = 0.002;
    var flat = removal(CutterProfile.flat(2 * r, 0.02), [slot(d)]);
    relative(flat.removed, (2 * r * length + Math.PI * r * r) * d,
      "flat slot removes a stadium prism");
    rays(flat.stock, new Vector(0.02, 0.01, 0.05), new Vector(0, 0, -1), 0.1,
      [{enter: 0.05 + d, exit: 0.05 + STOCK_Z}], "flat slot floor");
    rays(flat.stock, new Vector(-0.01, 0.01, -0.001), new Vector(1, 0, 0), 0.06,
      [{enter: 0.01, exit: 0.01 + a.x - r}, {enter: 0.01 + b.x + r, exit: 0.05}],
      "flat slot walls");
    flat.stock.close();

    // Ball end mill slot deeper than its radius.
    d = 0.004;
    var ballSection = Math.PI * r * r / 2 + 2 * r * (d - r);
    var ballTool = 2 / 3 * Math.PI * r * r * r + Math.PI * r * r * (d - r);
    var ball = removal(CutterProfile.ball(2 * r, 0.02), [slot(d)]);
    relative(ball.removed, ballSection * length + ballTool,
      "ball slot removes its section swept plus the tool");
    var offset = 0.002;
    rays(ball.stock, new Vector(0.02, 0.01 + offset, 0.05), new Vector(0, 0, -1), 0.1,
      [{enter: 0.05 + d - r + Math.sqrt(r * r - offset * offset), exit: 0.05 + STOCK_Z}],
      "ball slot floor follows the ball");
    ball.stock.close();

    // 90 degree V-bit engraving shallower than its cone.
    d = 0.002;
    var vee = removal(CutterProfile.vee(2 * r, Math.PI / 2, 0.02), [slot(d)]);
    relative(vee.removed, d * d * length + Math.PI * d * d * d / 3,
      "V groove removes a triangle swept plus a cone");
    vee.stock.close();

    // Bull-nose slot: integrate the profile numerically for the reference.
    var corner = 0.001;
    function bullRadius(h:Float):Float
      return h >= corner ? r
        : r - corner + Math.sqrt(corner * corner - (corner - h) * (corner - h));
    var bullSectionArea = integrate(h -> 2 * bullRadius(h), 0, d);
    var bullToolVolume = integrate(h -> Math.PI * bullRadius(h) * bullRadius(h), 0, d);
    var bull = removal(CutterProfile.bullNose(2 * r, corner, 0.02), [slot(d)]);
    relative(bull.removed, bullSectionArea * length + bullToolVolume,
      "bull-nose slot removes its section swept plus the tool");
    bull.stock.close();

    // Plunge from above the stock: only the part below the top is removed.
    var plunge = removal(CutterProfile.flat(2 * r, 0.02),
      [Line(new Point3(0.02, 0.01, 0.001), new Point3(0.02, 0.01, -0.003))]);
    relative(plunge.removed, Math.PI * r * r * 0.003, "plunge removes a cylinder");
    plunge.stock.close();

    // Half-circle arc with a ball mill: Pappus for the swept section.
    d = 0.004;
    var rho = 0.005;
    var arc = removal(CutterProfile.ball(2 * r, 0.02),
      [Arc(new Point3(0.02, 0.01, -d), rho, 0, Math.PI)]);
    relative(arc.removed, ballSection * rho * Math.PI + ballTool,
      "ball arc removes its section revolved plus the tool");
    rays(arc.stock, new Vector(0.02, 0.01 + rho, 0.05), new Vector(0, 0, -1), 0.1,
      [{enter: 0.05 + d, exit: 0.05 + STOCK_Z}], "ball arc floor at its apex");
    arc.stock.close();

    // Full circle: the section revolved all the way round, with no caps.
    var circle = removal(CutterProfile.ball(2 * r, 0.02),
      [Arc(new Point3(0.02, 0.01, -d), rho, Math.PI / 2, -2 * Math.PI)]);
    Assert.near(circle.removed / (ballSection * rho * 2 * Math.PI), 1.0,
      "full-circle arc removes its section revolved once", VOLUME_TOLERANCE);
    circle.stock.close();

    // Caps reaching round into the sweep are refused, not mis-checked.
    var refused = false;
    try ExactOracle.sweptSolid(CutterProfile.ball(2 * r, 0.02),
      Arc(new Point3(0.02, 0.01, -d), rho, 0, 1.9 * Math.PI)).close()
    catch (_:Dynamic) refused = true;
    Assert.check(refused, "an arc whose caps overlap its sweep is refused");

    camProgram();
  }

  /**
    A CamKit profile and pocket, cut exactly. This is the end-to-end reference
    for simulating real CAM output.
  **/
  static function camProgram():Void {
    var contour = new CamContour([
      new Point3(0.01, 0.005, 0), new Point3(0.03, 0.005, 0),
      new Point3(0.03, 0.015, 0), new Point3(0.01, 0.015, 0)
    ]);
    var tool = Tool.shaped(2, 0.0, CutterProfile.flat(0.002, 0.02));
    // One 2 mm level: a 10% ramp would need 20 mm, longer than the first
    // 18 mm pass, so CamKit plunges vertically. The oracle cannot build the
    // swept solid of a ramp (XY and Z together) and would refuse it.
    var program = new CamJob(0.005, 12000, new Point3(0, 0, 0.005))
      .pocket(contour, tool, -0.002, 0.01, 0.0015, 0.002)
      .finish();
    // Moves wholly above the stock cannot cut it; skipping them saves booleans.
    var moves = [for (move in CutMoves.fromOps(program.ops, program.tool))
      if (switch move.motion { case Path(geometry): lowestPoint(geometry) < 0.0; }) move];
    Assert.check(moves.length > 5, "CAM pocket produces cutting moves");
    var result = removalOf(moves);
    // A round cutter leaves a tool-radius fillet in each inside corner.
    var cornerRadius = 0.001;
    relative(result.removed,
      (0.02 * 0.01 - (4 - Math.PI) * cornerRadius * cornerRadius) * 0.002,
      "pocket clears its contour to depth apart from corner fillets");
    rays(result.stock, new Vector(0.02, 0.01, 0.05), new Vector(0, 0, -1), 0.1,
      [{enter: 0.052, exit: 0.06}], "pocket floor at depth");
    rays(result.stock, new Vector(0.02, -0.01, -0.001), new Vector(0, 1, 0), 0.04,
      [{enter: 0.01, exit: 0.015}, {enter: 0.025, exit: 0.03}],
      "pocket walls on the contour");
    result.stock.close();
  }

  static function lowestPoint(geometry:PathGeometry):Float
    return switch geometry {
      case Line(start, end): Math.min(start.z, end.z);
      case Arc(center, _, _, _): center.z;
      case Circular(center, _, _, _, _, rise): center.z + Math.min(0, rise);
    };

  static function removal(profile:CutterProfile,
      moves:Array<PathGeometry>):{stock:Part, removed:Float}
    return removalOf(ExactOracle.pathMoves(profile, moves));

  static function removalOf(moves:Array<CutMove>):{stock:Part, removed:Float} {
    var blank = Part.box(STOCK_X, STOCK_Y, STOCK_Z, Min, Min, Max);
    try {
      var stock = ExactOracle.cut(blank, moves);
      var removed = blank.volume() - stock.volume();
      blank.close();
      return {stock: stock, removed: removed};
    } catch (error:Dynamic) {
      blank.close();
      throw error;
    }
  }

  static function relative(actual:Float, expected:Float, message:String):Void
    Assert.near(actual, expected, message, Math.abs(expected) * VOLUME_TOLERANCE);

  static function rays(stock:Part,
      origin:Vector, direction:Vector, length:Float,
      expected:Array<{enter:Float, exit:Float}>, message:String):Void {
    var actual = ExactOracle.rayIntervals(stock, origin, direction, length);
    Assert.near(actual.length, expected.length, '$message: interval count', 0);
    for (i in 0...expected.length) {
      Assert.near(actual[i].enter, expected[i].enter, '$message: enter $i', DEPTH_TOLERANCE);
      Assert.near(actual[i].exit, expected[i].exit, '$message: exit $i', DEPTH_TOLERANCE);
    }
  }

  /** Composite Simpson's rule; the integrands here are smooth per piece. */
  static function integrate(f:Float->Float, a:Float, b:Float):Float {
    var n = 20000;
    var h = (b - a) / n, sum = f(a) + f(b);
    for (i in 1...n) sum += f(a + i * h) * (i % 2 == 1 ? 4 : 2);
    return sum * h / 3;
  }
}
