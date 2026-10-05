package fixtures;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import oracle.ExactOracle;
import haxeon.test.Shards.TestGroup;
import toolpathkit.tool.CutterProfile;

typedef ChainTool = {
  final name:String;
  final profile:CutterProfile;
  final depth:Float;
  final corners:Array<Float>;

  /** Roughly how long its cases take, in seconds, to balance shards. */
  final weight:Float;
}

/**
  Multi-move toolpaths whose moves join tangentially, cut with the exact
  oracle across tools, corner radii and stock positions. Their removed volume
  has a closed form: a tube around a smooth path whose curvature radius is at
  least the tool radius sweeps its cross-section along the path length
  (Pappus), plus one whole tool for the two open ends of an open path.
**/
class ChainFixtures {
  static inline final DEPTH_TOLERANCE = 1e-7; // relative volume
  static inline final RADIUS = 0.003;

  /** Every case, one after another. */
  public static function run():Void {
    for (group in groups())
      group.run();
  }

  /**
    The cases as independent groups, one per tool and one for the refusal check, so a test program can run them as parallel
    shards. Together they are what `run` does.
  **/
  public static function groups():Array<TestGroup> {
    var result:Array<TestGroup> = [for (tool in tools()) toolGroup(tool)];
    result.push({name: "ChainFixtures ball arc refused", run: ballArcRefused, weight: 0.1});
    return result;
  }

  static function toolGroup(tool:ChainTool):TestGroup
    return {name: 'ChainFixtures ${tool.name}', run: () -> runTool(tool), weight: tool.weight};

  static function tools():Array<ChainTool> {
    var r = RADIUS;
    // Corners at exactly the tool radius are CamKit's outside corners. The
    // oracle refuses them for tools with a curved edge at that radius, so
    // those tools use the smallest corners it accepts.
    return [
      {name: "flat", profile: CutterProfile.flat(2 * r, 0.02), depth: 0.002, corners: [r, 1.5 * r], weight: 9.4},
      {name: "ball", profile: CutterProfile.ball(2 * r, 0.02), depth: 0.004, corners: [1.01 * r, 1.5 * r], weight: 7.5},
      {name: "bull-nose", profile: CutterProfile.bullNose(2 * r, 0.001, 0.02), depth: 0.002,
        corners: [1.01 * r, 1.5 * r], weight: 13.5},
      {name: "V-bit", profile: CutterProfile.vee(2 * r, Math.PI / 2, 0.02), depth: 0.002,
        corners: [r, 1.5 * r], weight: 5.3}
    ];
  }

  static function origins():Array<Vector>
    return [new Vector(0, 0, 0), new Vector(0.0137, -0.0213, 0.0041), new Vector(0.25, 0.1, -0.05)];

  static function runTool(tool:ChainTool):Void {
    var origins = origins();
    for (corner in tool.corners)
      for (origin in origins) {
        var label = '${tool.name} corner ${corner} at (${origin.x}, ${origin.y}, ${origin.z})';
        roundedRectangle(tool.profile, tool.depth, corner, origin, label);
      }
    sCurve(tool.profile, tool.depth, 1.5 * RADIUS, origins[1], '${tool.name} S-curve');
  }

  static function ballArcRefused():Void {
    var refused = false;
    try ExactOracle.sweptSolid(CutterProfile.ball(2 * RADIUS, 0.02),
      Arc(new Point3(0, 0, 0), RADIUS, 0, Math.PI / 2)).close()
    catch (_:Dynamic) refused = true;
    Assert.check(refused, "a ball-mill arc of the ball's own radius is refused");
  }

  /** A closed loop of four lines and four quarter arcs, cut at one depth. */
  static function roundedRectangle(profile:CutterProfile, depth:Float,
      corner:Float, origin:Vector, label:String):Void {
    var a = 0.012, b = 0.007, z = origin.z - depth;
    var x0 = origin.x + 0.005, y0 = origin.y + 0.005;
    function p(x:Float, y:Float):Point3 return new Point3(x0 + x, y0 + y, z);
    var moves:Array<PathGeometry> = [
      Line(p(corner, 0), p(corner + a, 0)),
      Arc(p(corner + a, corner), corner, -Math.PI / 2, Math.PI / 2),
      Line(p(2 * corner + a, corner), p(2 * corner + a, corner + b)),
      Arc(p(corner + a, corner + b), corner, 0, Math.PI / 2),
      Line(p(corner + a, 2 * corner + b), p(corner, 2 * corner + b)),
      Arc(p(corner, corner + b), corner, Math.PI / 2, Math.PI / 2),
      Line(p(0, corner + b), p(0, corner)),
      Arc(p(corner, corner), corner, Math.PI, Math.PI / 2)
    ];
    var length = 2 * a + 2 * b + 2 * Math.PI * corner;
    var cut = profile.below(depth);
    check(profile, moves, origin, 2 * cut.halfSectionArea() * length, label);
  }

  /** Line, left turn, right turn, line: open, with tangent joins throughout. */
  static function sCurve(profile:CutterProfile, depth:Float, corner:Float,
      origin:Vector, label:String):Void {
    var a = 0.008, z = origin.z - depth;
    var x0 = origin.x + 0.005, y0 = origin.y + 0.005;
    function p(x:Float, y:Float):Point3 return new Point3(x0 + x, y0 + y, z);
    var moves:Array<PathGeometry> = [
      Line(p(0, 0), p(a, 0)),
      Arc(p(a, corner), corner, -Math.PI / 2, Math.PI / 2),
      Arc(p(a + 2 * corner, corner), corner, Math.PI, -Math.PI / 2),
      Line(p(a + 2 * corner, 2 * corner), p(2 * a + 2 * corner, 2 * corner))
    ];
    var length = 2 * a + Math.PI * corner;
    var cut = profile.below(depth);
    check(profile, moves, origin,
      2 * cut.halfSectionArea() * length + cut.volume(), label);
  }

  static function check(profile:CutterProfile, moves:Array<PathGeometry>,
      origin:Vector, expected:Float, label:String):Void {
    var width = 0.04, height = 0.03, thickness = 0.01;
    var box = Part.box(width, height, thickness, Min, Min, Max);
    var stock = box.translated(origin);
    box.close();
    try {
      var cutMoves = ExactOracle.pathMoves(profile, moves);
      var result = ExactOracle.cut(stock, cutMoves);
      var removed = stock.volume() - result.volume();
      CoreComparison.againstOracle(result, cutMoves, origin.x, origin.y, origin.z - thickness,
        origin.x + width, origin.y + height, origin.z, label);
      result.close();
      stock.close();
      Assert.near(removed / expected, 1.0, '$label removes a tube along its path',
        DEPTH_TOLERANCE);
    } catch (error:Dynamic) {
      stock.close();
      throw '$label: $error';
    }
  }
}
