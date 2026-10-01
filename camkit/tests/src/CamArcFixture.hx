import camkit.CamContour;
import camkit.CamJob;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.Point3;
import toolpathkit.tool.Tool;

/** A pocket in a polygonized circle is cut as arcs, as the circle was drawn. */
class CamArcFixture {
  public static function run(check:Bool->String->Void):Void {
    // A 20 mm circle as CAD edges sample it: vertices on the circle, chords within 20 µm.
    var circle = new CamContour([for (k in 0...128) new Point3(0.03 + 0.01 * Math.cos(2 * Math.PI * k / 128),
      0.03 + 0.01 * Math.sin(2 * Math.PI * k / 128), 0.0)]);
    var tool = new Tool(4, 0.0, 0.006);
    function pocket(arcTolerance:Float) return new CamJob(0.005, 10000,
      new Point3(0.03, 0.03, 0.02), 0.0, arcTolerance)
      .pocket(circle, tool, -0.002, 0.005, 0.002, 0.002, 0.001).finish();
    var lines = pocket(0.0), arcs = pocket(0.00002);
    var arcMoves = 0, radii:Array<Float> = [];
    for (op in arcs.ops) switch op {
      case Move(Cut, Arc(center, radius, _, sweep), _, _, _):
        arcMoves++;
        radii.push(Math.round(radius * 1e6) / 1e3);
        check(center.distanceTo(new Point3(0.03, 0.03, center.z)) < 1e-9,
          "each pocket ring is centred on the circle");
        check(Math.abs(Math.abs(sweep) - 2 * Math.PI) < 1e-9, "each pocket ring is one full arc");
      case _:
    }
    check(arcMoves >= 2 && arcs.ops.length * 10 < lines.ops.length,
      'pocket rings become arcs: ${arcs.ops.length} operations against ${lines.ops.length}, radii $radii mm');
    var machine = new CamTestRig();
    machine.toolLibrary.set(tool);
    var gcode = machine.export(arcs, CamTestSetup.standard());
    check(gcode.indexOf("G2 ") >= 0 || gcode.indexOf("G3 ") >= 0, "arcs reach the G-code as G2/G3");
    var imported = machine.compileDetailed(gcode);
    check(imported.diagnostics.length == 0 && imported.program.ops.length == arcs.ops.length,
      'arc G-code round trips: ${imported.diagnostics}');
    for (index in 0...arcs.ops.length) switch [arcs.ops[index], imported.program.ops[index]] {
      case [Move(_, a, _, _, _), Move(_, b, _, _, _)]:
        var arc = switch a {
          case Arc(_, _, _, _): true;
          case _: false;
        };
        // An arc keeps its length; the first move starts wherever the importing
        // machine stands, so lines are compared by where they end.
        if (arc) check(Math.abs(GeometryTools.length(a) - GeometryTools.length(b)) < 1e-8,
          'arc G-code keeps arc $index');
        else check(GeometryTools.pointAt(a, GeometryTools.length(a)).distanceTo(
          GeometryTools.pointAt(b, GeometryTools.length(b))) < 1e-8, 'arc G-code keeps move $index end');
      case _:
    }
  }
}
