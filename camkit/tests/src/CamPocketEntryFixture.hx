import camkit.CamContour;
import camkit.CamJob;
import camkit.CamProgram;
import toolpathkit.tool.Tool;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Point3;

/** Long and short pocket spans exercise ramp and plunge entry. */
class CamPocketEntryFixture {
  public static function run(check:Bool->String->Void):Void {
    var tool = new Tool(12, 0, 0.002);
    var large = rectangle(0.02, 0.01);
    var program = new CamJob(0.005, 10000)
      .pocket(large, tool, -0.002, 0.005, 0.001, 0.001, 0.001)
      .finish();
    var ramps = 0, plunges = 0;
    var firstRapid = false;
    for (index in 0...program.ops.length) switch program.ops[index] {
      case Move(Rapid, Line(a, b), _, _, _):
        if (!firstRapid) {
          firstRapid = true;
          check(Math.abs(a.x - b.x) < 1e-10 &&
            Math.abs(a.y - b.y) < 1e-10 &&
            Math.abs(b.z - 0.005) < 1e-10,
            "first pocket rapid raises vertically before XY travel");
        }
        if (Math.abs(a.x - b.x) + Math.abs(a.y - b.y) > 1e-9)
          check(Math.abs(a.z - 0.005) < 1e-10 &&
            Math.abs(b.z - 0.005) < 1e-10,
            "pocket XY rapids stay at safe Z");
      case Move(Cut, Line(a, b), speed, _, _)
        if (b.z < a.z - 1e-9):
        var xy = Math.sqrt(Math.pow(b.x - a.x, 2) +
          Math.pow(b.y - a.y, 2));
        if (xy > 1e-9) {
          ramps++;
          check(Math.abs(speed - 0.005) < 1e-10 &&
            (a.z - b.z) / xy <= 0.1 + 1e-9,
            "pocket ramp uses cutting feed and bounded slope");
          check(index + 1 < program.ops.length,
            "pocket ramp has a following retrace");
          switch program.ops[index + 1] {
            case Move(Cut, Line(c, d), retraceFeed, _, _):
              check(c.distanceTo(b) < 1e-9 &&
                Math.abs(d.x - a.x) < 1e-9 &&
                Math.abs(d.y - a.y) < 1e-9 &&
                Math.abs(d.z - b.z) < 1e-9 &&
                Math.abs(retraceFeed - 0.005) < 1e-10,
                "ramp entry retraces to its start at full depth");
            case _: check(false, "ramp entry must retrace");
          }
        } else {
          plunges++;
          check(Math.abs(speed - 0.001) < 1e-10,
            "vertical entry uses the separate plunge feed");
        }
      case _:
    }
    check(ramps > 0 && plunges > 0,
      "large pocket uses ramps and short inner rings use controlled plunges");

    var narrow = rectangle(0.005, 0.005);
    var narrowProgram = new CamJob(0.005, 10000)
      .pocket(narrow, tool, -0.001, 0.005, 0.001, 0.001, 0.001)
      .finish();
    var cuttingRamps = 0, cuttingPlunges = 0;
    for (op in narrowProgram.ops) switch op {
      case Move(Cut, Line(a, b), speed, _, _)
        if (b.z < a.z - 1e-9 && b.z < -1e-9):
        if (Math.abs(a.x - b.x) + Math.abs(a.y - b.y) > 1e-9)
          cuttingRamps++;
        else if (Math.abs(speed - 0.001) < 1e-10)
          cuttingPlunges++;
      case _:
    }
    check(cuttingRamps == 0 && cuttingPlunges > 0,
      "narrow pocket falls back to its configured plunge feed");

    var rejectedJob = new CamJob(0.005, 10000), rejected = false;
    try rejectedJob.pocket(large, tool, -0.001, 0.005, 0.001,
      0.001, 0.006)
    catch (_:Dynamic) rejected = true;
    check(rejected, "pocket rejects plunge feed above cutting feed");
    rejected = false;
    try rejectedJob.finish() catch (_:Dynamic) rejected = true;
    check(rejected, "invalid plunge feed leaves no partial CAM operation");
  }

  static function rectangle(width:Float, height:Float):CamContour
    return new CamContour([new Point3(0, 0, 0),
      new Point3(width, 0, 0), new Point3(width, height, 0),
      new Point3(0, height, 0)]);
}
