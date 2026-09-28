import camkit.CamContour;
import camkit.CamGCodeWriter;
import camkit.CamJob;
import cnckit.CncCompiler;
import cnckit.CncMachine;
import cnckit.CncTool;
import cnckit.ir.CncGeometryTools;
import cnckit.ir.CncOp;
import cnckit.ir.CncPoint;

/** Mixed profile and drill job verifies safe travel across tool changes. */
class CamSafeTravelFixture {
  public static function run(check:Bool->String->Void):Void {
    var contour = new CamContour([new CncPoint(0.01, 0.01, 0),
      new CncPoint(0.03, 0.01, 0),
      new CncPoint(0.03, 0.02, 0),
      new CncPoint(0.01, 0.02, 0)]);
    var profileTool = new CncTool(21, 0, 0.002);
    var drillTool = new CncTool(22, 0, 0.001);
    var job = new CamJob(0.005, 10000,
      new CncPoint(0.002, 0.002, -0.001));
    var program = job
      .profile(contour, profileTool, -0.003, 0.005,
        "outside", 0.002, 0.001)
      .drill([new CncPoint(0.018, 0.015, 0),
        new CncPoint(0.024, 0.015, 0)],
        drillTool, -0.004, 0.001, 0.0008)
      .profile(contour, profileTool, -0.001, 0.005,
        "inside", 0.002, 0.001)
      .finish();

    var current = new CncPoint(0.002, 0.002, -0.001);
    var firstRapid = false, toolChanges = 0;
    var profilePlunges = 0, drillFeeds = 0, xyRapids = 0;
    for (op in program.ops) switch op {
      case Rapid(Line(a, b), _):
        if (!firstRapid) {
          firstRapid = true;
          check(Math.abs(a.x - b.x) < 1e-10 &&
            Math.abs(a.y - b.y) < 1e-10 &&
            Math.abs(b.z - 0.005) < 1e-10,
            "first profile move retracts vertically from the initial pose");
        }
        if (Math.abs(a.x - b.x) + Math.abs(a.y - b.y) > 1e-9) {
          xyRapids++;
          check(Math.abs(a.z - 0.005) < 1e-10 &&
            Math.abs(b.z - 0.005) < 1e-10,
            "profile and drill XY rapids stay at safe Z");
        }
        current = b;
      case Feed(Line(a, b), speed, _, span):
        if (Math.abs(a.x - b.x) + Math.abs(a.y - b.y) < 1e-9 &&
            b.z < a.z - 1e-9) {
          if (span.line == 1 || span.line == 3) {
            profilePlunges++;
            check(Math.abs(speed - 0.001) < 1e-10,
              "profile descent uses its configured plunge feed");
          } else if (span.line == 2) {
            drillFeeds++;
            check(Math.abs(speed - 0.0008) < 1e-10,
              "drill descent retains its drilling feed");
          }
        }
        current = b;
      case Feed(geometry, _, _, _):
        current = CncGeometryTools.pointAt(geometry,
          CncGeometryTools.length(geometry));
      case ToolChange(_, _):
        toolChanges++;
        check(Math.abs(current.z - 0.005) < 1e-10,
          "tool changes occur after safe-Z retraction");
      case _:
    }
    check(toolChanges == 3 && xyRapids >= 3 &&
      profilePlunges >= 3 && drillFeeds == 2,
      "mixed job covers profile passes, two holes and two tool switches");

    var machine = new CncMachine("work", "x", "y", "z", 0.2);
    machine.setTool(profileTool); machine.setTool(drillTool);
    check(program.lower(machine).diagnostics.length == 0,
      "mixed tool job lowers through MotionKit");
    var imported = new CncCompiler(machine).compileDetailed(
      CamGCodeWriter.write(program));
    check(imported.diagnostics.length == 0 &&
      imported.ops.length == program.ops.length,
      "mixed tool G-code round trip keeps operation order");
    for (index in 0...program.ops.length) switch [program.ops[index], imported.ops[index]] {
      case [Feed(a, speedA, _, _), Feed(b, speedB, _, _)]:
        check(Math.abs(speedA - speedB) < 1e-8,
          "mixed tool G-code preserves feed selection");
        check(CncGeometryTools.pointAt(a, CncGeometryTools.length(a))
          .distanceTo(CncGeometryTools.pointAt(b, CncGeometryTools.length(b))) < 1e-8,
          "mixed tool G-code preserves feed endpoints");
      case [Rapid(a, _), Rapid(b, _)]:
        check(CncGeometryTools.pointAt(a, CncGeometryTools.length(a))
          .distanceTo(CncGeometryTools.pointAt(b, CncGeometryTools.length(b))) < 1e-8,
          "mixed tool G-code preserves rapid endpoints");
      case [ToolChange(a, _), ToolChange(b, _)]:
        check(a == b, "mixed tool G-code preserves tool number");
      case _:
    }

    var rejectedJob = new CamJob(0.005, 10000), rejected = false;
    try rejectedJob.profile(contour, profileTool, -0.001,
      0.005, "outside", 0.002, 0.006)
    catch (_:Dynamic) rejected = true;
    check(rejected, "profile rejects plunge feed above cutting feed");
    rejected = false;
    try rejectedJob.finish() catch (_:Dynamic) rejected = true;
    check(rejected, "invalid profile feed leaves no partial CAM operation");
    var badDrill = new CamJob(0.005, 10000);
    rejected = false;
    try badDrill.drill([new CncPoint(0.018, 0.015, 0),
      new CncPoint(0.024, 0.015, 0.01)],
      drillTool, -0.004, 0.001, 0.0008)
    catch (_:Dynamic) rejected = true;
    check(rejected, "drill validates all hole positions before cutting");
    rejected = false;
    try badDrill.finish() catch (_:Dynamic) rejected = true;
    check(rejected, "invalid later hole leaves no partial CAM operation");
  }
}
