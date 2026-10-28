import materia.casting.CastingFeature.CastingDimension;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.process.OrthogonalSections;

class SectionTests {
  static var assertions = 0;

  static function check(ok:Bool, label:String):Void {
    assertions++;
    if (!ok) throw label;
  }

  static function rejects(fn:Void -> Void, expected:String):Void {
    var message = "";
    try fn() catch (error:Dynamic) message = Std.string(error);
    check(message.indexOf(expected) >= 0, "Expected " + expected + ", got " + message);
  }

  static function close(a:Float, b:Float, label:String):Void check(
    Math.abs(a - b) < 1e-6,
    label + ": " + a + " vs " + b
  );

  public static function run(p:CastingMoldPlan, s:materia.tooling.process.CastingProcessSpec):Void {
    var box = {x: 0.0, y: 0.0, z: 0.0, width: 100.0, depth: 60.0, height: 20.0};
    var split = [
      {x: 0.0, y: 0.0, z: 0.0, width: 30.0, depth: 60.0, height: 20.0},
      {x: 30.0, y: 0.0, z: 0.0, width: 70.0, depth: 60.0, height: 20.0}
    ];
    var report = OrthogonalSections.analyze("body", "hash", split, []);
    check(report.spans.length == 3, "coplanar strips aggregate by physical endpoints");
    for (span in report.spans) {
      close(
        span.thicknessMm,
        span.axis == X ? 100 : span.axis == Y ? 60 : 20,
        "ray spans physical material, not subtraction fragments"
      );
      close(span.volumeMm3, 120000, "each axis covers the whole block");
    }
    check(report.maximumInscribedThicknessMm == 20, "rectangular inscribed thickness exact");
    var shape = split.concat([{x: 100.0, y: 0.0, z: 0.0, width: 20.0, depth: 20.0, height: 20.0}]);
    var junction = OrthogonalSections.analyze("body", "hash", shape, []);
    check(
      junction.maximumInscribedThicknessMm == null,
      "nonrectangular junction never uses a chord as an inscribed diameter"
    );
    check(
      junction.validation.filter(v -> v.rule == "process.sections.junction-thickness"
      && v.status == "pass").length == 1,
      "junction diameter has converged bounds without claiming thermal coverage"
    );
    for (axis in junction.coveredVolumeByAxisMm3) close(axis.volumeMm3, 128000, "junction coverage exact");
    shape.reverse();
    check(
      ToolingCodec.canonical(OrthogonalSections.analyze("body", "hash", shape, [])) == ToolingCodec.canonical(junction),
      "section ordering independent of box order"
    );
    var owners = OrthogonalSections.analyze(
      "body",
      "hash",
      [box],
      [{id: "core", box: {x: 0, y: 0, z: 20, width: 100, depth: 60, height: 20}}]
    );
    check(owners.spans.filter(x -> x.axis == Z)[0].upperSourceIds[0] == "core", "opposing face preserves feature ID");
    var effort = materia.tooling.export.ToolingFabrication.effort(p, FabricationTests.spec());
    check(effort.panels == p.panels.members.length + p.supportPanels.length,
      "effort counts the production mold panels");
    check(effort.epsCores == p.cores.length, "effort counts source EPS cores");
    Sys.println('SectionTests: $assertions assertions passed');
  }
}
