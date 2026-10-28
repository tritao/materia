import materia.tooling.process.JunctionThickness;
import materia.tooling.process.OrthogonalSections;
import materia.tooling.casting.CastingMoldPlan.MoldBox;
import materia.casting.BoxAlgebra;
import materia.tooling.ToolingCodec;

class JunctionTests {
  static var assertions = 0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }

  static function near(a:Float, b:Float, t:Float, message:String):Void check(
    Math.abs(a - b) <= t,
    message + ": " + a + " vs " + b
  );

  static function rejects(fn:Void -> Void, expected:String):Void {
    var message = "";
    try fn() catch (e:Dynamic) message = Std.string(e);
    check(message.indexOf(expected) >= 0, "Expected " + expected + ", got " + message);
  }

  static function b(x:Float, y:Float, z:Float, w:Float, d:Float, h:Float):MoldBox return {
    x: x,
    y: y,
    z: z,
    width: w,
    depth: d,
    height: h
  };

  static function bounded(r:JunctionThicknessReport, expected:Float):Void {
    check(
      r.lowerDiameterMm <= expected + 1e-7 && r.upperDiameterMm >= expected - 1e-7,
      "analytic diameter lies in certified interval"
    );
    check(
      r.converged && r.upperDiameterMm - r.lowerDiameterMm <= r.settings.diameterToleranceMm,
      "declared geometry tolerance is satisfied"
    );
  }

  public static function run():Void {
    var settings:JunctionSearchSettings = {diameterToleranceMm: 0.01, maximumCells: 50000};
    var block = [b(0, 0, 0, 100, 60, 20)];
    var rectangle = JunctionThickness.analyze("body", "source", block, [], settings);
    bounded(rectangle, 20);
    near(rectangle.witness.z, 10, 1e-7, "rectangular witness at shortest-axis midplane");
    var cross = [b(-5, -1, -4, 10, 2, 8), b(-1, -5, -4, 2, 10, 8)];
    var junction = JunctionThickness.analyze("body", "source", cross, [], settings);
    bounded(junction, 2 * Math.sqrt(2));
    check(junction.lowerDiameterMm > 2.8, "junction thicker than either arm's 2 mm wall");
    var before = ToolingCodec.canonical(junction);
    cross.reverse();
    check(
      ToolingCodec.canonical(JunctionThickness.analyze("body", "source", cross, [], settings)) == before,
      "input permutation leaves exact report unchanged"
    );
    var l = [b(0, 0, -4, 4, 2, 8), b(0, 0, -4, 2, 4, 8)];
    bounded(JunctionThickness.analyze("body", "source", l, [], settings), 8 - 4 * Math.sqrt(2));
    var disconnected = JunctionThickness.analyze(
      "body",
      "source",
      [b(0, 0, 0, 2, 2, 2), b(10, 0, 0, 6, 6, 6)],
      [],
      settings
    );
    bounded(disconnected, 6);
    var cavity = BoxAlgebra.subtract([b(-5, -5, -5, 10, 10, 10)], [b(-1, -1, -1, 2, 2, 2)]);
    var withHole = JunctionThickness.analyze(
      "body",
      "source",
      cavity,
      [{id: "hole", box: b(-1, -1, -1, 2, 2, 2)}],
      settings
    );
    check(withHole.upperDiameterMm < 10, "internal cavity limits inscribed diameter");
    check(
      withHole.faces.filter(f -> f.sourceIds.indexOf("hole") >= 0).length == 6,
      "internal cavity has six owned exposed faces"
    );
    var faceArea = 0.0;
    for (face in junction.faces) faceArea += face.axis == X ? face.region.depth * face.region.height
      : face.axis == Y ? face.region.width * face.region.height : face.region.width * face.region.depth;
    near(faceArea, 392, 1e-7, "cross surface excludes artificial internal partitions");
    var coarse = JunctionThickness.analyze("body", "source", cross, [], {diameterToleranceMm: 1e-8, maximumCells: 20});
    check(!coarse.converged, "exhausted work budget never claims convergence");
    check(
      coarse.lowerDiameterMm <= 2 * Math.sqrt(2) + 1e-7 && coarse.upperDiameterMm >= 2 * Math.sqrt(2) - 1e-7,
      "budget exhaustion retains valid global bounds"
    );
    rejects(
      () -> JunctionThickness.analyze("body", "s", block, [], {diameterToleranceMm: 0, maximumCells: 20}),
      "Invalid junction search settings"
    );
    rejects(() -> JunctionThickness.analyze("body", "s", [b(0, 0, 0, -1, 1, 1)], []), "Invalid junction box");
    Sys.println('JunctionTests: $assertions assertions passed');
  }
}
