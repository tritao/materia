import cadkit.modeling.FootprintGeometry;
import cadkit.modeling.RoundedRectangle;
import machinekit.assembly.AssemblyIntegrity;
import machinekit.hmi.*;
import machinekit.pendant.*;
import sys.io.File;

class PendantTests {
  static function check(ok:Bool, message:String):Void if (!ok) throw message;
  static function rejects(message:String, body:() -> Void):Void {
    var failed = false;
    try body() catch (_:Dynamic) failed = true;
    check(failed, 'Expected rejection: $message');
  }

  public static function run():Void {
    var layout = ReferencePendant.layout();
    check(layout.problems().length == 0, "The reference layout must pass its own checks: " + layout.problems().join(" | "));
    check(layout.controls.length == 12 && layout.width == 190.0, "Twelve parts on a 190 mm face");
    check(layout.requiredInnerDepth() == 43.0 && layout.overallHeight() == 48.0 + 3.0 + 38.0, "Depth 43, overall " + layout.overallHeight());
    check(Math.abs(layout.partsMassGrams() - 438.4) < 1e-9, "Parts mass " + layout.partsMassGrams());

    // The shell: six posts clear of every part and body, a cable hole, and parts that do not intersect.
    var pendantCase = PendantCase.plan(layout);
    check(pendantCase.posts.length == 6, "Six screw posts");
    for (post in pendantCase.posts) for (c in layout.controls) {
      check(FootprintGeometry.distanceTo(c.keepOut(), c.x, c.y, post.x, post.y) >= 6.0, 'A screw head lands on ${c.id}');
      check(FootprintGeometry.distanceTo(c.body(), c.x, c.y, post.x, post.y) >= 4.0 + 1.5 - 1e-9, 'A post hits the body of ${c.id}');
    }
    check(Math.abs(pendantCase.cableX) < layout.width / 2 - layout.wall - 15.0, "Cable hole inside the bottom wall");
    var shell = pendantCase.shell();
    var plate = pendantCase.plate();
    check(shell.solidCount() == 1 && plate.solidCount() == 1, "Shell and plate are each one solid");
    var bounds = shell.shape.bounds();
    check(Math.abs(bounds.get_min().get_z() + (48.0 + PendantCase.BACK_WALL)) < 1e-6 && Math.abs(bounds.get_max().get_z()) < 1e-6, "Shell depth");
    check(Math.abs(bounds.get_max().get_x() - 95.0) < 1e-6 && Math.abs(bounds.get_max().get_y() - 55.0) < 1e-6, "Shell outline");
    var touching = plate.intersect(shell);
    check(touching.volume() < 1e-3, "The plate must only rest on the shell: " + touching.volume());
    touching.close();
    // Outer box less the cavity, plus six posts, less the cable hole through a 3 mm wall and six pilot holes.
    var r = PanelFacePlate.CORNER_RADIUS;
    var shellExpected = RoundedRectangle.area(190.0, 110.0, r) * 51.0 - RoundedRectangle.area(184.0, 104.0, r - 3.0) * 48.0
      + 6 * Math.PI * 16.0 * 48.0 - Math.PI * 4.5 * 4.5 * 3.0 - 6 * Math.PI * 1.3 * 1.3 * PendantCase.PILOT_DEPTH;
    check(Math.abs(shell.volume() - shellExpected) / shellExpected < 0.003, "Shell volume " + shell.volume() + " vs " + shellExpected);

    // Every part's solid is clear of the others, the plate and the shell.
    var report = AssemblyIntegrity.check(PanelAssemblyBuilder.build(layout, () -> pendantCase.plate(), () -> pendantCase.shell()));
    check(report.errors().length == 0, "Interference: " + [for (e in report.errors()) e.message].join(" | "));

    var stepDir = Sys.getEnv("PENDANT_CASE_STEP_DIR");
    if (stepDir != null && stepDir != "") { shell.shape.exportStep(stepDir + "/pendant-shell.step"); plate.shape.exportStep(stepDir + "/pendant-plate.step"); }
    shell.close(); plate.close();
    var drawing = Sys.getEnv("PENDANT_SVG");
    if (drawing != null && drawing != "") File.saveContent(drawing, PanelLayoutSvg.render(layout, 4.0, pendantCase.marks()));

    rejects("no room for posts", () -> PendantCase.plan(new PanelLayout(60.0, 40.0, 3.0, 3.0, 30.0, [
      new PlacedPart("a", PartPresets.pushButton16(), 0.0, 0.0), new PlacedPart("b", PartPresets.pushButton16(), 18.0, 0.0), new PlacedPart("c", PartPresets.pushButton16(), -18.0, 0.0)])));
    trace("pendant tests passed");
  }
  public static function main():Void run();
}
