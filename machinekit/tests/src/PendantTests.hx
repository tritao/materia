import cadkit.modeling.Footprint;
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
    check(Math.abs(pendantCase.connectorX) < layout.width / 2 - layout.wall - 15.0, "Receptacle hole inside the bottom wall");
    var shell = pendantCase.shell();
    var plate = pendantCase.plate();
    check(shell.solidCount() == 1 && plate.solidCount() == 1, "Shell and plate are each one solid");
    var bounds = shell.shape.bounds();
    check(Math.abs(bounds.get_min().get_z() + (48.0 + PendantCase.BACK_WALL)) < 1e-6 && Math.abs(bounds.get_max().get_z()) < 1e-6, "Shell depth");
    check(Math.abs(bounds.get_max().get_x() - 95.0) < 1e-6 && Math.abs(bounds.get_max().get_y() - 55.0) < 1e-6, "Shell outline");
    var touching = plate.intersect(shell);
    check(touching.volume() < 1e-3, "The plate must only rest on the shell: " + touching.volume());
    touching.close();
    // Outer box less the cavity, plus six posts and four standoffs, less the receptacle hole through a 3 mm wall, two strap slots
    // through the back wall, the enabling pocket and the pilot holes.
    var r = PanelFacePlate.CORNER_RADIUS;
    var underside = pendantCase.boardTop - PendantCase.BOARD_THICKNESS;
    var shellExpected = RoundedRectangle.area(190.0, 110.0, r) * 51.0 - RoundedRectangle.area(184.0, 104.0, r - 3.0) * 48.0
      + 6 * Math.PI * 20.25 * 48.0 + 4 * Math.PI * 12.25 * (underside + 48.0)
      - Math.PI * 8.1 * 8.1 * 3.0 - 2 * 24.0 * 4.0 * 3.0 - Math.PI * 11.15 * 11.15 * 2.2
      - 6 * Math.PI * 2.1 * 2.1 * PendantCase.PILOT_DEPTH - 4 * Math.PI * 1.7 * 1.7 * PendantCase.STANDOFF_PILOT_DEPTH;
    check(Math.abs(shell.volume() - shellExpected) / shellExpected < 0.005, "Shell volume " + shell.volume() + " vs " + shellExpected);

    // What the plan added: standoffs, strap slots, a board behind the shallow parts, the enabling pocket and a tongue.
    check(pendantCase.standoffs.length == 4 && pendantCase.strapSlots.length == 2, "Four standoffs and two strap slots");
    for (slot in pendantCase.strapSlots) for (c in layout.controls)
      if (c.behind() > 40.0) check(FootprintGeometry.gap(c.body(), c.x, c.y, Rect(PendantCase.STRAP_SLOT_WIDTH, PendantCase.STRAP_SLOT_HEIGHT), slot.x, slot.y) >= 1.5 - 1e-9, "A strap slot behind the deep " + c.id);
    check(Math.abs(pendantCase.boardTop + 26.0) < 1e-9, "The board sits behind the 25 mm parts: " + pendantCase.boardTop);
    // Behind the shallow display module there is a free column for the enabling switch, left of centre.
    var pocket = pendantCase.enabling;
    check(pocket != null && pocket.x < 0 && pendantCase.notes.length == 0, "The enabling switch fits on the left: " + pendantCase.notes.join(" | "));
    var board = pendantCase.board();
    check(board.solidCount() == 1 && board.volume() > 0.5 * 180.0 * 100.0 * 1.6 && board.volume() < 180.0 * 100.0 * 1.6, "Board envelope " + board.volume());
    board.close();
    var plain = PanelFacePlate.part(layout, pendantCase.posts);
    check(plate.volume() > plain.volume() + 100.0 && plate.volume() < plain.volume() + 4000.0, "The tongue adds material: " + (plate.volume() - plain.volume()));
    plain.close();

    // Every part's solid is clear of the others, the plate and the shell.
    var report = AssemblyIntegrity.check(PanelAssemblyBuilder.build(layout, () -> pendantCase.plate(), () -> pendantCase.shell(), [{id: "board", solid: () -> pendantCase.board()}]));
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
