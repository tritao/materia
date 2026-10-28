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
    check(layout.controls.length == 12 && layout.width == 114.0 && layout.height == 210.0, "Twelve parts on a 114 x 210 mm face");
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
    check(Math.abs(bounds.get_max().get_x() - 57.0) < 1e-6 && Math.abs(bounds.get_max().get_y() - 105.0) < 1e-6, "Shell outline");
    var touching = plate.intersect(shell);
    check(touching.volume() < 1e-3, "The plate must only rest on the shell: " + touching.volume());
    touching.close();
    // Outer box less the cavity, plus six posts and four standoffs, less the receptacle hole through a 3 mm wall, two strap slots
    // through the back wall, the enabling switch's hole through a 3 mm side wall and the pilot holes.
    var r = PanelFacePlate.CORNER_RADIUS;
    var underside = pendantCase.boardTop - PendantCase.BOARD_THICKNESS;
    var shellExpected = RoundedRectangle.area(114.0, 210.0, r) * 51.0 - RoundedRectangle.area(108.0, 204.0, r - 3.0) * 48.0
      + 6 * Math.PI * 20.25 * 48.0 + 4 * Math.PI * 12.25 * (underside + 48.0)
      - Math.PI * 8.1 * 8.1 * 3.0 - 2 * 24.0 * 4.0 * 3.0 - Math.PI * 11.15 * 11.15 * 3.0
      - 6 * Math.PI * 2.1 * 2.1 * PendantCase.PILOT_DEPTH - 4 * Math.PI * 1.7 * 1.7 * PendantCase.STANDOFF_PILOT_DEPTH;
    // Ribs add material above the floor; posts that run into a wall and ribs that run into a tower overlap it, so the estimate is loose.
    var ribHeight = 48.0 + (pendantCase.boardTop - PendantCase.BOARD_THICKNESS - 1.0);
    for (rib in pendantCase.ribs) shellExpected += rib.w * rib.h * ribHeight;
    check(Math.abs(shell.volume() - shellExpected) / shellExpected < 0.03, "Shell volume " + shell.volume() + " vs " + shellExpected);

    // Every tower is part of the surrounding structure: it touches a wall, or a rib ties it to one.
    var towers:Array<{x:Float, y:Float, r:Float, name:String}> = [];
    for (p in pendantCase.posts) towers.push({x: p.x, y: p.y, r: PendantCase.POST_DIAMETER / 2, name: "post"});
    for (st in pendantCase.standoffs) towers.push({x: st.x, y: st.y, r: PendantCase.STANDOFF_DIAMETER / 2, name: "standoff"});
    var innerX = layout.width / 2 - layout.wall, innerY = layout.height / 2 - layout.wall;
    for (t in towers) {
      var wallGap = Math.min(Math.min(t.x + innerX, innerX - t.x), Math.min(t.y + innerY, innerY - t.y)) - t.r;
      var tied = wallGap <= 0.25;
      for (rib in pendantCase.ribs) if (FootprintGeometry.distanceTo(Rect(rib.w, rib.h), rib.x, rib.y, t.x, t.y) <= t.r + 0.3) tied = true;
      check(tied, "A free-standing " + t.name + " at " + t.x + ", " + t.y);
    }
    for (p in pendantCase.posts) check(Math.min(Math.min(p.x + innerX, innerX - p.x), Math.min(p.y + innerY, innerY - p.y)) - PendantCase.POST_DIAMETER / 2 <= 0.25,
      "A post stands off its wall at " + p.x + ", " + p.y + " and relies on a rib");

    // What the plan added: standoffs, strap slots, a board behind the shallow parts, the enabling switch and a tongue.
    check(pendantCase.standoffs.length == 4 && pendantCase.strapSlots.length == 2, "Four standoffs and two strap slots");
    for (slot in pendantCase.strapSlots) for (c in layout.controls)
      if (c.behind() > 40.0) check(FootprintGeometry.gap(c.body(), c.x, c.y, Rect(PendantCase.STRAP_SLOT_WIDTH, PendantCase.STRAP_SLOT_HEIGHT), slot.x, slot.y) >= 1.5 - 1e-9, "A strap slot behind the deep " + c.id);
    check(Math.abs(pendantCase.boardTop + 26.0) < 1e-9, "The board sits behind the 25 mm parts: " + pendantCase.boardTop);
    // The enabling switch lies against the back wall on the left side, where only the E-stop (on the right) is deep.
    var pocket = pendantCase.enabling;
    check(pocket != null && pocket.x < 0 && pendantCase.notes.length == 0, "The enabling switch fits on the left: " + pendantCase.notes.join(" | "));
    var board = pendantCase.board();
    check(board.solidCount() == 1 && board.volume() > 0.5 * 104.0 * 200.0 * 1.6 && board.volume() < 104.0 * 200.0 * 1.6, "Board envelope " + board.volume() + " solids " + board.solidCount());
    board.close();
    var plain = PanelFacePlate.part(layout, pendantCase.posts);
    check(plate.volume() > plain.volume() + 100.0 && plate.volume() < plain.volume() + 4000.0, "The tongue adds material: " + (plate.volume() - plain.volume()));
    plain.close();

    // Every part's solid is clear of the others, the plate and the shell.
    var assembly = PanelAssemblyBuilder.build(layout, () -> pendantCase.plate(), () -> pendantCase.shell(), [{id: "board", solid: () -> pendantCase.board()}], "FR4 PCB", ReferencePendant.groups());
    var report = AssemblyIntegrity.check(assembly);
    check(report.errors().length == 0, "Interference: " + [for (e in report.errors()) e.message].join(" | "));
    // The hierarchy reads case, jog, selectors, function and cycle keys, with the E-stop and display on their own, and each device is its bodies.
    var ids = [for (c in assembly.components()) c.id];
    for (id in ["case/plate", "case/shell", "case/board", "jog/wheel/housing", "jog/wheel/wheel", "jog/wheel-enable/housing", "jog/wheel-enable/cap", "selectors/axis/switch", "selectors/axis/knob",
        "function-keys/f2/housing", "cycle-keys/hold/cap", "estop/operator", "estop/head", "display"])
      check(ids.indexOf(id) >= 0, "Missing member " + id + " in " + ids.join(", "));
    check(ids.length == 26, "Twenty-six bodies: " + ids.length);
    // The wheel turns on its housing and each knob on its switch: three free joints, the wheel's named for its encoder.
    var joints = assembly.derived().definition.joints;
    var encoders = [for (j in joints) if (j.id == "jog/wheel/encoder") j];
    check(joints.length == 3 && encoders.length == 1, "Three free joints, one the wheel's encoder: " + [for (j in joints) j.id].join(", "));
    // Splitting a device does not change the bill of materials: one line per purchase, the bodies included in it.
    var bom = new machinekit.component.Bom();
    for (c in assembly.components()) bom.addComponent(c.component);
    var purchases = 0;
    for (line in bom.lines()) purchases += line.quantity;
    check(purchases == 15, "Fifteen purchases (plate, shell, board and twelve controls): " + purchases + " " + [for (l in bom.lines()) l.partNumber + " x" + l.quantity].join(", "));

    // The bodies of a device share its declared mass, and the head is the colour the part shows.
    var stop = PartPresets.xb2Es542();
    var stopBodies = stop.expand();
    check(stopBodies != null, "The E-stop expands into bodies");
    if (stopBodies != null) {
      var sum = 0.0;
      for (c in stopBodies.components()) sum += c.component.massProperties().mass;
      check(Math.abs(sum - stop.n("massGrams") / 1000.0) < 1e-9, "Body masses add up to the device: " + sum);
      for (c in stopBodies.components()) check(c.id != "head" || c.component.materialId == "abs-red", "The E-stop head is red");
      for (c in stopBodies.components()) check(c.id != "operator" || c.component.materialId == "abs-black", "The E-stop operator is black");
    }

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
