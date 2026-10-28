import machinekit.component.ComponentValues;
import cadkit.modeling.Footprint;
import cadkit.modeling.FootprintGeometry;
import cadkit.modeling.RoundedRectangle;
import machinekit.assembly.AssemblyIntegrity;
import machinekit.hmi.*;

class PanelLayoutTests {
  static function check(ok:Bool, message:String):Void if (!ok) throw message;
  static function rejects(message:String, body:() -> Void):Void {
    var failed = false;
    try body() catch (_:Dynamic) failed = true;
    check(failed, 'Expected rejection: $message');
  }
  static function hasProblem(layout:PanelLayout, text:String):Bool {
    for (p in layout.problems()) if (p.indexOf(text) >= 0) return true;
    return false;
  }

  /** A small panel with a wheel, an E-stop, a rotary switch and a button: 160 x 100 mm, 48 mm deep. */
  static function panel():PanelLayout return new PanelLayout(160.0, 100.0, 3.0, 3.0, 48.0, [
    new PlacedPart("wheel", PartPresets.yumoIsm6045(), 45.0, 0.0),
    new PlacedPart("estop", PartPresets.xb2Es542(), -50.0, 25.0),
    new PlacedPart("axis", PartPresets.rotarySwitch(12), -55.0, -15.0),
    new PlacedPart("button", PartPresets.pushButton16(), -20.0, -15.0)
  ]);

  public static function run():Void {
    var layout = panel();
    check(layout.problems().length == 0, "The sample panel must pass its own checks: " + layout.problems().join(" | "));
    check(layout.requiredInnerDepth() == 43.0, "The E-stop sets the depth");
    check(layout.overallHeight() == 48.0 + 3.0 + 38.0, "Overall height is case depth, plate and the wheel with its handle");
    check(Math.abs(layout.partsMassGrams() - (250.0 + 52.4 + 25.0 + 8.0)) < 1e-9, "Parts mass " + layout.partsMassGrams());
    check(layout.assumed().join(",") == "axis,button" && layout.control("wheel").source() == "vendor-drawing", "Only the switch and button are assumed");

    // Each check catches what it should.
    var wheel = layout.control("wheel");
    check(hasProblem(layout.withControl(wheel.movedTo(60.0, 0.0)), "wheel does not fit"), "Wheel off the panel");
    check(hasProblem(layout.withControl(layout.control("button").movedTo(-45.0, -15.0)), "axis and button"), "Overlap");
    check(hasProblem(layout.withControl(layout.control("estop").movedTo(-50.0, 31.0)), "estop does not fit"), "E-stop above the panel");
    check(hasProblem(layout.withControl(layout.control("estop").movedTo(-76.0, 25.0)), "body of estop reaches into the case wall"), "A rear body in the wall");
    var deep = new EmergencyStop(new ComponentValues().setToken("model", ControlCatalogs.CUSTOM).setNumber("blockLength", 38));
    check(hasProblem(layout.withControl(new PlacedPart("estop", deep, -50.0, 25.0)), "estop reaches 57 mm"), "An E-stop with a long block is too deep for the case");
    var overlapping = new PanelLayout(160.0, 100.0, 3.0, 3.0, 48.0, [new PlacedPart("a", PartPresets.pushButton16(), 0.0, 0.0), new PlacedPart("b", PartPresets.pushButton16(), 23.0, 0.0)]);
    check(hasProblem(overlapping, "bodies of a and b overlap"), "Rear bodies that overlap");
    var thick = new PanelLayout(160.0, 100.0, 4.0, 3.0, 48.0, [new PlacedPart("wheel", PartPresets.vallderMpg(), 0.0, 0.0)]);
    check(hasProblem(thick, "wheel allows 3"), "A panel thicker than the Vallder studs allow");
    check(new PanelLayout(160.0, 100.0, 6.0, 3.0, 48.0, [new PlacedPart("wheel", PartPresets.yumoIsm6045(), 0.0, 0.0)]).problems().length == 0, "The YUMO studs allow 6 mm");
    check(PanelLayout.gapBetween(wheel, layout.control("estop")) > 5.0, "Wheel and E-stop gap");
    rejects("duplicate ids", () -> new PanelLayout(160.0, 100.0, 3.0, 3.0, 48.0, [wheel, wheel]));
    rejects("no part", () -> new PlacedPart("x", null, 0.0, 0.0));
    rejects("a part that states no panel mount", () -> new PlacedPart("x", new machinekit.control.SafetyRelay(), 0.0, 0.0));

    // The wheel's panel template: a 44 mm opening and three 3.2 mm holes on a 50.8 mm circle at 120 degrees.
    check(wheel.holes().length == 3, "Three stud holes");
    for (h in wheel.holes()) check(Math.abs(Math.sqrt(h.x * h.x + h.y * h.y) - 25.4) < 1e-9 && h.diameter == 3.2, "Stud hole on the 50.8 mm circle");

    // The plate solid: one body, with the volume of the plate less every cutout.
    var plate = PanelFacePlate.part(layout);
    var removed = 0.0;
    for (c in layout.controls) {
      removed += FootprintGeometry.area(c.cutout());
      for (hole in c.holes()) removed += Math.PI * hole.diameter * hole.diameter / 4;
    }
    var expected = (RoundedRectangle.area(160.0, 100.0, PanelFacePlate.CORNER_RADIUS) - removed) * 3.0;
    check(plate.solidCount() == 1, "One solid");
    check(Math.abs(plate.volume() - expected) / expected < 1e-3, 'Plate volume ${plate.volume()} vs $expected');
    // No part overlaps another or the plate.
    plate.close();
    var report = AssemblyIntegrity.check(PanelAssemblyBuilder.build(layout, () -> PanelFacePlate.part(layout)));
    check(report.errors().length == 0, "Interference: " + [for (e in report.errors()) e.message].join(" | "));
    // Two buttons on top of each other are an error the assembly check reports.
    var stacked = new PanelLayout(160.0, 100.0, 3.0, 3.0, 48.0, [new PlacedPart("a", PartPresets.pushButton16(), 0.0, 0.0), new PlacedPart("b", PartPresets.pushButton16(), 10.0, 0.0)]);
    check(AssemblyIntegrity.check(PanelAssemblyBuilder.build(stacked, () -> PanelFacePlate.part(stacked))).errors().length > 0, "Overlapping parts are reported");
    var svg = PanelLayoutSvg.render(layout);
    check(svg.indexOf("<svg") == 0 && svg.indexOf(">wheel<") > 0, "SVG drawing");
    trace("panel layout tests passed");
  }
  public static function main():Void run();
}
