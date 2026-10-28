import cadkit.modeling.Part;
import machinekit.pendant.*;
import sys.io.File;

class PendantFaceTests {
  static function check(ok:Bool, message:String):Void if (!ok) throw message;
  static function rejects(message:String, body:() -> Void):Void {
    var failed = false;
    try body() catch (_:Dynamic) failed = true;
    check(failed, 'Expected rejection: $message');
  }
  static function hasProblem(face:PendantFace, text:String):Bool {
    for (p in face.problems()) if (p.indexOf(text) >= 0) return true;
    return false;
  }

  public static function run():Void {
    var face = ReferencePendant.face();
    check(face.problems().length == 0, "The reference layout must pass its own checks: " + face.problems().join(" | "));
    check(face.controls.length == 12, "Wheel, E-stop, display, two rotaries and seven buttons");
    check(face.requiredInnerDepth() == 45.0, "The E-stop sets the depth");
    check(face.overallHeight() == 50.0 + 3.0 + 41.0, "Overall height is case depth, plate and the wheel with its handle");
    check(Math.abs(face.partsMassGrams() - 472.0) < 1e-6, "Parts mass " + face.partsMassGrams());
    check(face.assumed().indexOf("wheel") < 0 && face.assumed().indexOf("estop") >= 0, "The wheel is vendor-drawing, the E-stop assumed");

    // Each check catches what it should.
    var wheel = face.control("wheel");
    check(hasProblem(face.withControl(wheel.movedTo(90.0, 0.0)), "wheel does not fit"), "Wheel off the face");
    check(hasProblem(face.withControl(face.control("f1").movedTo(-60.0, -8.0)), "step and f1"), "Overlap");
    check(hasProblem(face.withControl(face.control("estop").movedTo(-85.0, 36.0)), "estop does not fit"), "E-stop above the face");
    var shallow = new PendantFace(220.0, 110.0, 3.0, 3.0, 30.0, face.controls);
    check(hasProblem(shallow, "estop reaches 45 mm"), "A shallow case");
    var thick = new PendantFace(220.0, 110.0, 4.0, 3.0, 50.0, face.controls);
    check(hasProblem(thick, "wheel studs allow"), "A face plate thicker than the studs allow");
    check(PendantFace.gapBetween(wheel, face.control("estop")) > 5.0, "Wheel and E-stop gap");
    rejects("duplicate ids", () -> new PendantFace(220.0, 110.0, 3.0, 3.0, 50.0, [wheel, wheel]));
    rejects("unknown source", () -> new PendantControl("x", Button, 0.0, 0.0, Circle(5.0), Circle(6.0), 0.0, 0.0, 0.0, "guess"));

    // The wheel's panel template: a 45 mm hole and three 4.8 mm holes on a 72 mm circle at 120 degrees.
    check(wheel.extraHoles.length == 3, "Three stud holes");
    for (h in wheel.extraHoles) check(Math.abs(Math.sqrt(h.x * h.x + h.y * h.y) - 36.0) < 1e-9 && h.diameter == 4.8, "Stud hole on the 72 mm circle");

    // The face plate solid: one body, with the volume of the plate less every cutout.
    var plate = PendantFacePlate.part(face);
    var removed = 0.0;
    for (c in face.controls) {
      removed += switch c.cutout { case Circle(d): Math.PI * d * d / 4; case Rect(w, h): w * h; case _: 0.0; };
      for (hole in c.extraHoles) removed += Math.PI * hole.diameter * hole.diameter / 4;
    }
    var expected = (220.0 * 110.0 - removed) * 3.0;
    check(plate.solidCount() == 1, "One solid");
    check(Math.abs(plate.volume() - expected) / expected < 1e-3, 'Plate volume ${plate.volume()} vs $expected');
    var out = Sys.getEnv("PENDANT_FACE_STEP");
    if (out != null && out != "") plate.shape.exportStep(out);
    plate.close();
    var svg = PendantFaceSvg.render(face);
    check(svg.indexOf("<svg") == 0 && svg.indexOf(">wheel<") > 0 && svg.split("<circle").length > 10, "SVG drawing");
    var drawing = Sys.getEnv("PENDANT_FACE_SVG");
    if (drawing != null && drawing != "") File.saveContent(drawing, svg);
    trace("pendant face tests passed");
  }
  public static function main():Void run();
}
