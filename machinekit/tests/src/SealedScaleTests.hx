import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.component.MachineKitComponents;
import machinekit.motion.EncoderPart;
import machinekit.motion.PrecisionErrors;
import machinekit.motion.SealedScaleData;
import machinekit.motion.SealedScalePart;

/** Sealed absolute scales (JFT-40 / JFT-10): length selection, overall length, mass, connectors, tolerances, feedback. */
class SealedScaleTests {
  static function check(condition:Bool, message:String):Void if (!condition) throw 'Sealed scale: $message';
  static function near(a:Null<Float>, b:Float, message:String, tolerance:Float = 1e-6):Void check(
    a != null && Math.isFinite(a) && Math.abs(a - b) <= tolerance,
    '$message: $a != $b'
  );
  static function fails(run:() -> Void, message:String):Void {
    var failed = false;
    try run() catch (_:Dynamic) failed = true;
    check(failed, message);
  }
  static function part(series:String, ml:Float, variant:String = "housing", plate:Bool = false):SealedScalePart return new SealedScalePart(
    new ComponentValues().setToken("series", series).setNumber("measuringLength", ml).setToken("variant", variant).setBoolean("mountingPlate", plate)
  );

  public static function main():Void {
    MachineKitComponents.standard();
    lengths();
    sizes();
    connectorsAndGap();
    feedback();
    recipes();
    faces();
    Sys.println("Sealed scale: length selection, overall length, mass, connectors, tolerances and feedback passed");
  }

  static function lengths():Void {
    var slim = SealedScaleData.slim().lengths, full = SealedScaleData.full().lengths;
    check(slim.length == 27 && slim[0] == 70 && slim[15] == 820 && slim[16] == 920 && slim[17] == 1020 && slim[18] == 1140
      && slim[26] == 2040, "JFT-40 list: 16 steps of 50, then 920, 1020, 1140..1840 by 100, 2040");
    check(full.length == 30 && full[0] == 140 && full[17] == 1840 && full[18] == 2040 && full[29] == 4240, "JFT-10 list");
    near(SealedScaleData.standardLength("JFT-40", 95), 120, "95 -> 120");
    near(SealedScaleData.standardLength("JFT-40", 210), 220, "210 -> 220");
    near(SealedScaleData.standardLength("JFT-40", 30), 70, "30 -> 70");
    near(SealedScaleData.standardLength("JFT-40", 70), 70, "70 -> 70");
    near(SealedScaleData.standardLength("JFT-40", 270), 270, "270 -> 270");
    near(SealedScaleData.standardLength("JFT-40", 821), 920, "821 -> 920");
    near(SealedScaleData.standardLength("JFT-40", 1021), 1140, "1021 -> 1140");
    near(SealedScaleData.standardLength("JFT-40", 2040), 2040, "2040");
    near(SealedScaleData.standardLength("JFT-10", 100), 140, "JFT-10 100 -> 140");
    near(SealedScaleData.standardLength("JFT-10", 1900), 2040, "JFT-10 1900 -> 2040");
    near(SealedScaleData.standardLength("JFT-10", 3041), 3240, "JFT-10 3041 -> 3240");
    fails(() -> SealedScaleData.standardLength("JFT-40", 2041), "past 2040 has no standard length");
    fails(() -> SealedScaleData.standardLength("JFT-40", 0), "zero travel rejected");
    fails(() -> SealedScaleData.standardLength("JFT-99", 100), "unknown series rejected");
    fails(() -> part("JFT-40", 100), "non-standard ML rejected");
    var x = SealedScalePart.pair("JFT-40", 95), z = SealedScalePart.pair("JFT-40", 270, "5"), y = SealedScalePart.pair("JFT-40", 40, "5");
    near(x.housing.measuringLength, 120, "X ML 120");
    near(z.housing.measuringLength, 270, "Z ML 270");
    near(y.housing.measuringLength, 70, "Y ML 70");
  }

  static function sizes():Void {
    var h = part("JFT-40", 120);
    near(h.overallLength(), 258, "JFT-40 ML+138");
    near(part("JFT-40", 120, "housing", true).overallLength(), 225, "JFT-40 with plate ML+105");
    near(part("JFT-10", 140).overallLength(), 261, "JFT-10 ML+121");
    near(h.mounting().housingParallelism, 0.1, "JFT-40 housing parallelism");
    near(h.mounting().headParallelism, 0.05, "JFT-40 head parallelism");
    near(h.mounting().headLateral, 0.2, "JFT-40 head lateral");
    near(h.mounting().gap, 1, "JFT-40 gap");
    near(h.mounting().gapTolerance, 0.3, "JFT-40 gap tolerance");
    near(part("JFT-10", 140).mounting().headLateral, 0.1, "JFT-10 head lateral");
    near(part("JFT-10", 140).mounting().headParallelism, 0.2, "JFT-10 head parallelism");
    near(part("JFT-10", 140).mounting().gap, 1.5, "JFT-10 gap");
    // Mass: 0.2 + 0.5 kg/m (JFT-40), 0.4 + 2.6 kg/m (JFT-10), head included in the fixed part.
    var total40 = part("JFT-40", 520).massProperties().mass + part("JFT-40", 520, "head").massProperties().mass;
    near(total40, 0.2 + 0.5 * 0.52 + 0.2, "JFT-40 housing+head mass", 1e-9);
    near(part("JFT-10", 1040).massProperties().mass, 0.4 + 2.6 * 1.04, "JFT-10 housing mass", 1e-9);
    // Geometry.
    var g = h.geometry(), b = g.shape.bounds();
    near(b.get_min().get_z(), 0, "housing starts at z=0");
    near(b.get_max().get_z(), 258, "housing ends at ML+138");
    near(b.get_max().get_y(), 13.5, "housing height");
    near(b.get_max().get_x(), 9, "housing half depth");
    check(g.solidCount() == 1 && g.volume() < h.geometry(Envelope).volume(), "end holes cut");
    g.close();
    var head = part("JFT-40", 120, "head").geometry(), hb = head.shape.bounds();
    near(hb.get_max().get_z() - hb.get_min().get_z(), 115.3, "head length 115.3");
    near(hb.get_max().get_y(), 0, "head face toward housing at y=0");
    head.close();
    fails(() -> part("JFT-40", 1340), "JFT-40 needs the plate from ML 1340");
    check(part("JFT-40", 1340, "housing", true).mountingPlate, "plate accepted");
    check(SealedScalePart.pair("JFT-40", 1300).housing.mountingPlate, "pair adds the mandatory plate");
    fails(() -> part("JFT-10", 140, "housing", true), "JFT-10 has no plate");
    fails(() -> new SealedScalePart(new ComponentValues().setToken("series", "JFT-10").setNumber("measuringLength", 3240)), "+-3 um stops at 3040");
    check(new SealedScalePart(new ComponentValues().setToken("series", "JFT-10").setNumber("measuringLength", 3240).setToken("accuracy", "5")).accuracy == 5, "+-5 um beyond");
  }

  static function connectorsAndGap():Void {
    var h = part("JFT-40", 120), head = part("JFT-40", 120, "head");
    near(h.connector("mount").frame.y, 13.5, "housing mounting face");
    near(h.connector("axis").frame.z, 16.5, "measuring start");
    near(h.connector("end1").frame.z, 11.5, "end hole 1");
    near(h.connector("end2").frame.z - h.connector("end1").frame.z, 120 + 115, "hole spacing ML+115");
    check([for (c in head.connectors()) if (c.name == "end1") c].length == 0, "head has no end holes");
    near(head.connector("hole2").frame.z - head.connector("hole1").frame.z, 97.5, "head M5 pitch");
    near(head.connector("mount").frame.y, -11.2, "head mount face away from housing");
    check(head.hasPort("biss") && head.port("biss").kind == Signal, "BiSS-C signal port on the head");
    check(h.hasPort("air"), "air inlet on the housing");
    check(part("JFT-10", 140).connectors().length == 2 && part("JFT-10", 140, "head").connectors().length == 4, "JFT-10 connectors");
    near(part("JFT-10", 140, "head").connector("hole2").frame.z - part("JFT-10", 140, "head").connector("hole1").frame.z, 76, "JFT-10 head pitch 76");
    // No contact: head is `gap` below the housing, so a 1 mm running gap is what a mate leaves.
    near(h.gap(), 1, "running gap data");
  }

  static function feedback():Void {
    var h = part("JFT-40", 120);
    var asEncoder:EncoderPart = h;
    var e = asEncoder.encoder("x", "X");
    check(e.kind == "absolute" && e.joint == "X" && e.index == null, "absolute, no index");
    near(e.counts, 200000, "5 nm -> 200000 counts/mm");
    near(new SealedScalePart(new ComponentValues().setToken("resolution", "2.5")).encoder("x", "X").counts, 400000, "2.5 nm");
    near(new SealedScalePart(new ComponentValues().setToken("resolution", "10")).encoder("x", "X").counts, 100000, "10 nm");
    var head:EncoderPart = part("JFT-40", 120, "head");
    near(head.encoder("x", "X").counts, 200000, "head reads the same");
    var errors = PrecisionErrors.of(h).all();
    check(errors[0].quantity == "scaleAccuracy" && errors[0].bound == 3 && errors[0].unit == "um", "accuracy +-3 um");
    near(errors[1].bound, 8, "glass expansion 8e-6/K");
    near(part("JFT-40", 120, "housing", true).thermalExpansion(), 9e-6, "with plate 9e-6/K");
    // A scale on a carriage joint wires like the tape did.
    var a = new MachineAssembly();
    a.addComponent("housing", h);
    a.addComponent("head", part("JFT-40", 120, "head"));
    check(a.components().length == 2, "both parts in an assembly");
  }

  /** The fastening faces: the housing's two M8 through the end holes (Ø15 counterbore from the head side only), the head's tapped M5 pair. */
  static function faces():Void {
    var h = part("JFT-40", 120), head = part("JFT-40", 120, "head");
    var housing = h.fasteningFaces(), tapped = head.fasteningFaces();
    check(housing.length == 1 && tapped.length == 1, "one face each");
    var clamped = housing[0], holes:Array<machinekit.component.BoltedFace.BoltHole> = clamped.holes ?? [], pair:Array<machinekit.component.BoltedFace.BoltHole> = tapped[0].holes ?? [];
    check(clamped.isClamped() && holes.length == 2 && Math.abs(holes[0].x - 11.5) < 1e-9 && Math.abs(holes[1].x - (h.overallLength() - 11.5)) < 1e-9,
      "the housing's two M8 sit 11.5 mm from each end");
    switch clamped.role {
      case Clamped(size, clearance, grip, recess):
        check(size == "M8" && clearance == 9.6 && recess != null && recess.diameter == 15 && Math.abs(grip + recess.depth - 13.5) < 1e-9, "Ø9.6 holes, Ø15 counterbore, floor under it");
      default:
        check(false, "the housing's face is clamped");
    }
    check(pair.length == 2 && Math.abs(pair[1].x - 48.75) < 1e-9 && Math.abs(pair[0].x + 48.75) < 1e-9, "the head's M5 pair at 97.5 mm");
    // The counterbore is cut only from the head side: the mounting face (y = 13.5) keeps the bearing area around the Ø9.6 hole.
    var body = h.geometry(), full = machinekit.component.Solids.named(cadkit.modeling.Part.prism([new cadkit.modeling.Vector(-9, 0), new cadkit.modeling.Vector(9, 0), new cadkit.modeling.Vector(9, 13.5), new cadkit.modeling.Vector(-9, 13.5)], 0, h.overallLength()), "ref");
    var removed = full.volume() - body.volume(), expected = 2 * Math.PI / 4 * (9.6 * 9.6 * 13.5 + (15 * 15 - 9.6 * 9.6) * 9.5);
    check(Math.abs(removed - expected) < 0.5 * expected / 100, 'the end holes remove ${removed} mm3, expected ${expected}; a through counterbore would remove more');
    body.close();
    full.close();
    var tappedBody = head.geometry();
    var block = 18.25 * 11.2 * 115.3, drill = machinekit.standard.SocketHeadCapScrew.catalog().get("M5").tapDrill;
    check(Math.abs(block - tappedBody.volume() - 2 * Math.PI / 4 * drill * drill * 8) < 1.0, "the head's two tapped holes are drawn, 8 mm deep");
    tappedBody.close();
  }

  static function recipes():Void {
    var h = part("JFT-40", 270);
    check(h.designation == "JFT-40-ML270-A3-R5-BISSC", "designation " + h.designation);
    var rebuilt:SealedScalePart = cast h.componentType().create(h.values());
    check(rebuilt.designation == h.designation && rebuilt.measuringLength == 270, "recipe rebuilds");
    check(h.bom.procurement != null, "bought");
    var tag = part("JFT-40", 270, "head").designation;
    check(tag == "JFT-40-ML270-A3-R5-BISSC-HEAD", "head designation " + tag);
  }
}
