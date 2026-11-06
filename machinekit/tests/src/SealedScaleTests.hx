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
    options();
    var compared = SealedScaleStepTests.run();
    Sys.println('Sealed scale: length selection, overall length, mass, connectors, tolerances, feedback and ${compared} of 3 supplier STEP comparisons passed');
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
    near(b.get_max().get_y(), 18, "housing thickness (along the fixing holes)");
    near(b.get_min().get_x(), -13.5, "hole axis 13.5 from the bottom face");
    near(b.get_max().get_x(), 18.7, "reading face 18.7 from the hole axis");
    check(g.solidCount() == 1 && g.volume() < h.geometry(Envelope).volume(), "end holes cut");
    g.close();
    var head = part("JFT-40", 120, "head").geometry(), hb = head.shape.bounds();
    var body = part("JFT-40", 120, "head").bodyBox();
    near(body[5] - body[4], 97.5, "head body 97.5");
    near(body[4] - SealedScaleData.slim().bootLength, hb.get_min().get_z() + SealedScaleData.slim().cableStraight, "boot and cable stub beyond the body");
    near(hb.get_max().get_z(), body[5], "head ends at its body");
    near(hb.get_min().get_x(), 19.7, "head body 1 mm beyond the reading face");
    near(hb.get_max().get_x(), 32.7, "head body 13 high");
    near(hb.get_min().get_y(), 0, "head level with the housing, free face at y=0");
    near(hb.get_max().get_y(), 18, "head 18 thick");
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
    near(h.connector("mount").frame.y, 18, "housing mounting face");
    near(h.connector("axis").frame.z, 16.5, "measuring start");
    near(h.connector("end1").frame.z, 11.5, "end hole 1");
    near(h.connector("end2").frame.z - h.connector("end1").frame.z, 120 + 115, "hole spacing ML+115");
    check([for (c in head.connectors()) if (c.name == "end1") c].length == 0, "head has no end holes");
    near(head.connector("hole2").frame.z - head.connector("hole1").frame.z, 56, "head slot pitch");
    near(head.connector("hole1").frame.z, -17.5, "slot 1 from the reading finger");
    near(head.connector("hole1").frame.x, 28.7, "slot axis 9 above the gap face");
    near(head.connector("mount").frame.y, 0, "head carrier face on the free side");
    near(head.connector("cable").frame.z, -100.5, "cable stub end");
    check(head.hasPort("biss") && head.port("biss").kind == Signal, "BiSS-C signal port on the head");
    check(h.hasPort("air"), "air inlet on the housing");
    check(part("JFT-10", 140).connectors().length == 2 && part("JFT-10", 140, "head").connectors().length == 4, "JFT-10 connectors");
    near(part("JFT-10", 140, "head").connector("hole2").frame.z - part("JFT-10", 140, "head").connector("hole1").frame.z, 76, "JFT-10 head pitch 76");
    // No contact: the head stands `gap` beyond the reading face, level with the housing.
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

  /** The fastening faces: the housing's two M8 through the end holes (Ø15 counterbore 5.75 from each face), the head's two M4 through its slots into the carrier. */
  static function faces():Void {
    var h = part("JFT-40", 120), head = part("JFT-40", 120, "head");
    var housing = h.fasteningFaces(), clampedHead = head.fasteningFaces();
    check(housing.length == 1 && clampedHead.length == 1, "one face each");
    var clamped = housing[0], holes:Array<machinekit.component.BoltedFace.BoltHole> = clamped.holes ?? [], pair:Array<machinekit.component.BoltedFace.BoltHole> = clampedHead[0].holes ?? [];
    check(clamped.isClamped() && holes.length == 2 && Math.abs(holes[0].x - 11.5) < 1e-9 && Math.abs(holes[1].x - (h.overallLength() - 11.5)) < 1e-9,
      "the housing's two M8 sit 11.5 mm from each end");
    switch clamped.role {
      case Clamped(size, clearance, grip, recess):
        check(size == "M8" && clearance == 9.6 && recess != null && recess.diameter == 15 && Math.abs(recess.depth - 5.75) < 1e-9 && Math.abs(grip + recess.depth - 18) < 1e-9,
          "Ø9.6 holes, Ø15 counterbore 5.75 deep, the rest of the 18 mm under it");
      default:
        check(false, "the housing's face is clamped");
    }
    check(clampedHead[0].isClamped(), "the head's face is clamped (its slots are through)");
    switch clampedHead[0].role {
      case Clamped(size, clearance, grip, recess):
        check(size == "M4" && clearance == 4.6 && recess != null && recess.diameter == 7.6 && recess.depth == 5 && Math.abs(grip - 13) < 1e-9, "Ø4.6 slots for M4, Ø7.6 counterbore 5 deep");
      default:
    }
    check(pair.length == 2 && Math.abs(pair[1].x - 38.5) < 1e-9 && Math.abs(pair[0].x + 17.5) < 1e-9, "the head's M4 pair 56 mm apart, 10.5 mm past the reading finger's centre");
    // The counterbores are cut from both faces and the holes run through the thickness: the mounting face keeps no Ø15 bearing ring beyond the relief.
    var body = h.geometry(), full = machinekit.component.Solids.named(cadkit.modeling.Part.prism([new cadkit.modeling.Vector(-13.5, 0), new cadkit.modeling.Vector(18.7, 0), new cadkit.modeling.Vector(18.7, 18), new cadkit.modeling.Vector(-13.5, 18)], 0, h.overallLength()), "ref");
    var removed = full.volume() - body.volume(), expected = 2 * Math.PI / 4 * (9.6 * 9.6 * 18 + 2 * (15 * 15 - 9.6 * 9.6) * 5.75);
    check(Math.abs(removed - expected) < 0.5 * expected / 100, 'the end holes remove ${removed} mm3, expected ${expected}');
    body.close();
    full.close();
    var headBody = head.geometry(), envelope = head.geometry(Envelope);
    // The drawn volume is the envelope's less both slots (each a Ø4.6 x 2.4 stadium through the 18 mm, with a Ø7.6 x 2.4 stadium 5 deep from each face).
    var oneSlot = Math.PI / 4 * 4.6 * 4.6 * 18 + 4.6 * 2.4 * 18 + 2 * (Math.PI / 4 * (7.6 * 7.6 - 4.6 * 4.6) * 5 + (7.6 - 4.6) * 2.4 * 5);
    check(Math.abs(envelope.volume() - headBody.volume() - 2 * oneSlot) < 0.01 * 2 * oneSlot, 'the head\'s two slots are drawn: ${envelope.volume() - headBody.volume()} mm3, expected ${2 * oneSlot}');
    headBody.close();
    envelope.close();
  }

  /** Installation choices: head side mirrors the pair, the air port and the cable's bend add their envelopes. */
  static function options():Void {
    var mirrored = new SealedScalePart(new ComponentValues().setNumber("measuringLength", 120).setToken("variant", "head").setToken("headSide", "-x"));
    var g = mirrored.geometry(), b = g.shape.bounds();
    near(b.get_min().get_x(), -32.7, "mirrored head at -X");
    near(b.get_max().get_x(), -19.7, "mirrored head at -X, far face");
    g.close();
    var housing = new SealedScalePart(new ComponentValues().setNumber("measuringLength", 120).setToken("variant", "housing").setToken("airPort", "end"));
    var a = housing.geometry(), ab = a.shape.bounds();
    check(a.solidCount() == 1, "air fitting joins the housing");
    near(ab.get_max().get_z(), 258 + 18 + 25, "fitting and tube past the end face", 1e-6);
    near(housing.connector("air").frame.x, 11.7, "air port 7 below the reading face");
    near(housing.connector("air").frame.z, 258, "air port on the far end face");
    a.close();
    var bent = new SealedScalePart(new ComponentValues().setNumber("measuringLength", 120).setToken("variant", "head").setToken("cableBend", "-y"));
    var c = bent.geometry(), cb = c.shape.bounds();
    check(c.solidCount() == 1, "cable bend joins the head");
    // Stub end at z -100.5, y 9: quarter bend of 75 toward -y ends at y -66, then 37 straight on.
    near(cb.get_min().get_y(), 9 - 75 - 37, "cable end after the bend and its straight run", 0.5);
    c.close();
    fails(() -> new SealedScalePart(new ComponentValues().setToken("series", "JFT-10").setNumber("measuringLength", 140).setToken("airPort", "end")), "JFT-10 has no air port");
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
