package machinekit.motion;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;

/** 23HS22-2804S exterior reconstructed from the pinned 2018 supplier STEP.
 * See tools/suppliers/stepperonline-23hs22.json for provenance and measured datums.
 * Mechanical frame: face Z=0, housing toward -Z, shaft along +Z.
 * Omits lead exit/wires, casing screws and small chamfers; no guessed screw access bores.
 */
class StepperOnline23Geometry {
  public static inline final REFERENCE_SHA256:String =
    "2683a79867bfd965b62b3fc73803ce65c430efbecf5678f793347f19f188f793";
  public static inline final PROFILE:String = "stepperonline-23hs22-cad2018-v1";

  /**
   * The housing `length` long. Exact cylindrical side faces in the reference: outer R4 at (24.5, 15.57), concave R4 about the
   * mounting-hole centre (23.57, 23.57).
   */
  public static function record(b:DocumentBuilder, length:Float):Feature {
    if (!Math.isFinite(length) || length < 16) throw "StepperOnline-profile motor must be at least 16 mm long";
    var up = Vector.Z(), axis = new Vector(0, 0, 0);
    var housing = b.label(Nema23Outline.prism(b, Nema23Outline.edges(28.5, 4, 23.57, 4), 0, length), "Housing");
    var flange = b.extrude(b.roundedRectangle(57, 57, 4, new Plane(new Vector(0, 0, -5), Vector.X(), up)), 5);
    var holes:Array<Feature> = [for (x in [-23.57, 23.57]) for (y in [-23.57, 23.57]) b.cylinderAlong(new Vector(x, y, 0), up, 2.55, -5.1, 0.1)];
    var shaft = b.cylinderAlong(axis, up, 3.175, 0, 21);
    return b.label(b.unionAll([
      housing,
      b.label(b.cutAll(flange, holes, [for (k in 0...holes.length) 'mounting hole ${k + 1}']), "Flange"),
      b.label(b.cylinderAlong(axis, up, 19.05, 0, 1.6), "Pilot"),
      b.label(b.cutAll(shaft, [b.boxBetween(2.675, 8, -4, 4, 6, 21.1)], ["flat"]), "Shaft")
    ]), "StepperOnline 23HS22");
  }
}
