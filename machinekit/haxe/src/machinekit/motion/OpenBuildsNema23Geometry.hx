package machinekit.motion;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;

/** Adapted from Gumyr's bd_warehouse, Apache-2.0.
 * See machinekit/third-party/bd_warehouse/NOTICE for pinned source and changes.
 */
class OpenBuildsNema23Geometry {
  static inline final HALF = 28.2;
  static inline final FILLET = 2.5;
  static inline final HOLE = 23.57;
  static inline final RELIEF = 5.0;

  /** The motor `length` long: two end caps on the housing section, the core inset 0.1 mm, the filleted flange, pilot and flatted shaft. */
  public static function record(b:DocumentBuilder, length:Float):Feature {
    if (!Math.isFinite(length) || length <= 0) throw "Invalid OpenBuilds motor length";
    var cap = Math.min(length / 2, 11), flangeDepth = Math.min(length, 4.8);
    var section = Nema23Outline.edges(HALF, FILLET, HOLE, RELIEF), core = Nema23Outline.edges(HALF, FILLET, HOLE, RELIEF, 0.1);
    var up = Vector.Z(), axis = new Vector(0, 0, 0);
    var parts:Array<Feature> = [
      b.label(Nema23Outline.prism(b, section, 0, cap), "Front cap"),
      b.label(Nema23Outline.prism(b, section, -length + cap, cap), "Rear cap"),
      // Upstream's inset stack starts at the front plane and overlaps the caps.
      b.label(Nema23Outline.prism(b, core, 0, length - cap), "Core")
    ];
    var flange = b.extrude(b.roundedRectangle(56.4, 56.4, 4.6, new Plane(new Vector(0, 0, -flangeDepth), Vector.X(), up)), flangeDepth);
    var holes:Array<Feature> = [for (x in [-HOLE, HOLE]) for (y in [-HOLE, HOLE]) b.cylinderAlong(new Vector(x, y, 0), up, 2.55, -flangeDepth - 0.1, 0.1)];
    parts.push(b.label(b.cutAll(flange, holes, [for (k in 0...holes.length) 'mounting hole ${k + 1}']), "Flange"));
    parts.push(b.label(b.cylinderAlong(axis, up, 19.05, 0, 1.6), "Pilot"));
    var shaft = b.cylinderAlong(axis, up, 3.175, 0, 20.6);
    parts.push(b.label(b.cutAll(shaft, [b.boxBetween(-4, 4, 2.675, 8, 5.6, 20.7)], ["flat"]), "Shaft"));
    return b.label(b.unionAll(parts), "OpenBuilds NEMA 23");
  }
}
