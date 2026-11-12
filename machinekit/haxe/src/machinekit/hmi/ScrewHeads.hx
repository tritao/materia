package machinekit.hmi;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;

/** Screw heads for the terminals of bought devices, drawn pointing back (-z) with their base at `z`. */
class ScrewHeads {
  /**
   * A pan head `height` tall with a cross recess `recess` across and `depth` deep, of radius `radius` at its widest, on the axis
   * through (x, y) with its base at z.
   */
  public static function pan(b:DocumentBuilder, radius:Float, height:Float, recess:Float, depth:Float, x:Float = 0, y:Float = 0,
      z:Float = 0):Feature {
    var section = [[0.0, 0.0], [radius, 0.0], [radius * 1.05, -height * 0.4], [radius * 0.86, -height * 0.9], [radius * 0.55, -height],
      [0.0, -height]];
    var head = b.revolve(b.polygon([for (p in section) new Vector(p[0], z + p[1], 0)], new Plane(new Vector(x, y, 0), Vector.X(), Vector.Y().scale(-1))),
      new Vector(x, y, 0), Vector.Z(), 2 * Math.PI);
    var narrow = 0.3 * recess / 2.8 + 0.3;
    function slot(w:Float, l:Float):Feature return b.boxBetween(x - w / 2, x + w / 2, y - l / 2, y + l / 2, z - height - 0.2, z - height + depth);
    return b.cutAll(head, [slot(recess, narrow), slot(narrow, recess)], ["recess across", "recess along"]);
  }
}
