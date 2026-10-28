package machinekit.hmi;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;

/** Screw heads for the terminals of bought devices, drawn pointing back (-z) with their base on z = 0. */
class ScrewHeads {
  /**
   * A pan head `height` tall with a cross recess `recess` across and `depth` deep, of radius `radius` at its widest. The
   * caller owns the solid and closes it.
   */
  public static function pan(radius:Float, height:Float, recess:Float, depth:Float):Part {
    var head = Part.revolve([
      {r: 0.0, z: 0.0},
      {r: radius, z: 0.0},
      {r: radius * 1.05, z: -height * 0.4},
      {r: radius * 0.86, z: -height * 0.9},
      {r: radius * 0.55, z: -height},
      {r: 0.0, z: -height}
    ]);
    var cross = [
      slot(recess, 0.3 * recess / 2.8 + 0.3, height, depth),
      slot(0.3 * recess / 2.8 + 0.3, recess, height, depth)
    ];
    return Solids.cut(head, cross);
  }

  static function slot(width:Float, length:Float, height:Float, depth:Float):Part {
    var raw = Part.box(width, length, depth + 0.2);
    var moved = raw.translated(new Vector(0, 0, -height - 0.2));
    raw.close();
    return moved;
  }
}
