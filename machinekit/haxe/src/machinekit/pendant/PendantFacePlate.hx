package machinekit.pendant;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/**
 * The face plate as a solid: a plate with the cutouts of every control and the extra holes of the
 * wheel. Printed on its own it is the layout mock-up: drop the real parts into it. Corners are
 * square for now.
 */
class PendantFacePlate {
  public static function part(face:PendantFace):Part {
    var plate = Part.box(face.width, face.height, face.thickness);
    var tools:Array<Part> = [];
    try {
      for (c in face.controls) {
        tools.push(tool(c.cutout, c.x, c.y, face.thickness));
        for (hole in c.extraHoles) tools.push(tool(Circle(hole.diameter), c.x + hole.x, c.y + hole.y, face.thickness));
      }
      var result = plate.subtractAll(tools);
      plate.close();
      for (t in tools) t.close();
      return result;
    } catch (error:Dynamic) {
      plate.close();
      for (t in tools) t.close();
      throw error;
    }
  }

  /** A cutting tool through the whole plate, with a millimetre either side so the cut is clean. */
  static function tool(f:Footprint, x:Float, y:Float, thickness:Float):Part return switch f {
    case Circle(d): Part.cylinderSpan(d / 2, -1.0, thickness + 1.0, x, y);
    case Rect(w, h):
      var raw = Part.box(w, h, thickness + 2.0);
      var moved = raw.translated(new Vector(x, y, -1.0));
      raw.close();
      moved;
  };
}
