package machinekit.hmi;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/**
 * The panel plate as a solid: a rounded plate with the cutouts of every control and the extra holes of
 * the wheel. Printed on its own it is the layout mock-up: drop the real parts into it. With screw
 * positions (see PendantCase) it also carries M3 counterbored holes for the posts of the shell.
 */
class PanelFacePlate {
  public static inline var CORNER_RADIUS:Float = 8.0;
  public static inline var SCREW_HOLE:Float = 3.4;
  public static inline var COUNTERBORE:Float = 6.2;
  public static inline var COUNTERBORE_DEPTH:Float = 1.8;

  public static function part(face:PanelLayout, ?fasteners:Array<{x:Float, y:Float}>):Part {
    var plate = PanelOutline.box(face.width, face.height, CORNER_RADIUS, 0.0, face.thickness);
    var tools:Array<Part> = [];
    try {
      for (c in face.controls) {
        tools.push(tool(c.cutout(), c.x, c.y, face.thickness));
        for (hole in c.holes()) tools.push(tool(Circle(hole.diameter), c.x + hole.x, c.y + hole.y, face.thickness));
      }
      if (fasteners != null) for (screw in fasteners) {
        tools.push(tool(Circle(SCREW_HOLE), screw.x, screw.y, face.thickness));
        tools.push(Part.cylinderSpan(COUNTERBORE / 2, face.thickness - COUNTERBORE_DEPTH, face.thickness + 1.0, screw.x, screw.y));
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
    case _: throw "Unknown footprint";
  };
}
