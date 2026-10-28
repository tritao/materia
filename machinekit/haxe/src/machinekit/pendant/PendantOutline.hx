package machinekit.pendant;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** The pendant's rounded-rectangle outline, extruded in z. */
class PendantOutline {
  /** Plan area of the rounded rectangle, for volume checks. */
  public static function area(width:Float, height:Float, radius:Float):Float
    return width * height - (4.0 - Math.PI) * radius * radius;

  /** A rounded box centred on x and y, spanning z0 to z1. */
  public static function box(width:Float, height:Float, radius:Float, z0:Float, z1:Float):Part {
    if (!(radius > 0) || radius * 2 >= Math.min(width, height) || !(z1 > z0)) throw "Outline needs a radius below half its size and a positive height";
    var pieces:Array<Part> = [];
    var h = z1 - z0;
    var cross = Part.box(width - 2 * radius, height, h);
    pieces.push(shifted(cross, z0));
    var wide = Part.box(width, height - 2 * radius, h);
    pieces.push(shifted(wide, z0));
    for (sx in [-1.0, 1.0]) for (sy in [-1.0, 1.0])
      pieces.push(Part.cylinderSpan(radius, z0, z1, sx * (width / 2 - radius), sy * (height / 2 - radius)));
    var union = pieces[0];
    for (i in 1...pieces.length) {
      var next = union.combine(pieces[i]);
      union.close();
      pieces[i].close();
      union = next;
    }
    return union;
  }

  static function shifted(part:Part, z:Float):Part {
    var moved = part.translated(new Vector(0.0, 0.0, z));
    part.close();
    return moved;
  }
}
