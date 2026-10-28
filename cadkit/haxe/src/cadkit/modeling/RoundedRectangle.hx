package cadkit.modeling;

/** A rounded rectangle centred on x and y, extruded in z. */
class RoundedRectangle {
  /** Plan area, for volume checks. */
  public static function area(width:Float, height:Float, radius:Float):Float
    return width * height - (4.0 - Math.PI) * radius * radius;

  /** The rounded box spanning z0 to z1; the radius must be below half the smaller side. */
  public static function box(width:Float, height:Float, radius:Float, z0:Float, z1:Float):Part {
    if (!(radius > 0) || radius * 2 >= Math.min(width, height) || !(z1 > z0)) throw "A rounded rectangle needs a radius below half its size and a positive height";
    var h = z1 - z0;
    var pieces:Array<Part> = [shifted(Part.box(width - 2 * radius, height, h), z0), shifted(Part.box(width, height - 2 * radius, h), z0)];
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
