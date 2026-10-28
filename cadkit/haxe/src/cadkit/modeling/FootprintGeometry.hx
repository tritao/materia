package cadkit.modeling;

/** What can be asked of a Footprint: its extent, distances and gaps, and the solid that cuts it. */
class FootprintGeometry {
  /** Half the shape's width and height. */
  public static function halfExtent(f:Footprint):{x:Float, y:Float} return switch f {
    case Circle(d): {x: d / 2, y: d / 2};
    case Rect(w, h): {x: w / 2, y: h / 2};
    case _: throw "Unknown footprint";
  };

  /** The shape grown by a margin on every side. */
  public static function grow(f:Footprint, margin:Float):Footprint return switch f {
    case Circle(d): Circle(d + 2 * margin);
    case Rect(w, h): Rect(w + 2 * margin, h + 2 * margin);
    case _: throw "Unknown footprint";
  };

  /** Radius of the smallest circle around the shape. */
  public static function maxRadius(f:Footprint):Float return switch f {
    case Circle(d): d / 2;
    case Rect(w, h): Math.sqrt(w * w + h * h) / 2;
    case _: throw "Unknown footprint";
  };

  public static function area(f:Footprint):Float return switch f {
    case Circle(d): Math.PI * d * d / 4;
    case Rect(w, h): w * h;
    case _: throw "Unknown footprint";
  };

  /** Distance from a point to the edge of the shape centred at (fx, fy), negative inside it. */
  public static function distanceTo(f:Footprint, fx:Float, fy:Float, px:Float, py:Float):Float return switch f {
    case Circle(d): Math.sqrt(sq(px - fx) + sq(py - fy)) - d / 2;
    case Rect(w, h):
      var dx = Math.abs(px - fx) - w / 2, dy = Math.abs(py - fy) - h / 2;
      dx > 0 || dy > 0 ? Math.sqrt(sq(Math.max(dx, 0.0)) + sq(Math.max(dy, 0.0))) : Math.max(dx, dy);
    case _: throw "Unknown footprint";
  };

  /** Gap between two shapes (negative when they overlap), each centred at its own position. */
  public static function gap(a:Footprint, ax:Float, ay:Float, b:Footprint, bx:Float, by:Float):Float return switch [a, b] {
    case [Circle(da), Circle(db)]: Math.sqrt(sq(ax - bx) + sq(ay - by)) - da / 2 - db / 2;
    case [Circle(_), Rect(_, _)]: distanceTo(b, bx, by, ax, ay) - radius(a);
    case [Rect(_, _), Circle(_)]: distanceTo(a, ax, ay, bx, by) - radius(b);
    case [Rect(wa, ha), Rect(wb, hb)]:
      var gx = Math.abs(ax - bx) - (wa + wb) / 2, gy = Math.abs(ay - by) - (ha + hb) / 2;
      gx > 0 && gy > 0 ? Math.sqrt(gx * gx + gy * gy) : Math.max(gx, gy);
    case _: throw "Unknown footprint";
  };

  /** The solid that cuts the shape through z0 to z1, centred at (x, y). */
  public static function solid(f:Footprint, x:Float, y:Float, z0:Float, z1:Float):Part return switch f {
    case Circle(d): Part.cylinderSpan(d / 2, z0, z1, x, y);
    case Rect(w, h):
      var raw = Part.box(w, h, z1 - z0);
      var moved = raw.translated(new Vector(x, y, z0));
      raw.close();
      moved;
    case _: throw "Unknown footprint";
  };

  static function radius(f:Footprint):Float return switch f {
    case Circle(d): d / 2;
    case _: throw "Not a circle";
  };

  static function sq(v:Float):Float return v * v;
}
