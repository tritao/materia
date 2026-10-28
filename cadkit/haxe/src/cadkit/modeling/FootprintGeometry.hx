package cadkit.modeling;

/** What can be asked of a Footprint: its extent, distances and gaps, and the solid that cuts it. */
class FootprintGeometry {
  /** The shape's centre and what it is made of: a `Shifted` footprint unwrapped to its plain shape and total offset. */
  public static function resolve(f:Footprint): {shape: Footprint, dx: Float, dy: Float} return switch f {
    case Shifted(inner, dx, dy):
      var r = resolve(inner);
      {shape: r.shape, dx: r.dx + dx, dy: r.dy + dy};
    case _:
      {shape: f, dx: 0.0, dy: 0.0};
  };

  /** Half the width and height of the box about the footprint's origin that contains the shape. */
  public static function halfExtent(f:Footprint): {x: Float, y: Float} {
    var r = resolve(f);
    return switch r.shape {
      case Circle(d):
        {x: d / 2 + Math.abs(r.dx), y: d / 2 + Math.abs(r.dy)};
      case Rect(w, h):
        {x: w / 2 + Math.abs(r.dx), y: h / 2 + Math.abs(r.dy)};
      case _:
        throw "Unknown footprint";
    };
  }

  /** The box that exactly contains the shape when its footprint's origin is at (x, y). */
  public static function extent(f:Footprint, x:Float, y:Float): {x0: Float, x1: Float, y0: Float, y1: Float} {
    var r = resolve(f);
    var hx = switch r.shape {
      case Circle(d):
        d / 2;
      case Rect(w, _):
        w / 2;
      case _:
        throw "Unknown footprint";
    };
    var hy = switch r.shape {
      case Circle(d):
        d / 2;
      case Rect(_, h):
        h / 2;
      case _:
        throw "Unknown footprint";
    };
    var cx = x + r.dx, cy = y + r.dy;
    return {x0: cx - hx, x1: cx + hx, y0: cy - hy, y1: cy + hy};
  }

  /** The shape grown by a margin on every side. */
  public static function grow(f:Footprint, margin:Float):Footprint return switch f {
    case Circle(d):
      Circle(d + 2 * margin);
    case Rect(w, h):
      Rect(w + 2 * margin, h + 2 * margin);
    case Shifted(inner, dx, dy):
      Shifted(grow(inner, margin), dx, dy);
    case _:
      throw "Unknown footprint";
  };

  /** Radius of the smallest circle around the footprint's origin that contains the shape. */
  public static function maxRadius(f:Footprint):Float {
    var r = resolve(f);
    return switch r.shape {
      case Circle(d):
        Math.sqrt(sq(r.dx) + sq(r.dy)) + d / 2;
      case Rect(w, h):
        Math.sqrt(sq(Math.abs(r.dx) + w / 2) + sq(Math.abs(r.dy) + h / 2));
      case _:
        throw "Unknown footprint";
    };
  }

  public static function area(f:Footprint):Float return switch f {
    case Circle(d):
      Math.PI * d * d / 4;
    case Rect(w, h):
      w * h;
    case Shifted(inner, _, _):
      area(inner);
    case _:
      throw "Unknown footprint";
  };

  /** Distance from a point to the edge of the shape centred at (fx, fy), negative inside it. */
  public static function distanceTo(f:Footprint, fx:Float, fy:Float, px:Float, py:Float):Float return switch f {
    case Circle(d):
      Math.sqrt(sq(px - fx) + sq(py - fy)) - d / 2;
    case Rect(w, h):
      var dx = Math.abs(px - fx) - w / 2, dy = Math.abs(py - fy) - h / 2;
      dx > 0 || dy > 0 ? Math.sqrt(sq(Math.max(dx, 0.0)) + sq(Math.max(dy, 0.0))) : Math.max(dx, dy);
    case Shifted(inner, sx, sy):
      distanceTo(inner, fx + sx, fy + sy, px, py);
    case _:
      throw "Unknown footprint";
  };

  /** Gap between two shapes (negative when they overlap), each centred at its own position. */
  public static function gap(a:Footprint, ax:Float, ay:Float, b:Footprint, bx:Float, by:Float):Float {
    var ra = resolve(a), rb = resolve(b);
    return plainGap(ra.shape, ax + ra.dx, ay + ra.dy, rb.shape, bx + rb.dx, by + rb.dy);
  }

  static function plainGap(a:Footprint, ax:Float, ay:Float, b:Footprint, bx:Float, by:Float):Float return switch [
    a,
    b
  ] {
    case [Circle(da), Circle(db)]:
      Math.sqrt(sq(ax - bx) + sq(ay - by)) - da / 2 - db / 2;
    case [Circle(_), Rect(_, _)]:
      distanceTo(b, bx, by, ax, ay) - radius(a);
    case [Rect(_, _), Circle(_)]:
      distanceTo(a, ax, ay, bx, by) - radius(b);
    case [Rect(wa, ha), Rect(wb, hb)]:
      var gx = Math.abs(ax - bx) - (wa + wb) / 2, gy = Math.abs(ay - by) - (ha + hb) / 2;
      gx > 0 && gy > 0 ? Math.sqrt(gx * gx + gy * gy) : Math.max(gx, gy);
    case _:
      throw "Unknown footprint";
  };

  /** The solid that cuts the shape through z0 to z1, centred at (x, y). */
  public static function solid(f:Footprint, x:Float, y:Float, z0:Float, z1:Float):Part return switch f {
    case Circle(d):
      Part.cylinderSpan(d / 2, z0, z1, x, y);
    case Rect(w, h):
      var raw = Part.box(w, h, z1 - z0);
      var moved = raw.translated(new Vector(x, y, z0));
      raw.close();
      moved;
    case Shifted(inner, dx, dy):
      solid(inner, x + dx, y + dy, z0, z1);
    case _:
      throw "Unknown footprint";
  };

  static function radius(f:Footprint):Float return switch f {
    case Circle(d):
      d / 2;
    case _:
      throw "Not a circle";
  };

  static function sq(v:Float):Float return v * v;
}
