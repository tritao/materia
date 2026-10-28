package machinekit.hmi;

/**
 * A panel (a pendant face, a door panel) and what sits on it. `problems()` is the layout check: every
 * part inside the usable face with an edge margin, no two parts closer than the minimum gap, the
 * case deep enough for the parts that reach into it, and the face plate thin enough for the wheel's
 * studs. Coordinates are millimetres, origin at the face centre, x to the right, y up.
 */
class PanelLayout {
  /** Outer size of the face plate and its thickness. */
  public final width:Float;
  public final height:Float;
  public final thickness:Float;
  /** Case wall thickness; the usable face is inside it. */
  public final wall:Float;
  /** Free depth inside the case behind the face plate. */
  public final caseInnerDepth:Float;
  public final controls:Array<PlacedPart>;
  /** Space kept clear between a part and the usable face edge. */
  public final edgeMargin:Float;
  /** Smallest gap between the footprints of two parts. */
  public final minGap:Float;

  public function new(width:Float, height:Float, thickness:Float, wall:Float, caseInnerDepth:Float, controls:Array<PlacedPart>,
      edgeMargin:Float = 2.0, minGap:Float = 5.0, maxPanelThickness:Float = 3.0) {
    for (v in [width, height, thickness, wall, caseInnerDepth, edgeMargin, minGap])
      if (!(v > 0) || !Math.isFinite(v)) throw "Pendant face dimensions must be positive";
    if (wall * 2 >= width || wall * 2 >= height) throw "Wall is thicker than the face";
    var ids:Array<String> = [];
    for (c in controls) {
      if (ids.indexOf(c.id) >= 0) throw 'Duplicate control ${c.id}';
      ids.push(c.id);
    }
    this.width = width; this.height = height; this.thickness = thickness; this.wall = wall;
    this.caseInnerDepth = caseInnerDepth; this.controls = controls.copy();
    this.edgeMargin = edgeMargin; this.minGap = minGap;
  }

  public function control(id:String):PlacedPart {
    for (c in controls) if (c.id == id) return c;
    throw 'Unknown control $id';
  }

  /** A copy with one control moved, for layout search. */
  public function withControl(moved:PlacedPart):PanelLayout
    return new PanelLayout(width, height, thickness, wall, caseInnerDepth, [for (c in controls) c.id == moved.id ? moved : c],
      edgeMargin, minGap);

  public function usableHalfWidth():Float return width / 2 - wall - edgeMargin;
  public function usableHalfHeight():Float return height / 2 - wall - edgeMargin;

  /** The deepest any part reaches behind the face plate. */
  public function requiredInnerDepth():Float {
    var depth = 0.0;
    for (c in controls) depth = Math.max(depth, c.behind());
    return depth;
  }

  /** Height of the case above the table with the face up, including whatever stands highest on the face. */
  public function overallHeight():Float {
    var above = 0.0;
    for (c in controls) above = Math.max(above, c.above());
    return caseInnerDepth + thickness + above;
  }

  public function partsMassGrams():Float {
    var mass = 0.0;
    for (c in controls) mass += c.massGrams();
    return mass;
  }

  /** The controls whose numbers are still placeholders. */
  public function assumed():Array<String> return [for (c in controls) if (c.source() == "assumed") c.id];

  public function problems():Array<String> {
    var found:Array<String> = [];
    var hw = usableHalfWidth(), hh = usableHalfHeight();
    for (c in controls) {
      var half = halfSize(c.keepOut());
      if (Math.abs(c.x) + half.x > hw + 1e-9 || Math.abs(c.y) + half.y > hh + 1e-9)
        found.push('${c.id} does not fit inside the usable face (needs x within ${fmt(hw)}, y within ${fmt(hh)} with its keep-out)');
      for (hole in c.holes()) {
        var r = hole.diameter / 2;
        if (Math.abs(c.x + hole.x) + r > width / 2 - wall || Math.abs(c.y + hole.y) + r > height / 2 - wall)
          found.push('a hole of ${c.id} reaches the case wall');
      }
    }
    for (c in controls) {
      var half = halfSize(c.body());
      if (Math.abs(c.x) + half.x > width / 2 - wall + 1e-9 || Math.abs(c.y) + half.y > height / 2 - wall + 1e-9)
        found.push('the body of ${c.id} reaches into the case wall');
    }
    for (i in 0...controls.length) for (j in i + 1...controls.length) {
      var gap = gapBetween(controls[i], controls[j]);
      if (gap < minGap - 1e-9)
        found.push('${controls[i].id} and ${controls[j].id} are ${fmt(gap)} mm apart, less than ${fmt(minGap)}');
    }
    for (i in 0...controls.length) for (j in i + 1...controls.length) {
      var overlap = bodyGapBetween(controls[i], controls[j]);
      if (overlap < -1e-9) found.push('the bodies of ${controls[i].id} and ${controls[j].id} overlap behind the face by ${fmt(-overlap)} mm');
    }
    if (requiredInnerDepth() > caseInnerDepth + 1e-9) {
      var deepest = controls[0];
      for (c in controls) if (c.behind() > deepest.behind()) deepest = c;
      found.push('${deepest.id} reaches ${fmt(deepest.behind())} mm behind the face but the case has ${fmt(caseInnerDepth)}');
    }
    for (c in controls) if (thickness > c.part.maxPanelThickness() + 1e-9)
      found.push('the panel is ${fmt(thickness)} mm thick but ${c.id} allows ${fmt(c.part.maxPanelThickness())}');
    return found;
  }

  static function halfSize(f:Footprint):{x:Float, y:Float} return switch f {
    case Circle(d): {x: d / 2, y: d / 2};
    case Rect(w, h): {x: w / 2, y: h / 2};
  };

  /** Distance from a point to the edge of a footprint centred at (fx, fy), negative inside it. */
  public static function distanceToFootprint(f:Footprint, fx:Float, fy:Float, px:Float, py:Float):Float return switch f {
    case Circle(d): Math.sqrt(sq(px - fx) + sq(py - fy)) - d / 2;
    case Rect(w, h):
      var dx = Math.abs(px - fx) - w / 2, dy = Math.abs(py - fy) - h / 2;
      dx > 0 || dy > 0 ? Math.sqrt(sq(Math.max(dx, 0.0)) + sq(Math.max(dy, 0.0))) : Math.max(dx, dy);
    case _: throw "Unknown footprint";
  };

  /** Gap between two footprints (negative when they overlap). */
  public static function gapBetween(a:PlacedPart, b:PlacedPart):Float return footprintGap(a.keepOut(), a.x, a.y, b.keepOut(), b.x, b.y);

  /** Gap between the rear bodies of two controls (negative when they overlap). */
  public static function bodyGapBetween(a:PlacedPart, b:PlacedPart):Float return footprintGap(a.body(), a.x, a.y, b.body(), b.x, b.y);

  static function footprintGap(fa:Footprint, ax:Float, ay:Float, fb:Footprint, bx:Float, by:Float):Float {
    return switch [fa, fb] {
      case [Circle(da), Circle(db)]: Math.sqrt(sq(ax - bx) + sq(ay - by)) - da / 2 - db / 2;
      case [Circle(d), Rect(w, h)]: circleToRect(ax, ay, d / 2, bx, by, w, h);
      case [Rect(w, h), Circle(d)]: circleToRect(bx, by, d / 2, ax, ay, w, h);
      case [Rect(wa, ha), Rect(wb, hb)]:
        var gx = Math.abs(ax - bx) - (wa + wb) / 2, gy = Math.abs(ay - by) - (ha + hb) / 2;
        gx > 0 && gy > 0 ? Math.sqrt(gx * gx + gy * gy) : Math.max(gx, gy);
      case _: throw "Unknown footprint";
    };
  }

  static function circleToRect(cx:Float, cy:Float, r:Float, rx:Float, ry:Float, w:Float, h:Float):Float {
    var dx = Math.max(Math.abs(cx - rx) - w / 2, 0.0), dy = Math.max(Math.abs(cy - ry) - h / 2, 0.0);
    return Math.sqrt(dx * dx + dy * dy) - r;
  }

  static function sq(v:Float):Float return v * v;
  static function fmt(v:Float):String return Std.string(Math.round(v * 10) / 10);
}
