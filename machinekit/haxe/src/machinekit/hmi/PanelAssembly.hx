package machinekit.hmi;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/**
 * A panel as solids: every part placed with its origin at the panel's front face, tested against the
 * others, the panel plate and an optional shell. Intersections are volumes of overlap, so touching faces
 * are not interference.
 */
class PanelAssembly {
  /** A part's solid in panel coordinates. The caller closes it. */
  public static function placed(layout:PanelLayout, control:PlacedPart):Part {
    var raw = control.part.geometry(Preview);
    var moved = raw.translated(new Vector(control.x, control.y, layout.thickness));
    raw.close();
    return moved;
  }

  /** What overlaps what, as messages; empty when the parts, the plate and the shell are mutually clear. The plate and shell are borrowed. */
  public static function interferences(layout:PanelLayout, plate:Part, ?shell:Part, tolerance:Float = 1e-3):Array<String> {
    var found:Array<String> = [];
    var solids:Array<Part> = [];
    try {
      for (c in layout.controls) solids.push(placed(layout, c));
      for (i in 0...solids.length) {
        var c = layout.controls[i];
        var common = solids[i].intersect(plate);
        if (common.volume() > tolerance) found.push('${c.id} overlaps the panel plate by ${Math.round(common.volume() * 100) / 100} mm3');
        common.close();
        if (shell != null) {
          var inShell = solids[i].intersect(shell);
          if (inShell.volume() > tolerance) found.push('${c.id} overlaps the shell by ${Math.round(inShell.volume() * 100) / 100} mm3');
          inShell.close();
        }
        for (j in i + 1...solids.length) {
          if (!planOverlap(c, layout.controls[j])) continue;
          var both = solids[i].intersect(solids[j]);
          if (both.volume() > tolerance) found.push('${c.id} and ${layout.controls[j].id} overlap by ${Math.round(both.volume() * 100) / 100} mm3');
          both.close();
        }
      }
    } catch (error:Dynamic) {
      for (s in solids) s.close();
      throw error;
    }
    for (s in solids) s.close();
    return found;
  }

  /** Cheap pre-test: parts whose plan extents are apart cannot overlap. */
  static function planOverlap(a:PlacedPart, b:PlacedPart):Bool {
    var ea = extent(a), eb = extent(b);
    return Math.abs(a.x - b.x) < ea + eb && Math.abs(a.y - b.y) < ea + eb;
  }

  static function extent(c:PlacedPart):Float {
    var keep = switch c.keepOut() { case Circle(d): d / 2; case Rect(w, h): Math.max(w, h) / 2; case _: 0.0; };
    var body = switch c.body() { case Circle(d): d / 2; case Rect(w, h): Math.max(w, h) / 2; case _: 0.0; };
    return Math.max(keep, body);
  }
}
