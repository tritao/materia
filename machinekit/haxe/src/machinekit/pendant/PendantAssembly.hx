package machinekit.pendant;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/**
 * The pendant as solids: every part placed on the face with its origin at the panel's front face, tested
 * against the others, the face plate and the shell. Intersections are volumes of overlap, so touching
 * faces are not interference.
 */
class PendantAssembly {
  /** A control's part solid in face coordinates, or null for a control without a part. Closed by the caller. */
  public static function placed(face:PendantFace, control:PendantControl):Null<Part> {
    if (control.part == null) return null;
    var raw = control.part.geometry(Preview);
    var moved = raw.translated(new Vector(control.x, control.y, face.thickness));
    raw.close();
    return moved;
  }

  /** What overlaps what, as messages; empty when the parts, plate and shell are mutually clear. */
  public static function interferences(face:PendantFace, ?pendantCase:PendantCase, tolerance:Float = 1e-3):Array<String> {
    var found:Array<String> = [];
    var solids:Array<Null<Part>> = [for (c in face.controls) placed(face, c)];
    var plate = pendantCase == null ? PendantFacePlate.part(face) : pendantCase.plate();
    var shell = pendantCase == null ? null : pendantCase.shell();
    try {
      for (i in 0...solids.length) {
        var a = solids[i];
        if (a == null) continue;
        var common = a.intersect(plate);
        if (common.volume() > tolerance) found.push('${face.controls[i].id} overlaps the face plate by ${Math.round(common.volume() * 100) / 100} mm3');
        common.close();
        if (shell != null) {
          var inShell = a.intersect(shell);
          if (inShell.volume() > tolerance) found.push('${face.controls[i].id} overlaps the shell by ${Math.round(inShell.volume() * 100) / 100} mm3');
          inShell.close();
        }
        for (j in i + 1...solids.length) {
          var b = solids[j];
          if (b == null || !planOverlap(face.controls[i], face.controls[j])) continue;
          var both = a.intersect(b);
          if (both.volume() > tolerance) found.push('${face.controls[i].id} and ${face.controls[j].id} overlap by ${Math.round(both.volume() * 100) / 100} mm3');
          both.close();
        }
      }
    } catch (error:Dynamic) {
      for (s in solids) if (s != null) s.close();
      plate.close();
      if (shell != null) shell.close();
      throw error;
    }
    for (s in solids) if (s != null) s.close();
    plate.close();
    if (shell != null) shell.close();
    return found;
  }

  /** Cheap pre-test: parts whose plan extents are apart cannot overlap. */
  static function planOverlap(a:PendantControl, b:PendantControl):Bool {
    var ea = extent(a), eb = extent(b);
    return Math.abs(a.x - b.x) < ea + eb && Math.abs(a.y - b.y) < ea + eb;
  }

  static function extent(c:PendantControl):Float {
    var keep = switch c.keepOut { case Circle(d): d / 2; case Rect(w, h): Math.max(w, h) / 2; case _: 0.0; };
    var body = switch c.body { case Circle(d): d / 2; case Rect(w, h): Math.max(w, h) / 2; case _: 0.0; };
    return Math.max(keep, body);
  }
}
