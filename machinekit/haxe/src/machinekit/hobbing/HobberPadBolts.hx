package machinekit.hobbing;

import machinekit.component.BoltedFace;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * The screws that hold an X or Z drive pad bar down on the drive-seat insert of the casting (FRAME_PLAN, "Pads to casting inserts"). One declaration
 * feeds both ends: the pad's clamped face (`face`, which derives its clearance holes and counterbores) and the insert's tapped holes
 * (`HobberMountingHoles` adds them to the cast insert from `points`), so the pattern is never written twice.
 *
 * The heads stand in the groove the nut block's travel cuts in the pad (`HobberDrives.nutGroove`): that is the one place on the bar's top that is
 * clear of the BK12/BF12 feet, the motor mount's relief and the feet's own screws. The bar is counterbored from the groove floor so each head ends
 * `HEAD_SINK` below it and the block passes over it. Four screws, two stations at `inset` from the groove's ends and `across` either side of the
 * screw axis; X takes M4 (the groove floor is 7.5 mm thick), Z M6 (32.5 mm).
 */
class HobberPadBolts {
  /** Head top below the groove floor, mm. */
  public static inline final HEAD_SINK = 0.5;

  /** The pad bar's thickness on `d`'s axis, mm (X 15, Z 40). */
  public static function thickness(d:HobberDriveAxis):Float return d.id == "X" ? 15.0 : 40.0;

  /** The screw size of the pad's bolts on `d`'s axis. */
  public static function thread(d:HobberDriveAxis):String return d.id == "X" ? "M4" : "M6";

  /** Distance of the two stations from the groove's ends, mm. */
  static function inset(d:HobberDriveAxis):Float return d.id == "X" ? 40.0 : 60.0;

  /** Distance of the holes either side of the screw axis, mm. */
  static function across(d:HobberDriveAxis):Float return d.id == "X" ? 20.0 : 22.0;

  static function indices(d:HobberDriveAxis):{n:Int, a:Int, c:Int} {
    var ni = -1, ai = -1;
    for (i in 0...3) {
      if (Math.abs(d.axis.normal[i]) > 0.5) ni = i;
      if (Math.abs(d.axis.direction[i]) > 0.5) ai = i;
    }
    return {n: ni, a: ai, c: 3 - ni - ai};
  }

  static function groove(d:HobberDriveAxis):Array<Float> {
    var g = HobberDrives.nutGroove(d);
    if (g == null) throw "The " + d.id + " pad has no nut groove to hold the insert screws in";
    return g;
  }

  /** The coordinate along the normal of the pad's underside (on the insert) and of the groove floor, work frame mm. */
  static function planes(d:HobberDriveAxis):{under:Float, floor:Float} {
    var i = indices(d), b = HobberDrives.padBoundsBoth(d, HobberDrives.PAD_WIDTH, thickness(d)), g = groove(d), up = d.axis.normal[i.n] > 0;
    return {under: up ? b[i.n] : b[i.n + 3], floor: up ? g[i.n] : g[i.n + 3]};
  }

  /** The holes on the pad's underside, work frame: the insert's top face. */
  public static function points(d:HobberDriveAxis):Array<{x:Float, y:Float, z:Float}> {
    var i = indices(d), g = groove(d), plane = planes(d).under, out = [];
    for (s in [g[i.a] + inset(d), g[i.a + 3] - inset(d)]) for (u in [-across(d), across(d)]) {
      var p = [0.0, 0, 0];
      p[i.n] = plane;
      p[i.a] = s;
      p[i.c] = d.axis.start[i.c] + u;
      out.push({x: p[0], y: p[1], z: p[2]});
    }
    return out;
  }

  /** Direction from the pad into the insert (the thread's axis). */
  public static function intoInsert(d:HobberDriveAxis):Array<Float> return [for (v in d.axis.normal) -v];

  /** The pad's face on the insert: clearance holes for the screws, counterbored from the groove floor, in the pad's frame (the work frame). */
  public static function face(d:HobberDriveAxis):BoltedFace {
    var planes = planes(d), pts = points(d), screw = SocketHeadCapScrew.catalog().get(thread(d));
    var origin = {x: pts[0].x, y: pts[0].y, z: pts[0].z};
    var n = {x: d.axis.normal[0], y: d.axis.normal[1], z: d.axis.normal[2]}, a = {x: d.axis.direction[0], y: d.axis.direction[1], z: d.axis.direction[2]};
    var frame = BoltedFace.frameOf(origin, n, a), inverse = F.inverse(frame);
    var holes = [for (p in pts) {
      var q = F.transformPoint(inverse, p.x, p.y, p.z);
      ({x: q.x, y: q.y} : BoltedFace.BoltHole);
    }];
    var floorThickness = Math.abs(planes.floor - planes.under), depth = screw.headHeight + HEAD_SINK;
    return BoltedFace.clamped("insert", frame, holes, screw.clearanceMedium, floorThickness - depth, thread(d), {diameter: screw.headDiameter + 2, depth: depth}, false);
  }
}
