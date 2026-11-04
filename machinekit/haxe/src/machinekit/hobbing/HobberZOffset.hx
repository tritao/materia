package machinekit.hobbing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.AxisBuilder;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.motion.BallNut;
import machinekit.motion.ErobActuatorData as E;
import materia.assembly.AssemblyFrames as F;

/** What fixed the Z screw's offset across the machine, mm. */
typedef ZOffsetResult = {
  /** Signed screw position across the machine (work y). */
  var y:Float;
  /** The least |y| that keeps every running gap and hole margin (before rounding up to `STEP`). */
  var minimum:Float;
  /** The largest |y| the drive seat's casting cover allows. */
  var maximum:Float;
  /** The obstacle that set `minimum`. */
  var governing:String;
  /** Gap at `y` to each obstacle (name, mm). */
  var gaps:Array<{name:String, gap:Float}>;
  /** Half the drive seat insert needs around the screw line: the seat holes' reach plus the casting cover. */
  var insertHalfWidth:Float;
}

/**
 * Where the Z screw stands across the machine (work y).
 *
 * The A axis's eRob housing (Ø110, 88 mm deep, rear boss and connectors) stands behind the Z saddle at y = 0, where the Z
 * screw used to be. The screw moves off the housing by the least distance at which the fixed supports (BK12 mount, BF12
 * support) keep `RUNNING_GAP` from the housing over the whole Z travel, the nut and its block (which travel with the
 * housing) keep it, and the floating pad's seat holes keep 1 mm from the pad's relief (`relief`), measured on the parts
 * posed at the trial offset. The offset is rounded up to `STEP`.
 *
 * The side is `SIDE`: the Z moving mass (saddle, eRob and the head with its spindle seat at y 70..222) has its centre of mass
 * on the +y side, so the screw's thrust and the weight meet nearer one another there (`HobberFrameChecks.zOffsetMoments`
 * verifies the sign against the built model and prints the moments).
 *
 * The casting limits the other way: the drive seat insert must keep `SOURCE_STEEL_COVER_MM` from the column's lifting lands
 * (which start at `COLUMN_HEAD_WIDTH / 2 + cover`), so its outer edge stays inside the head's side face: |y| + half the insert
 * width at most `COLUMN_HEAD_WIDTH / 2`. The insert needs at least the seat holes' reach plus the cover each side. If no y
 * satisfies both, this fails with the numbers.
 */
class HobberZOffset {
  /** The running gap the housing keeps from every fixed or moving part of the drive, mm. */
  public static inline final RUNNING_GAP = 2.0;
  /** Extra cut on the pad bar's relief and nut groove beyond the running gap, mm. */
  public static inline final PAD_RELIEF_MARGIN = 0.5;
  /** The side of the housing the screw stands on: +1 is +y. */
  public static inline final SIDE = 1.0;
  /** The offset is rounded up to a whole millimetre. */
  public static inline final STEP = 1.0;
  /** Clearance of the floating pad's seat holes from its relief, mm. */
  public static inline final HOLE_MARGIN = 1.0;
  /** Widest drive seat insert the casting wants (the width of the old central one), mm. */
  public static inline final MAX_INSERT_WIDTH = 100.0;

  static var cache:Null<ZOffsetResult>;

  public static function axisY():Float return result().y;

  public static function result():ZOffsetResult {
    if (cache == null) cache = derive();
    return cache;
  }

  /** Width of the drive seat insert at the derived offset: the widest the casting allows, up to `MAX_INSERT_WIDTH`. */
  public static function insertWidth():Float {
    var r = result();
    return Math.min(MAX_INSERT_WIDTH, 2 * (FoundationLayout.COLUMN_HEAD_WIDTH / 2 - Math.abs(r.y)));
  }

  /** The housing envelope (body, rear boss, connectors) swept over the Z joint range `lo`..`hi`, grown by `grow`, in the work frame. */
  public static function envelope(lo:Float, hi:Float, grow:Float):Part {
    var front = HobberSwivelLayout.OUTPUT_X + E.PILOT_HEIGHT;
    // Segments along work x: the Ø110 body from the front face, the Ø85 rear boss, the connectors (the boss's envelope to the overall length).
    var bodyEnd = HobberSwivelLayout.OUTPUT_X + E.BODY_LENGTH - E.REAR_SPIGOT_HEIGHT;
    var bossEnd = HobberSwivelLayout.OUTPUT_X + E.BODY_LENGTH;
    var connectorEnd = HobberSwivelLayout.OUTPUT_X + E.OVERALL_LENGTH;
    var segments = [
      {x0: front - grow, x1: bodyEnd + grow, r: E.BODY_DIAMETER / 2 + grow},
      {x0: bodyEnd, x1: bossEnd + grow, r: E.REAR_SPIGOT_DIAMETER / 2 + grow},
      {x0: bossEnd, x1: connectorEnd + grow, r: E.REAR_SPIGOT_DIAMETER / 2 + grow}
    ];
    var pieces:Array<Part> = [];
    var zLow = HobberZLayout.SEAT_Z + lo, zHigh = HobberZLayout.SEAT_Z + hi;
    for (s in segments) {
      for (z in (hi > lo ? [zLow, zHigh] : [zLow])) {
        var c = Part.cylinderSpan(s.r, s.x0, s.x1);
        pieces.push(PosedParts.moved(c, AxisBuilder.orient(0, 0, z, [0.0, 0, 1], [1.0, 0, 0])));
        c.close();
      }
      if (hi > lo) {
        var raw = Part.box(s.x1 - s.x0, 2 * s.r, zHigh - zLow), moved = raw.translated(new Vector((s.x0 + s.x1) / 2, 0, zLow));
        raw.close();
        pieces.push(moved);
      }
    }
    return Solids.union(pieces);
  }

  /** The relief the floating pad needs: the housing's envelope over the Z travel grown by the running gap. */
  public static function relief():Part {
    var s = HobberScales.stroke("Z");
    return envelope(s.lower, s.upper, RUNNING_GAP);
  }

  /**
   * The relief cut in the pad bar: the same envelope with `PAD_RELIEF_MARGIN` more, so that the machined edge, which comes out at the
   * running gap to within the cut's tolerance, never lands under it.
   */
  public static function padRelief():Part {
    var s = HobberScales.stroke("Z");
    return envelope(s.lower, s.upper, RUNNING_GAP + PAD_RELIEF_MARGIN);
  }

  static function derive():ZOffsetResult {
    var s = HobberScales.stroke("Z");
    var drive = HobberDrives.zAt(0.0);
    var nut = new BallNut(), poses = HobberDrives.poses(drive);
    var swept = envelope(s.lower, s.upper, 0), here = envelope(0, 0, 0), grown = relief();
    var fixedMount = PosedParts.moved(drive.drive.fixed.geometry(), poses.get("mount"));
    var floatingSupport = PosedParts.moved(drive.drive.floating.geometry(), poses.get("floating"));
    var nutPart = PosedParts.moved(nut.geometry(), poses.get("nut"));
    var block = HobberDrives.nutBlock(drive, nut), blockPart = PosedParts.moved(block.geometry(), HobberDrives.nutBlockPose(drive, nut));
    var tools:Array<Part> = [];
    var holes = HobberDrives.seatHoles(drive, "floating"), reach = 0.0;
    for (support in ["fixed", "floating"]) for (h in HobberDrives.seatHoles(drive, support)) reach = Math.max(reach, Math.abs(h.position[1]) + h.diameter / 2);
    for (h in holes) {
      var cylinder = Part.cylinderSpan(h.diameter / 2, -1, h.depth);
      var up = Math.abs(h.direction[2]) < 0.9 ? [0.0, 0, 1] : [1.0, 0, 0];
      tools.push(PosedParts.moved(cylinder, AxisBuilder.orient(h.position[0], h.position[1], h.position[2], up, h.direction)));
      cylinder.close();
    }
    // Obstacles: name, part at y = 0, the housing it is measured against.
    var obstacles = [
      {name: "BK12 mount (fixed)", part: fixedMount, against: swept},
      {name: "BF12 support (floating)", part: floatingSupport, against: swept},
      {name: "ball nut", part: nutPart, against: here},
      {name: "nut block", part: blockPart, against: here}
    ];
    function gapsAt(a:Float):Array<{name:String, gap:Float}> {
      var shift = new Vector(0, SIDE * a, 0), out = [];
      for (o in obstacles) {
        var moved = o.part.translated(shift), d = moved.distance(o.against);
        moved.close();
        out.push({name: o.name, gap: d});
      }
      var worst = 1e9;
      for (t in tools) {
        var moved = t.translated(shift), d = moved.distance(grown);
        moved.close();
        worst = Math.min(worst, d);
      }
      out.push({name: "floating pad seat holes to relief", gap: worst});
      return out;
    }
    function ok(g:Array<{name:String, gap:Float}>):Bool {
      for (item in g) if (item.gap < (item.name.indexOf("holes") >= 0 ? HOLE_MARGIN : RUNNING_GAP) - 1e-6) return false;
      return true;
    }
    function governing(g:Array<{name:String, gap:Float}>):String {
      var name = "", worst = 1e9;
      for (item in g) {
        var need = item.name.indexOf("holes") >= 0 ? HOLE_MARGIN : RUNNING_GAP;
        if (item.gap - need < worst) {
          worst = item.gap - need;
          name = item.name;
        }
      }
      return name;
    }
    var found:Null<Float> = null, previous = 0.0;
    var a = 0.0;
    while (a <= 130) {
      if (ok(gapsAt(a))) {
        found = a;
        break;
      }
      previous = a;
      a += 1.0;
    }
    var maximum = FoundationLayout.COLUMN_HEAD_WIDTH / 2 - (reach + FoundationLayout.SOURCE_STEEL_COVER_MM);
    var below = found == null ? [] : gapsAt(Math.max(0.0, found - 1.0));
    var g = found == null ? [] : gapsAt(found);
    for (o in obstacles) o.part.close();
    for (t in tools) t.close();
    swept.close();
    here.close();
    grown.close();
    if (found == null) throw "Z screw offset: no |y| up to 130 mm keeps " + RUNNING_GAP + " mm from the A axis housing";
    var y = SIDE * found;
    // The step is 1 mm and the scan runs in steps of 1 mm, so `found` is already rounded up.
    if (found > maximum + 1e-9)
      throw "Z screw offset: the housing needs |y| >= " + found + " mm (" + governing(below) + " at " + (found - 1) + " mm), the drive seat's casting cover allows at most "
        + maximum + " mm (seat holes reach " + reach + " mm from the screw line, cover " + FoundationLayout.SOURCE_STEEL_COVER_MM + " mm, head half-width "
        + FoundationLayout.COLUMN_HEAD_WIDTH / 2 + " mm)";
    return {y: y, minimum: found, maximum: maximum, governing: governing(below), gaps: g, insertHalfWidth: reach + FoundationLayout.SOURCE_STEEL_COVER_MM};
  }
}
