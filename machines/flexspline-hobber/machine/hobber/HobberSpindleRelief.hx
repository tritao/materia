package hobber;

import machinekit.hobbing.*;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import machinekit.motion.LeadshineServo;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingFeature;
import machinekit.structural.CastingFeaturePatterns as Patterns;
import materia.casting.CastingPatterns as C;

/** The relief the bed casting needs under the hob spindle's servo, in the casting's frame (mm). */
typedef SpindleRelief = {
  /** The pocket: centre-based x and y as `CastingPatterns.box` takes them, the floor height `z0` from the casting's bottom, and the sizes. */
  var x:Float;
  var y:Float;
  var z0:Float;
  var width:Float;
  var depth:Float;
  var height:Float;
  /** The pocket's extent along each axis (casting frame), before the casting's outline cut it. */
  var x0:Float;
  var x1:Float;
  var y0:Float;
  var y1:Float;
  /** The part of the pocket inside the casting's outline, mm3. */
  var volume:Float;
  /** The outline faces the pocket reaches (it opens through each) and the top. */
  var faces:Array<String>;
  /** The sweep pose that sets the floor, as text. */
  var governing:String;
  /** How many poses were swept. */
  var poses:Int;
}

/**
 * Where the spindle servo passes close to the bed casting, and the pocket that keeps it `RUNNING_GAP` clear.
 *
 * The servo (`HobSpindle.motor`: a Leadshine ELM2H-1000LA80F with its connector envelopes, `LeadshineServo.clearanceBoxes`) hangs
 * off the cartridge, behind the A axis' swing, and at the Z lower limit it stands below the top of the bed's side beam. The relief is cut
 * from that beam's top. It is derived, not drawn: the servo's boxes (body and connectors), each grown by the running gap, are posed over
 * the whole sweep (X, Y and Z to their limits and `HobberSkeleton.reference().overtravel` beyond, A over its whole range), and what lies inside the
 * casting's outline below its top face is the volume to remove. The pocket is the box around that volume.
 *
 * X runs over the design stroke (`HobberScales.stroke`, from `HobberSkeleton.reference().xMin`), which is at most the machine's own lower limit (that one
 * follows the mandrel and hob radii of the gear being cut, which the casting cannot know): the pocket is a little longer than the cut needs.
 *
 * Poses: the head moves as `T(X, 0, Z) * Rx(-A about the swivel axis) * T(0, -Y, 0)` (positive A turns about work -x, the actuator's output direction) from its construction pose (the swivel axis is the
 * work x direction through y = 0 at `HobberSkeleton.reference().nativeHobCentreZ`; Y comes after A because the Y rails ride on the swivel plate). Only the
 * extremes of X, Y and Z can set the pocket (its footprint grows as the servo goes lower, and the beam's top is flat along x), A is
 * sampled every `A_STEP_DEGREES`. `HobberFrameChecks.spindleRelief` checks this chain against the solved machine and the clearance
 * against the finished casting.
 *
 * Nothing here needs the casting's body (the relief is a feature of it): where the casting has cores of its own below its top, the cut
 * only removes more air. `HobberFrameChecks.spindleRelief` checks the 25 mm wall left under the pocket's floor in the built casting.
 */
class HobberSpindleRelief {
  /** The running gap the servo keeps from the casting (the same as the integrity sweep's), mm. */
  public static inline final RUNNING_GAP = 2.0;
  /** Angular step of the A samples, degrees. */
  public static inline final A_STEP_DEGREES = 2.5;
  /** The pocket's edges are rounded outward to this, mm. */
  public static inline final STEP = 0.5;
  /** How far the pocket's box rises above the casting's top, so the cut is clean, mm. */
  public static inline final ABOVE_TOP = 10.0;

  static final cache = new machinekit.cache.Memo<Null<SpindleRelief>>("hobber-spindle-reliefs", 8);

  /** The relief for the servo's clocking `HobSpindle.MOTOR_CLOCKING`, or null if the servo never comes within the running gap of the casting. */
  public static function result():Null<SpindleRelief> return of(HobSpindle.MOTOR_CLOCKING);

  /** The casting feature for the relief (empty when none is needed). */
  public static function features():Array<CastingFeature> {
    var r = result();
    if (r == null) return [];
    var openings = [for (face in r.faces) C.face("body." + face)];
    return [Patterns.box("spindle-motor-relief", Clearance, openings, r.width, r.depth, r.height, r.x, r.y, r.z0)];
  }

  /** The relief for the servo turned `clocking` quarter turns about the spindle axis, to choose among the four (`HobSpindle.MOTOR_CLOCKING`). */
  public static function reliefFor(clocking:Int):Null<SpindleRelief> return of(clocking);

  /** The head's frame change in the work frame at joint values `x`, `y`, `z` and swivel angle `a` (radians), from the construction pose. */
  public static function motion(x:Float, y:Float, z:Float, a:Float):AssemblyFrame {
    var c = Math.cos(a), s = Math.sin(a), seat = HobberSkeleton.reference().nativeHobCentreZ;
    var turn = F.compose(F.translation(0, 0, seat), F.compose(F.fromRotationMatrix(0, 0, 0, [1.0, 0, 0, 0, c, s, 0, -s, c]), F.translation(0, 0, -seat)));
    return F.compose(F.translation(x, 0, z), F.compose(turn, F.translation(0, -y, 0)));
  }

  /** The servo's frame in the work frame at the construction pose, for a quarter-turn clocking. */
  public static function motorZeroPose(clocking:Int):AssemblyFrame {
    var spindle = new HobSpindle(null, null, clocking), solved = spindle.solvedPoses();
    return F.compose(HobberSkeleton.reference().spindleDatum(), solved.get("motor"));
  }

  /** The casting's outline in the work frame, as a box. */
  static function outline():{x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float} {
    var d = HobberSkeleton.reference().foundationDimensions("bed"), at = HobberSkeleton.reference().foundationDatum("bed");
    if (Math.abs(at.qx) + Math.abs(at.qy) + Math.abs(at.qz) > 1e-9) throw "The bed's frame is rotated: the relief derivation assumes it only moves the casting";
    return {x0: at.x - d[0] / 2, x1: at.x + d[0] / 2, y0: at.y - d[1] / 2, y1: at.y + d[1] / 2, z0: at.z, z1: at.z + d[2]};
  }

  static function of(clocking:Int):Null<SpindleRelief> {
    return cache.get(Std.string(clocking), () -> derive(clocking));
  }

  static function derive(clocking:Int):Null<SpindleRelief> {
    var servo = LeadshineServo.driven(HobSpindle.MOTOR), boxes = servo.clearanceBoxes();
    var zero = motorZeroPose(clocking), o = outline();
    var slab = Part.box(o.x1 - o.x0, o.y1 - o.y0, o.z1 - o.z0, Align.Min, Align.Min, Align.Min);
    var wall = slab.translated(new cadkit.modeling.Vector(o.x0, o.y0, o.z0));
    slab.close();
    var x = HobberScales.stroke("X"), y = HobberScales.stroke("Y"), z = HobberScales.stroke("Z");
    var ot = HobberSkeleton.reference().overtravel, range = HobberSkeleton.reference().swivelRange;
    // Z: the lowest pose sets the footprint and the floor (the servo can only come lower than the top there).
    var zs = [z.lower - ot], xs = [x.lower - ot, x.upper + ot], ys = [y.lower - ot, y.upper + ot];
    var steps = Math.ceil(range * 180 / Math.PI / A_STEP_DEGREES), as = [for (i in -steps...steps + 1) range * i / steps];
    var minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9, minZ = 1e9, count = 0;
    var governing = "";
    for (xv in xs) for (yv in ys) for (zv in zs) for (a in as) {
      count++;
      var pose = F.compose(motion(xv, yv, zv, a), zero);
      for (b in boxes) {
        var g = RUNNING_GAP;
        var raw = Part.box(b.x1 - b.x0 + 2 * g, b.y1 - b.y0 + 2 * g, b.z1 - b.z0 + 2 * g, Align.Center, Align.Center, Align.Center);
        var local = F.compose(pose, F.translation((b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2, (b.z0 + b.z1) / 2));
        var placed = PosedParts.moved(raw, local);
        raw.close();
        var inside = placed.intersect(wall);
        placed.close();
        if (inside.volume() > 1e-6) {
          var bb = PosedParts.boxOf(inside);
          minX = Math.min(minX, bb.minX);
          maxX = Math.max(maxX, bb.maxX);
          minY = Math.min(minY, bb.minY);
          maxY = Math.max(maxY, bb.maxY);
          if (bb.minZ < minZ) {
            minZ = bb.minZ;
            governing = '${b.name} at X $xv, Y $yv, Z $zv, A ${Math.round(a * 1800 / Math.PI) / 10} deg';
          }
        }
        inside.close();
      }
    }
    wall.close();
    if (minZ > 1e8) return null;
    function down(v:Float):Float return Math.floor(v / STEP + 1e-9) * STEP;
    function up(v:Float):Float return Math.ceil(v / STEP - 1e-9) * STEP;
    var bedX = HobberSkeleton.reference().foundationDatum("bed").x, bedZ = HobberSkeleton.reference().foundationDatum("bed").z;
    var x0 = Math.max(o.x0, down(minX)) - bedX, x1 = Math.min(o.x1, up(maxX)) - bedX;
    var y0 = Math.max(o.y0, down(minY)), y1 = Math.min(o.y1, up(maxY));
    var floor = down(minZ) - bedZ, top = o.z1 - bedZ;
    var faces = ["z-max"];
    if (down(minX) <= o.x0 + 1e-9) faces.push("x-min");
    if (up(maxX) >= o.x1 - 1e-9) faces.push("x-max");
    if (down(minY) <= o.y0 + 1e-9) faces.push("y-min");
    if (up(maxY) >= o.y1 - 1e-9) faces.push("y-max");
    return {
      x: (x0 + x1) / 2,
      y: (y0 + y1) / 2,
      z0: floor,
      width: x1 - x0,
      depth: y1 - y0,
      height: top - floor + ABOVE_TOP,
      x0: x0,
      x1: x1,
      y0: y0,
      y1: y1,
      volume: (x1 - x0) * (y1 - y0) * (top - floor),
      faces: faces,
      governing: governing,
      poses: count
    };
  }
}
