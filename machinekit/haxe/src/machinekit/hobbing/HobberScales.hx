package machinekit.hobbing;

import machinekit.assembly.AxisBuilder;
import machinekit.component.BoltedFace;
import machinekit.component.ManufacturingRoute.RouteHole;
import machinekit.motion.SealedScalePart;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * One axis's sealed scale (JFT-40, `SealedScalePart`) placed in the hobber, all in the work frame at joint zero.
 *
 * The scale is a sandwich along `toSeat`: fixed seat, housing, running gap, reading head, carriage seat. The housing sits on
 * the fixed part (a ground bar, or the swivel plate itself) with its travel along `along`, the joint's positive direction;
 * the head is carried by the moving part, centred `headAlong` mm from the housing's start end along the travel.
 */
typedef HobberScaleAxis = {
  var id:String;
  var housing:SealedScalePart;
  var head:SealedScalePart;
  /** Poses at joint zero (the part frames of `SealedScalePart`: travel along +Z, +Y from the head toward the seat). */
  var housingPose:AssemblyFrame;
  var headPose:AssemblyFrame;
  /** Unit vectors: from the head toward the seat, and along the travel. */
  var toSeat:Array<Float>;
  var along:Array<Float>;
  /** The ground bar under the housing as {x0, x1, y0, y1, z0, z1}, or null when the housing sits on a plate. */
  var seatBar:Null<Array<Float>>;
}

/**
 * Where the X, Y and Z sealed scales go (FRAME_PLAN H4). The measuring length comes from the axis stroke plus the overtravel
 * at both ends (`SealedScalePart.pair` picks the smallest standard length); the housing is placed so the head stays inside the
 * measuring window over the whole stroke with margin at both ends.
 *
 * - **X**: housing on a 10 x 20 ground bar on the bed beam's flat top (z 210) beside the right rail, head under a rib of the
 *   carriage (the rib is as deep as the scale stack leaves).
 * - **Y**: housing on the swivel plate's front face between the guide rails, on the A axis line; head on a boss on the Y saddle.
 * - **Z**: housing on a 10 x 20 ground bar on the column channel's inner wall (y 120), head on an arm milled out of a bar and
 *   bolted to the Z saddle's back face.
 */
class HobberScales {
  /** Top of the bed beam beside the X guide (work z), mm, measured on the casting. */
  public static inline final BED_BEAM_TOP = 210.0;
  /** Thickness of the ground bar the housing stands on, mm. */
  public static inline final SEAT_BAR = 10.0;
  /** Width of that bar, mm: the housing is 18 wide. */
  public static inline final SEAT_BAR_WIDTH = 20.0;
  /** Inner face of the column channel wall (work y), mm: the casting's wall stands at +120 and -120. */
  public static inline final COLUMN_WALL_Y = 120.0;

  /** Which channel wall the Z scale stands on: the one away from the Z screw's pads (+1 is the +y wall). */
  public static function zWallSign():Float return HobberZLayout.axisY() < 0 ? 1.0 : -1.0;
  /** Where the Z scale stands along x (centre), mm: just inside the channel's open front (the casting wall starts at x 186). */
  public static inline final Z_COLUMN_X = 198.0;
  /** Arm that carries the Z head, thickness across the wall and bolt-up height, mm. */
  public static inline final Z_ARM_THICKNESS = 10.0;
  /** How far the head can overhang the carrier that holds it, mm (it is wider than the holes' span). */
  public static inline final HEAD_CARRIER_MARGIN = 6.0;

  /** Each axis' scale, derived once per axis and shared: no caller changes it. */
  static final cache = new machinekit.cache.Memo<HobberScaleAxis>("hobber-scales", 8);

  static function overtravel():Float return HobberXLayout.OVERTRAVEL;

  /** Axis travel the scale must cover (lower, upper joint values), conservative: the design stroke, not the cut-dependent one. */
  public static function stroke(id:String):{lower:Float, upper:Float} return switch id {
    case "X": {lower: HobberXLayout.MIN, upper: HobberXLayout.RETRACT};
    case "Y": {lower: -10.0, upper: 10.0};
    case "Z": {lower: -10.0, upper: 190.0};
    default: throw 'No scale for axis $id';
  };

  public static function x():HobberScaleAxis return axis("X");
  public static function y():HobberScaleAxis return axis("Y");
  public static function z():HobberScaleAxis return axis("Z");
  public static function all():Array<HobberScaleAxis> return [x(), y(), z()];

  /** Where the housing's seat face starts, how it lies, and where the head's centre sits at joint zero (work frame). */
  static function layout(id:String, startFor:Float, len:Float):{toSeat:Array<Float>, along:Array<Float>, start:Array<Float>, bar:Null<Array<Float>>, headCentre:Float} {
    return switch id {
      case "X":
        // The head centre is at x 262 at joint zero; the housing starts so the head's travel sits in the window.
        var at = 262.0 - startFor;
        {toSeat: [0.0, 0, -1], along: [1.0, 0, 0], start: [at, 178.0, BED_BEAM_TOP + SEAT_BAR],
          bar: [at, at + len, 178 - SEAT_BAR_WIDTH / 2, 178 + SEAT_BAR_WIDTH / 2, BED_BEAM_TOP, BED_BEAM_TOP + SEAT_BAR], headCentre: startFor};
      case "Y":
        // The joint runs along -y; the scale is read along +y (the sign is set in the drive). The plate is 230 long (y +-115).
        {toSeat: [1.0, 0, 0], along: [0.0, 1, 0], start: [85.0, -104.0, HobberZLayout.SEAT_Z], bar: null, headCentre: startFor};
      case "Z":
        var at = HobberZLayout.SEAT_Z + 50.0 - startFor, w = zWallSign();
        {toSeat: [0.0, w, 0], along: [0.0, 0, 1], start: [Z_COLUMN_X, w * (COLUMN_WALL_Y - SEAT_BAR), at],
          bar: [Z_COLUMN_X - SEAT_BAR_WIDTH / 2, Z_COLUMN_X + SEAT_BAR_WIDTH / 2, w > 0 ? COLUMN_WALL_Y - SEAT_BAR : -COLUMN_WALL_Y, w > 0 ? COLUMN_WALL_Y : -COLUMN_WALL_Y + SEAT_BAR, at, at + len], headCentre: startFor};
      default:
        throw 'No scale for axis $id';
    };
  }

  static function axis(id:String):HobberScaleAxis return cache.get(id, () -> derive(id));

  static function derive(id:String):HobberScaleAxis {
    var s = stroke(id), accuracy = id == "X" ? "3" : "5";
    var span = s.upper - s.lower + 2 * overtravel();
    var pair = SealedScalePart.pair("JFT-40", span, accuracy);
    var h = pair.housing, len = h.overallLength(), ml = h.measuringLength;
    // The head's centre at the lowest reachable joint value sits `margin` inside the window's start; the same margin is left at the far end.
    var margin = (ml - span) / 2;
    // The head centre's distance from the housing start at joint zero: window start + margin - (lower - overtravel).
    var headFromStart = h.startOffset() + margin - (s.lower - overtravel());
    var place = layout(id, headFromStart, len);
    var hh = h.series.housingHeight, gap = h.gap(), up = place.toSeat, along = place.along, p = place.start;
    var housingOrigin = [p[0] - up[0] * hh, p[1] - up[1] * hh, p[2] - up[2] * hh];
    var housingPose = AxisBuilder.orient(housingOrigin[0], housingOrigin[1], housingOrigin[2], up, along);
    var headOrigin = [for (i in 0...3) housingOrigin[i] + along[i] * place.headCentre - up[i] * gap];
    var headPose = AxisBuilder.orient(headOrigin[0], headOrigin[1], headOrigin[2], up, along);
    var result:HobberScaleAxis = {id: id, housing: pair.housing, head: pair.head, housingPose: housingPose, headPose: headPose,
      toSeat: up, along: along, seatBar: place.bar};
    return result;
  }

  /** The point on the head's carrier face (the plane the head's mounting face rests on), at the head's centre, work frame. */
  public static function headFace(a:HobberScaleAxis):Array<Float> {
    var f = a.head.connector("mount").frame, p = F.transformPoint(a.headPose, f.x, f.y, f.z);
    return [p.x, p.y, p.z];
  }

  /** The point on the seat face under the housing (its mounting face, at its middle), work frame. */
  public static function housingFace(a:HobberScaleAxis):Array<Float> {
    var f = a.housing.connector("mount").frame, p = F.transformPoint(a.housingPose, f.x, f.y, f.z);
    return [p.x, p.y, p.z];
  }

  /** The housing's two end fixing holes in the seat: tapped M8, mouth on the seat face, into the seat `depth` mm. */
  public static function seatHoles(a:HobberScaleAxis, depth:Float):Array<RouteHole> {
    var thread = "M8", tap = SocketHeadCapScrew.catalog().get(thread).tapDrill, n = 0, out = [];
    var into = F.transformVector(a.housingPose, 0, 1, 0);
    for (name in ["end1", "end2"]) {
      var f = a.housing.connector(name).frame, p = F.transformPoint(a.housingPose, f.x, f.y, f.z);
      out.push({id: "S" + (++n), kind: "tapped", thread: thread, diameter: tap, nominal: 8.0, fit: null, depth: depth,
        position: [p.x, p.y, p.z], direction: [into.x, into.y, into.z]});
    }
    return out;
  }

  /** A screw head sunk below the face it is counterbored into, mm. */
  public static inline final HEAD_SINK = 0.5;

  /**
   * The seat's receiving face under the housing: the two M8 tapped holes at the housing's end connectors, through the seat (`thickness` of bar or plate),
   * in the seat part's frame (`origin` is that frame's origin in the work frame). The holes are the seat's own (`seatHoles`); the housing's face is its own.
   */
  public static function seatFace(a:HobberScaleAxis, origin:Array<Float>, thickness:Float):BoltedFace {
    var mid = housingFace(a), up = a.toSeat, along = a.along;
    var frame = BoltedFace.frameOf({x: mid[0] - origin[0], y: mid[1] - origin[1], z: mid[2] - origin[2]}, {x: up[0], y: up[1], z: up[2]}, {x: along[0], y: along[1], z: along[2]});
    var inverse = F.inverse(frame);
    var holes = [for (name in ["end1", "end2"]) {
      var f = a.housing.connector(name).frame, p = F.transformPoint(a.housingPose, f.x, f.y, f.z);
      var q = F.transformPoint(inverse, p.x - origin[0], p.y - origin[1], p.z - origin[2]);
      ({x: q.x, y: q.y} : BoltedFace.BoltHole);
    }];
    return BoltedFace.tapped("scale-housing", frame, holes, "M8", thickness, thickness, true);
  }

  /** Depth a counterbore must have for an M5 head to end `HEAD_SINK` below its mouth, mm. */
  public static function headRecess():Float return SocketHeadCapScrew.catalog().get("M5").headHeight + HEAD_SINK;

  /**
   * The carrier's face under the head: M5 clearance holes drilled to suit the head's tapped pair, in the carrier part's frame. `grip` is the carrier's
   * material from the head's face to the counterbore floor; `recess` (null for none) is the counterbore's depth from that floor, which a machine
   * may make deeper than the head (a well through more material, for the driver); the head ends `HEAD_SINK` below the floor's top.
   */
  public static function carrierFace(a:HobberScaleAxis, origin:Array<Float>, grip:Float, ?recess:Float):BoltedFace {
    var p = headFace(a), into = F.transformVector(a.headPose, 0, -1, 0), along = a.along, screw = SocketHeadCapScrew.catalog().get("M5");
    var frame = BoltedFace.frameOf({x: p[0] - origin[0], y: p[1] - origin[1], z: p[2] - origin[2]}, {x: into.x, y: into.y, z: into.z}, {x: along[0], y: along[1], z: along[2]});
    return BoltedFace.clampedToSuit("scale-head", frame, a.head.series.headHolePitch + 4 * screw.diameter, 4 * screw.diameter, screw.clearanceMedium,
      grip, "M5", recess == null ? null : {diameter: screw.headDiameter + 2, depth: recess});
  }
}
