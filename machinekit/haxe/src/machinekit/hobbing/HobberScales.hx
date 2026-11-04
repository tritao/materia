package machinekit.hobbing;

import machinekit.assembly.AxisBuilder;
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
  /** Where the Z scale stands along x (centre), mm: just inside the channel's open front (the casting wall starts at x 186). */
  public static inline final Z_COLUMN_X = 198.0;
  /** Arm that carries the Z head, thickness across the wall and bolt-up height, mm. */
  public static inline final Z_ARM_THICKNESS = 10.0;
  /** How far the head can overhang the carrier that holds it, mm (it is wider than the holes' span). */
  public static inline final HEAD_CARRIER_MARGIN = 6.0;

  static var cache:Map<String, HobberScaleAxis> = new Map();

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
        var at = HobberZLayout.SEAT_Z + 50.0 - startFor;
        {toSeat: [0.0, 1, 0], along: [0.0, 0, 1], start: [Z_COLUMN_X, COLUMN_WALL_Y - SEAT_BAR, at],
          bar: [Z_COLUMN_X - SEAT_BAR_WIDTH / 2, Z_COLUMN_X + SEAT_BAR_WIDTH / 2, COLUMN_WALL_Y - SEAT_BAR, COLUMN_WALL_Y, at, at + len], headCentre: startFor};
      default:
        throw 'No scale for axis $id';
    };
  }

  static function axis(id:String):HobberScaleAxis {
    var found = cache.get(id);
    if (found != null) return found;
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
    cache.set(id, result);
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

  /** The head's two M5 screws through its carrier: clearance holes, mouth on the carrier face, into the carrier `depth` mm. */
  public static function carrierHoles(a:HobberScaleAxis, depth:Float):Array<RouteHole> {
    var out = [], n = 0;
    var into = F.transformVector(a.headPose, 0, -1, 0);
    for (name in ["hole1", "hole2"]) {
      var f = a.head.connector(name).frame, p = F.transformPoint(a.headPose, f.x, f.y, f.z);
      out.push({id: "H" + (++n), kind: "clearance", thread: "M5", diameter: 5.5, nominal: 5.5, fit: null, depth: depth,
        position: [p.x, p.y, p.z], direction: [into.x, into.y, into.z]});
    }
    return out;
  }
}
