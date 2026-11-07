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
 * The housing is bolted by its side (the fixing holes run through its 18 mm thickness, from the STEP) to a seat wall under `toSeat`, with its travel along
 * `along`, the joint's positive direction; the reading head stands level with it on the reading face's side, `across` from the hole line, carried from
 * the free side by the moving part, its reading finger `headAlong` mm from the housing's start end along the travel.
 */
typedef HobberScaleAxis = {
  var id:String;
  var housing:SealedScalePart;
  var head:SealedScalePart;
  /** Poses at joint zero (the part frames of `SealedScalePart`: travel along +Z, +Y from the free face toward the seat, X across). */
  var housingPose:AssemblyFrame;
  var headPose:AssemblyFrame;
  /** Unit vectors: from the free face toward the seat, along the travel, and across it from the hole line toward the head. */
  var toSeat:Array<Float>;
  var along:Array<Float>;
  var across:Array<Float>;
  /** The ground bar under the housing as {x0, x1, y0, y1, z0, z1}, or null when the housing sits on a plate. */
  var seatBar:Null<Array<Float>>;
}

/**
 * Where the X, Y and Z sealed scales go (FRAME_PLAN H4, reworked to the maker's STEP 2026-11-06). The measuring length comes from the axis stroke plus the overtravel
 * at both ends (`SealedScalePart.pair` picks the smallest standard length); the housing is placed so the reading finger stays inside the
 * measuring window over the whole stroke with margin at both ends.
 *
 * The housing is bolted by its side and the head stands beside it (see `HobberScaleAxis`), so every axis has the housing on a narrow seat and the head's carrier reaching
 * round to the free side:
 * - **X**: housing on a 32 x 15 ground bar on the bed beam's flat top (z 210) beside the right rail, the head on the bed's inboard side of it, under a rib of the carriage.
 * - **Y**: housing on the swivel plate's front face between the guide rails, near the A axis line, with a 3 mm relief milled in the plate under the head's path (the head runs
 *   beside the housing at its seat plane); head on a boss on the Y saddle.
 * - **Z**: housing on a 32 x 15 ground bar on the column channel's inner wall (y 120), head on an arm milled out of a bar and bolted to the Z saddle's back face.
 * The head's M4 enter from the seat side, so each is fitted with its carrier off the machine (`HobberFrameChecks.integrityOptions`).
 * The cable leaves each head toward its housing's start end, along the travel, straight for 54.5 mm past the body (the STEP's stub), then bends in a radius of at least 75 mm
 * (`SealedScaleData.CABLE_BEND_RADIUS`). Only the Y bend is drawn (across the swivel plate's edge, free); the X and Z bends are not, because no direction is free at the end
 * of their stubs (FRAME_PLAN, open item): X leaves toward the swivel stack, where the bend meets the guide rails and blocks one way and the plate's swing at A +15 deg the
 * other, and Z leaves downward 32 mm above the carriage deck at joint zero.
 * The air purge is the M5 port on each housing's end face (`airPort` "end": the tail end, where the housing is 121.5 mm past the measuring window).
 */
class HobberScales {
  /** Top of the bed beam beside the X guide (work z), mm, measured on the casting. */
  public static inline final BED_BEAM_TOP = 210.0;
  /**
   * Thickness of the ground bar the housing stands on, mm: the two M8 (DIN 7984 low head, the JFT-40's 5.75 mm counterbores) through the 12.25 mm left under the head need
   * 8 mm of thread in the seat, which takes the 25 mm screw, and its 12.75 mm of engagement is tapped through 15 mm.
   */
  public static inline final SEAT_BAR = 15.0;
  /**
   * Width of that bar, mm: the standard ground flat (32) under the JFT-40's 32.2 mm housing (STEP). The head stands beside it, level with the housing, 1 mm past the reading face and
   * moving: the bar's edge on that side stops `RUNNING_CLEARANCE` short of the head (1.2 mm inside the reading face) and it runs on past the housing's other face.
   */
  public static inline final SEAT_BAR_WIDTH = 32.0;
  /** Running gap the sweep wants between a moving part and a fixed one, mm (`AssemblyIntegrity`'s 2), and the clearance kept just beyond it. */
  public static inline final RUNNING_CLEARANCE = 2.2;
  /** Inner face of the column channel wall (work y), mm: the casting's wall stands at +120 and -120. */
  public static inline final COLUMN_WALL_Y = 120.0;

  /** Which channel wall the Z scale stands on: the one away from the Z screw's pads (+1 is the +y wall). */
  public static function zWallSign():Float return HobberZLayout.axisY() < 0 ? 1.0 : -1.0;
  /**
   * Where the Z scale's hole line runs along x, mm. The head stands deeper in the channel than the housing (away from the open front, `across` +x): the housing's bar (32 wide,
   * centred on it) then covers x 184.6..216.6 on the wall, just inside the casting's front edge at x 186 (the bottom face is 13.5 from the hole line), and the head (13 wide)
   * stands at x 219.7..232.7, reached by an arm from the saddle.
   */
  public static inline final Z_COLUMN_X = 200.0;
  /**
   * Where the X scale's hole line runs along y, mm: the housing covers y 161.3..193.5, clear of the guide blocks (y 196) by the running gap, and the bar y 162.5..194.5 (they reach 2.5
   * mm and 3.7 mm past the beam's inner edge at y 165: the flat strip between that edge and the rail seat at y 195 is 30 mm wide, narrower than the 32.2 mm housing).
   */
  public static inline final X_HOLE_LINE_Y = 180.0;
  /** Arm that carries the Z head, thickness across the wall and bolt-up height, mm. */
  public static inline final Z_ARM_THICKNESS = 12.0;
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
  public static function y():HobberScaleAxis return cache.get("Y",()->{
    var s=HobberYLayout.moduleSpec(),mount=HobberYLayout.modulePose();
    var hp=F.compose(mount,s.scalePose()),tp=F.compose(mount,s.scalePose(true));
    var normal=F.transformVector(hp,0,1,0),along=F.transformVector(hp,0,0,1),across=F.transformVector(hp,-1,0,0);
    var body=new machinekit.motion.ScrewLinearModulePart(s.values().setToken("role","scaleSeat")).geometry(),b=machinekit.assembly.PosedParts.boxOf(body);body.close();
    return {id:"Y",housing:s.scaleHousing,head:s.scaleHead,housingPose:hp,headPose:tp,toSeat:[normal.x,normal.y,normal.z],along:[along.x,along.y,along.z],across:[across.x,across.y,across.z],seatBar:worldBox(mount,[b.minX,b.maxX,b.minY,b.maxY,b.minZ,b.maxZ])};
  });
  public static function member(id:String,head:Bool=false):String return id=="Y"?"Ymodule/"+(head?"scaleHead":"scaleHousing"):id+(head?"scaleHead":"scale");
  public static function z():HobberScaleAxis return axis("Z");
  public static function all():Array<HobberScaleAxis> return [x(), y(), z()];

  /** Which side of the housing the head stands (the part's `headSide`), where the hole line runs, and where the head's reading finger sits at joint zero. */
  static function layout(id:String, startFor:Float, len:Float):{toSeat:Array<Float>, along:Array<Float>, start:Array<Float>, side:Float, plate:Bool, headCentre:Float, air:String, bend:String} {
    return switch id {
      case "X":
        // The head centre is at x 262 at joint zero; the housing starts so the head's travel sits in the window.
        var at = 262.0 - startFor;
        {toSeat: [0.0, 0, -1], along: [1.0, 0, 0], start: [at, X_HOLE_LINE_Y, BED_BEAM_TOP + SEAT_BAR], side: 1.0, plate: false, headCentre: startFor, air: "end", bend: "none"};
      case "Z":
        var at = HobberZLayout.SEAT_Z + 50.0 - startFor, w = zWallSign();
        {toSeat: [0.0, w, 0], along: [0.0, 0, 1], start: [Z_COLUMN_X, w * (COLUMN_WALL_Y - SEAT_BAR), at], side: w, plate: false, headCentre: startFor, air: "end", bend: "none"};
      default:
        throw 'No scale for axis $id';
    };
  }

  static function axis(id:String):HobberScaleAxis return cache.get(id, () -> derive(id));

  static function cross(a:Array<Float>, b:Array<Float>):Array<Float> return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

  static function derive(id:String):HobberScaleAxis {
    var s = stroke(id), accuracy = id == "X" ? "3" : "5";
    var span = s.upper - s.lower + 2 * overtravel();
    var probe = SealedScalePart.pair("JFT-40", span, accuracy);
    var h0 = probe.housing, len = h0.overallLength(), ml = h0.measuringLength;
    // The head's centre at the lowest reachable joint value sits `margin` inside the window's start; the same margin is left at the far end.
    var margin = (ml - span) / 2;
    // The head centre's distance from the housing start at joint zero: window start + margin - (lower - overtravel).
    var headFromStart = h0.startOffset() + margin - (s.lower - overtravel());
    var place = layout(id, headFromStart, len);
    var values = new machinekit.component.ComponentValues().setToken("headSide", place.side > 0 ? "+x" : "-x").setToken("airPort", place.air).setToken("cableBend", place.bend);
    var pair = SealedScalePart.pair("JFT-40", span, accuracy, "5", false, values);
    var h = pair.housing, t = h.series.housingThickness, up = place.toSeat, along = place.along, p = place.start;
    var housingOrigin = [p[0] - up[0] * t, p[1] - up[1] * t, p[2] - up[2] * t];
    var housingPose = AxisBuilder.orient(housingOrigin[0], housingOrigin[1], housingOrigin[2], up, along);
    var headOrigin = [for (i in 0...3) housingOrigin[i] + along[i] * place.headCentre];
    var headPose = AxisBuilder.orient(headOrigin[0], headOrigin[1], headOrigin[2], up, along);
    var x = cross(up, along), across = [for (v in x) place.side * v];
    // The bar under the housing (the plate needs none: the housing sits on its front face).
    var bb = h.bodyBox(), bar:Null<Array<Float>> = null;
    // The bar's edge toward the head stops short of it; its thickness runs beyond the seat plane.
    var edge = h.gap() - RUNNING_CLEARANCE;
    if (!place.plate) bar = worldBox(housingPose, place.side > 0 ? [bb[1] + edge - SEAT_BAR_WIDTH, bb[1] + edge, t, t + SEAT_BAR, 0, len]
      : [bb[0] - edge, bb[0] - edge + SEAT_BAR_WIDTH, t, t + SEAT_BAR, 0, len]);
    var result:HobberScaleAxis = {id: id, housing: pair.housing, head: pair.head, housingPose: housingPose, headPose: headPose,
      toSeat: up, along: along, across: across, seatBar: bar};
    return result;
  }



  /** A box of a part frame ({x0, x1, y0, y1, z0, z1}) as the box of the work frame that holds it, grown by `grow`. */
  public static function worldBox(pose:AssemblyFrame, box:Array<Float>, grow:Float = 0):Array<Float> {
    var lo = [1e18, 1e18, 1e18], hi = [-1e18, -1e18, -1e18];
    for (xl in [box[0], box[1]]) for (yl in [box[2], box[3]]) for (zl in [box[4], box[5]]) {
      var p = F.transformPoint(pose, xl, yl, zl), v = [p.x, p.y, p.z];
      for (k in 0...3) {
        lo[k] = Math.min(lo[k], v[k]);
        hi[k] = Math.max(hi[k], v[k]);
      }
    }
    return [lo[0] - grow, hi[0] + grow, lo[1] - grow, hi[1] + grow, lo[2] - grow, hi[2] + grow];
  }

  /**
   * The box a carrier of the head must lie within across and along the travel, work frame: the head body grown by `grow`, except on the housing's side, where it stops `RUNNING_CLEARANCE`
   * beyond the housing's reading face (the carrier moves, the housing is fixed, and the head stands only 1 mm from it).
   */
  public static function carrierBox(a:HobberScaleAxis, grow:Float):Array<Float> {
    var hb = a.head.bodyBox(), bb = a.housing.bodyBox(), x0 = hb[0] - grow, x1 = hb[1] + grow;
    if (a.housing.side > 0) x0 = Math.max(x0, bb[1] + RUNNING_CLEARANCE) else x1 = Math.min(x1, bb[0] - RUNNING_CLEARANCE);
    return worldBox(a.headPose, [x0, x1, hb[2] - grow, hb[3] + grow, hb[4] - grow, hb[5] + grow]);
  }

  /** The head body's box in the work frame, grown by `grow`. */
  public static function headBox(a:HobberScaleAxis, grow:Float = 0):Array<Float> return worldBox(a.headPose, a.head.bodyBox(), grow);

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

  /**
   * The carrier's face under the head: tapped to suit the head's two slots (the head's `headThread`, M4), `depth` deep in a carrier `thickness` thick (`origin` is that
   * frame's origin in the work frame; the footprint is centred on the slot pair). The slots run through the head's thickness, so the screws enter from the seat side
   * and thread into the carrier: the head's own face is clamped (`SealedScalePart.fasteningFaces`).
   */
  public static function carrierFace(a:HobberScaleAxis, origin:Array<Float>, depth:Float, thickness:Float):BoltedFace {
    var into = F.transformVector(a.headPose, 0, -1, 0), along = a.along, screw = SocketHeadCapScrew.catalog().get(a.head.series.headThread);
    var f1 = a.head.connector("hole1").frame, f2 = a.head.connector("hole2").frame;
    var p1 = F.transformPoint(a.headPose, f1.x, f1.y, f1.z), p2 = F.transformPoint(a.headPose, f2.x, f2.y, f2.z);
    var mid = {x: (p1.x + p2.x) / 2 - origin[0], y: (p1.y + p2.y) / 2 - origin[1], z: (p1.z + p2.z) / 2 - origin[2]};
    var frame = BoltedFace.frameOf(mid, {x: into.x, y: into.y, z: into.z}, {x: along[0], y: along[1], z: along[2]});
    return BoltedFace.drilledToSuit("scale-head", frame, a.head.series.headHolePitch + 4 * screw.diameter, 4 * screw.diameter, depth, thickness, BoltedFace.ThreadMaterial.Steel);
  }
}
