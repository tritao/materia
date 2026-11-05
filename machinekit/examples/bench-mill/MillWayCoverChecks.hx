import machinekit.cover.WayCoverCatalog;
import machinekit.cover.WayCoverPlan;
import machinekit.motion.ScrewSupportUnit;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** One place the mill's screws are open to chips and what the way cover for it would have to be. */
typedef MillCoverSpot = {
  /** `<axis> <end>`: the covered screw and the side of the carrier it is on. */
  var name:String;
  /** The screw's clear opening (the screw with running clearance), mm. */
  var opening:Float;
  var stroke:Float;
  /** Length between the two mounting planes where they are closest, mm. */
  var space:Float;
  /** The most the cover's skins may reach from the screw axis across and up (the nearest other member), mm. */
  var roomAcross:Float;
  var roomUp:Float;
  var roomDown:Float;
  /** What bounds that room, for the report. */
  var limits:String;
  /** The smaller side of the face each flange would bolt to (the carrier of the moving end, the support of the fixed end), mm, and what those are. */
  var movingFace:Float;
  var fixedFace:Float;
  var hosts:String;
};

/**
 * The way-cover study of the bench mill: for each exposed screw, the cover the catalogue gives for its stroke and the space between
 * the support that would carry its fixed flange and the carrier that moves its other end (`WayCoverPlan`: folds or sections,
 * folded and extended length, nested skins and the outer size of the flanges), against the room the mill leaves around the screw.
 * The numbers come from the mill's solved poses; the verdicts are asserted, so a change of the mill that makes a cover fit (or
 * stops one fitting) shows here. `WayCover.install` is not applied: no cover fits the mill as it is built (README, "Way covers").
 */
class MillWayCoverChecks {
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw 'Mill way covers: $message';
  }

  static function r(value:Float):Float return Math.round(value * 10) / 10;

  /** "Nothing else limits it" for a room, mm. */
  static inline var OPEN:Float = 1e6;

  /** The nut bracket's plate, mm (`BenchMill.MillNutBracket`). */
  static inline var NUT_PLATE:Float = 8;
  /** Clear opening round the 16 mm screw: running clearance each side. */
  static inline var OPENING:Float = 24;

  public static function spots(mill:BenchMill):Array<MillCoverSpot> {
    var poses:Map<String, AssemblyFrame> = mill.solvedPoses();
    var bk = ScrewSupportUnit.bk12(), bf = ScrewSupportUnit.bf12();
    var out:Array<MillCoverSpot> = [];
    function pose(id:String):AssemblyFrame return poses.get(id);
    function spec(id:String):BenchMill.MillAxisSpec {
      for (candidate in mill.specs) if (candidate.id == id) return candidate;
      throw 'The mill has no axis "$id"';
    }
    // Y: the saddle moves along -Y as its coordinate grows (the joint is made along [0, -1, 0]).
    var y = spec("y"), saddle = pose("saddle"), halfDepth = mill.saddle.depth / 2;
    var forward = y.initial - y.lower, back = y.upper - y.initial;
    // The saddle's front face at its frontmost stop is the nearest to the fixed (front) support; the nut bracket's outer face at its rearmost
    // stop is the nearest to the floating support.
    var frontSupport = pose("screwYFixed").y + bk.length / 2;
    var rearSupport = pose("screwYFloating").y - bf.length / 2;
    var saddleFrontNearest = saddle.y - halfDepth - back;
    var bracketRearNearest = saddle.y + halfDepth + NUT_PLATE + forward;
    // The carrier of the moving end is the nut's bracket plate (the saddle's own front face for Y front); the fixed end bolts to a support housing.
    var nutFlange = new machinekit.motion.BallNut().flangeDiameter;
    var bkFace = Math.min(bk.width, bk.height), bfFace = Math.min(bf.width, bf.height);
    var yRoomAcross = mill.base.railSpacing / 2 - 17;
    var yRoomDown = pose("screwY").z - mill.base.height;
    out.push({name: "Y front", opening: OPENING, stroke: y.upper - y.lower, space: saddleFrontNearest - frontSupport,
      roomAcross: yRoomAcross, roomUp: OPEN, roomDown: yRoomDown, limits: "Y rails and blocks at 53 mm from the axis, base top 63 mm below it",
      movingFace: Math.min(mill.saddle.height, mill.saddle.width), fixedFace: bkFace, hosts: "the saddle's front face and the BK12 housing"});
    out.push({name: "Y rear", opening: OPENING, stroke: y.upper - y.lower, space: rearSupport - bracketRearNearest,
      roomAcross: yRoomAcross, roomUp: OPEN, roomDown: yRoomDown, limits: "Y rails and blocks at 53 mm from the axis, base top 63 mm below it",
      movingFace: nutFlange + 22, fixedFace: bfFace, hosts: "the nut bracket's plate and the BF12 housing"});
    // X: the table moves along -X as its coordinate grows. The nut body's left end and its bracket's right face bound the covers.
    var x = spec("x"), nut = pose("screwXNut"), bracket = pose("screwXNutBracket");
    var left = x.initial - x.lower, right = x.upper - x.initial;
    var leftSupport = pose("screwXFixed").x + bk.length / 2;
    var rightSupport = pose("screwXFloating").x - bf.length / 2;
    var nutLeftNearest = nut.x - right;
    var bracketRightNearest = bracket.x + NUT_PLATE + left;
    var xRoom = pose("screwX").z - (mill.base.height + 28 + mill.saddle.height);
    out.push({name: "X left", opening: OPENING, stroke: x.upper - x.lower, space: nutLeftNearest - leftSupport,
      roomAcross: mill.saddle.railSpacing / 2 - 17, roomUp: 14, roomDown: xRoom,
      limits: "saddle top 14 mm below the screw, table underside 14 mm above it, X blocks 28 mm from the axis",
      movingFace: nutFlange + 2, fixedFace: bkFace, hosts: "the nut and its bracket plate and the BK12 housing"});
    out.push({name: "X right", opening: OPENING, stroke: x.upper - x.lower, space: rightSupport - bracketRightNearest,
      roomAcross: mill.saddle.railSpacing / 2 - 17, roomUp: 14, roomDown: xRoom,
      limits: "saddle top 14 mm below the screw, table underside 14 mm above it, X blocks 28 mm from the axis",
      movingFace: nutFlange + 2, fixedFace: bfFace, hosts: "the nut bracket's plate and the BF12 housing"});
    // Z: the head moves along +Z as its coordinate grows (lower is -250, the top stop is 0); the supports stand 26 mm from the column face.
    var z = spec("z"), zNut = pose("screwZNut"), zBracket = pose("screwZNutBracket");
    var down = z.upper - z.initial, up = z.initial - z.lower;
    var topSupport = pose("screwZFixed").z - bk.length / 2;
    var bottomSupport = pose("screwZFloating").z + bf.length / 2;
    var zAcross = pose("screwZ").x - mill.column.width / 2;
    out.push({name: "Z top", opening: OPENING, stroke: z.upper - z.lower, space: topSupport - (zNut.z + 0),
      roomAcross: zAcross, roomUp: OPEN, roomDown: OPEN, limits: "column side face 26 mm from the axis",
      movingFace: nutFlange + 2, fixedFace: bkFace, hosts: "the nut and the BK12 housing"});
    out.push({name: "Z bottom", opening: OPENING, stroke: z.upper - z.lower, space: (zBracket.z - NUT_PLATE - up) - bottomSupport,
      roomAcross: zAcross, roomUp: OPEN, roomDown: OPEN, limits: "column side face 26 mm from the axis",
      movingFace: nutFlange + 2, fixedFace: bfFace, hosts: "the nut bracket's plate and the BF12 housing"});
    return out;
  }

  /** The cover a spot would need from `rowId`: its plan, or the reason it cannot be made. */
  public static function verdict(spot:MillCoverSpot, rowId:String):{fits:Bool, text:String} {
    var plan:WayCoverPlan;
    try plan = new WayCoverPlan(WayCoverCatalog.get(rowId), spot.opening, spot.opening, spot.stroke, spot.space) catch (e:Dynamic)
      return {fits: false, text: Std.string(e)};
    var halfWidth = plan.outerWidth() / 2, halfHeight = plan.outerHeight() / 2;
    var text = '${plan.folds > 0 ? plan.folds + " folds" : plan.sections + " sections"}, ${r(plan.compressed)} to ${r(plan.extended)} mm, '
      + '${plan.levels} nested skins, flanges ${r(plan.outerWidth())} x ${r(plan.outerHeight())} mm';
    var room = halfWidth <= spot.roomAcross + 1e-9 && halfHeight <= Math.min(spot.roomUp, spot.roomDown) + 1e-9;
    var faces = plan.outerWidth() <= Math.min(spot.movingFace, spot.fixedFace) + 1e-9;
    var why = [];
    if (!room) why.push('the room round the screw is ${r(spot.roomAcross)} across and ${spot.roomUp >= OPEN && spot.roomDown >= OPEN ? "open" : r(Math.min(spot.roomUp, spot.roomDown))} up/down from the axis');
    if (!faces) why.push('the flanges would bolt to ${spot.hosts}, ${r(Math.min(spot.movingFace, spot.fixedFace))} mm across at most');
    return {fits: room && faces, text: text + (why.length == 0 ? "" : "; " + why.join("; "))};
  }

  public static function main():Void {
    var mill = new BenchMill(false, false);
    var found = 0, fitting = 0;
    for (spot in spots(mill)) {
      Sys.println('${spot.name}: stroke ${r(spot.stroke)} mm, ${r(spot.space)} mm between the flange planes at the nearest stop (${spot.limits})');
      for (row in WayCoverCatalog.ids()) {
        var v = verdict(spot, row);
        Sys.println('  $row: ${v.fits ? "fits" : "does not fit"}: ${v.text}');
        found++;
        if (v.fits) fitting++;
      }
    }
    // The study's findings, asserted: every cover as built is refused, by space or by the room round the screw.
    check(fitting == 0, 'no cover is expected to fit the mill as built, but $fitting do; install them with WayCover.install');
    Sys.println('mill way covers: $found cover/row pairs studied, none fits the mill as built');
  }
}
