package hobber;

import machinekit.hobbing.*;

import cadkit.modeling.Part;
import machinekit.motion.ErobActuatorPart;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.ManufacturingRoute.MachinedFace;
import machinekit.structural.GroundStock;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** GE-motor-cradle; owns its stock, interfaces and manufacturing route. */
class HobberMotorCradle extends HobberMadePart {
  // ---- B motor cradle (FRAME_PLAN head redesign) -------------------------------------------------------------------------------

  /**
   * The cradle wall: it stands behind the cartridge's rear face (y 240), the motor flange on its front face; the pulleys stand clear of its rear
   * face and the flange bolts' nuts. The servo slides along the belt's line (work z) in slots through the wall, so the nuts are on the rear face.
   */
  public static inline final CRADLE_WALL = 8.0;
  /** The wall's front face (the motor flange face) in work y. */
  /**
   * The floor sits on the spindle seat block's top (z `SEAT_HALF`). Behind `CRADLE_FLOOR_STEP` (where the seat block ends and the wall's foot
   * overhangs it) it is `CRADLE_FLOOR` thick; ahead, where it only carries the first bolt row, `CRADLE_FLOOR_THIN`.
   */
  public static inline final CRADLE_FLOOR = 12.0;
  public static inline final CRADLE_FLOOR_THIN = 8.0;
  /**
   * Under the wall the floor is a `CRADLE_FOOT` thick foot (from the seat's end `SEAT_Y1`), and its rear bolt row stands just ahead of the wall's front
   * face, so the wall's foot overhangs the bolts by a few millimetres, not a floor's length: the foot rests on the seat's tongue (`SEAT_TONGUE_*`), which
   * carries the bolts' tapped holes under the wall's own footprint. The flange stays above the foot (the foot top is 5 mm under the flange's underside).
   */
  public static inline final CRADLE_FOOT = 18.0;
  public static inline final CRADLE_REAR_BOLT_Y = 232.0;
  public static inline final CRADLE_FLOOR_Y0 = 125.0;
  public static inline final CRADLE_FLOOR_STEP = 195.0;
  /** The wall stands clear of the Y drive's motor and bracket (x from 35) on the +x side: the servo is 8 mm toward -x of the cartridge (the flange's holes are 31.8 from its axis). */
  public static inline final CRADLE_X0 = -56.0;
  public static inline final CRADLE_X1 = 31.0;
  /** The floor ahead of the step is trimmed to the bolts' width; the thick part reaches farther on the +x side (x 38) for its bolts. */
  public static inline final CRADLE_FLOOR_THIN_X0 = -44.0;
  public static inline final CRADLE_FLOOR_X1 = 38.0;
  /** The servo's axis in work x. */
  public static inline final CRADLE_MOTOR_X = -HobSpindle.MOTOR_OFFSET;
  /** One rib stiffens the wall against the servo rocking on its offset centre of mass: 6 thick, outside the flange, from y `CRADLE_RIB_Y0` to the wall's rear, with a lightening window. */
  public static inline final CRADLE_RIB = 6.0;
  public static inline final CRADLE_RIB_Y0 = 192.0;
  /** The rib's window (work y, z above the cartridge axis): a 17 mm front post and a 15 mm rear post stay, and the foot below z 95. */
  public static inline final CRADLE_RIB_WINDOW_Y0 = 209.0;
  public static inline final CRADLE_RIB_WINDOW_Y1 = 232.0;
  public static inline final CRADLE_RIB_WINDOW_Z0 = 95.0;
  public static inline final CRADLE_RIB_WINDOW_Z1 = 160.0;
  /** The slit through the floor, over the seat's own, so the clamp's two arms are not bridged by the floor (it stops short of the wall by this much). */
  public static inline final CRADLE_SLIT = 4.0;
  public static inline final CRADLE_SLIT_END = 3.0;
  /** The jack screw's lug under the servo flange: it stands on the floor against the wall's front face, `CRADLE_LUG` above the foot top (the underside of the flange is 5 mm above the foot), M8 vertical in the flange's y middle. */
  public static inline final CRADLE_LUG = 4.0;
  public static inline final CRADLE_LUG_Y0 = 224.0;
  public static inline final CRADLE_LUG_HALF = 13.0;
  /** Width of the flange-bolt slots and of the rest, mm: M6 clearance 6.6 plus 0.4. */
  public static inline final CRADLE_SLOT = 7.0;
  /**
   * The rest block under the servo's body (FE, 2026-11-06): the servo hangs 46 mm ahead of the wall on a plate wall, and with the floor alone its first mode is 340 Hz.
   * A block on the floor from `CRADLE_REST_Y0` to `CRADLE_REST_Y1` across `CRADLE_REST_X0..X1` (the dowels' x +- 4) carries the body's underside through a ground shim
   * pack set at assembly: the belt's pull and the weight press the body on it, so the contact stays closed under any vibration that the cutting load excites.
   * The top stands `TENSION_TRAVEL` plus `CRADLE_REST_CLEAR` under the body's underside at the nominal centres so the servo can still slide down to the bottom of its slots
   * to fit the belt; the pack is that gap plus the tension travel (peel shims). The slit runs through it.
   */
  public static inline final CRADLE_REST_Y0 = 145.0;
  public static inline final CRADLE_REST_Y1 = 200.0;
  public static inline final CRADLE_REST_X0 = -44.0;
  public static inline final CRADLE_REST_X1 = 36.0;
  public static inline final CRADLE_REST_CLEAR = 0.5;
  /** The floor's two dowels into the seat block's top: 8 mm at these x, y 190, reamed through the 8 mm floor and 12 mm into the seat. */
  public static final CRADLE_PIN_Y = 190.0;
  public static final CRADLE_PIN_X = [-40.0, 24.0];
  /** The floor's four M8 into the seat block's top and its tongue: two rows 67 apart along the spindle, 60 apart across it. */
  public static function cradleFloorBolts():Array<{x:Float, y:Float}> return [{x: -30.0, y: CRADLE_FRONT_BOLT_Y}, {x: 30.0, y: CRADLE_FRONT_BOLT_Y}, {x: -30.0, y: CRADLE_REAR_BOLT_Y}, {x: 30.0, y: CRADLE_REAR_BOLT_Y}];
  public static inline final CRADLE_FRONT_BOLT_Y = 135.0;
  /** The rear row's counterbore for the M8 heads (Ø13 x 8): the heads end below the foot's top, under the servo's flange. */
  public static inline final CRADLE_BORE = 14.0;
  public static inline final CRADLE_BORE_DEPTH = 9.0;
  /** The floor's top at work y: the thin part ahead of the step. */
  public static function cradleFloorTop(y:Float):Float return HobberSpindleSeat.SEAT_HALF + (y >= HobberSpindleSeat.SEAT_Y1 ? CRADLE_FOOT : y < CRADLE_FLOOR_STEP ? CRADLE_FLOOR_THIN : CRADLE_FLOOR);
  /** Work z (seat frame) of the rest block's top: under the servo body's underside at the bottom of its slot travel by `CRADLE_REST_CLEAR`. */
  public static function cradleRestTop():Float {
    var flange = cradleMotor().mounting();
    if (flange == null) flange = machinekit.motion.ServoMotor.assumedMounting(cradleMotor().rating.designation);
    return HobSpindle.motorHeight() - flange.face / 2 - HobSpindle.TENSION_TRAVEL - CRADLE_REST_CLEAR;
  }
  /** The rest block's footprint and top, mm (seat frame): the FE couples its top to the servo group through the shim pack. */
  public static function cradleRest():{x0:Float, x1:Float, y0:Float, y1:Float, top:Float} return {x0: CRADLE_REST_X0, x1: CRADLE_REST_X1, y0: CRADLE_REST_Y0, y1: CRADLE_REST_Y1, top: cradleRestTop()};
  /** Work y of the wall's front face (the motor flange face). */
  public static function cradleFaceY():Float return HobberLayout.get().spindleNativeY - HobSpindle.MOTOR_FACE_Z;
  /** The servo's axis in work x (the cradle's authored frame). */
  public static inline function cradleMotorX():Float return CRADLE_MOTOR_X;
  /**
   * The dimensions that decide the cradle's FE (and the hand model's inputs) as one string: the FE reference in `HobberFeResults` is valid only for this signature.
   */
  public static function cradleSignature():String {
    var rest = cradleRest(), r1 = function(v:Float) return Std.string(Math.round(v * 10) / 10);
    return "wall " + r1(CRADLE_WALL) + ", rib " + r1(CRADLE_RIB) + " from y " + r1(CRADLE_RIB_Y0) + " windowed y " + r1(CRADLE_RIB_WINDOW_Y0) + ".." + r1(CRADLE_RIB_WINDOW_Y1) + " z " + r1(CRADLE_RIB_WINDOW_Z0) + ".." + r1(CRADLE_RIB_WINDOW_Z1)
      + ", floor " + r1(CRADLE_FLOOR) + "/" + r1(CRADLE_FLOOR_THIN) + " from y " + r1(CRADLE_FLOOR_Y0) + ", foot " + r1(CRADLE_FOOT) + ", bolts " + [for (b in cradleFloorBolts()) r1(b.x) + "," + r1(b.y)].join(" ")
      + ", rest x " + r1(rest.x0) + ".." + r1(rest.x1) + " y " + r1(rest.y0) + ".." + r1(rest.y1) + " top " + r1(rest.top) + ", x " + r1(CRADLE_X0) + ".." + r1(CRADLE_X1);
  }
  /** Work y of the wall's rear face. */
  public static function cradleRearY():Float return cradleFaceY() + CRADLE_WALL;
  /**
   * The cradle's section for the drive-mode estimate: wall thickness and width, height above the floor, the rib's thickness, x and depth ahead of the
   * wall's rear face, the floor bolts, mm.
   */
  public static function cradleSection():{wall:Float, width:Float, x0:Float, height:Float, ribThickness:Float, ribDepth:Float, ribX:Float, floorTop:Float, bolts:Array<{x:Float, y:Float}>} {
    return {wall: CRADLE_WALL, width: CRADLE_X1 - CRADLE_X0, x0: CRADLE_X0, ribX: CRADLE_X0 - 2.0, height: HobSpindle.motorHeight() + 47 - (HobberSpindleSeat.SEAT_HALF + CRADLE_FOOT), ribThickness: CRADLE_RIB,
      ribDepth: cradleRearY() - CRADLE_RIB_Y0, floorTop: HobberSpindleSeat.SEAT_HALF + CRADLE_FOOT, bolts: cradleFloorBolts()};
  }
  /**
   * The cradle's wall section at height `z` (seat frame) as x slabs `{x0, x1, y0, y1}` (mm): the wall at full thickness (the pilot recess, the shaft slot and the bolt slots are bypassed by the
   * load, which flows along the strips between them: FE of the cradle alone, `fem/`), and with `withRib` the rib beside the wall, whose window leaves its front post (from the rib's front
   * edge to the window) and its rear post between z `CRADLE_RIB_WINDOW_Z0` and `Z1`. Overlapping slabs are merged by x into one y extent each.
   */
  public static function cradleSlabs(z:Float, withRib:Bool):Array<{x0:Float, x1:Float, y0:Float, y1:Float}> {
    var rects = [{x0: CRADLE_X0, x1: CRADLE_X1, y0: cradleFaceY(), y1: cradleRearY()}];
    if (withRib) {
      var rx0 = CRADLE_X0 - 5.0, rx1 = CRADLE_X0 + 1.0;
      if (z >= CRADLE_RIB_WINDOW_Z0 && z <= CRADLE_RIB_WINDOW_Z1) {
        rects.push({x0: rx0, x1: rx1, y0: CRADLE_RIB_Y0, y1: CRADLE_RIB_WINDOW_Y0});
        rects.push({x0: rx0, x1: rx1, y0: CRADLE_RIB_WINDOW_Y1, y1: cradleRearY()});
      } else rects.push({x0: rx0, x1: rx1, y0: CRADLE_RIB_Y0, y1: cradleRearY()});
    }
    var xs:Array<Float> = [];
    for (r in rects) for (x in [r.x0, r.x1]) if (xs.indexOf(x) < 0) xs.push(x);
    xs.sort((a, b) -> a < b ? -1 : a > b ? 1 : 0);
    var slabs = [];
    for (i in 0...xs.length - 1) {
      var mid = 0.5 * (xs[i] + xs[i + 1]), lo = 1e9, hi = -1e9;
      for (r in rects) if (r.x0 <= mid && mid <= r.x1) {
        lo = Math.min(lo, r.y0);
        hi = Math.max(hi, r.y1);
      }
      if (hi > lo) slabs.push({x0: xs[i], x1: xs[i + 1], y0: lo, y1: hi});
    }
    return slabs;
  }
  /** The thicknesses the drive check reports: wall, thick floor, thin floor, jack lug height above the floor, slot travel (each way), slot width. */
  public static function cradleDimensions():{wall:Float, floor:Float, floorThin:Float, foot:Float, lug:Float, travel:Float, slot:Float, jackY:Float} {
    return {wall: CRADLE_WALL, floor: CRADLE_FLOOR, floorThin: CRADLE_FLOOR_THIN, foot: CRADLE_FOOT, lug: CRADLE_LUG, travel: HobSpindle.TENSION_TRAVEL, slot: CRADLE_SLOT, jackY: cradleFaceY() - 4.0};
  }
  public static function cradleMotor():machinekit.motion.ServoMotor return machinekit.motion.LeadshineServo.driven(HobSpindle.MOTOR);
  /** A slot as `holeTool` ends and the box between them: vertical (work z) with the round ends at `zc` +- `travel`, cut into +y from the mouth at `y0`. */
  public static function slotCuts(x:Float, zc:Float, width:Float, travel:Float, y0:Float, depth:Float):Array<Part> {
    return [HobberPartGeometry.holeTool([x, y0, zc - travel], [0.0, 1, 0], width, depth), HobberPartGeometry.holeTool([x, y0, zc + travel], [0.0, 1, 0], width, depth),
      HobberPartGeometry.box(x - width / 2, x + width / 2, y0 - 1, y0 + depth, zc - travel, zc + travel)];
  }
  /**
   * The cradle's holes in the work frame lowered by the seat height (`inSeat` false), or the seat block's matching tapped holes (true): the
   * floor's four M8 and two dowels into the seat's top, and in the cradle the motor's pilot register, four flange bolts and the
   * shaft's through bore, all three vertical slots (listed as their two round ends), and the jack screw's tapped M8 in the lug.
   */
  public static function cradleHoles(inSeat:Bool):Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var list:Array<machinekit.component.ManufacturingRoute.RouteHole> = [], m8 = machinekit.standard.SocketHeadCapScrew.catalog().get("M8").tapDrill;
    var n = 0;
    // The floor's four M8 (clearance, the rear row's counterbore, the seat's thread) are derived (`fasteningFaces`).
    if (inSeat) return list;
    var motor = cradleMotor(), flange = motor.mounting();
    if (flange == null) flange = machinekit.motion.ServoMotor.assumedMounting(motor.rating.designation);
    var c = HobSpindle.motorHeight(), t = HobSpindle.TENSION_TRAVEL, k = 0;
    // The pilot register and the shaft bore are slots too (the servo slides on its flange bolts, the pilot has the travel to spare).
    list.push({id: "P1", kind: "bore", thread: null, diameter: flange.pilotDiameter + 0.2, nominal: flange.pilotDiameter, fit: "H7",
      depth: flange.pilotHeight, position: [CRADLE_MOTOR_X, cradleFaceY(), c], direction: [0.0, 1, 0], slotLength: 2 * t, slotAxis: [0.0, 0, 1]});
    list.push({id: "P2", kind: "bore", thread: null, diameter: motor.rating.shaftDiameter + 8, nominal: motor.rating.shaftDiameter + 8, fit: null,
      depth: CRADLE_WALL, position: [CRADLE_MOTOR_X, cradleFaceY(), c], direction: [0.0, 1, 0], slotLength: 2 * t, slotAxis: [0.0, 0, 1]});
    // The flange bolts: M6 from the motor side through the slot and a washer and nut on the wall's rear face.
    for (h in flange.holes) {
      k++;
      list.push({id: "M" + k, kind: "clearance", thread: "M" + Std.int(flange.holeDiameter - 0.5), diameter: CRADLE_SLOT, nominal: flange.holeDiameter - 0.5, fit: null,
        depth: CRADLE_WALL, position: [CRADLE_MOTOR_X + h.x, cradleFaceY(), c + h.y], direction: [0.0, 1, 0], slotLength: 2 * t, slotAxis: [0.0, 0, 1]});
    }
    // The jack screw: M8 up through the lug, its tip on the flange's underside.
    list.push({id: "J1", kind: "tapped", thread: "M8", diameter: m8, nominal: 8, fit: null, depth: CRADLE_LUG + 12.0,
      position: [CRADLE_MOTOR_X, cradleFaceY() - 4.0, cradleFloorTop(cradleFaceY()) + CRADLE_LUG], direction: [0.0, 0, -1]});
    return list;
  }
  /** The motor cradle: a milled steel part, a floor on the seat block's top, a wall with the servo's slotted flange register, a jack lug, one windowed rib and the rest block under the servo's body. */
  public static function motorCradle():Part {
    var c = HobSpindle.motorHeight(), top = c + 47, rear = cradleRearY(), low = HobberSpindleSeat.SEAT_HALF, floor = low + CRADLE_FLOOR, thin = low + CRADLE_FLOOR_THIN, foot = low + CRADLE_FOOT;
    var body = Solids.union([HobberPartGeometry.box(CRADLE_FLOOR_THIN_X0, CRADLE_FLOOR_X1, CRADLE_FLOOR_Y0, CRADLE_FLOOR_STEP, low, thin),
      HobberPartGeometry.box(CRADLE_X0, CRADLE_FLOOR_X1, CRADLE_FLOOR_STEP, HobberSpindleSeat.SEAT_Y1, low, floor),
      HobberPartGeometry.box(CRADLE_X0, CRADLE_FLOOR_X1, HobberSpindleSeat.SEAT_Y1, rear, low, foot),
      HobberPartGeometry.box(CRADLE_REST_X0, CRADLE_REST_X1, CRADLE_REST_Y0, CRADLE_REST_Y1, low, cradleRestTop()),
      HobberPartGeometry.box(CRADLE_X0, CRADLE_X1, cradleFaceY(), rear, foot, top),
      HobberPartGeometry.box(CRADLE_X0 - 5, CRADLE_X0 + 1, CRADLE_RIB_Y0, rear, low, top),
      HobberPartGeometry.box(CRADLE_MOTOR_X - CRADLE_LUG_HALF, CRADLE_MOTOR_X + CRADLE_LUG_HALF, CRADLE_LUG_Y0, cradleFaceY() + 0.5, foot, foot + CRADLE_LUG)]);
    var slit = HobberPartGeometry.box(-CRADLE_SLIT / 2, CRADLE_SLIT / 2, CRADLE_FLOOR_Y0 - 1, cradleFaceY() - CRADLE_SLIT_END, low - 1, Math.max(foot, cradleRestTop()) + 1);
    var window = HobberPartGeometry.box(CRADLE_X0 - 6, CRADLE_X0 + 2, CRADLE_RIB_WINDOW_Y0, CRADLE_RIB_WINDOW_Y1, CRADLE_RIB_WINDOW_Z0, CRADLE_RIB_WINDOW_Z1);
    var flange = cradleMotor().mounting();
    if (flange == null) flange = machinekit.motion.ServoMotor.assumedMounting(cradleMotor().rating.designation);
    var t = HobSpindle.TENSION_TRAVEL, cuts = [slit, window];
    // Pilot recess, shaft bore and the four flange-bolt slots, then the plain holes.
    cuts = cuts.concat(slotCuts(CRADLE_MOTOR_X, c, flange.pilotDiameter + 0.2, t, cradleFaceY(), flange.pilotHeight));
    cuts = cuts.concat(slotCuts(CRADLE_MOTOR_X, c, cradleMotor().rating.shaftDiameter + 8, t, cradleFaceY(), CRADLE_WALL));
    for (h in flange.holes) cuts = cuts.concat(slotCuts(CRADLE_MOTOR_X + h.x, c + h.y, CRADLE_SLOT, t, cradleFaceY(), CRADLE_WALL));
    var plain = [for (h in cradleHoles(false)) if (h.slotLength == null) h];
    return HobberPartGeometry.withHoles(Solids.cut(body, cuts), plain);
  }
  public function new(?values:ComponentValues) { super("GE-motor-cradle",values); finish(true); }
  override public function machinable():Bool return true;
  override public function pinnedFaces():Array<PinnedFace> return {
    [PinnedFace.atPoints("floor-pins", BoltedFace.frameOf({x: 0.0, y: CRADLE_PIN_Y, z: HobberSpindleSeat.SEAT_HALF}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0}),
      [for (x in CRADLE_PIN_X) {x: x, y: CRADLE_PIN_Y, z: HobberSpindleSeat.SEAT_HALF}], 8.0, cradleFloorTop(CRADLE_PIN_Y) - HobberSpindleSeat.SEAT_HALF, true)];
  };
  override public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return {
    derivedRouteHoles().concat(cradleHoles(false));
  };
  override public function pieces():Array<Part> return {
    [motorCradle()];
  };
  override public function manufacturingRoute():Null<ManufacturingRoute> return {
    ManufacturingRoute.milled(2, "fine", [
      {name: "motor flange face, front (square to the shaft bore; pilot register)", tolerance: 0.02, roughnessRa: 1.6},
      {name: "floor underside, on the seat block's top", tolerance: 0.02, roughnessRa: 1.6},
      {name: "rest block top, ground flat (the shim pack's seat under the servo body)", tolerance: 0.02, roughnessRa: 1.6}
    ], 180);
  };
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace>=[];
    // The floor's four M8 into the seat: heads on the floor top, clearance holes through it; the rear row's heads are sunk (Ø14 x 9) under the servo's flange.
        for (row in [CRADLE_FRONT_BOLT_Y, CRADLE_REAR_BOLT_Y]) {
          var rear = row >= HobberSpindleSeat.SEAT_Y1, floor = cradleFloorTop(row) - HobberSpindleSeat.SEAT_HALF;
          faces.push(BoltedFace.clamped(rear ? "floorRear" : "floorFront", BoltedFace.frameOf({x: 0.0, y: row, z: HobberSpindleSeat.SEAT_HALF}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0}),
            [for (b in cradleFloorBolts()) if (b.y == row) {x: b.x, y: 0.0}], machinekit.standard.SocketHeadCapScrew.catalog().get("M8").clearanceMedium,
            rear ? floor - CRADLE_BORE_DEPTH : floor, "M8", rear ? {diameter: CRADLE_BORE, depth: CRADLE_BORE_DEPTH} : null, false));
        }
        // The servo's flange screws pass through the wall's vertical tension slots (drawn by `cradleHoles`) and take a washer
        // and nut on its rear face: a through plate, so the screws, washers and nuts derive while the slots stay the part's own.
        var motor = cradleMotor(), flange = motor.mounting();
        if (flange == null) flange = machinekit.motion.ServoMotor.assumedMounting(motor.rating.designation);
        faces.push(BoltedFace.through("motor", BoltedFace.frameOf({x: CRADLE_MOTOR_X, y: cradleFaceY(), z: HobSpindle.motorHeight()},
          {x: 0, y: 1, z: 0}, {x: 1, y: 0, z: 0}), [for (h in flange.holes) {x: h.x, y: -h.y}], flange.holeDiameter, CRADLE_WALL,
          "M" + Std.int(flange.holeDiameter - 0.5)).withSlots(CRADLE_SLOT, HobSpindle.TENSION_TRAVEL, "y"));
    return faces;
  }
}
