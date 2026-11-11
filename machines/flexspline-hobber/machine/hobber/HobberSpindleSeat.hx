package hobber;

import machinekit.hobbing.*;

import cadkit.modeling.Part;
import machinekit.motion.ErobActuatorPart;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.ManufacturingRoute.MachinedFace;
import machinekit.structural.GroundStock;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * GE-spindle-seat: the milled split-clamp block that holds the Ø80 spindle cartridge on the Y saddle. Recorded in its own
 * frame, the spindle axis (bore along +Y): the block and the cradle tongue, the bore and the clamp slit, trimmed to the swivel
 * head's clearance, its clamp holes, and datums for its back face, its top faces and their dowels.
 */
class HobberSpindleSeat extends HobberMadePart {
  static final seatRecipe = new machinekit.cache.Once<ComponentType>();

  public static function spindleSeatType():ComponentType return seatRecipe.get(() -> new ComponentType("machinekit.hobbing.spindle-seat", [],
    _ -> new HobberSpindleSeat(), false, false, null, false).versioned(1).numbered(_ -> "HOBBER-GE-SPINDLE-SEAT")
    .procured(_ -> Make("steel fabrication and machining")).requiring(_ -> route().requirements()));

  static function backFrame():AssemblyFrame return BoltedFace.frameOf({x: SEAT_X1, y: (seatY0() + SEAT_Y1) / 2, z: (SEAT_LOW + SEAT_HALF) / 2},
    {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0});
  static function topFrontFrame():AssemblyFrame return BoltedFace.frameOf({x: -1.5, y: 132.5, z: SEAT_HALF}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0});
  static function topRearFrame():AssemblyFrame return BoltedFace.frameOf({x: (SEAT_X0 + SEAT_TONGUE_X1) / 2, y: 223.0, z: SEAT_HALF},
    {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0});
  static function topPinsFrame():AssemblyFrame return BoltedFace.frameOf({x: 0.0, y: HobberMotorCradle.CRADLE_PIN_Y, z: SEAT_HALF},
    {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0});
  public static function seatY0():Float return HobberSkeleton.reference().spindleNativeY - Bt30SpindleData.CLAMP_TO;
  /** Derive every X stack plane from the fixed bed seat and catalogue guide.
   * Replicated block pads occupy relieved plate material, adding no axis height. */
  public static function headFaceX():Float return SEAT_X1;
  /** The seat's two drawing dowels: 4 mm at Y80 and Z +/-35, through the carriage and 6 mm into the seat. Bolts carry the load; no pin strength credit. */
  public static final SEAT_PIN_Y = 80.0;
  public static final SEAT_PIN_Z = 35.0;
  /** Spindle seat block: x from the bore's front side to the saddle plate, y along the cartridge, z across. */
  public static inline final SEAT_X0 = -48.0;
  public static inline final SEAT_X1 = 45.0;
  public static inline final SEAT_Y1 = 222.0;
  /** Half height on the +z side (the slit and clamp screws); the -z side is cut back to `SEAT_LOW` so the head clears the X rail at A +15 deg. */
  public static inline final SEAT_HALF = 58.0;
  public static inline final SEAT_LOW = -50.0;
  public static inline final SEAT_BORE = 80.0;
  public static inline final SEAT_SLIT = 2.0;
  public static inline final SEAT_CLAMP_Z = 49.5;
  /** The tongue behind the clamp that the cradle's foot rests on and bolts into (it reaches x 42 so the rear bolts at x 30 keep 12 mm, 1.5 M8 diameters, to its edge): above the cartridge (its top is z 40), cut by the same slit as the floor. */
  public static inline final SEAT_TONGUE_X1 = 42.0;
  public static inline final SEAT_TONGUE_Y1 = 251.0;
  public static inline final SEAT_TONGUE_Z0 = 44.0;
  /** The seat's hand-drilled holes in its own frame (and the matching ones in the Y saddle): 2 dowels, 3 clamp screws M6 across the slit. */
  public static function seatHoles(inSaddle:Bool):Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var m6 = machinekit.standard.SocketHeadCapScrew.catalog().get("M6").tapDrill;
    var list:Array<machinekit.component.ManufacturingRoute.RouteHole> = [], n = 0, back = SEAT_X1;
    // The four M8 into the seat's back face are derived (`fasteningFaces`: the Y saddle's plate clamps, the seat takes the thread); the dowels are derived too (`pinnedFaces`).
    if (!inSaddle) {
      n = 0;
      for (y in [100.0, 150.0, 200.0]) {
        n++;
        list.push({id: "C" + n, kind: "clearance", thread: "M6", diameter: 6.6, nominal: 6, fit: null, depth: -SEAT_SLIT / 2 - SEAT_X0,
          position: [SEAT_X0, y, SEAT_CLAMP_Z], direction: [1.0, 0, 0]});
        list.push({id: "K" + n, kind: "tapped", thread: "M6", diameter: m6, nominal: 6, fit: null, depth: 20.0,
          position: [SEAT_SLIT / 2, y, SEAT_CLAMP_Z], direction: [1.0, 0, 0]});
      }
    }
    return list;
  }
  /** The four M8 that hold the seat to the Y saddle: y along the cartridge, z across it; 1.5 diameters from the seat's edges (z -50 and 58). */
  public static function seatBolts():Array<machinekit.component.BoltedFace.BoltHole> return [for (y in [95.0, 115.0]) for (z in [-38.0, 46.0])
    {x: y, y: z}];
  /** The spindle seat's extent along the spindle axis (work y) and the y of its three clamp screws. */
  public static function spindleSeatRange():{y0:Float, y1:Float, clampYs:Array<Float>} return {y0: seatY0(), y1: SEAT_Y1, clampYs: [100.0, 150.0, 200.0]};

  // ---- B motor cradle (FRAME_PLAN head redesign) -------------------------------------------------------------------------------
  public function new(?values:ComponentValues) {
    super("GE-spindle-seat", values, new ComponentValues());
    finish(false);
  }
  override public function componentType():Null<ComponentType> return spindleSeatType();

  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    var block = b.label(b.boxBetween(SEAT_X0, SEAT_X1, seatY0(), SEAT_Y1, SEAT_LOW, SEAT_HALF), "Clamp block");
    var tongue = b.label(b.boxBetween(SEAT_X0, SEAT_TONGUE_X1, SEAT_Y1 - 1, SEAT_TONGUE_Y1, SEAT_TONGUE_Z0, SEAT_HALF), "Cradle tongue");
    var body = b.label(b.unionAll([block, tongue]), "Block");
    var bore = b.cylinderAlong(new Vector(0, 0, 0), Vector.Y(), SEAT_BORE / 2, seatY0() - 0.1, SEAT_Y1 + 0.1);
    var slit = b.boxBetween(-SEAT_SLIT / 2, SEAT_SLIT / 2, seatY0() - 0.1, HobberMotorCradle.cradleFaceY() - HobberMotorCradle.CRADLE_SLIT_END, 0, SEAT_HALF + 0.1);
    var bored = b.label(b.cutAll(body, [bore, slit], ["spindle bore", "clamp slit"]), "Bore and slit");
    // The swivel head turns about the spindle by +-swivelRange: keep the side of the inclined plane that clears the X rail.
    var skeleton = HobberSkeleton.reference(), a = skeleton.swivelRange, c = skeleton.railClearance(), sn = Math.sin(a), cs = Math.cos(a);
    var profile = b.polygon([new Vector(-1000, (c + 1000 * sn) / cs, 0), new Vector(0, c / cs, 0), new Vector(1000, (c + 1000 * sn) / cs, 0),
      new Vector(1000, 1000, 0), new Vector(-1000, 1000, 0)], new Plane(new Vector(0, 0, 0), Vector.Y(), Vector.X()));
    var keep = b.extrude(profile, 2000, null, false, true);
    var trimmed = b.label(b.intersect(bored, keep), "Swivel clearance");
    var drilled = b.label(b.holes(trimmed, [for (hole in seatHoles(false)) RouteHoles.spec(hole, F.identity())]), "Clamp screw holes");
    HobberRecording.datum(b, "back", backFrame());
    HobberRecording.datum(b, "top-front", topFrontFrame());
    HobberRecording.datum(b, "top-rear", topRearFrame());
    HobberRecording.datum(b, "top-pins", topPinsFrame());
    return drilled;
  }
  override public function machinable():Bool return true;
  override public function pinnedFaces():Array<PinnedFace> return {
    [PinnedFace.atPoints("back-pins", backFrame(), [for (z in [-SEAT_PIN_Z, SEAT_PIN_Z]) {x: SEAT_X1, y: SEAT_PIN_Y, z: z}], 4.0, 6.0, false)
        .at(recordedDatum("back")),
      PinnedFace.atPoints("top-pins", topPinsFrame(), [for (x in HobberMotorCradle.CRADLE_PIN_X) {x: x, y: HobberMotorCradle.CRADLE_PIN_Y, z: SEAT_HALF}], 8.0,
        12.0, false).at(recordedDatum("top-pins"))];
  };
  override public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return {
    derivedRouteHoles().concat(seatHoles(false)).concat(HobberMotorCradle.cradleHoles(true));
  };
  override public function manufacturingRoute():Null<ManufacturingRoute> return route();

  static function route():ManufacturingRoute return {
    ManufacturingRoute.milled(2, "tight", [
      {name: "spindle bore, clamp slit closed at the screw torque (cylindricity)", tolerance: 0.01, roughnessRa: 0.8},
      {name: "back face, square to the bore", tolerance: 0.02, roughnessRa: 1.6}
    ], 160);
  };
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace>=[];
    // The back face on the Y saddle's plate: tapped to suit the plate's clearance holes (M8, 12 mm plate).
        // The cradle floor's M8 into the top (z 58): ahead of y 195 over the bore (the wall under the thread is what is left above its Ø80), behind it into the 14 mm tongue, tapped through.
        faces.push(BoltedFace.drilledToSuit("topFront", recordedDatum("top-front"), SEAT_X1 - SEAT_X0, 125, 16,
          SEAT_HALF - SEAT_LOW, BoltedFace.ThreadMaterial.Steel, null, false, (u, _) -> SEAT_HALF - Math.sqrt(Math.max(0.0, SEAT_BORE * SEAT_BORE / 4 - (u - 1.5) * (u - 1.5)))));
        faces.push(BoltedFace.drilledToSuit("topRear", recordedDatum("top-rear"),
          SEAT_TONGUE_X1 - SEAT_X0, 56, 14, SEAT_HALF - SEAT_TONGUE_Z0, BoltedFace.ThreadMaterial.Steel, null, true));
        faces.push(BoltedFace.drilledToSuit("back", recordedDatum("back"),
          SEAT_Y1 - seatY0(), SEAT_HALF - SEAT_LOW, 16, SEAT_X1 - SEAT_X0, BoltedFace.ThreadMaterial.Steel));
    return faces;
  }
}
