package machinekit.hobbing;

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

/** GE-Z-saddle; owns its stock, interfaces and manufacturing route. */
class HobberZSaddle extends HobberMadePart {
  public static function zSaddleOriginZ():Float return HobberLayout.get().nativeHobCentreZ;
  /** The Z saddle plate's front face and back (block) face, x. */
  public static inline final SWIVEL_PLATE_FRONT = 135.0;
  public static inline final SWIVEL_PLATE_BACK = 148.0;
  /** The swivel plate's back face, where the adapter ring ends. */
  public static inline final SWIVEL_PLATE_X = 100.0;
  /** Adapter ring: outside and inside diameter, mm. The inside leaves the output flange more than half its footprint to bear on. */
  public static inline final SWIVEL_RING_OUTSIDE = 110.0;
  public static inline final SWIVEL_RING_INSIDE = 40.0;
  /**
   * Adapter-to-plate screws: four M6 on this pitch circle, in the ring's plate end, at 45 deg to the plate's axes. They have to stand in the strip |z| 24..40 of the
   * plate's front: the Y scale's housing lies on it along y at z 0, and the Y guide blocks (z 43 to 77, on the rails at +-60) run over everything beyond z 40.
   * Four M6 12.9 clamp the ring (70 % of proof, ~390 N m of friction at mu 0.15 on this radius), which holds the eRob's 204 N m peak but not its 369 N m momentary
   * torque with the 1.2 the slip check wants (1.05); the ring was first drawn with eight M5, whose heads fell under the scale and the guide blocks. The torque is
   * therefore taken by two dowels (`RING_PIN_*`) and the friction is the reserve (`JointSlip`: pins carry, friction not counted).
   */
  public static inline final SWIVEL_PLATE_PCD = 95.0;
  public static inline final SWIVEL_PLATE_SCREWS = 4;
  public static inline final SWIVEL_PLATE_PHASE:Float = Math.PI / 4;
  /**
   * The two dowels between the adapter ring and the swivel plate, on opposite sides of the axis: Ø6 (ISO 8734) on this radius, 15 deg off the output flange's
   * screw at 0 deg so the ring's sunk heads (Ø7.5 on r 38.5) keep 2.5 mm from them, each hole `RING_PIN_DEPTH` deep from the joint plane into the ring and into
   * the plate (blind: the plate's front, where the Y scale lies, stays whole). Two Ø6 pins bearing 6 mm in aluminium take ~740 N m (the check says).
   */
  public static inline final SWIVEL_RING_PIN_DIAMETER = 6.0;
  public static inline final SWIVEL_RING_PIN_RADIUS = 43.0;
  public static inline final SWIVEL_RING_PIN_PHASE:Float = Math.PI / 12;
  public static inline final SWIVEL_RING_PIN_DEPTH = 10.0;
  /** How far the output face stands in front of the saddle plate's front face (its bore is 2.2 mm wider in radius than the pilot). */
  public static function swivelRunningGap():Float return HobberZSaddle.SWIVEL_PLATE_FRONT - HobberLayout.get().swivelNativeX;
  /** Depth of the pocket from the plate's back face to the housing's front face. */
  public static function swivelPocketDepth():Float return HobberZSaddle.SWIVEL_PLATE_BACK - (HobberLayout.get().swivelNativeX - ErobActuatorPart.SEAT_Z);
  /** The pocket, output bore and screw clearance holes the Z saddle needs, in the saddle's frame (the work frame lowered by `SEAT_Z`). */
  public static function swivelHousingCutout():Part {
    var tool = new ErobActuatorPart().mountingCutout(HobberZSaddle.swivelPocketDepth(), HobberZSaddle.swivelRunningGap(), 0.4, 0, ErobActuatorPart.PILOT_BORE_CLEARANCE);
    var placed = PosedParts.moved(tool, F.fromRotationMatrix(HobberLayout.get().swivelNativeX, 0, 0, HobberLayout.get().swivelRotation));
    tool.close();
    return placed;
  }
  public static inline final PLATE_BLOCK_SOLE = 6.0;
  // ---- A axis and spindle seat (FRAME_PLAN R1, R2) ------------------------------------------------------------------------------
  public static function platePositions():Array<{y:Float, z:Float}> {
    var r = HobberZSaddle.SWIVEL_PLATE_PCD / 2, n = HobberZSaddle.SWIVEL_PLATE_SCREWS;
    return [for (i in 0...n) {y: r * Math.cos(2 * Math.PI * i / n + HobberZSaddle.SWIVEL_PLATE_PHASE), z: r * Math.sin(2 * Math.PI * i / n + HobberZSaddle.SWIVEL_PLATE_PHASE)}];
  }
  /** The Z saddle's eRob output bore (the housing's screws and the pocket are derived or cut from the actuator's mounting cut-out), in the saddle's frame. */
  public static function erobPilotBore():Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var web = HobberLayout.get().swivelNativeX - machinekit.motion.ErobActuatorPart.SEAT_Z - HobberZSaddle.SWIVEL_PLATE_FRONT;
    var d = machinekit.motion.ErobActuatorData.OUTPUT_PILOT_DIAMETER + machinekit.motion.ErobActuatorPart.PILOT_BORE_CLEARANCE;
    return [{id: "P1", kind: "bore", thread: null, diameter: d, nominal: d, fit: null, depth: web, position: [HobberZSaddle.SWIVEL_PLATE_FRONT, 0, 0], direction: [1.0, 0, 0]}];
  }
  /** The ring's end screws into the output flange are heads sunk this deep (the head is 4: flush with the plate end, 0.2 over). */
  public static inline final RING_HEAD_RECESS = 4.2;
  public function new(?values:ComponentValues) { super("GE-Z-saddle",values); finish(true); }
  override public function machinable():Bool return true;
  override public function pinnedFaces():Array<PinnedFace> return {
    [HobberDriveBar.nutBlockPins(HobberDrives.z(), [0.0, 0, zSaddleOriginZ()])];
  };
  override public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return {
    derivedRouteHoles().concat(erobPilotBore());
  };
  override public function pieces():Array<Part> return {
    // One milled plate between the swivel seat (x 135) and the block faces (x 148), with the nut block's bolts.
    // The eRob's pocket opens in the back face (x 148); the web in front of the housing carries its 16 screws and the output bore.
    [Solids.cut(HobberPartGeometry.box(135, 148, -175, 175, -65, 170), [HobberZSaddle.swivelHousingCutout()])];
  };
  override public function manufacturingRoute():Null<ManufacturingRoute> return {
    ManufacturingRoute.milled(2, "fine", [
      {name: "block faces, back", tolerance: 0.02, roughnessRa: 1.6},
      {name: "eRob seat: output bore, front face web and pocket", tolerance: 0.05, roughnessRa: 1.6}
    ], 350);
  };
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace>=[];
    faces=faces.concat(HobberLayout.get().zBellowsSaddleReceivers());
        var block = machinekit.motion.LinearRailBlock.metric("HGH15CA", machinekit.motion.LinearRailBlock.SUPPLIER);
        for (side in [-1, 1]) for (row in [0.0, HobberLayout.get().zBlockSpacing])
          faces.push(HobberPartGeometry.blockFace("block" + (side < 0 ? "L" : "R") + (row == 0 ? "1" : "2"), [148.0, side * 145.0, row], [-1.0, 0, 0], [0.0, 1, 0], block, 13, PLATE_BLOCK_SOLE));
        // Flush heads keep the fixed Z chip shield clear throughout saddle travel.
        faces.push(HobberDriveBar.nutBlockFace(HobberDrives.z(), 6.5, [0.0, 0, zSaddleOriginZ()], {diameter: 12.0, depth: 6.5}));
        // The eRob housing's 16 M4 into its front face, passed through the 5 mm web from the front (x 135): the plane is the housing's front face.
        faces.push(BoltedFace.clamped("erob", BoltedFace.frameOf({x: HobberLayout.get().swivelNativeX - machinekit.motion.ErobActuatorPart.SEAT_Z, y: 0.0, z: 0.0}, {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0}),
          [for (i in 0...machinekit.motion.ErobActuatorData.FIXED_BOLT_COUNT) {
            var b = machinekit.motion.ErobActuatorPart.fixedBolt(i);
            {x: b.x, y: b.y};
          }], machinekit.standard.SocketHeadCapScrew.catalog().get("M4").clearanceMedium,
          HobberLayout.get().swivelNativeX - machinekit.motion.ErobActuatorPart.SEAT_Z - HobberZSaddle.SWIVEL_PLATE_FRONT, "M4", null, false));
    return faces;
  }
}
