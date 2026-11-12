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

/**
 * GE-outboard-carrier: the milled frame on the Y saddle that holds the outboard bearing housing coaxial with the spindle.
 * Recorded in its own frame, the spindle axis at the hob centre: a ring round the housing seat and two arms back to the
 * saddle face, with datums at the saddle face and the housing seat.
 */
class HobberOutboardCarrier extends HobberMadePart {
  static final carrierRecipe = new machinekit.cache.Once<ComponentType>();

  public static function carrierType():ComponentType return carrierRecipe.get(() -> new ComponentType("machinekit.hobbing.outboard-carrier", [],
    _ -> new HobberOutboardCarrier(), false, false, null, false).versioned(1).numbered(_ -> "HOBBER-GE-OUTBOARD-CARRIER")
    .procured(_ -> Make("steel fabrication and machining")).requiring(_ -> route().requirements()));
  public static function outboardHousingMountFrame():AssemblyFrame return BoltedFace.frameOf({x: 30.0, y: -100.0, z: 0.0},
    {x: 1, y: 0, z: 0}, {x: 0, y: 1, z: 0});
  public static function outboardHousingHoles():Array<machinekit.component.BoltedFace.BoltHole> return [
    for (y in [-8.0, 8.0]) for (z in [-22.0, 22.0]) {x: y, y: z}
  ];
  public static function outboardHousingPins():PinnedFace return PinnedFace.atPoints("housing-pins", HobberOutboardCarrier.outboardHousingMountFrame(),
    [for (z in [-22.0, 22.0]) {x: 30.0, y: -100.0, z: z}], 4, 6, false);
  /** The swivel plate's thickness (ground plate), mm: the Y scale housing's M8 are tapped through it. */
  /** The wall of the outboard support's arm in front of the Y saddle's plate (x 30..45), mm. */
  public static inline final OUTBOARD_ARM_WALL = 15.0;
  public static function outboardCarrierFrame(into:Float):AssemblyFrame return BoltedFace.frameOf(
    {x: 45.0, y: 30.0, z: 0.0}, {x: into, y: 0, z: 0}, {x: 0, y: 1, z: 0});
  public static function outboardCarrierPins(into:Float):machinekit.component.BoltedFace.PinnedFace return PinnedFace.atPoints("outboard-pins", outboardCarrierFrame(into),
    [for (z in [-45.0, 45.0]) {x: 45.0, y: 30.0, z: z}], 4.0, 6.0, false);
  public function new() {
    super("GE-outboard-carrier", new ComponentValues());
    finish(false);
  }
  override public function componentType():Null<ComponentType> return carrierType();

  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    var ring = b.label(b.cutAll(b.boxBetween(-45, 45, -115, -85, -55, 55), [b.boxBetween(-30, 30, -115.1, -84.9, -30, 30)], ["housing window"]), "Ring");
    var upper = b.label(b.boxBetween(30, 45, -86, 45, 35, 55), "Upper arm");
    var lower = b.label(b.boxBetween(30, 45, -86, 45, -55, -35), "Lower arm");
    var carrier = b.label(b.unionAll([ring, upper, lower]), "Carrier");
    HobberRecording.datum(b, "saddle", outboardCarrierFrame(-1));
    HobberRecording.datum(b, "housing", outboardHousingMountFrame());
    return carrier;
  }
  override public function machinable():Bool return true;
  override public function pinnedFaces():Array<PinnedFace> return {
    [outboardCarrierPins(-1).at(recordedDatum("saddle")), outboardHousingPins().at(recordedDatum("housing"))];
  };
  override public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return {
    derivedRouteHoles();
  };
  override public function manufacturingRoute():Null<ManufacturingRoute> return route();

  static function route():ManufacturingRoute return {
    ManufacturingRoute.milled(2, "fine", [
      {name: "bearing housing seat, perpendicular to saddle face", tolerance: 0.01, roughnessRa: 1.6},
      {name: "carrier bolts and dowels (ream at alignment)", tolerance: 0.02, roughnessRa: 1.6}
    ], 110);
  };
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace>=[];
    faces.push(BoltedFace.drilledToSuit("saddle", recordedDatum("saddle"), 30, 110, 10, 15, BoltedFace.ThreadMaterial.Steel));
    faces.push(BoltedFace.clamped("housing", recordedDatum("housing"), outboardHousingHoles(), 4.5, 9.5, "M4", {diameter: 9.5, depth: 5.5}, false, true));
    return faces;
  }
}
