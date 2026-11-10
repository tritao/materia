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

/** GE-outboard-carrier; owns its stock, interfaces and manufacturing route. */
class HobberOutboardCarrier extends HobberMadePart {
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
  public function new(?values:ComponentValues) { super("GE-outboard-carrier",values); finish(false); }
  override public function machinable():Bool return true;
  override public function pinnedFaces():Array<PinnedFace> return {
    [outboardCarrierPins(-1), HobberOutboardCarrier.outboardHousingPins()];
  };
  override public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return {
    derivedRouteHoles();
  };
  override public function pieces():Array<Part> return {
    [Solids.union([Solids.cut(HobberPartGeometry.box(-45,45,-115,-85,-55,55),[HobberPartGeometry.box(-30,30,-115.1,-84.9,-30,30)]),HobberPartGeometry.box(30,45,-86,45,35,55),HobberPartGeometry.box(30,45,-86,45,-55,-35)])];
  };
  override public function manufacturingRoute():Null<ManufacturingRoute> return {
    ManufacturingRoute.milled(2, "fine", [
      {name: "bearing housing seat, perpendicular to saddle face", tolerance: 0.01, roughnessRa: 1.6},
      {name: "carrier bolts and dowels (ream at alignment)", tolerance: 0.02, roughnessRa: 1.6}
    ], 110);
  };
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace>=[];
    faces.push(BoltedFace.drilledToSuit("saddle", outboardCarrierFrame(-1), 30, 110, 10, 15, BoltedFace.ThreadMaterial.Steel));
        faces.push(BoltedFace.clamped("housing", HobberOutboardCarrier.outboardHousingMountFrame(), HobberOutboardCarrier.outboardHousingHoles(), 4.5, 9.5, "M4", {diameter: 9.5, depth: 5.5}, false, true));
    return faces;
  }
}
