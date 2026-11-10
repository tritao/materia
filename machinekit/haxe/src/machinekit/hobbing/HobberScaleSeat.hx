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

/** GE-X-scale-seat; owns its stock, interfaces and manufacturing route. */
class HobberScaleSeat extends HobberMadePart {
  /** The seat bar under a scale housing as a box (it is authored in the work frame, so no origin shift). */
  public static function barBox(axis:HobberScaleAxis):Array<Float> {
    var b = axis.seatBar;
    if (b == null) throw "Scale " + axis.id + " has no seat bar";
    return [b[0], b[2], b[4], b[1], b[3], b[5]];
  }
  public function new(axis:String="X", ?values:ComponentValues) {
    super("GE-"+axis+"-scale-seat",values);
    if(axis!="X"&&axis!="Z")throw "Unknown scale seat";
    finish(true);
  }
  function scaleAxis():HobberScaleAxis return designId=="GE-X-scale-seat"?HobberScales.x():HobberScales.z();
  override public function machinable():Bool return false;
  override public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return {
    HobberScales.seatHoles(scaleAxis(), HobberScales.SEAT_BAR);
  };
  override public function pieces():Array<Part> return {
    [HobberPartGeometry.withHoles(HobberPartGeometry.padBox(barBox(scaleAxis())), HobberScales.seatHoles(scaleAxis(), HobberScales.SEAT_BAR))];
  };
  override public function manufacturingRoute():Null<ManufacturingRoute> return {
    // Ground bar: the housing's mounting face parallelism comes from the datasheet (0.1 over the length), the grinding gives 0.01.
    ManufacturingRoute.groundStock(GroundStock.bar(HobberScales.SEAT_BAR_WIDTH, HobberScales.SEAT_BAR),
      [{name: "housing seat, tapped holes (ground after cutting)", tolerance: HobberScales.x().housing.mounting().housingParallelism, roughnessRa: null}]);
  };
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace>=[];
    faces.push(HobberScales.seatFace(scaleAxis(), [0.0, 0, 0], HobberScales.SEAT_BAR));
    return faces;
  }
}
