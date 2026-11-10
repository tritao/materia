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

/** GE-Z-scale-arm; owns its stock, interfaces and manufacturing route. */
class HobberZScaleArm extends HobberMadePart {
  public function new(?values:ComponentValues) { super("GE-Z-scale-arm",values); finish(true); }
  override public function machinable():Bool return true;
  override public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return {
    derivedRouteHoles();
  };
  override public function pieces():Array<Part> return {
    // Milled arm from the Z saddle's back face (x 148) into the column channel, 12 thick across it; it carries the head. The housing stands beside the head with its free
    // face in the head's carrier plane, so the arm bridges over it a running gap above that plane and steps down to the plane only at the head (a pad as wide as the head's).
    var z = HobberScales.z(), face = HobberScales.headFace(z), hb = HobberScales.carrierBox(z, HobberScales.HEAD_CARRIER_MARGIN);
    var toward = -HobberScales.zWallSign(), gap = HobberScales.RUNNING_CLEARANCE, thick = HobberScales.Z_ARM_THICKNESS;
    function band(from:Float, to:Float):Array<Float> return toward > 0 ? [face[1] + from, face[1] + to] : [face[1] - to, face[1] - from];
    var bridge = band(gap, gap + thick), pad = band(0, gap + thick), zr = [hb[4] - HobberZSaddle.zSaddleOriginZ(), hb[5] - HobberZSaddle.zSaddleOriginZ()];
    [Solids.union([HobberPartGeometry.box(148, hb[1], bridge[0], bridge[1], zr[0], zr[1]), HobberPartGeometry.box(hb[0], hb[1], pad[0], pad[1], zr[0], zr[1])])];
  };
  override public function manufacturingRoute():Null<ManufacturingRoute> return {
    ManufacturingRoute.milled(1, "fine", [{name: "scale head seat, face", tolerance: HobberScales.z().head.mounting().headParallelism, roughnessRa: 1.6}], 230);
  };
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace>=[];
    // The Z scale head's two M4 thread into the arm (10 thick) from the seat side.
        faces.push(HobberScales.carrierFace(HobberScales.z(), [0.0, 0, HobberZSaddle.zSaddleOriginZ()], HobberScales.Z_ARM_THICKNESS - 1, HobberScales.Z_ARM_THICKNESS + HobberScales.RUNNING_CLEARANCE));
    return faces;
  }
}
