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

/** GE-Z-scale-arm; owns its stock, interfaces and manufacturing route. */
/**
 * The milled arm that carries the Z scale head off the Z saddle's back face: a bridge over the scale housing, a running gap
 * above its carrier plane, stepping down to a pad under the head. Recorded in its own frame, the head's seat (`frameFor`).
 */
class HobberZScaleArm extends HobberMadePart {
  static final armRecipe = new machinekit.cache.Once<ComponentType>();

  /** Its own recipe: the arm follows from the scale layout, so it has no inputs. */
  public static function armType():ComponentType return armRecipe.get(() -> new ComponentType("machinekit.hobbing.z-scale-arm", [],
    _ -> new HobberZScaleArm(), false, false, null, false).versioned(1).numbered(_ -> "HOBBER-GE-Z-SCALE-ARM")
    .procured(_ -> Make("steel fabrication and machining")).requiring(_ -> route().requirements()));

  /** The head's seat face in the Z saddle's native frame (the frame the arm used to be authored in). */
  static function headSeat():BoltedFace return HobberScales.carrierFace(HobberScales.z(), [0.0, 0, HobberZSaddle.zSaddleOriginZ()],
    HobberScales.Z_ARM_THICKNESS - 1, HobberScales.Z_ARM_THICKNESS + HobberScales.RUNNING_CLEARANCE);

  /** The arm's own frame in the Z saddle's native frame: the head's seat, +Z into the arm. */
  public static function frameFor():AssemblyFrame return headSeat().frame;

  public function new(?values:ComponentValues) {
    super("GE-Z-scale-arm", values, new ComponentValues());
    finish(true);
  }
  override public function componentType():Null<ComponentType> return armType();

  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    // From the Z saddle's back face (x 148) into the column channel; the housing stands beside the head with its free face in
    // the head's carrier plane, so the arm bridges over it a running gap above that plane and steps down to it only at the head.
    var z = HobberScales.z(), face = HobberScales.headFace(z), hb = HobberScales.carrierBox(z, HobberScales.HEAD_CARRIER_MARGIN);
    var toward = -HobberScales.zWallSign(), toOwn = F.inverse(frameFor());
    var gap = b.length("arm.gap", HobberScales.RUNNING_CLEARANCE).fixedBy("scale running clearance (HobberScales)").value;
    var thick = b.length("arm.thickness", HobberScales.Z_ARM_THICKNESS).fixedBy("arm section the scale layout gives (HobberScales)").value;
    function band(from:Float, to:Float):Array<Float> return toward > 0 ? [face[1] + from, face[1] + to] : [face[1] - to, face[1] - from];
    var bridge = band(gap, gap + thick), pad = band(0, gap + thick), zr = [hb[4] - HobberZSaddle.zSaddleOriginZ(), hb[5] - HobberZSaddle.zSaddleOriginZ()];
    var over = b.label(HobberRecording.box(b, toOwn, 148, hb[1], bridge[0], bridge[1], zr[0], zr[1]), "Bridge");
    var under = b.label(HobberRecording.box(b, toOwn, hb[0], hb[1], pad[0], pad[1], zr[0], zr[1]), "Head pad");
    var arm = b.label(b.unionAll([over, under]), "Arm");
    HobberRecording.datum(b, "head", F.identity());
    return arm;
  }
  override public function machinable():Bool return true;
  override public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return {
    derivedRouteHoles();
  };
  override public function manufacturingRoute():Null<ManufacturingRoute> return route();

  static function route():ManufacturingRoute return {
    ManufacturingRoute.milled(1, "fine", [{name: "scale head seat, face", tolerance: HobberScales.z().head.mounting().headParallelism, roughnessRa: 1.6}], 230);
  };
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace>=[];
    // The Z scale head's two M4 thread into the arm (10 thick) from the seat side.
    faces.push(headSeat().at(recordedDatum("head")));
    return faces;
  }
}
