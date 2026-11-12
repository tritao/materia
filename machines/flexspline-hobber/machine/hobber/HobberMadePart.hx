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

/** Shared identity and manufacturing lifecycle for the machine's made parts. Each concrete part has its own recipe;
 * geometry, fastener interfaces and routes belong to it. */
class HobberMadePart extends MachineComponent {
  public final designId:String;
  /** `values`: the values of the part's own recipe (its `componentType`). */
  public function new(design:String, values:ComponentValues) {
    super(
      "HOBBER-" + design,
      "Designed steel " + design + "; machined seats, assumed structural adequacy",
      "steel",
      false,
      values.copy()
    );
    addFacet(EngineeringMetadata.assumed(""));
    designId = design;
    addConnector("datum", Mount, F.identity());
  }
  /** Called after the concrete part has initialized its dimensions. */
  public function finish(collision:Bool):Void {
    var route=manufacturingRoute();
    if(route!=null) { addFacet(route); addFacet(route.intent()); }
    if(collision) {
      var parts=pieces();
      try addFacet(ExtrusionCollision.pieces(parts)) catch(e:Dynamic) {
        for(p in parts)p.close();
        throw e;
      }
      for(p in parts)p.close();
    }
  }
  override public function hasGeometry():Bool return true;
  override public function pinnedFaces():Array<PinnedFace> return [];
  override public function fasteningFaces():Array<BoltedFace> return [];
  public function manufacturingRoute():Null<ManufacturingRoute> return null;
  public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return [];
  /** The stock pieces (no derived holes), for the collision pieces: a recorded part's recorded body. */
  public function pieces():Array<Part> {
    if (!isRecorded()) throw "Concrete made part must declare its stock pieces";
    var self = this;
    var document = cadkit.parametric.recording.DocumentBuilder.build(b -> b.output(self.record(b, Preview)));
    return [PartRecordings.solid(document)];
  }
  /** A part that records its geometry (`record`) takes it from the recording; the others still build from `pieces()`. */
  override public function geometry(detail:ComponentDetail = Preview):Part
    return isRecorded() ? super.geometry(detail) : machined(Solids.union(pieces()));
}
