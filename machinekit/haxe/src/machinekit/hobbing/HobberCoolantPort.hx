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

/** GE-coolant; owns its stock, interfaces and manufacturing route. */
class HobberCoolantPort extends HobberMadePart {
  public function new(?values:ComponentValues) { super("GE-coolant",values); finish(false); }
  override public function machinable():Bool return false;
  override public function pieces():Array<Part> return {
    [Solids.cut(HobberPartGeometry.box(33, 45, 59.2, 69.5, 60, 84), [HobberPartGeometry.boreX(2, 32.9, 45.1, 64.35, 72)])];
  };
}
