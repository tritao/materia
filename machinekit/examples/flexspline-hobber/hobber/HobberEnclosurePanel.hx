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

/** GE-enclosure; owns its stock, interfaces and manufacturing route. */
class HobberEnclosurePanel extends HobberMadePart {
  public function new(?values:ComponentValues) { super("GE-enclosure",values); finish(false); }
  override public function machinable():Bool return false;
  override public function pieces():Array<Part> return {
    [HobberPartGeometry.box(-270, 780, -360, -356, 0, 950), HobberPartGeometry.box(-270, 780, 356, 360, 0, 950), HobberPartGeometry.box(776, 780, -360, 360, 0, 950)];
  };
}
