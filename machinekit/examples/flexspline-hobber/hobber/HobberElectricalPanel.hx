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

/** electrical-panel; owns its stock, interfaces and manufacturing route. */
class HobberElectricalPanel extends HobberMadePart {
  public static inline final ELECTRICAL_HEIGHT = MineralCastBed.PLINTH_HEIGHT - machinekit.enclosure.TraySeat.DEPTH - 6;
  public function new(?values:ComponentValues) { super("electrical-panel",values); finish(false); }
  override public function machinable():Bool return false;
  override public function pieces():Array<Part> return {
    var parts = [HobberPartGeometry.box(-210, 210, -10, 0, 0, ELECTRICAL_HEIGHT), HobberPartGeometry.box(-230, 230, -150, 50, 0, 20)];
    // Two welded triangular webs tie the upright to its floor base.
    for (x in [-190.0, 190.0]) {
      var web = Part.prism([new Vector(0, 20), new Vector(-130, 20), new Vector(0, 200)], -3, 3);
      var placed = PosedParts.moved(web, F.fromRotationMatrix(x, 0, 0, [0, 0, 1, 1, 0, 0, 0, 1, 0]));
      web.close();
      parts.push(placed);
    }
    parts;
  };
}
