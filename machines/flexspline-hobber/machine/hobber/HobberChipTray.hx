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

/** GE-chip-tray; owns its stock, interfaces and manufacturing route. */
class HobberChipTray extends HobberMadePart {
  public function new(?values:ComponentValues) { super("GE-chip-tray",values); finish(false); }
  override public function machinable():Bool return false;
  override public function pieces():Array<Part> return {
    // Removable stationary skirt: 5 mm radial face clearance, 15 mm housing
    // clearance and 2 mm sheet. The lower skirt covers the full table height.
    var lip = parameterNumber("tableFaceDiameter") / 2 + 5;
    var skirt = Math.max(parameterNumber("tableHousingDiameter") / 2 + 15, lip + 5);
    var top = parameterNumber("tableFaceHeight") - 3, bottom = parameterNumber("tableSeatHeight");
    var shoulder = Math.max(bottom + 5, top - 61);
    var front = MineralCastBed.WORK_WELL_CENTRE_X
      - MineralCastBed.WORK_WELL_WIDTH / 2 + MineralCastBed.CHIP_TRAY_WELL_CLEARANCE_MM;
    [
      Part.revolve([
        {r: lip + 2, z: top},
        {r: skirt + 2, z: shoulder},
        {r: skirt + 2, z: bottom},
        {r: skirt, z: bottom},
        {r: skirt, z: shoulder},
        {r: lip, z: top - 2}
      ]),
      Solids.cut(
        Solids.union([
          HobberPartGeometry.box(front, 90, -170, 170, 135, 139),
          HobberPartGeometry.box(front, 90, -174, -170, 135, 164),
          HobberPartGeometry.box(front, 90, 170, 174, 135, 164)
        ]),
        [HobberPartGeometry.box(-skirt, skirt, -skirt, skirt, 134, 165)]
      )
    ];
  };
}
