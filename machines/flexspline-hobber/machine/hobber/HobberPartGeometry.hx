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
 * The guide-block land a made part offers a linear-rail block: the bolted face `JointFasteners` derives the block's screws,
 * holes and counterbores from. Parts now record their shapes (`MachineComponent.record`); only these interfaces stay shared.
 */
class HobberPartGeometry {
  /** The face a guide block is bolted to, at `frame` (origin on the land, +Z into the part, +X along the rail): clampedToSuit, the pocket from the far side leaving `sole` under the heads. */
  public static function blockFaceAt(id:String, frame:AssemblyFrame, block:machinekit.motion.LinearRailBlock, thickness:Float, sole:Float):BoltedFace {
    var screw = machinekit.standard.SocketHeadCapScrew.catalog().get(block.spec.blockMountThread);
    var side = Math.max(block.spec.blockHoleSpacingAcross, block.spec.blockHoleSpacingAlong) + 4 * screw.diameter;
    return BoltedFace.clampedToSuit(id, frame, side, side, screw.clearanceMedium, sole, block.spec.blockMountThread,
      {diameter: screw.headDiameter + 2, depth: thickness - sole});
  }
}
