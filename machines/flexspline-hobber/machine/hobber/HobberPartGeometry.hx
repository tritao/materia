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

/** Shared primitive and drilling operations used by made parts. */
class HobberPartGeometry {
  public static function box(x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float):Part {
    var raw = Part.box(x1 - x0, y1 - y0, z1 - z0), p = raw.translated(new Vector((x0 + x1) / 2, (y0 + y1) / 2, z0));
    raw.close();
    return p;
  }
  public static function boreX(radius:Float, x0:Float, x1:Float, y:Float, z:Float):Part {
    var p = Part.cylinderSpan(radius, x0, x1);
    var q = machinekit.assembly.PosedParts.moved(
      p,
      machinekit.assembly.AxisBuilder.orient(0, y, z, [0.0, 0, 1], [1.0, 0, 0])
    );
    p.close();
    return q;
  }
  public static function boreY(radius:Float, y0:Float, y1:Float, x:Float, z:Float):Part return Part.cylinderAlongY(
    radius,
    y0,
    y1,
    x,
    z
  );
  /** Milled 40 x 40 coupling service window with R3 internal corners. */
  public static function couplingWindow(x:Float, z0:Float, z1:Float):Part {
    var cuts = [box(x - 17, x + 17, -20, 20, z0, z1), box(x - 20, x + 20, -17, 17, z0, z1)];
    for (sx in [-1, 1]) for (sy in [-1, 1]) cuts.push(Part.cylinderSpan(3, z0, z1, x + sx * 17, sy * 17));
    return Solids.union(cuts);
  }
  public static function radialPlate(x0:Float, x1:Float, section:Array<{y:Float, z:Float}>):Part {
    var raw = Part.prism([for (p in section) new Vector(p.y, p.z)], x0, x1);
    var shaped = machinekit.assembly.PosedParts.moved(raw,
      F.fromRotationMatrix(0, 0, 0, [0.0, 0, 1, 1, 0, 0, 0, 1, 0]));
    raw.close();
    return shaped;
  }
  /** A hole's tool: a cylinder of `diameter` from `position` along `direction`, `depth` long, starting 1 mm outside the mouth. */
  public static function holeTool(position:Array<Float>, direction:Array<Float>, diameter:Float, depth:Float):Part {
    var cylinder = Part.cylinderSpan(diameter / 2, -1, depth);
    var up = Math.abs(direction[2]) < 0.9 ? [0.0, 0, 1] : [1.0, 0, 0];
    var placed = PosedParts.moved(cylinder, machinekit.assembly.AxisBuilder.orient(position[0], position[1], position[2], up, direction));
    cylinder.close();
    return placed;
  }
  /** Where the Z saddle's own frame sits in the work frame (z), mm: `FlexsplineHobber` places it at z 363. The drive's holes are in the work frame. */
  /** Depth of the head screws' tapped holes in the rib / saddle plate, mm (M4: 1.5 diameters in steel, with the thread's run-out). */
  public static inline final X_HEAD_HOLE_DEPTH = 8.0;
  public static inline final Y_HEAD_HOLE_DEPTH = 8.0;
  /** The scale holes `list` (work frame) in the frame of a part whose origin is at z `originZ` in the work frame. */
  public static function shifted(list:Array<machinekit.component.ManufacturingRoute.RouteHole>, originZ:Float):Array<machinekit.component.ManufacturingRoute.RouteHole> {
    return [for (h in list) ({
      id: h.id, kind: h.kind, thread: h.thread, diameter: h.diameter, nominal: h.nominal, fit: h.fit, depth: h.depth,
      position: [h.position[0], h.position[1], h.position[2] - originZ], direction: h.direction
    } : machinekit.component.ManufacturingRoute.RouteHole)];
  }
  /** `body` with the holes `list` cut (each a tool of its cut diameter from 1 mm outside the mouth to its depth). */
  public static function withHoles(body:Part, list:Array<machinekit.component.ManufacturingRoute.RouteHole>):Part {
    return Solids.cut(body, [for (h in list) holeTool(h.position, h.direction, h.diameter, h.depth)]);
  }
  public static function padBox(b:Array<Float>):Part return box(b[0], b[3], b[1], b[4], b[2], b[5]);

  // ---- Fastening faces (BOM_PLAN F1): the joints whose screws, holes and clearances `deriveFasteners` derives ------------------
  /** The face a guide block is bolted to: clampedToSuit with the pocket from the far side leaving `sole` under the heads. */
  public static function blockFace(id:String, origin:Array<Float>, into:Array<Float>, across:Array<Float>, block:machinekit.motion.LinearRailBlock, thickness:Float, sole:Float):BoltedFace
    return blockFaceAt(id, BoltedFace.frameOf({x: origin[0], y: origin[1], z: origin[2]}, {x: into[0], y: into[1], z: into[2]},
      {x: across[0], y: across[1], z: across[2]}), block, thickness, sole);

  /** The same face at `frame` (origin on the land, +Z into the part, +X along the rail). */
  public static function blockFaceAt(id:String, frame:AssemblyFrame, block:machinekit.motion.LinearRailBlock, thickness:Float, sole:Float):BoltedFace {
    var screw = machinekit.standard.SocketHeadCapScrew.catalog().get(block.spec.blockMountThread);
    var side = Math.max(block.spec.blockHoleSpacingAcross, block.spec.blockHoleSpacingAlong) + 4 * screw.diameter;
    return BoltedFace.clampedToSuit(id, frame, side, side, screw.clearanceMedium, sole, block.spec.blockMountThread,
      {diameter: screw.headDiameter + 2, depth: thickness - sole});
  }
}
