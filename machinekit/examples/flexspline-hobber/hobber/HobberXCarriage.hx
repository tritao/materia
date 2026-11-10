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

/** GE-X-carriage; owns its stock, interfaces and manufacturing route. */
class HobberXCarriage extends HobberMadePart {
  /** One joint declaration also reserves its source drilling before cast-anchor
   * selection. Anchor hardware must never be selected through a service thread. */
  public static function columnSoleFace(pad:hobber.HobberLayout.ColumnSolePad):BoltedFace {
    var cx=(pad.x0+pad.x1)/2;
    var cy=(pad.y0+pad.y1)/2;
    return BoltedFace.clamped("column-sole-"+pad.id,
      BoltedFace.frameOf({x:cx,y:cy,z:xCarriageTop()},{x:0,y:0,z:-1},{x:1,y:0,z:0}),
      [for (x in pad.boltXs) {x:x-cx,y:cy-pad.boltY}],
      9.0,11.0,"M8",{diameter:15.0,depth:9.0},false);
  }
  /** Material under the heads of the guide-block screws (the pocket is cut from the far face), mm: M6 into a 20 mm deck, M4 into the 13 and 12 mm plates. */
  public static inline final X_BLOCK_SOLE = 10.0;
  public static inline final X_CARRIAGE_THICKNESS = 20.0;
  /** Expanded deck keepout, retaining the open front window. */
  public static function xDeckKeepout(gap:Float,drop:Float):Part return Solids.cut(
    HobberPartGeometry.box(60-gap,386+gap,-xCarriageHalfWidth()-gap,xCarriageHalfWidth()+gap,xCarriageBottom()-gap-drop,xCarriageTop()+gap),
    [HobberPartGeometry.box(55+gap,140-gap,-150+gap,150-gap,xCarriageBottom()-gap-drop-1,xCarriageTop()+gap+1)]);
  public static function xGuideOriginZ():Float return HobberLayout.get().heights().xGuideOriginZ - HobberFrame.PART_DATUM_Z;
  public static function xGuideSpec():machinekit.motion.LinearRailProfile.LinearRailProfileSpec {
    var spec=machinekit.motion.LinearRailSystem.catalog().get("HGH25CA");
    if (spec==null) throw "X guide specification unavailable";
    return spec;
  }
  public static function xCarriageBottom():Float return HobberLayout.get().heights().xCarriageBottomZ - HobberFrame.PART_DATUM_Z;
  public static function xCarriageTop():Float return HobberLayout.get().heights().xCarriageTopZ - HobberFrame.PART_DATUM_Z;
  /** Buy the carriage width outward from the outermost guide-block bolt and its
   * edge/head wall requirement, in 10 mm full-width stock increments. */
  public static function xCarriageHalfWidth():Float {
    var block = machinekit.motion.LinearRailBlock.metric("HGH25CA");
    var screw = machinekit.standard.SocketHeadCapScrew.catalog().get(block.spec.blockMountThread);
    var edge = machinekit.assembly.JointFasteners.requiredEdgeRadius(screw.diameter, screw.headDiameter + 2);
    return Math.ceil((HobberLayout.get().bedRailCentre + block.spec.blockHoleSpacingAcross / 2 + edge) / 5) * 5;
  }
  public function new(?values:ComponentValues) { super("GE-X-carriage",values); finish(true); }
  override public function machinable():Bool return true;
  override public function pinnedFaces():Array<PinnedFace> return {
    var pins = [HobberDriveBar.nutBlockPins(HobberDrives.x(), [0.0, 0, 0])];
    for (role in HobberCarriageRib.ROLES) {
      var rib = new HobberCarriageRib(new ComponentValues().setToken("role", role));
      pins.push(new PinnedFace("rib-" + role, rib.interfaceFrame(1),
        [for (x in rib.pinXs()) {x: x - 263, y: 0.0}], 4, 8));
    }
    pins;
  };
  override public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return {
    derivedRouteHoles();
  };
  override public function pieces():Array<Part> return {
    // Ground plate; separate bolted ribs carry their own joints and scale seat.
    var cuts = [HobberPartGeometry.box(55, 140, -150, 150, 259, 281)];
    [Solids.cut(HobberPartGeometry.box(60, 386, -xCarriageHalfWidth(), xCarriageHalfWidth(), xCarriageBottom(), xCarriageTop()), cuts)];
  };
  override public function manufacturingRoute():Null<ManufacturingRoute> return {
    ManufacturingRoute.groundStock(GroundStock.plate(GroundStock.STEEL, X_CARRIAGE_THICKNESS), [
      {name: "inspect/select ground stock or finish-grind four block lands in one setup to0.01 mm; no replicated pads", tolerance: 0.01, roughnessRa: 1.6},
      {name: "column sole seat and rib bond lands", tolerance: 0.02, roughnessRa: 1.6}
    ]);
  };
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace>=[];
    var block = machinekit.motion.LinearRailBlock.metric("HGH25CA", machinekit.motion.LinearRailBlock.SUPPLIER);
        for (side in [-1, 1]) for (row in [HobberLayout.get().xBlockFront, HobberLayout.get().xBlockRear])
          faces.push(HobberPartGeometry.blockFace("block" + (side < 0 ? "L" : "R") + (row == HobberLayout.get().xBlockFront ? "1" : "2"), [row, side * HobberLayout.get().bedRailCentre, xCarriageBottom()],
            [0.0, 0, 1], [1.0, 0, 0], block, 20, X_BLOCK_SOLE));
        // Two derived M8 per ground corner pad, spread over the whole column base.
        for (pad in HobberLayout.get().columnSolePads()) faces.push(columnSoleFace(pad));
        faces.push(HobberDriveBar.nutBlockFace(HobberDrives.x(), X_CARRIAGE_THICKNESS, [0.0, 0, 0]));
        faces = faces.concat(HobberChipCover.carriageReceivers()).concat(HobberLayout.get().zBellowsDeckReceivers());
        // The X scale head's two M4 enter from the seat side (below, before the carriage is set on the rails) and thread into the rib's underside.
        for (role in HobberCarriageRib.ROLES) {
          var rib = new HobberCarriageRib(new ComponentValues().setToken("role", role));
          var b = rib.band();
          faces.push(BoltedFace.drilledToSuit("rib-" + role, rib.interfaceFrame(1),
            246, b.high - b.low, 15, 20, BoltedFace.ThreadMaterial.Steel, rib.boltSize()));
        }
    return faces;
  }
}
