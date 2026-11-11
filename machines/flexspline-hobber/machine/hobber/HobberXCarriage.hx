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
 * GE-X-carriage: the ground steel deck the column stands on, riding four HGH25 blocks. Recorded in its own frame (the deck's
 * underside, centred between the X rails and along the deck: `HobberSkeleton.xCarriageSeat`), so its history reads as its
 * steps and its interfaces are expressed in that frame. Owns its stock, interfaces and manufacturing route.
 */
class HobberXCarriage extends HobberMadePart {
  static final carriageRecipe = new machinekit.cache.Once<ComponentType>();

  /** Its own recipe: every dimension comes from the skeleton or from stock, so it has no free inputs. */
  public static function carriageType():ComponentType return carriageRecipe.get(() -> new ComponentType("machinekit.hobbing.x-carriage", [],
    _ -> new HobberXCarriage(), false, false, null, false).versioned(1).numbered(_ -> "HOBBER-GE-X-CARRIAGE")
    .procured(_ -> Make("steel fabrication and machining")).requiring(_ -> route().requirements()));

  /** The layout this carriage is made for. */
  public final skeleton:HobberSkeleton;
  /** One joint declaration also reserves its source drilling before cast-anchor
   * selection. Anchor hardware must never be selected through a service thread. */
  public static function columnSoleFace(pad:hobber.HobberSkeleton.ColumnSolePad):BoltedFace {
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
  public static function xGuideOriginZ():Float return HobberSkeleton.reference().heights().xGuideOriginZ - HobberFrame.PART_DATUM_Z;
  public static function xGuideSpec():machinekit.motion.LinearRailProfile.LinearRailProfileSpec {
    var spec=machinekit.motion.LinearRailSystem.catalog().get("HGH25CA");
    if (spec==null) throw "X guide specification unavailable";
    return spec;
  }
  public static function xCarriageBottom():Float return HobberSkeleton.reference().heights().xCarriageBottomZ - HobberFrame.PART_DATUM_Z;
  public static function xCarriageTop():Float return HobberSkeleton.reference().heights().xCarriageTopZ - HobberFrame.PART_DATUM_Z;
  /** Buy the carriage width outward from the outermost guide-block bolt and its
   * edge/head wall requirement, in 10 mm full-width stock increments. */
  public static function xCarriageHalfWidth():Float {
    var block = machinekit.motion.LinearRailBlock.metric("HGH25CA");
    var screw = machinekit.standard.SocketHeadCapScrew.catalog().get(block.spec.blockMountThread);
    var edge = machinekit.assembly.JointFasteners.requiredEdgeRadius(screw.diameter, screw.headDiameter + 2);
    return Math.ceil((HobberSkeleton.reference().bedRailCentre + block.spec.blockHoleSpacingAcross / 2 + edge) / 5) * 5;
  }
  public function new(?values:ComponentValues, ?skeleton:HobberSkeleton) {
    super("GE-X-carriage", values, true);
    this.skeleton = skeleton == null ? HobberSkeleton.reference() : skeleton;
    finish(true);
  }
  override public function componentType():Null<ComponentType> return carriageType();

  /** Part datum frame to this carriage's own frame. */
  function fromDatum():AssemblyFrame return F.inverse(skeleton.xCarriageSeat());

  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    var plate:Null<cadkit.parametric.Feature> = null;
    b.group("Stock", () -> {
      b.length("plate.length", skeleton.columnRear - skeleton.xDeckFront).derivedBy("X deck front to column rear (skeleton)");
      var half = b.length("plate.halfWidth", xCarriageHalfWidth())
        .derivedBy("rail centre + block bolt spacing / 2 + bolt edge distance, rounded up to 5 mm");
      var thickness = b.length("plate.thickness", X_CARRIAGE_THICKNESS).fixedBy("20 mm ground steel flat stock");
      var front = b.expression("plate.front", cadkit.parametric.ParameterKind.Length, "mm", "-plate.length / 2");
      var rear = b.expression("plate.rear", cadkit.parametric.ParameterKind.Length, "mm", "plate.length / 2");
      var right = b.expression("plate.right", cadkit.parametric.ParameterKind.Length, "mm", "-plate.halfWidth");
      plate = b.label(b.boxBetween(front, rear, right, half, 0, thickness), "Ground plate");
    });
    // Open at the front (it starts 5 mm ahead of the deck), 80 mm into the deck and 300 mm wide: clears the hob and work.
    var front = -(skeleton.columnRear - skeleton.xDeckFront) / 2;
    var window = b.boxBetween(front - 5, front + 80, -150, 150, -1, X_CARRIAGE_THICKNESS + 1);
    return b.label(b.cutAll(plate, [window], ["front window"]), "Front window");
  }
  override public function machinable():Bool return true;
  override public function pinnedFaces():Array<PinnedFace> return {
    var toOwn = fromDatum();
    var pins = [HobberDriveBar.nutBlockPins(HobberDrives.x(), [0.0, 0, 0])];
    for (role in HobberCarriageRib.ROLES) {
      var rib = new HobberCarriageRib(new ComponentValues().setToken("role", role));
      pins.push(new PinnedFace("rib-" + role, rib.interfaceFrame(1),
        [for (x in rib.pinXs()) {x: x - 263, y: 0.0}], 4, 8));
    }
    [for (pin in pins) pin.movedBy(toOwn)];
  };
  override public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return {
    derivedRouteHoles();
  };
  /** The recorded body without the derived holes, for the collision pieces; separate bolted ribs carry their own joints. */
  override public function pieces():Array<Part> return {
    var document = cadkit.parametric.recording.DocumentBuilder.build(b -> b.output(record(b, Preview)));
    [PartRecordings.solid(document)];
  };
  override public function manufacturingRoute():Null<ManufacturingRoute> return route();

  static function route():ManufacturingRoute return {
    ManufacturingRoute.groundStock(GroundStock.plate(GroundStock.STEEL, X_CARRIAGE_THICKNESS), [
      {name: "inspect/select ground stock or finish-grind four block lands in one setup to0.01 mm; no replicated pads", tolerance: 0.01, roughnessRa: 1.6},
      {name: "column sole seat and rib bond lands", tolerance: 0.02, roughnessRa: 1.6}
    ]);
  };
  /** The carriage's bolted faces, worked out in the part datum frame (the shared helpers' frame) and given in its own. */
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace>=[];
    var block = machinekit.motion.LinearRailBlock.metric("HGH25CA", machinekit.motion.LinearRailBlock.SUPPLIER);
        for (side in [-1, 1]) for (row in [skeleton.xBlockFront, skeleton.xBlockRear])
          faces.push(HobberPartGeometry.blockFace("block" + (side < 0 ? "L" : "R") + (row == skeleton.xBlockFront ? "1" : "2"), [row, side * skeleton.bedRailCentre, xCarriageBottom()],
            [0.0, 0, 1], [1.0, 0, 0], block, 20, X_BLOCK_SOLE));
        // Two derived M8 per ground corner pad, spread over the whole column base.
        for (pad in skeleton.columnSolePads()) faces.push(columnSoleFace(pad));
        faces.push(HobberDriveBar.nutBlockFace(HobberDrives.x(), X_CARRIAGE_THICKNESS, [0.0, 0, 0]));
        faces = faces.concat(HobberChipCover.carriageReceivers()).concat(skeleton.zBellowsDeckReceivers());
        // The X scale head's two M4 enter from the seat side (below, before the carriage is set on the rails) and thread into the rib's underside.
        for (role in HobberCarriageRib.ROLES) {
          var rib = new HobberCarriageRib(new ComponentValues().setToken("role", role));
          var b = rib.band();
          faces.push(BoltedFace.drilledToSuit("rib-" + role, rib.interfaceFrame(1),
            246, b.high - b.low, 15, 20, BoltedFace.ThreadMaterial.Steel, rib.boltSize()));
        }
    var toOwn = fromDatum();
    return [for (face in faces) face.movedBy(toOwn)];
  }
}
