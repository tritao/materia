package machinekit.hobbing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.ManufacturingRoute.MachinedFace;
import machinekit.structural.GroundStock;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Bolted structural members and adjustment interfaces with machined datums.
 * Dimensions are mm; construction is unwelded. Strength and modal stiffness
 * estimates retain their stated assumptions pending Phase D. */
class HobberSaddle extends MachineComponent {
  final designId:String;
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return [
    P.choice(
      "design",
      [
        "GE-X-carriage",
        "GE-Z-saddle",
        "GE-X-drive-pad",
        "GE-Z-drive-pad",
        "GE-Y-drive-fixed",
        "GE-Y-drive-floating",
        "GE-Y-nut-seat",
        "GE-X-scale-seat",
        "GE-Z-scale-seat",
        "GE-Z-scale-arm",
        "GE-chip-tray",
        "GE-enclosure",
        "GE-coolant",
        "GE-swivel-plate",
        "GE-swivel-adapter",
        "GE-spindle-seat",
        "GE-motor-cradle",
        "GE-Y-saddle",
        "GE-outboard-carrier",
        "electrical-panel"
      ],
      "GE-Z-saddle"
    ),
    P.quantity("tableFaceDiameter", 200, "mm"),
    P.quantity("tableHousingDiameter", 260, "mm"),
    P.quantity("tableFaceHeight", 228, "mm"),
    P.quantity("tableSeatHeight", 18, "mm")
  ];
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.hobbing.saddle",
      parameters(),
      v -> new HobberSaddle(v.token("design"), v)
    ).versioned(2).numbered(v -> 'HOBBER-${v.token("design").toUpperCase()}')
      .procured(_ -> Make("steel fabrication and machining"))
      .requiringInputs(["design", "tableFaceDiameter", "tableHousingDiameter", "tableFaceHeight", "tableSeatHeight"]));
  }
  public function new(design:String, ?values:ComponentValues) {
    var supplied = values == null ? new ComponentValues() : values.copy();
    supplied.setToken("design", design);
    var v = P.resolve(parameters(), supplied);
    super(
      "HOBBER-" + design,
      "Designed steel " + design + "; machined seats, assumed structural adequacy",
      "steel",
      false,
      v
    );
    addFacet(new EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", "")));
    designId = design;
    // The adapter ring turns with the head: light alloy.
    if (design == "GE-swivel-adapter") setMaterial("aluminium 6061");
    addConnector("datum", Mount, F.identity());
    var route = routeOf(design);
    if (route != null) {
      addFacet(route);
      addFacet(route.intent());
    }
    if (["GE-X-carriage", "GE-Z-saddle", "GE-X-drive-pad", "GE-Z-drive-pad",
      "GE-Y-saddle", "GE-motor-cradle", "GE-Y-drive-fixed", "GE-Y-drive-floating", "GE-Y-nut-seat", "GE-X-scale-seat", "GE-Z-scale-seat", "GE-Z-scale-arm"].indexOf(design) >= 0) {
      var parts = pieces();
      try {
        addFacet(ExtrusionCollision.pieces(parts));
      } catch (e:Dynamic) {
        for (p in parts) p.close();
        throw e;
      }
      for (p in parts) p.close();
    }
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  static function box(x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float):Part {
    var raw = Part.box(x1 - x0, y1 - y0, z1 - z0), p = raw.translated(new Vector((x0 + x1) / 2, (y0 + y1) / 2, z0));
    raw.close();
    return p;
  }
  static function boreX(radius:Float, x0:Float, x1:Float, y:Float, z:Float):Part {
    var p = Part.cylinderSpan(radius, x0, x1);
    var q = machinekit.assembly.PosedParts.moved(
      p,
      machinekit.assembly.AxisBuilder.orient(0, y, z, [0.0, 0, 1], [1.0, 0, 0])
    );
    p.close();
    return q;
  }
  static function boreY(radius:Float, y0:Float, y1:Float, x:Float, z:Float):Part return Part.cylinderAlongY(
    radius,
    y0,
    y1,
    x,
    z
  );
  /** Milled 40 x 40 coupling service window with R3 internal corners. */
  static function couplingWindow(x:Float, z0:Float, z1:Float):Part {
    var cuts = [box(x - 17, x + 17, -20, 20, z0, z1), box(x - 20, x + 20, -17, 17, z0, z1)];
    for (sx in [-1, 1]) for (sy in [-1, 1]) cuts.push(Part.cylinderSpan(3, z0, z1, x + sx * 17, sy * 17));
    return Solids.union(cuts);
  }

  static function radialPlate(x0:Float, x1:Float, section:Array<{y:Float, z:Float}>):Part {
    var raw = Part.prism([for (p in section) new Vector(p.y, p.z)], x0, x1);
    var shaped = machinekit.assembly.PosedParts.moved(raw,
      F.fromRotationMatrix(0, 0, 0, [0.0, 0, 1, 1, 0, 0, 0, 1, 0]));
    raw.close();
    return shaped;
  }
  /** Width of the ground flat bar a drive pad is sawn from. */
  static inline final PAD_WIDTH = HobberDrives.PAD_WIDTH;

  /** A hole's tool: a cylinder of `diameter` from `position` along `direction`, `depth` long, starting 1 mm outside the mouth. */
  static function holeTool(position:Array<Float>, direction:Array<Float>, diameter:Float, depth:Float):Part {
    var cylinder = Part.cylinderSpan(diameter / 2, -1, depth);
    var up = Math.abs(direction[2]) < 0.9 ? [0.0, 0, 1] : [1.0, 0, 0];
    var placed = PosedParts.moved(cylinder, machinekit.assembly.AxisBuilder.orient(position[0], position[1], position[2], up, direction));
    cylinder.close();
    return placed;
  }

  /** Where the Z saddle's own frame sits in the work frame (z), mm: `FlexsplineHobber` places it at z 363. The drive's holes are in the work frame. */
  static inline final Z_SADDLE_ORIGIN_Z = HobberZLayout.SEAT_Z;
  /** Depth of the head screws' tapped holes in the rib / saddle plate, mm (M4: 1.5 diameters in steel, with the thread's run-out). */
  static inline final X_HEAD_HOLE_DEPTH = 8.0;
  static inline final Y_HEAD_HOLE_DEPTH = 8.0;
  /** The swivel plate's thickness (ground plate), mm: the Y scale housing's M8 are tapped through it. */
  static inline final SWIVEL_PLATE_THICKNESS = 15.0;
  /** The wall of the outboard support's arm in front of the Y saddle's plate (x 30..45), mm. */
  static inline final OUTBOARD_ARM_WALL = 15.0;

  /** The scale holes `list` (work frame) in the frame of a part whose origin is at z `originZ` in the work frame. */
  static function shifted(list:Array<machinekit.component.ManufacturingRoute.RouteHole>, originZ:Float):Array<machinekit.component.ManufacturingRoute.RouteHole> {
    return [for (h in list) ({
      id: h.id, kind: h.kind, thread: h.thread, diameter: h.diameter, nominal: h.nominal, fit: h.fit, depth: h.depth,
      position: [h.position[0], h.position[1], h.position[2] - originZ], direction: h.direction
    } : machinekit.component.ManufacturingRoute.RouteHole)];
  }

  /** `body` with the holes `list` cut (each a tool of its cut diameter from 1 mm outside the mouth to its depth). */
  static function withHoles(body:Part, list:Array<machinekit.component.ManufacturingRoute.RouteHole>):Part {
    return Solids.cut(body, [for (h in list) holeTool(h.position, h.direction, h.diameter, h.depth)]);
  }

  /** The seat bar under a scale housing as a box (it is authored in the work frame, so no origin shift). */
  static function barBox(axis:HobberScaleAxis):Array<Float> {
    var b = axis.seatBar;
    if (b == null) throw "Scale " + axis.id + " has no seat bar";
    return [b[0], b[2], b[4], b[1], b[3], b[5]];
  }

  static function padBox(b:Array<Float>):Part return box(b[0], b[3], b[1], b[4], b[2], b[5]);

  // ---- Fastening faces (BOM_PLAN F1): the joints whose screws, holes and clearances `deriveFasteners` derives ------------------

  /** The mating plane of the parts bolted on a pad or bracket: the faces of `axis`'s `support` seat, drilled and tapped to suit what stands on it. */
  static function seatFace(axis:HobberDriveAxis, support:String, thickness:Float, ?clip:Array<Float>):BoltedFace {
    var b = clip ?? HobberDrives.padBounds(axis, support, PAD_WIDTH, thickness), n = axis.axis.normal, a = axis.axis.direction, ni = 0, ai = 0;
    for (i in 0...3) {
      if (Math.abs(n[i]) > 0.5) ni = i;
      if (Math.abs(a[i]) > 0.5) ai = i;
    }
    var ci = 3 - ni - ai, origin = [0.0, 0, 0], into = [0.0, 0, 0], across = [0.0, 0, 0];
    origin[ni] = n[ni] > 0 ? b[ni + 3] : b[ni];
    origin[ai] = (b[ai] + b[ai + 3]) / 2;
    origin[ci] = (b[ci] + b[ci + 3]) / 2;
    into[ni] = n[ni] > 0 ? -1 : 1;
    across[ai] = 1;
    return BoltedFace.drilledToSuit("seat-" + support, BoltedFace.frameOf({x: origin[0], y: origin[1], z: origin[2]}, {x: into[0], y: into[1], z: into[2]},
      {x: across[0], y: across[1], z: across[2]}), b[ai + 3] - b[ai], b[ci + 3] - b[ci], Math.min(thickness, 20.0), thickness, BoltedFace.ThreadMaterial.Steel,
      null, thickness <= 15);
  }

  /** The face a guide block is bolted to: clampedToSuit with the pocket from the far side leaving `sole` under the heads. */
  static function blockFace(id:String, origin:Array<Float>, into:Array<Float>, across:Array<Float>, block:machinekit.motion.LinearRailBlock, thickness:Float, sole:Float):BoltedFace {
    var screw = machinekit.standard.SocketHeadCapScrew.catalog().get(block.spec.blockMountThread);
    var side = Math.max(block.spec.blockHoleSpacingAcross, block.spec.blockHoleSpacingAlong) + 4 * screw.diameter;
    return BoltedFace.clampedToSuit(id, BoltedFace.frameOf({x: origin[0], y: origin[1], z: origin[2]}, {x: into[0], y: into[1], z: into[2]},
      {x: across[0], y: across[1], z: across[2]}), side, side, screw.clearanceMedium, sole, block.spec.blockMountThread,
      {diameter: screw.headDiameter + 2, depth: thickness - sole});
  }

  /** One joint declaration also reserves its source drilling before cast-anchor
   * selection. Anchor hardware must never be selected through a service thread. */
  public static function columnSoleFace(pad:machinekit.hobbing.FoundationLayout.ColumnSolePad):BoltedFace {
    var cx=(pad.x0+pad.x1)/2;
    var cy=(pad.y0+pad.y1)/2;
    return BoltedFace.clamped("column-sole-"+pad.id,
      BoltedFace.frameOf({x:cx,y:cy,z:xCarriageTop()},{x:0,y:0,z:-1},{x:1,y:0,z:0}),
      [for (x in pad.boltXs) {x:x-cx,y:cy-pad.boltY}],
      9.0,11.0,"M8",{diameter:15.0,depth:9.0},false);
  }

  /** The nut block's base end on the carriage, saddle or seat that carries it: two clearance holes for the block's tapped base, `grip` of material, in this part's frame (`origin` is the part frame's origin in the work frame). */
  static function nutBlockFace(axis:HobberDriveAxis, grip:Float, origin:Array<Float>, ?recess:machinekit.component.BoltedFace.BoltRecess):BoltedFace {
    var block = HobberDrives.nutBlock(axis, new machinekit.motion.BallNut());
    var screw = machinekit.standard.SocketHeadCapScrew.catalog().get(machinekit.motion.NutBlock.BASE_THREAD);
    return BoltedFace.clamped("nut-block", nutBlockFrame(axis, origin), [for (u in block.baseHoleXs()) {x: u, y: 0.0}], screw.clearanceMedium, grip,
      machinekit.motion.NutBlock.BASE_THREAD, recess, false);
  }

  /** The face frame of the nut block's base end on its carrier (this part's frame; `origin` is the part frame's origin in the work frame). */
  static function nutBlockFrame(axis:HobberDriveAxis, origin:Array<Float>):materia.assembly.AssemblyRecord.AssemblyFrame {
    var block = HobberDrives.nutBlock(axis, new machinekit.motion.BallNut()), pose = HobberDrives.nutBlockPose(axis, new machinekit.motion.BallNut());
    var at = F.transformPoint(pose, 0, -block.centreHeight, block.thickness / 2), out = F.transformVector(pose, 0, -1, 0), x = F.transformVector(pose, 1, 0, 0);
    return BoltedFace.frameOf({x: at.x - origin[0], y: at.y - origin[1], z: at.z - origin[2]}, {x: out.x, y: out.y, z: out.z}, {x: x.x, y: x.y, z: x.z});
  }

  /** The dowels of a pinned nut block in its carrier (X deck, Z saddle plate): the block's two, blind `NUT_PIN_DEPTH` deep. */
  static function nutBlockPins(axis:HobberDriveAxis, origin:Array<Float>):PinnedFace {
    var block = HobberDrives.nutBlock(axis, new machinekit.motion.BallNut());
    return new PinnedFace("nut-block-pins", nutBlockFrame(axis, origin), [for (u in block.pinXs()) {x: u, y: 0.0}], machinekit.motion.NutBlock.PIN_DIAMETER, NUT_PIN_DEPTH);
  }
  /** Depth of the carrier's dowel bores for the nut block, mm: one diameter of engagement plus the relief, with a millimetre over (the carrier is 20 and 13 thick). */
  static inline final NUT_PIN_DEPTH = 8.0;
  /** The key on the Y nut seat: two ribs, either side of the block's base, each KEY_THICKNESS thick and KEY_HEIGHT proud of the seat (the block's 20 mm base sits between them). */
  public static inline final KEY_HEIGHT = 3.0;
  public static inline final KEY_THICKNESS = 3.5;
  /** The Y nut seat's extent across the screw (x 40..82), which is the key's length. */
  static inline final NUT_SEAT_WIDTH = 42.0;

  /** Material under the heads of the guide-block screws (the pocket is cut from the far face), mm: M6 into a 20 mm deck, M4 into the 13 and 12 mm plates. */
  static inline final X_BLOCK_SOLE = 10.0;
  public static inline final X_CARRIAGE_THICKNESS = 20.0;
  /** Derive every X stack plane from the fixed bed seat and catalogue guide.
   * Replicated block pads occupy relieved plate material, adding no axis height. */
  public static function xGuideOriginZ():Float return FoundationLayout.bedRailSeatTop() + xGuideSpec().railHeight;
  static function xGuideSpec():machinekit.motion.LinearRailProfile.LinearRailProfileSpec {
    var spec=machinekit.motion.LinearRailSystem.catalog().get("HGH25CA");
    if (spec==null) throw "X guide specification unavailable";
    return spec;
  }
  public static function xCarriageBottom():Float return FoundationLayout.bedRailSeatTop() + xGuideSpec().blockHeight;
  public static function xCarriageTop():Float return xCarriageBottom() + X_CARRIAGE_THICKNESS;


  /** Buy the carriage width outward from the outermost guide-block bolt and its
   * edge/head wall requirement, in 10 mm full-width stock increments. */
  public static function xCarriageHalfWidth():Float {
    var block = machinekit.motion.LinearRailBlock.metric("HGH25CA");
    var screw = machinekit.standard.SocketHeadCapScrew.catalog().get(block.spec.blockMountThread);
    var edge = machinekit.assembly.JointFasteners.requiredEdgeRadius(screw.diameter, screw.headDiameter + 2);
    return Math.ceil((FoundationLayout.BED_RAIL_CENTRE + block.spec.blockHoleSpacingAcross / 2 + edge) / 5) * 5;
  }
  static inline final PLATE_BLOCK_SOLE = 6.0;

  /** The middle y of the Y bracket under `support` (its foot's bolts are centred on it). */
  static function bracketMid(support:String):Float {
    if (support == "floating") return (-114.6 + -55.4) / 2;
    var lo = 1e9, hi = -1e9;
    for (h in HobberDrives.seatHoles(HobberDrives.y(), "fixed")) {
      lo = Math.min(lo, h.position[1]);
      hi = Math.max(hi, h.position[1]);
    }
    return (lo + hi) / 2;
  }

  /** A bracket's foot on the swivel plate's top edge: two M5 `BRACKET_BOLT_SPAN` apart on the foot's middle (x 92.5), heads on the bridge's top, 15 mm of foot under them. */
  static function bracketFoot(mid:Float):BoltedFace {
    return BoltedFace.clamped("plate", BoltedFace.frameOf({x: 92.5, y: mid, z: 458.0}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0}),
      [{x: 0.0, y: -BRACKET_BOLT_SPAN / 2}, {x: 0.0, y: BRACKET_BOLT_SPAN / 2}], machinekit.standard.SocketHeadCapScrew.catalog().get("M5").clearanceMedium, 15, "M5", null, false);
  }
  static inline final BRACKET_BOLT_SPAN = 40.0;
  /** The nut seat's two M4 into the saddle's top edge, mm apart (the nut block stands between them). */
  static inline final NUT_SEAT_BOLT_SPAN = 40.0;
  /** The nut seat's thickness (ground flat bar): see `HobberYLayout.NUT_BLOCK_HEIGHT`. */
  static inline final NUT_SEAT_THICKNESS = 16.0;

  override public function machinable():Bool return ["GE-X-carriage", "GE-Z-saddle", "GE-Y-saddle", "GE-X-drive-pad", "GE-Z-drive-pad", "GE-Y-drive-fixed",
    "GE-Y-drive-floating", "GE-swivel-plate", "GE-swivel-adapter", "GE-spindle-seat", "GE-motor-cradle", "GE-Y-nut-seat", "GE-Z-scale-arm", "GE-outboard-carrier"].indexOf(designId) >= 0;

  /** The dowels of the supports on a pad or bracket: each support's two, the seat's half of `SupportedScrewDrive.dowelPoints` (the foot's half is the support's own). */
  function padPins(axis:HobberDriveAxis, supports:Array<String>, thickness:Float, ?clip:Array<Float>):Array<machinekit.component.BoltedFace.PinnedFace> {
    var faces = [];
    for (support in supports) {
      var holes = [for (h in HobberDrives.seatRouteHoles(axis, support)) if (h.kind == "dowel") h];
      if (holes.length == 0) continue;
      faces.push(PinnedFace.atPoints("pins-" + support, seatFace(axis, support, thickness, clip).frame,
        [for (h in holes) {x: h.position[0], y: h.position[1], z: h.position[2]}], holes[0].nominal, holes[0].depth, holes[0].depth >= thickness - 1e-6));
    }
    return faces;
  }

  /** The seat's two dowels into the Y saddle's plate: 8 mm, 35 mm either side of the plane through the cartridge, reamed through the 12 mm plate and 12 mm into the seat. */
  static final SEAT_PIN_Y = 150.0;
  static final SEAT_PIN_Z = 35.0;

  /** The Y nut seat's key (the ribs either side of the block's base) locates the block against the screw thrust in both directions. */
  override public function locatingKeys():Array<machinekit.component.BoltedFace.LocatingKey> {
    return designId == "GE-Y-nut-seat" ? [{face: "nut-block", height: KEY_HEIGHT, length: NUT_SEAT_WIDTH, thickness: KEY_THICKNESS}] : [];
  }

  static function outboardCarrierFrame(into:Float):AssemblyFrame return BoltedFace.frameOf(
    {x: 45.0, y: -100.0, z: 0.0}, {x: into, y: 0, z: 0}, {x: 0, y: 1, z: 0});
  static function outboardCarrierPins(into:Float):machinekit.component.BoltedFace.PinnedFace return PinnedFace.atPoints("outboard-pins", outboardCarrierFrame(into),
    [for (z in [-45.0, 45.0]) {x: 45.0, y: -100.0, z: z}], 4.0, 6.0, false);

  override public function pinnedFaces():Array<machinekit.component.BoltedFace.PinnedFace> {
    return switch designId {
      case "GE-X-carriage":
        var pins = [nutBlockPins(HobberDrives.x(), [0.0, 0, 0])];
        for (role in HobberCarriageRib.ROLES) {
          var rib = new HobberCarriageRib(new ComponentValues().setToken("role", role));
          pins.push(new PinnedFace("rib-" + role, rib.interfaceFrame(1),
            [for (x in rib.pinXs()) {x: x - 263, y: 0.0}], 4, 8));
        }
        pins;
      case "GE-Z-saddle": [nutBlockPins(HobberDrives.z(), [0.0, 0, Z_SADDLE_ORIGIN_Z])];
      case "GE-X-drive-pad": padPins(HobberDrives.x(), ["fixed", "floating"], 15);
      case "GE-Z-drive-pad": padPins(HobberDrives.z(), ["fixed", "floating"], 40);
      case "GE-Y-drive-fixed": padPins(HobberDrives.y(), ["fixed"], 10);
      case "GE-Y-drive-floating": padPins(HobberDrives.y(), ["floating"], 10, [35.0, -114.6, 463, 45, -55.4, 548]);
      case "GE-outboard-carrier": [outboardCarrierPins(-1), HobberOutboardLayout.carrierPins()];
      case "GE-Y-saddle":
        [outboardCarrierPins(1), PinnedFace.atPoints("seat-pins", BoltedFace.frameOf({x: 45.0, y: 0.0, z: 0.0}, {x: 1, y: 0, z: 0}, {x: 0, y: 1, z: 0}),
          [for (z in [-SEAT_PIN_Z, SEAT_PIN_Z]) {x: 57.0, y: SEAT_PIN_Y, z: z}], 8.0, 12.0, true)];
      case "GE-spindle-seat":
        [PinnedFace.atPoints("back-pins", BoltedFace.frameOf({x: SEAT_X1, y: (SEAT_Y0 + SEAT_Y1) / 2, z: (SEAT_LOW + SEAT_HALF) / 2}, {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0}),
          [for (z in [-SEAT_PIN_Z, SEAT_PIN_Z]) {x: SEAT_X1, y: SEAT_PIN_Y, z: z}], 8.0, 12.0, false),
          PinnedFace.atPoints("top-pins", BoltedFace.frameOf({x: 0.0, y: CRADLE_PIN_Y, z: SEAT_HALF}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0}),
            [for (x in CRADLE_PIN_X) {x: x, y: CRADLE_PIN_Y, z: SEAT_HALF}], 8.0, 12.0, false)];
      case "GE-motor-cradle":
        [PinnedFace.atPoints("floor-pins", BoltedFace.frameOf({x: 0.0, y: CRADLE_PIN_Y, z: SEAT_HALF}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0}),
          [for (x in CRADLE_PIN_X) {x: x, y: CRADLE_PIN_Y, z: SEAT_HALF}], 8.0, cradleFloorTop(CRADLE_PIN_Y) - SEAT_HALF, true)];
      case "GE-swivel-adapter":
        [ringPins(BoltedFace.frameOf({x: HobberSwivelLayout.PLATE_X, y: 0.0, z: 0.0}, {x: 1, y: 0, z: 0}, {x: 0, y: 1, z: 0}))];
      case "GE-swivel-plate":
        [ringPins(BoltedFace.frameOf({x: HobberSwivelLayout.PLATE_X, y: 0.0, z: 0.0}, {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0}))];
      default: [];
    };
  }

  /** The two dowels between the adapter ring and the swivel plate (`HobberSwivelLayout.RING_PIN_*`), on the joint plane, the face frame being that side's. */
  static function ringPins(frame:materia.assembly.AssemblyRecord.AssemblyFrame):PinnedFace {
    var r = HobberSwivelLayout.RING_PIN_RADIUS;
    return PinnedFace.atPoints("ring-pins", frame, [for (k in 0...2) {
      var a = HobberSwivelLayout.RING_PIN_PHASE + k * Math.PI;
      {x: HobberSwivelLayout.PLATE_X, y: r * Math.cos(a), z: r * Math.sin(a)};
    }], HobberSwivelLayout.RING_PIN_DIAMETER, HobberSwivelLayout.RING_PIN_DEPTH);
  }

  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace> = [];
    switch designId {
      case "GE-X-carriage":
        var block = machinekit.motion.LinearRailBlock.metric("HGH25CA", machinekit.motion.LinearRailBlock.SUPPLIER);
        for (side in [-1, 1]) for (row in [HobberXLayout.BLOCK_FRONT, HobberXLayout.BLOCK_REAR])
          faces.push(blockFace("block" + (side < 0 ? "L" : "R") + (row == HobberXLayout.BLOCK_FRONT ? "1" : "2"), [row, side * FoundationLayout.BED_RAIL_CENTRE, xCarriageBottom()],
            [0.0, 0, 1], [1.0, 0, 0], block, 20, X_BLOCK_SOLE));
        // Two derived M8 per ground corner pad, spread over the whole column base.
        for (pad in FoundationLayout.columnSolePads()) faces.push(columnSoleFace(pad));
        faces.push(nutBlockFace(HobberDrives.x(), X_CARRIAGE_THICKNESS, [0.0, 0, 0]));
        faces = faces.concat(HobberChipCover.carriageReceivers());
        // The X scale head's two M4 enter from the seat side (below, before the carriage is set on the rails) and thread into the rib's underside.
        for (role in HobberCarriageRib.ROLES) {
          var rib = new HobberCarriageRib(new ComponentValues().setToken("role", role));
          var b = rib.band();
          faces.push(BoltedFace.drilledToSuit("rib-" + role, rib.interfaceFrame(1),
            246, b.high - b.low, 15, 20, BoltedFace.ThreadMaterial.Steel, rib.boltSize()));
        }
      case "GE-Z-saddle":
        var block = machinekit.motion.LinearRailBlock.metric("HGH15CA", machinekit.motion.LinearRailBlock.SUPPLIER);
        for (side in [-1, 1]) for (row in [0.0, 110.0])
          faces.push(blockFace("block" + (side < 0 ? "L" : "R") + (row == 0 ? "1" : "2"), [148.0, side * 145.0, row], [-1.0, 0, 0], [0.0, 1, 0], block, 13, PLATE_BLOCK_SOLE));
        // Flush heads keep the fixed Z chip shield clear throughout saddle travel.
        faces.push(nutBlockFace(HobberDrives.z(), 6.5, [0.0, 0, Z_SADDLE_ORIGIN_Z], {diameter: 12.0, depth: 6.5}));
        // The eRob housing's 16 M4 into its front face, passed through the 5 mm web from the front (x 135): the plane is the housing's front face.
        faces.push(BoltedFace.clamped("erob", BoltedFace.frameOf({x: HobberSwivelLayout.OUTPUT_X - machinekit.motion.ErobActuatorPart.SEAT_Z, y: 0.0, z: 0.0}, {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0}),
          [for (i in 0...machinekit.motion.ErobActuatorData.FIXED_BOLT_COUNT) {
            var b = machinekit.motion.ErobActuatorPart.fixedBolt(i);
            {x: b.x, y: b.y};
          }], machinekit.standard.SocketHeadCapScrew.catalog().get("M4").clearanceMedium,
          HobberSwivelLayout.OUTPUT_X - machinekit.motion.ErobActuatorPart.SEAT_Z - HobberSwivelLayout.PLATE_FRONT, "M4", null, false));
      case "GE-outboard-carrier":
        faces.push(BoltedFace.drilledToSuit("saddle", outboardCarrierFrame(-1), 30, 110, 10, 15, BoltedFace.ThreadMaterial.Steel));
        faces.push(BoltedFace.clamped("housing", HobberOutboardLayout.carrierMountFrame(), HobberOutboardLayout.carrierHoles(), 4.5, 9.5, "M4", {diameter: 9.5, depth: 5.5}, false, true));
      case "GE-Y-saddle":
        // Heads are reached from the guide side, between the two Y rails. This
        // avoids burying a driver in the carrier's closed upper/lower beams.
        faces.push(BoltedFace.clamped("outboard-carrier", outboardCarrierFrame(1),
          [for (y in [-8.0, 8.0]) for (z in [-45.0, 45.0]) {x: y, y: z}], 4.5, 12.0, "M4", null, false, true));
        // The Y scale head's two M4 enter from the seat side and thread into the boss and the saddle plate behind it.
        faces.push(HobberScales.carrierFace(HobberScales.y(), [0.0, 0, Z_SADDLE_ORIGIN_Z], Y_HEAD_HOLE_DEPTH, HobberScales.headFace(HobberScales.y())[0] - 45));
        // The spindle seat's four M8: the plate's face at x 45, +Z into the plate (+x), y across the plate (z).
        faces.push(BoltedFace.clamped("seat", BoltedFace.frameOf({x: 45.0, y: 0.0, z: 0.0}, {x: 1, y: 0, z: 0}, {x: 0, y: 1, z: 0}), seatBolts(), 9.0, 12.0, "M8", null, false));
        // The nut seat's M4 into the plate's top edge (z 95, x 45..57: 12 thick, so the screws run down its middle).
        faces.push(BoltedFace.drilledToSuit("topEdge", BoltedFace.frameOf({x: 51.0, y: -20.0, z: 458.0 - Z_SADDLE_ORIGIN_Z}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0}),
          12, 220, 8, 170, BoltedFace.ThreadMaterial.Steel));
        var block = machinekit.motion.LinearRailBlock.metric("HGH15CA", machinekit.motion.LinearRailBlock.SUPPLIER);
        for (side in [-1, 1]) for (row in [-40.0, 40.0])
          faces.push(blockFace("block" + (side < 0 ? "L" : "R") + (row < 0 ? "1" : "2"), [57.0, row, side * 60.0], [-1.0, 0, 0], [0.0, 1, 0], block, 12, PLATE_BLOCK_SOLE));
      case "GE-Y-nut-seat":
        // The nut block's two M6 pass the seat from below (x 70 +- 8: between the saddle's plate and the swivel plate, see `HobberDrives.Y_BLOCK_BASE_EDGE`) into its tapped
        // base; the seat is held to the saddle's top edge by two M4 from above, outside the block's thickness, fitted first.
        faces.push(nutBlockFace(HobberDrives.y(), NUT_SEAT_THICKNESS, [0.0, 0, 0]));
        faces.push(BoltedFace.clamped("saddle", BoltedFace.frameOf({x: 51.0, y: -10.0, z: 458.0}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0}),
          [{x: 0.0, y: -NUT_SEAT_BOLT_SPAN / 2}, {x: 0.0, y: NUT_SEAT_BOLT_SPAN / 2}], machinekit.standard.SocketHeadCapScrew.catalog().get("M4").clearanceMedium, NUT_SEAT_THICKNESS, "M4", null, false));
      case "GE-X-scale-seat":
        faces.push(HobberScales.seatFace(HobberScales.x(), [0.0, 0, 0], HobberScales.SEAT_BAR));
      case "GE-Z-scale-seat":
        faces.push(HobberScales.seatFace(HobberScales.z(), [0.0, 0, 0], HobberScales.SEAT_BAR));
      case "GE-Z-scale-arm":
        // The Z scale head's two M4 thread into the arm (10 thick) from the seat side.
        faces.push(HobberScales.carrierFace(HobberScales.z(), [0.0, 0, Z_SADDLE_ORIGIN_Z], HobberScales.Z_ARM_THICKNESS - 1, HobberScales.Z_ARM_THICKNESS + HobberScales.RUNNING_CLEARANCE));
      case "GE-X-drive-pad":
        // Support seats end at the support foot, leaving separate edge lands for the guards.
        for (support in ["fixed", "floating"]) {
          var seat = HobberDrives.padBounds(HobberDrives.x(), support, 70, 15);
          faces.push(seatFace(HobberDrives.x(), support, 15, seat));
        }
        // The four screws that hold the bar on the bed's drive-seat insert, in the groove floor (`HobberPadBolts`).
        faces.push(HobberPadBolts.face(HobberDrives.x()));
        faces = faces.concat(HobberChipCover.padReceivers());
      case "GE-Z-drive-pad":
        faces.push(seatFace(HobberDrives.z(), "fixed", 40));
        faces.push(seatFace(HobberDrives.z(), "floating", 40));
        faces.push(HobberPadBolts.face(HobberDrives.z()));
      case "GE-Y-drive-fixed":
        faces.push(seatFace(HobberDrives.y(), "fixed", 10));
        faces.push(bracketFoot(bracketMid("fixed")));
      case "GE-Y-drive-floating":
        faces.push(seatFace(HobberDrives.y(), "floating", 10, [35.0, -114.6, 463, 45, -55.4, 548]));
        faces.push(bracketFoot(bracketMid("floating")));
      case "GE-swivel-adapter":
        // Aluminium ring from the output flange (x 124) to the swivel plate (x 100): its 12 M4 clearance holes (heads sunk at the plate end) onto the output flange
        // and 4 M6 tapped into the plate end. The actuator's CAD x is the work frame's y, its y is -z.
        var ring = HobberSwivelLayout.OUTPUT_X - HobberSwivelLayout.PLATE_X;
        faces.push(BoltedFace.clamped("output", BoltedFace.frameOf({x: HobberSwivelLayout.OUTPUT_X, y: 0.0, z: 0.0}, {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0}),
          [for (i in 0...machinekit.motion.ErobActuatorData.OUTPUT_BOLT_COUNT) {
            var b = machinekit.motion.ErobActuatorPart.outputBolt(i);
            {x: b.x, y: b.y};
          }], machinekit.motion.ErobActuatorData.OUTPUT_CLEARANCE_HOLE, ring - RING_HEAD_RECESS, "M4", {diameter: 7.5, depth: RING_HEAD_RECESS}, false));
        // Tapped to suit the plate's holes (the ring is an annulus of Ø110 and Ø40: the footprint is its square, the screws' 1.5 diameters inside it).
        faces.push(BoltedFace.drilledToSuit("plate", BoltedFace.frameOf({x: HobberSwivelLayout.PLATE_X, y: 0.0, z: 0.0}, {x: 1, y: 0, z: 0}, {x: 0, y: 1, z: 0}),
          HobberSwivelLayout.RING_OUTSIDE, HobberSwivelLayout.RING_OUTSIDE, 16, ring, BoltedFace.ThreadMaterial.Aluminium));
      case "GE-swivel-plate":
        // The Y scale housing's two M8 are tapped through the plate from its front face (x 85).
        faces.push(HobberScales.seatFace(HobberScales.y(), [0.0, 0, Z_SADDLE_ORIGIN_Z], SWIVEL_PLATE_THICKNESS));
        // The ring's 4 M6 pass the plate from its front face into the ring.
        faces.push(BoltedFace.clamped("ring", BoltedFace.frameOf({x: HobberSwivelLayout.PLATE_X, y: 0.0, z: 0.0}, {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0}),
          [for (p in platePositions()) {x: p.y, y: -p.z}], machinekit.standard.SocketHeadCapScrew.catalog().get("M6").clearanceMedium, 15, "M6", null, false));
        // The Y rails' screws are tapped through the plate from its front face (x 85), the plate's frame is the Z saddle's.
        // The brackets' M5 into the plate's top edge (z 95): the edge is 15 thick, so the screws run down its middle (7.5 = 1.5 d from each side).
        faces.push(BoltedFace.drilledToSuit("topEdge", BoltedFace.frameOf({x: 92.5, y: 0.0, z: 458.0 - Z_SADDLE_ORIGIN_Z}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0}),
          15, 230, 12, 190, BoltedFace.ThreadMaterial.Steel));
        for (side in [-1, 1]) faces.push(BoltedFace.drilledToSuit(side < 0 ? "railYLeft" : "railYRight",
          BoltedFace.frameOf({x: 85.0, y: 0.0, z: side * 60.0}, {x: 1, y: 0, z: 0}, {x: 0, y: 1, z: 0}), 230, 36, 15, 15, BoltedFace.ThreadMaterial.Steel, null, true));
      case "GE-spindle-seat":
        // The back face on the Y saddle's plate: tapped to suit the plate's clearance holes (M8, 12 mm plate).
        // The cradle floor's M8 into the top (z 58): ahead of y 195 over the bore (the wall under the thread is what is left above its Ø80), behind it into the 14 mm tongue, tapped through.
        faces.push(BoltedFace.drilledToSuit("topFront", BoltedFace.frameOf({x: -1.5, y: 132.5, z: SEAT_HALF}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0}), SEAT_X1 - SEAT_X0, 125, 16,
          SEAT_HALF - SEAT_LOW, BoltedFace.ThreadMaterial.Steel, null, false, (u, _) -> SEAT_HALF - Math.sqrt(Math.max(0.0, SEAT_BORE * SEAT_BORE / 4 - (u - 1.5) * (u - 1.5)))));
        faces.push(BoltedFace.drilledToSuit("topRear", BoltedFace.frameOf({x: (SEAT_X0 + SEAT_TONGUE_X1) / 2, y: 223.0, z: SEAT_HALF}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0}),
          SEAT_TONGUE_X1 - SEAT_X0, 56, 14, SEAT_HALF - SEAT_TONGUE_Z0, BoltedFace.ThreadMaterial.Steel, null, true));
        faces.push(BoltedFace.drilledToSuit("back", BoltedFace.frameOf({x: SEAT_X1, y: (SEAT_Y0 + SEAT_Y1) / 2, z: (SEAT_LOW + SEAT_HALF) / 2}, {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0}),
          SEAT_Y1 - SEAT_Y0, SEAT_HALF - SEAT_LOW, 16, SEAT_X1 - SEAT_X0, BoltedFace.ThreadMaterial.Steel));
      case "GE-motor-cradle":
        // The floor's four M8 into the seat: heads on the floor top, clearance holes through it; the rear row's heads are sunk (Ø14 x 9) under the servo's flange.
        for (row in [CRADLE_FRONT_BOLT_Y, CRADLE_REAR_BOLT_Y]) {
          var rear = row >= SEAT_Y1, floor = cradleFloorTop(row) - SEAT_HALF;
          faces.push(BoltedFace.clamped(rear ? "floorRear" : "floorFront", BoltedFace.frameOf({x: 0.0, y: row, z: SEAT_HALF}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0}),
            [for (b in cradleFloorBolts()) if (b.y == row) {x: b.x, y: 0.0}], machinekit.standard.SocketHeadCapScrew.catalog().get("M8").clearanceMedium,
            rear ? floor - CRADLE_BORE_DEPTH : floor, "M8", rear ? {diameter: CRADLE_BORE, depth: CRADLE_BORE_DEPTH} : null, false));
        }
        // The servo's flange screws pass through the wall's vertical tension slots (drawn by `cradleHoles`) and take a washer
        // and nut on its rear face: a through plate, so the screws, washers and nuts derive while the slots stay the part's own.
        var motor = cradleMotor(), flange = motor.mounting();
        if (flange == null) flange = machinekit.motion.ServoMotor.assumedMounting(motor.rating.designation);
        faces.push(BoltedFace.through("motor", BoltedFace.frameOf({x: CRADLE_MOTOR_X, y: CRADLE_FACE_Y, z: HobSpindle.motorHeight()},
          {x: 0, y: 1, z: 0}, {x: 1, y: 0, z: 0}), [for (h in flange.holes) {x: h.x, y: -h.y}], flange.holeDiameter, CRADLE_WALL,
          "M" + Std.int(flange.holeDiameter - 0.5)).withSlots(CRADLE_SLOT, HobSpindle.TENSION_TRAVEL, "y"));
      default:
    }
    return faces;
  }

  /** The holes the derivation cut into this part, as a hole table (part frame): tapped, clearance and counterbore, from the mating plane. */
  function derivedHoles():Array<machinekit.component.ManufacturingRoute.RouteHole> return derivedRouteHoles();

  // ---- A axis and spindle seat (FRAME_PLAN R1, R2) ------------------------------------------------------------------------------

  static function platePositions():Array<{y:Float, z:Float}> {
    var r = HobberSwivelLayout.PLATE_PCD / 2, n = HobberSwivelLayout.PLATE_SCREWS;
    return [for (i in 0...n) {y: r * Math.cos(2 * Math.PI * i / n + HobberSwivelLayout.PLATE_PHASE), z: r * Math.sin(2 * Math.PI * i / n + HobberSwivelLayout.PLATE_PHASE)}];
  }

  /** The Z saddle's eRob output bore (the housing's screws and the pocket are derived or cut from the actuator's mounting cut-out), in the saddle's frame. */
  static function erobPilotBore():Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var web = HobberSwivelLayout.OUTPUT_X - machinekit.motion.ErobActuatorPart.SEAT_Z - HobberSwivelLayout.PLATE_FRONT;
    var d = machinekit.motion.ErobActuatorData.OUTPUT_PILOT_DIAMETER + machinekit.motion.ErobActuatorPart.PILOT_BORE_CLEARANCE;
    return [{id: "P1", kind: "bore", thread: null, diameter: d, nominal: d, fit: null, depth: web, position: [HobberSwivelLayout.PLATE_FRONT, 0, 0], direction: [1.0, 0, 0]}];
  }

  /** The ring's end screws into the output flange are heads sunk this deep (the head is 4: flush with the plate end, 0.2 over). */
  static inline final RING_HEAD_RECESS = 4.2;

  /** Spindle seat block: x from the bore's front side to the saddle plate, y along the cartridge, z across. */
  static inline final SEAT_X0 = -48.0;
  static inline final SEAT_X1 = 45.0;
  static inline final SEAT_Y0 = HobberSwivelLayout.SPINDLE_Y - Bt30SpindleData.CLAMP_TO;
  static inline final SEAT_Y1 = 222.0;
  /** Half height on the +z side (the slit and clamp screws); the -z side is cut back to `SEAT_LOW` so the head clears the X rail at A +15 deg. */
  static inline final SEAT_HALF = 58.0;
  static inline final SEAT_LOW = -50.0;
  static inline final SEAT_BORE = 80.0;
  static inline final SEAT_SLIT = 2.0;
  static inline final SEAT_CLAMP_Z = 49.5;
  /** The tongue behind the clamp that the cradle's foot rests on and bolts into (it reaches x 42 so the rear bolts at x 30 keep 12 mm, 1.5 M8 diameters, to its edge): above the cartridge (its top is z 40), cut by the same slit as the floor. */
  static inline final SEAT_TONGUE_X1 = 42.0;
  static inline final SEAT_TONGUE_Y1 = 251.0;
  static inline final SEAT_TONGUE_Z0 = 44.0;

  /** The seat's hand-drilled holes in its own frame (and the matching ones in the Y saddle): 2 dowels, 3 clamp screws M6 across the slit. */
  static function seatHoles(inSaddle:Bool):Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var m6 = machinekit.standard.SocketHeadCapScrew.catalog().get("M6").tapDrill;
    var list:Array<machinekit.component.ManufacturingRoute.RouteHole> = [], n = 0, back = SEAT_X1;
    // The four M8 into the seat's back face are derived (`fasteningFaces`: the Y saddle's plate clamps, the seat takes the thread); the dowels are derived too (`pinnedFaces`).
    if (!inSaddle) {
      n = 0;
      for (y in [100.0, 150.0, 200.0]) {
        n++;
        list.push({id: "C" + n, kind: "clearance", thread: "M6", diameter: 6.6, nominal: 6, fit: null, depth: -SEAT_SLIT / 2 - SEAT_X0,
          position: [SEAT_X0, y, SEAT_CLAMP_Z], direction: [1.0, 0, 0]});
        list.push({id: "K" + n, kind: "tapped", thread: "M6", diameter: m6, nominal: 6, fit: null, depth: 20.0,
          position: [SEAT_SLIT / 2, y, SEAT_CLAMP_Z], direction: [1.0, 0, 0]});
      }
    }
    return list;
  }

  /** The four M8 that hold the seat to the Y saddle: y along the cartridge, z across it; 1.5 diameters from the seat's edges (z -50 and 58). */
  static function seatBolts():Array<machinekit.component.BoltedFace.BoltHole> return [for (y in [95.0, 205.0]) for (z in [-38.0, 46.0]) {x: y, y: z}];

  /** The spindle seat's extent along the spindle axis (work y) and the y of its three clamp screws. */
  public static function spindleSeatRange():{y0:Float, y1:Float, clampYs:Array<Float>} return {y0: SEAT_Y0, y1: SEAT_Y1, clampYs: [100.0, 150.0, 200.0]};

  /** The split-clamp block that holds the Ø80 cartridge: bore, slit up the +z side, the holes of `seatHoles` and the cradle floor's tapped holes in its top. */
  static function spindleSeat():Part {
    var block = box(SEAT_X0, SEAT_X1, SEAT_Y0, SEAT_Y1, SEAT_LOW, SEAT_HALF);
    var tongue = box(SEAT_X0, SEAT_TONGUE_X1, SEAT_Y1 - 1, SEAT_TONGUE_Y1, SEAT_TONGUE_Z0, SEAT_HALF);
    var cuts = [boreY(SEAT_BORE / 2, SEAT_Y0 - 0.1, SEAT_Y1 + 0.1, 0, 0), box(-SEAT_SLIT / 2, SEAT_SLIT / 2, SEAT_Y0 - 0.1, CRADLE_FACE_Y - CRADLE_SLIT_END, 0, SEAT_HALF + 0.1)];
    return withHoles(Solids.cut(Solids.union([block, tongue]), cuts), seatHoles(false).concat(cradleHoles(true)));
  }

  // ---- B motor cradle (FRAME_PLAN head redesign) -------------------------------------------------------------------------------

  /**
   * The cradle wall: it stands behind the cartridge's rear face (y 240), the motor flange on its front face; the pulleys stand clear of its rear
   * face and the flange bolts' nuts. The servo slides along the belt's line (work z) in slots through the wall, so the nuts are on the rear face.
   */
  static inline final CRADLE_WALL = 8.0;
  /** The wall's front face (the motor flange face) in work y. */
  static inline final CRADLE_FACE_Y = HobberSwivelLayout.SPINDLE_Y - HobSpindle.MOTOR_FACE_Z;
  /**
   * The floor sits on the spindle seat block's top (z `SEAT_HALF`). Behind `CRADLE_FLOOR_STEP` (where the seat block ends and the wall's foot
   * overhangs it) it is `CRADLE_FLOOR` thick; ahead, where it only carries the first bolt row, `CRADLE_FLOOR_THIN`.
   */
  static inline final CRADLE_FLOOR = 12.0;
  static inline final CRADLE_FLOOR_THIN = 8.0;
  /**
   * Under the wall the floor is a `CRADLE_FOOT` thick foot (from the seat's end `SEAT_Y1`), and its rear bolt row stands just ahead of the wall's front
   * face, so the wall's foot overhangs the bolts by a few millimetres, not a floor's length: the foot rests on the seat's tongue (`SEAT_TONGUE_*`), which
   * carries the bolts' tapped holes under the wall's own footprint. The flange stays above the foot (the foot top is 5 mm under the flange's underside).
   */
  static inline final CRADLE_FOOT = 18.0;
  static inline final CRADLE_REAR_BOLT_Y = 232.0;
  static inline final CRADLE_FLOOR_Y0 = 125.0;
  static inline final CRADLE_FLOOR_STEP = 195.0;
  /** The wall stands clear of the Y drive's motor and bracket (x from 35) on the +x side: the servo is 8 mm toward -x of the cartridge (the flange's holes are 31.8 from its axis). */
  static inline final CRADLE_X0 = -56.0;
  static inline final CRADLE_X1 = 31.0;
  /** The floor ahead of the step is trimmed to the bolts' width; the thick part reaches farther on the +x side (x 38) for its bolts. */
  static inline final CRADLE_FLOOR_THIN_X0 = -44.0;
  static inline final CRADLE_FLOOR_X1 = 38.0;
  /** The servo's axis in work x. */
  static inline final CRADLE_MOTOR_X = -HobSpindle.MOTOR_OFFSET;
  /** One rib stiffens the wall against the servo rocking on its offset centre of mass: 6 thick, outside the flange, from y `CRADLE_RIB_Y0` to the wall's rear, with a lightening window. */
  static inline final CRADLE_RIB = 6.0;
  static inline final CRADLE_RIB_Y0 = 192.0;
  /** The rib's window (work y, z above the cartridge axis): a 17 mm front post and a 15 mm rear post stay, and the foot below z 95. */
  static inline final CRADLE_RIB_WINDOW_Y0 = 209.0;
  static inline final CRADLE_RIB_WINDOW_Y1 = 232.0;
  static inline final CRADLE_RIB_WINDOW_Z0 = 95.0;
  static inline final CRADLE_RIB_WINDOW_Z1 = 160.0;
  /** The slit through the floor, over the seat's own, so the clamp's two arms are not bridged by the floor (it stops short of the wall by this much). */
  static inline final CRADLE_SLIT = 4.0;
  static inline final CRADLE_SLIT_END = 3.0;
  /** The jack screw's lug under the servo flange: it stands on the floor against the wall's front face, `CRADLE_LUG` above the foot top (the underside of the flange is 5 mm above the foot), M8 vertical in the flange's y middle. */
  static inline final CRADLE_LUG = 4.0;
  static inline final CRADLE_LUG_Y0 = 224.0;
  static inline final CRADLE_LUG_HALF = 13.0;
  /** Width of the flange-bolt slots and of the rest, mm: M6 clearance 6.6 plus 0.4. */
  static inline final CRADLE_SLOT = 7.0;

  /**
   * The rest block under the servo's body (FE, 2026-11-06): the servo hangs 46 mm ahead of the wall on a plate wall, and with the floor alone its first mode is 340 Hz.
   * A block on the floor from `CRADLE_REST_Y0` to `CRADLE_REST_Y1` across `CRADLE_REST_X0..X1` (the dowels' x +- 4) carries the body's underside through a ground shim
   * pack set at assembly: the belt's pull and the weight press the body on it, so the contact stays closed under any vibration that the cutting load excites.
   * The top stands `TENSION_TRAVEL` plus `CRADLE_REST_CLEAR` under the body's underside at the nominal centres so the servo can still slide down to the bottom of its slots
   * to fit the belt; the pack is that gap plus the tension travel (peel shims). The slit runs through it.
   */
  static inline final CRADLE_REST_Y0 = 145.0;
  static inline final CRADLE_REST_Y1 = 200.0;
  static inline final CRADLE_REST_X0 = -44.0;
  static inline final CRADLE_REST_X1 = 36.0;
  static inline final CRADLE_REST_CLEAR = 0.5;

  /** The floor's two dowels into the seat block's top: 8 mm at these x, y 190, reamed through the 8 mm floor and 12 mm into the seat. */
  static final CRADLE_PIN_Y = 190.0;
  static final CRADLE_PIN_X = [-40.0, 24.0];

  /** The floor's four M8 into the seat block's top and its tongue: two rows 67 apart along the spindle, 60 apart across it. */
  public static function cradleFloorBolts():Array<{x:Float, y:Float}> return [{x: -30.0, y: CRADLE_FRONT_BOLT_Y}, {x: 30.0, y: CRADLE_FRONT_BOLT_Y}, {x: -30.0, y: CRADLE_REAR_BOLT_Y}, {x: 30.0, y: CRADLE_REAR_BOLT_Y}];
  static inline final CRADLE_FRONT_BOLT_Y = 135.0;
  /** The rear row's counterbore for the M8 heads (Ø13 x 8): the heads end below the foot's top, under the servo's flange. */
  static inline final CRADLE_BORE = 14.0;
  static inline final CRADLE_BORE_DEPTH = 9.0;

  /** The floor's top at work y: the thin part ahead of the step. */
  static function cradleFloorTop(y:Float):Float return SEAT_HALF + (y >= SEAT_Y1 ? CRADLE_FOOT : y < CRADLE_FLOOR_STEP ? CRADLE_FLOOR_THIN : CRADLE_FLOOR);

  /** Work z (seat frame) of the rest block's top: under the servo body's underside at the bottom of its slot travel by `CRADLE_REST_CLEAR`. */
  public static function cradleRestTop():Float {
    var flange = cradleMotor().mounting();
    if (flange == null) flange = machinekit.motion.ServoMotor.assumedMounting(cradleMotor().rating.designation);
    return HobSpindle.motorHeight() - flange.face / 2 - HobSpindle.TENSION_TRAVEL - CRADLE_REST_CLEAR;
  }

  /** The rest block's footprint and top, mm (seat frame): the FE couples its top to the servo group through the shim pack. */
  public static function cradleRest():{x0:Float, x1:Float, y0:Float, y1:Float, top:Float} return {x0: CRADLE_REST_X0, x1: CRADLE_REST_X1, y0: CRADLE_REST_Y0, y1: CRADLE_REST_Y1, top: cradleRestTop()};

  /** Work y of the wall's front face (the motor flange face). */
  public static inline function cradleFaceY():Float return CRADLE_FACE_Y;

  /** The servo's axis in work x (the cradle's authored frame). */
  public static inline function cradleMotorX():Float return CRADLE_MOTOR_X;

  /**
   * The dimensions that decide the cradle's FE (and the hand model's inputs) as one string: the FE reference in `HobberFeResults` is valid only for this signature.
   */
  public static function cradleSignature():String {
    var rest = cradleRest(), r1 = function(v:Float) return Std.string(Math.round(v * 10) / 10);
    return "wall " + r1(CRADLE_WALL) + ", rib " + r1(CRADLE_RIB) + " from y " + r1(CRADLE_RIB_Y0) + " windowed y " + r1(CRADLE_RIB_WINDOW_Y0) + ".." + r1(CRADLE_RIB_WINDOW_Y1) + " z " + r1(CRADLE_RIB_WINDOW_Z0) + ".." + r1(CRADLE_RIB_WINDOW_Z1)
      + ", floor " + r1(CRADLE_FLOOR) + "/" + r1(CRADLE_FLOOR_THIN) + " from y " + r1(CRADLE_FLOOR_Y0) + ", foot " + r1(CRADLE_FOOT) + ", bolts " + [for (b in cradleFloorBolts()) r1(b.x) + "," + r1(b.y)].join(" ")
      + ", rest x " + r1(rest.x0) + ".." + r1(rest.x1) + " y " + r1(rest.y0) + ".." + r1(rest.y1) + " top " + r1(rest.top) + ", x " + r1(CRADLE_X0) + ".." + r1(CRADLE_X1);
  }

  /** Work y of the wall's rear face. */
  public static inline function cradleRearY():Float return CRADLE_FACE_Y + CRADLE_WALL;

  /**
   * The cradle's section for the drive-mode estimate: wall thickness and width, height above the floor, the rib's thickness, x and depth ahead of the
   * wall's rear face, the floor bolts, mm.
   */
  public static function cradleSection():{wall:Float, width:Float, x0:Float, height:Float, ribThickness:Float, ribDepth:Float, ribX:Float, floorTop:Float, bolts:Array<{x:Float, y:Float}>} {
    return {wall: CRADLE_WALL, width: CRADLE_X1 - CRADLE_X0, x0: CRADLE_X0, ribX: CRADLE_X0 - 2.0, height: HobSpindle.motorHeight() + 47 - (SEAT_HALF + CRADLE_FOOT), ribThickness: CRADLE_RIB,
      ribDepth: cradleRearY() - CRADLE_RIB_Y0, floorTop: SEAT_HALF + CRADLE_FOOT, bolts: cradleFloorBolts()};
  }

  /**
   * The cradle's wall section at height `z` (seat frame) as x slabs `{x0, x1, y0, y1}` (mm): the wall at full thickness (the pilot recess, the shaft slot and the bolt slots are bypassed by the
   * load, which flows along the strips between them: FE of the cradle alone, `fem/`), and with `withRib` the rib beside the wall, whose window leaves its front post (from the rib's front
   * edge to the window) and its rear post between z `CRADLE_RIB_WINDOW_Z0` and `Z1`. Overlapping slabs are merged by x into one y extent each.
   */
  public static function cradleSlabs(z:Float, withRib:Bool):Array<{x0:Float, x1:Float, y0:Float, y1:Float}> {
    var rects = [{x0: CRADLE_X0, x1: CRADLE_X1, y0: CRADLE_FACE_Y, y1: cradleRearY()}];
    if (withRib) {
      var rx0 = CRADLE_X0 - 5.0, rx1 = CRADLE_X0 + 1.0;
      if (z >= CRADLE_RIB_WINDOW_Z0 && z <= CRADLE_RIB_WINDOW_Z1) {
        rects.push({x0: rx0, x1: rx1, y0: CRADLE_RIB_Y0, y1: CRADLE_RIB_WINDOW_Y0});
        rects.push({x0: rx0, x1: rx1, y0: CRADLE_RIB_WINDOW_Y1, y1: cradleRearY()});
      } else rects.push({x0: rx0, x1: rx1, y0: CRADLE_RIB_Y0, y1: cradleRearY()});
    }
    var xs:Array<Float> = [];
    for (r in rects) for (x in [r.x0, r.x1]) if (xs.indexOf(x) < 0) xs.push(x);
    xs.sort((a, b) -> a < b ? -1 : a > b ? 1 : 0);
    var slabs = [];
    for (i in 0...xs.length - 1) {
      var mid = 0.5 * (xs[i] + xs[i + 1]), lo = 1e9, hi = -1e9;
      for (r in rects) if (r.x0 <= mid && mid <= r.x1) {
        lo = Math.min(lo, r.y0);
        hi = Math.max(hi, r.y1);
      }
      if (hi > lo) slabs.push({x0: xs[i], x1: xs[i + 1], y0: lo, y1: hi});
    }
    return slabs;
  }

  /** The thicknesses the drive check reports: wall, thick floor, thin floor, jack lug height above the floor, slot travel (each way), slot width. */
  public static function cradleDimensions():{wall:Float, floor:Float, floorThin:Float, foot:Float, lug:Float, travel:Float, slot:Float, jackY:Float} {
    return {wall: CRADLE_WALL, floor: CRADLE_FLOOR, floorThin: CRADLE_FLOOR_THIN, foot: CRADLE_FOOT, lug: CRADLE_LUG, travel: HobSpindle.TENSION_TRAVEL, slot: CRADLE_SLOT, jackY: CRADLE_FACE_Y - 4.0};
  }

  static function cradleMotor():machinekit.motion.ServoMotor return machinekit.motion.LeadshineServo.driven(HobSpindle.MOTOR);

  /** A slot as `holeTool` ends and the box between them: vertical (work z) with the round ends at `zc` +- `travel`, cut into +y from the mouth at `y0`. */
  static function slotCuts(x:Float, zc:Float, width:Float, travel:Float, y0:Float, depth:Float):Array<Part> {
    return [holeTool([x, y0, zc - travel], [0.0, 1, 0], width, depth), holeTool([x, y0, zc + travel], [0.0, 1, 0], width, depth),
      box(x - width / 2, x + width / 2, y0 - 1, y0 + depth, zc - travel, zc + travel)];
  }

  /**
   * The cradle's holes in the work frame lowered by the seat height (`inSeat` false), or the seat block's matching tapped holes (true): the
   * floor's four M8 and two dowels into the seat's top, and in the cradle the motor's pilot register, four flange bolts and the
   * shaft's through bore, all three vertical slots (listed as their two round ends), and the jack screw's tapped M8 in the lug.
   */
  static function cradleHoles(inSeat:Bool):Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var list:Array<machinekit.component.ManufacturingRoute.RouteHole> = [], m8 = machinekit.standard.SocketHeadCapScrew.catalog().get("M8").tapDrill;
    var n = 0;
    // The floor's four M8 (clearance, the rear row's counterbore, the seat's thread) are derived (`fasteningFaces`).
    if (inSeat) return list;
    var motor = cradleMotor(), flange = motor.mounting();
    if (flange == null) flange = machinekit.motion.ServoMotor.assumedMounting(motor.rating.designation);
    var c = HobSpindle.motorHeight(), t = HobSpindle.TENSION_TRAVEL, k = 0;
    // The pilot register and the shaft bore are slots too (the servo slides on its flange bolts, the pilot has the travel to spare).
    list.push({id: "P1", kind: "bore", thread: null, diameter: flange.pilotDiameter + 0.2, nominal: flange.pilotDiameter, fit: "H7",
      depth: flange.pilotHeight, position: [CRADLE_MOTOR_X, CRADLE_FACE_Y, c], direction: [0.0, 1, 0], slotLength: 2 * t, slotAxis: [0.0, 0, 1]});
    list.push({id: "P2", kind: "bore", thread: null, diameter: motor.rating.shaftDiameter + 8, nominal: motor.rating.shaftDiameter + 8, fit: null,
      depth: CRADLE_WALL, position: [CRADLE_MOTOR_X, CRADLE_FACE_Y, c], direction: [0.0, 1, 0], slotLength: 2 * t, slotAxis: [0.0, 0, 1]});
    // The flange bolts: M6 from the motor side through the slot and a washer and nut on the wall's rear face.
    for (h in flange.holes) {
      k++;
      list.push({id: "M" + k, kind: "clearance", thread: "M" + Std.int(flange.holeDiameter - 0.5), diameter: CRADLE_SLOT, nominal: flange.holeDiameter - 0.5, fit: null,
        depth: CRADLE_WALL, position: [CRADLE_MOTOR_X + h.x, CRADLE_FACE_Y, c + h.y], direction: [0.0, 1, 0], slotLength: 2 * t, slotAxis: [0.0, 0, 1]});
    }
    // The jack screw: M8 up through the lug, its tip on the flange's underside.
    list.push({id: "J1", kind: "tapped", thread: "M8", diameter: m8, nominal: 8, fit: null, depth: CRADLE_LUG + 12.0,
      position: [CRADLE_MOTOR_X, CRADLE_FACE_Y - 4.0, cradleFloorTop(CRADLE_FACE_Y) + CRADLE_LUG], direction: [0.0, 0, -1]});
    return list;
  }

  /** The motor cradle: a milled steel part, a floor on the seat block's top, a wall with the servo's slotted flange register, a jack lug, one windowed rib and the rest block under the servo's body. */
  static function motorCradle():Part {
    var c = HobSpindle.motorHeight(), top = c + 47, rear = cradleRearY(), low = SEAT_HALF, floor = low + CRADLE_FLOOR, thin = low + CRADLE_FLOOR_THIN, foot = low + CRADLE_FOOT;
    var body = Solids.union([box(CRADLE_FLOOR_THIN_X0, CRADLE_FLOOR_X1, CRADLE_FLOOR_Y0, CRADLE_FLOOR_STEP, low, thin),
      box(CRADLE_X0, CRADLE_FLOOR_X1, CRADLE_FLOOR_STEP, SEAT_Y1, low, floor),
      box(CRADLE_X0, CRADLE_FLOOR_X1, SEAT_Y1, rear, low, foot),
      box(CRADLE_REST_X0, CRADLE_REST_X1, CRADLE_REST_Y0, CRADLE_REST_Y1, low, cradleRestTop()),
      box(CRADLE_X0, CRADLE_X1, CRADLE_FACE_Y, rear, foot, top),
      box(CRADLE_X0 - 5, CRADLE_X0 + 1, CRADLE_RIB_Y0, rear, low, top),
      box(CRADLE_MOTOR_X - CRADLE_LUG_HALF, CRADLE_MOTOR_X + CRADLE_LUG_HALF, CRADLE_LUG_Y0, CRADLE_FACE_Y + 0.5, foot, foot + CRADLE_LUG)]);
    var slit = box(-CRADLE_SLIT / 2, CRADLE_SLIT / 2, CRADLE_FLOOR_Y0 - 1, CRADLE_FACE_Y - CRADLE_SLIT_END, low - 1, Math.max(foot, cradleRestTop()) + 1);
    var window = box(CRADLE_X0 - 6, CRADLE_X0 + 2, CRADLE_RIB_WINDOW_Y0, CRADLE_RIB_WINDOW_Y1, CRADLE_RIB_WINDOW_Z0, CRADLE_RIB_WINDOW_Z1);
    var flange = cradleMotor().mounting();
    if (flange == null) flange = machinekit.motion.ServoMotor.assumedMounting(cradleMotor().rating.designation);
    var t = HobSpindle.TENSION_TRAVEL, cuts = [slit, window];
    // Pilot recess, shaft bore and the four flange-bolt slots, then the plain holes.
    cuts = cuts.concat(slotCuts(CRADLE_MOTOR_X, c, flange.pilotDiameter + 0.2, t, CRADLE_FACE_Y, flange.pilotHeight));
    cuts = cuts.concat(slotCuts(CRADLE_MOTOR_X, c, cradleMotor().rating.shaftDiameter + 8, t, CRADLE_FACE_Y, CRADLE_WALL));
    for (h in flange.holes) cuts = cuts.concat(slotCuts(CRADLE_MOTOR_X + h.x, c + h.y, CRADLE_SLOT, t, CRADLE_FACE_Y, CRADLE_WALL));
    var plain = [for (h in cradleHoles(false)) if (h.slotLength == null) h];
    return withHoles(Solids.cut(body, cuts), plain);
  }

  /**
   * The ground bar under both screw supports of an axis: one box over the fixed support's mount-flange relief and both supports'
   * seat holes; the Z one is also cut by the A actuator's housing envelope swept over Z.
   */
  static function drivePad(axis:HobberDriveAxis, thickness:Float, housingRelief:Bool):Part {
    var cutters = [padBox(HobberDrives.mountRelief(axis))];
    if (housingRelief) cutters.push(HobberZOffset.padRelief());
    var groove = HobberDrives.nutGroove(axis);
    if (groove != null) cutters.push(padBox(groove));
    var relieved = Solids.cut(padBox(HobberDrives.padBoundsBoth(axis, axis.id == "X" ? HobberDrives.X_PAD_WIDTH : PAD_WIDTH, thickness)), cutters);
    for (c in cutters) c.close();
    return relieved;
  }

  /** How each made design is produced; null for the designs that are not part of the moving structure. */
  static function routeOf(design:String):Null<ManufacturingRoute> {
    function bar(thickness:Float, faces:Array<MachinedFace>):ManufacturingRoute return ManufacturingRoute.groundStock(GroundStock.bar(design == "GE-X-drive-pad" ? HobberDrives.X_PAD_WIDTH : PAD_WIDTH, thickness), faces);
    return switch design {
      case "GE-X-carriage":
        ManufacturingRoute.groundStock(GroundStock.plate(GroundStock.STEEL, X_CARRIAGE_THICKNESS), [
          {name: "four block pads replicated against one master to 0.01 mm; stock flatness fails", tolerance: 0.01, roughnessRa: 1.6},
          {name: "column sole seat and rib bond lands", tolerance: 0.02, roughnessRa: 1.6}
        ]);
      case "GE-Z-saddle":
        ManufacturingRoute.milled(2, "fine", [
          {name: "block faces, back", tolerance: 0.02, roughnessRa: 1.6},
          {name: "eRob seat: output bore, front face web and pocket", tolerance: 0.05, roughnessRa: 1.6}
        ], 350);
      case "GE-X-drive-pad":
        // One bar under both supports: the grinding makes both seats coplanar, and it stays the re-grindable height spacer.
        bar(15, [{name: "seat plane under both supports (ground together after cutting)", tolerance: 0.01, roughnessRa: 1.6}, {name: "seat holes", tolerance: 0.1, roughnessRa: null},
          {name: "groove for the nut block's travel (block + 2.5 mm each way)", tolerance: 0.5, roughnessRa: null}]);
      case "GE-Z-drive-pad":
        bar(40, [{name: "seat plane under both supports (ground together after cutting)", tolerance: 0.01, roughnessRa: 1.6}, {name: "seat holes", tolerance: 0.1, roughnessRa: null},
          {name: "groove for the nut block's travel (block + 2.5 mm each way)", tolerance: 0.5, roughnessRa: null},
          {name: "relief for the A housing envelope (swept over Z, 2.5 mm gap)", tolerance: 0.5, roughnessRa: null}]);
      case "GE-Y-drive-fixed" | "GE-Y-drive-floating":
        ManufacturingRoute.milled(1, "fine", [{name: "seat face, bolts and dowels", tolerance: 0.05, roughnessRa: 3.2}], 160);
      case "GE-Y-nut-seat":
        // Milled from 19 mm bar in one setup: the seat plane and the two key ribs (20 mm apart for the block's ground 20 mm base).
        ManufacturingRoute.milled(1, "fine", [{name: "nut block seat plane and key ribs (20 mm between them)", tolerance: 0.02, roughnessRa: 1.6}], 60);
      case "GE-X-scale-seat" | "GE-Z-scale-seat":
        // Ground bar: the housing's mounting face parallelism comes from the datasheet (0.1 over the length), the grinding gives 0.01.
        ManufacturingRoute.groundStock(GroundStock.bar(HobberScales.SEAT_BAR_WIDTH, HobberScales.SEAT_BAR),
          [{name: "housing seat, tapped holes (ground after cutting)", tolerance: HobberScales.x().housing.mounting().housingParallelism, roughnessRa: null}]);
      case "GE-Z-scale-arm":
        ManufacturingRoute.milled(1, "fine", [{name: "scale head seat, face", tolerance: HobberScales.z().head.mounting().headParallelism, roughnessRa: 1.6}], 230);
      case "GE-outboard-carrier":
        ManufacturingRoute.milled(2, "fine", [
          {name: "bearing housing seat, perpendicular to saddle face", tolerance: 0.01, roughnessRa: 1.6},
          {name: "carrier bolts and dowels (ream at alignment)", tolerance: 0.02, roughnessRa: 1.6}
        ], 110);
      case "GE-Y-saddle":
        ManufacturingRoute.milled(2, "fine", [
          {name: "block faces, back", tolerance: 0.02, roughnessRa: 1.6},
          {name: "scale head seat, boss", tolerance: HobberScales.y().head.mounting().headParallelism, roughnessRa: 1.6}
        ], 400);
      case "GE-swivel-plate":
        // Ground plate (the faces are as ground; the Y rails seat on the front face against an edge); the holes are drilled and tapped.
        ManufacturingRoute.groundStock(GroundStock.plate(GroundStock.STEEL, 15), [
          {name: "scale housing seat, front (ground after cutting)", tolerance: HobberScales.y().housing.mounting().housingParallelism, roughnessRa: 1.6},
          {name: "scale head relief, front, 3 mm deep (milled after grinding)", tolerance: 0.1, roughnessRa: 3.2},
          {name: "adapter ring seat, back (ground after cutting)", tolerance: 0.05, roughnessRa: 1.6}
        ]);
      case "GE-swivel-adapter":
        ManufacturingRoute.turned(2, "fine", [
          {name: "output flange face (bore and ends turned together)", tolerance: 0.02, roughnessRa: 1.6},
          {name: "plate end face, parallel to the flange face", tolerance: 0.02, roughnessRa: 1.6}
        ]);
      case "GE-motor-cradle":
        ManufacturingRoute.milled(2, "fine", [
          {name: "motor flange face, front (square to the shaft bore; pilot register)", tolerance: 0.02, roughnessRa: 1.6},
          {name: "floor underside, on the seat block's top", tolerance: 0.02, roughnessRa: 1.6},
          {name: "rest block top, ground flat (the shim pack's seat under the servo body)", tolerance: 0.02, roughnessRa: 1.6}
        ], 180);
      case "GE-spindle-seat":
        ManufacturingRoute.milled(2, "tight", [
          {name: "spindle bore, clamp slit closed at the screw torque (cylindricity)", tolerance: 0.01, roughnessRa: 0.8},
          {name: "back face, square to the bore", tolerance: 0.02, roughnessRa: 1.6}
        ], 160);
      default: null;
    };
  }

  /** The holes this design carries, in the world frame the geometry is authored in: the drive seats' tapped holes and dowels, the nut block's carriage bolts. Empty for designs without any. */
  public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> {
    return switch designId {
      case "GE-X-carriage": derivedHoles();
      case "GE-Z-saddle": derivedHoles().concat(erobPilotBore());
      case "GE-Y-saddle" | "GE-outboard-carrier": derivedHoles();
      case "GE-swivel-plate": derivedHoles().concat(shifted(HobberScales.seatHoles(HobberScales.y(), SWIVEL_PLATE_THICKNESS), Z_SADDLE_ORIGIN_Z));
      case "GE-swivel-adapter": derivedHoles();
      case "GE-spindle-seat": derivedHoles().concat(seatHoles(false)).concat(cradleHoles(true));
      case "GE-motor-cradle": derivedHoles().concat(cradleHoles(false));
      case "GE-X-scale-seat": HobberScales.seatHoles(HobberScales.x(), HobberScales.SEAT_BAR);
      case "GE-Z-scale-seat": HobberScales.seatHoles(HobberScales.z(), HobberScales.SEAT_BAR);
      case "GE-Z-scale-arm": derivedHoles();
      case "GE-X-drive-pad": derivedHoles();
      case "GE-Z-drive-pad": derivedHoles();
      case "GE-Y-drive-fixed": derivedHoles();
      case "GE-Y-drive-floating": derivedHoles();
      case "GE-Y-nut-seat": derivedHoles();
      default: [];
    };
  }

  override public function geometry(detail:ComponentDetail = Preview):Part return machined(Solids.union(pieces()));
  function pieces():Array<Part> {
    return switch designId {
      case "GE-X-carriage":
        // Ground plate; separate bolted ribs carry their own joints and scale seat.
        var cuts = [box(55, 140, -150, 150, 259, 281)];
        var block = machinekit.motion.LinearRailBlock.metric("HGH25CA");
        var layer = HobberCarriageStockCheck.padLayer().nominalMm;
        for (side in [-1, 1]) for (row in [HobberXLayout.BLOCK_FRONT, HobberXLayout.BLOCK_REAR]) {
          var y = side * FoundationLayout.BED_RAIL_CENTRE;
          cuts.push(box(row - block.spec.blockLength / 2, row + block.spec.blockLength / 2,
            y - block.spec.blockWidth / 2, y + block.spec.blockWidth / 2, xCarriageBottom() - 1, xCarriageBottom() + layer));
        }
        [Solids.cut(box(60, 386, -xCarriageHalfWidth(), xCarriageHalfWidth(), xCarriageBottom(), xCarriageTop()), cuts)];
      case "GE-Z-saddle":
        // One milled plate between the swivel seat (x 135) and the block faces (x 148), with the nut block's bolts.
        // The eRob's pocket opens in the back face (x 148); the web in front of the housing carries its 16 screws and the output bore.
        [Solids.cut(box(135, 148, -175, 175, -65, 170), [HobberSwivelLayout.housingCutout()])];
      case "GE-swivel-plate":
        // The Y scale housing's seat is the front face (x 85), relieved 3 mm beside it for the head; its end screws are tapped into the plate. Ground plate, 15 thick.
        // A 3 mm relief in the front face under the head's path leaves the head room to run beside the housing at the housing's seat plane (the head is as thick as the housing).
        var relief = HobberScales.reliefBox(HobberScales.y());
        [withHoles(Solids.cut(box(85, 100, -115, 115, -95, 95), [box(relief[0], relief[1], relief[2], relief[3], relief[4] - Z_SADDLE_ORIGIN_Z, relief[5] - Z_SADDLE_ORIGIN_Z)]),
          shifted(HobberScales.seatHoles(HobberScales.y(), SWIVEL_PLATE_THICKNESS), Z_SADDLE_ORIGIN_Z))];
      case "GE-swivel-adapter":
        // Turned ring from the output flange (x 124) to the swivel plate (x 100): Ø110, Ø40 through, M4 clearance holes for the actuator and M5 tapped into the plate end.
        [Solids.cut(boreX(HobberSwivelLayout.RING_OUTSIDE / 2, HobberSwivelLayout.PLATE_X, HobberSwivelLayout.OUTPUT_X, 0, 0),
          [boreX(HobberSwivelLayout.RING_INSIDE / 2, HobberSwivelLayout.PLATE_X - 1, HobberSwivelLayout.OUTPUT_X + 1, 0, 0)])];
      case "GE-spindle-seat":
        [spindleSeat()];
      case "GE-motor-cradle":
        [motorCradle()];
      case "GE-X-scale-seat":
        [withHoles(padBox(barBox(HobberScales.x())), HobberScales.seatHoles(HobberScales.x(), HobberScales.SEAT_BAR))];
      case "GE-Z-scale-seat":
        [withHoles(padBox(barBox(HobberScales.z())), HobberScales.seatHoles(HobberScales.z(), HobberScales.SEAT_BAR))];
      case "GE-Z-scale-arm":
        // Milled arm from the Z saddle's back face (x 148) into the column channel, 12 thick across it; it carries the head. The housing stands beside the head with its free
        // face in the head's carrier plane, so the arm bridges over it a running gap above that plane and steps down to the plane only at the head (a pad as wide as the head's).
        var z = HobberScales.z(), face = HobberScales.headFace(z), hb = HobberScales.carrierBox(z, HobberScales.HEAD_CARRIER_MARGIN);
        var toward = -HobberScales.zWallSign(), gap = HobberScales.RUNNING_CLEARANCE, thick = HobberScales.Z_ARM_THICKNESS;
        function band(from:Float, to:Float):Array<Float> return toward > 0 ? [face[1] + from, face[1] + to] : [face[1] - to, face[1] - from];
        var bridge = band(gap, gap + thick), pad = band(0, gap + thick), zr = [hb[4] - Z_SADDLE_ORIGIN_Z, hb[5] - Z_SADDLE_ORIGIN_Z];
        [Solids.union([box(148, hb[1], bridge[0], bridge[1], zr[0], zr[1]), box(hb[0], hb[1], pad[0], pad[1], zr[0], zr[1])])];
      case "GE-outboard-carrier":
        [Solids.cut(box(-45, 45, -115, -85, -55, 55), [box(-30, 30, -115.1, -84.9, -30, 30)])];
      case "GE-Y-saddle":
        // A 2.3 mm boss on the back face (x 57) is the head's seat; the head's screws go through it and the plate.
        var yHead = HobberScales.y(), yFace = HobberScales.headFace(yHead), yBox = HobberScales.carrierBox(yHead, HobberScales.HEAD_CARRIER_MARGIN);
        [
          Solids.cut(radialPlate(
            45,
            57,
            [
              {y: -130.0, z: -40.0},
              {y: -65.0, z: -75.0},
              {y: 65.0, z: -75.0},
              {y: 90.0, z: -55.0},
              {y: 90.0, z: 95.0},
              {y: -130.0, z: 95.0}
            ]
          ), [boreY(Bt30SpindleData.FLANGE_DIAMETER / 2 + 3, -132, HobberSwivelLayout.SPINDLE_Y - Bt30SpindleData.FLANGE_FROM + 2, 0, 0)]),
          box(57, yFace[0], -130, 72, yBox[4] - Z_SADDLE_ORIGIN_Z, yBox[5] - Z_SADDLE_ORIGIN_Z),
          box(57, yFace[0], yBox[2], yBox[3], yBox[4] - Z_SADDLE_ORIGIN_Z, yBox[5] - Z_SADDLE_ORIGIN_Z),
          // The plate continues under the spindle seat block (y 70-230) and its screws.
          box(45, 57, 90, SEAT_Y1, SEAT_LOW + 2, 95)
        ];
      case "GE-X-drive-pad":
        [drivePad(HobberDrives.x(), 15, false)];
      case "GE-Z-drive-pad":
        [drivePad(HobberDrives.z(), 40, true)];
      case "GE-Y-drive-fixed" | "GE-Y-drive-floating":
        // An L bracket milled in one setup: the 10 mm wall carrying the support (seat plane x 45), a 10 mm bridge 5 mm over the Y
        // saddle's top edge (z 458) and a foot that bolts to the swivel plate's top edge (x 85-100). The floating one stops 2 mm
        // short of the nut seat bar's travel (y -57), the fixed one clears its +35.
        var support = designId == "GE-Y-drive-fixed" ? "fixed" : "floating";
        var holes = HobberDrives.seatHoles(HobberDrives.y(), support), lo = 1e9, hi = -1e9;
        for (h in holes) {
          lo = Math.min(lo, h.position[1]);
          hi = Math.max(hi, h.position[1]);
        }
        var y0 = support == "fixed" ? lo - 18 : -114.6, y1 = support == "fixed" ? hi + 18 : -55.4;
        [
          support == "fixed" ? Solids.cut(box(35, 45, y0, y1, 463, 548), [padBox(HobberDrives.mountRelief(HobberDrives.y()))]) : box(35, 45, y0, y1, 463, 548),
          box(35, 100, y0, y1, 463, 473),
          box(85, 100, y0, y1, 458, 473)
        ];
      case "GE-Y-nut-seat":
        // Ground bar, 16 thick, 42 x 60, on the Y saddle's top edge (x 45-57, z 458) under the nut block, 3 mm short of the swivel plate.
        // Two ribs stand KEY_HEIGHT proud of the seat's top either side of the nut block's base (y -17.5..2.5, the block's 20 mm thickness): the key.
        var block = HobberDrives.nutBlock(HobberDrives.y(), new machinekit.motion.BallNut());
        var y0 = F.transformPoint(HobberDrives.nutBlockPose(HobberDrives.y(), new machinekit.motion.BallNut()), 0, 0, 0).y, top = 458 + NUT_SEAT_THICKNESS;
        [box(40, 82, -40, 20, 458, top), box(40, 82, y0 - KEY_THICKNESS, y0, top, top + KEY_HEIGHT), box(40, 82, y0 + block.thickness, y0 + block.thickness + KEY_THICKNESS, top, top + KEY_HEIGHT)];
      case "GE-chip-tray":
        // Removable stationary skirt: 5 mm radial face clearance, 15 mm housing
        // clearance and 2 mm sheet. The lower skirt covers the full table height.
        var lip = parameterNumber("tableFaceDiameter") / 2 + 5;
        var skirt = Math.max(parameterNumber("tableHousingDiameter") / 2 + 15, lip + 5);
        var top = parameterNumber("tableFaceHeight") - 3, bottom = parameterNumber("tableSeatHeight");
        var shoulder = Math.max(bottom + 5, top - 61);
        var front = FoundationLayout.WORK_WELL_CENTRE_X
          - FoundationLayout.WORK_WELL_WIDTH / 2 + FoundationLayout.CHIP_TRAY_WELL_CLEARANCE_MM;
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
              box(front, 90, -170, 170, 135, 139),
              box(front, 90, -174, -170, 135, 164),
              box(front, 90, 170, 174, 135, 164)
            ]),
            [box(-skirt, skirt, -skirt, skirt, 134, 165)]
          )
        ];
      case "GE-coolant":
        [Solids.cut(box(33, 45, 59.2, 69.5, 60, 84), [boreX(2, 32.9, 45.1, 64.35, 72)])];
      case "GE-enclosure":
        [box(-270, 780, -360, -356, 0, 950), box(-270, 780, 356, 360, 0, 950), box(776, 780, -360, 360, 0, 950)];
      case "electrical-panel":
        var parts = [box(-210, 210, -10, 0, 0, 850), box(-230, 230, -150, 50, 0, 20)];
        // Two welded triangular webs tie the upright to its floor base.
        for (x in [-190.0, 190.0]) {
          var web = Part.prism([new Vector(0, 20), new Vector(-130, 20), new Vector(0, 200)], -3, 3);
          var placed = PosedParts.moved(web, F.fromRotationMatrix(x, 0, 0, [0, 0, 1, 1, 0, 0, 0, 1, 0]));
          web.close();
          parts.push(placed);
        }
        parts;
      default:
        throw "unknown hobber saddle design";
    };
  }
}
