package machinekit.hobbing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.ManufacturingRoute.MachinedFace;
import machinekit.structural.GroundStock;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Separate designed weldments, with machined mounting datums. Dimensions are mm.
    Strength, weld distortion and modal stiffness remain assumed pending Phase D. */
class HobberSaddle extends MachineComponent {
  final designId:String;
  static var recipe:Null<ComponentType>;
  public static function parameters():Array<ComponentParameter> return [
    P.choice(
      "design",
      [
        "GE-X-carriage",
        "GE-Z-saddle",
        "GE-X-drive-fixed",
        "GE-X-drive-floating",
        "GE-Z-drive-fixed",
        "GE-Z-drive-floating",
        "GE-Y-drive-fixed",
        "GE-Y-drive-floating",
        "GE-Y-nut-seat",
        "GE-chip-tray",
        "GE-enclosure",
        "GE-coolant",
        "GE-swivel-plate",
        "GE-Y-saddle",
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
    if (recipe == null) recipe = new ComponentType(
      "machinekit.hobbing.saddle",
      parameters(),
      v -> new HobberSaddle(v.token("design"), v)
    ).numbered(v -> 'HOBBER-${v.token("design").toUpperCase()}')
      .procured(_ -> Make("steel fabrication and machining"))
      .requiringInputs(["design", "tableFaceDiameter", "tableHousingDiameter", "tableFaceHeight", "tableSeatHeight"]);
    return recipe;
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
    addConnector("datum", Mount, F.identity());
    var route = routeOf(design);
    if (route != null) {
      addFacet(route);
      addFacet(new ProcurementIntent(Make(route.process())));
    }
    if (["GE-X-carriage", "GE-Z-saddle", "GE-X-drive-fixed", "GE-X-drive-floating", "GE-Z-drive-fixed", "GE-Z-drive-floating",
      "GE-Y-saddle", "GE-Y-drive-fixed", "GE-Y-drive-floating", "GE-Y-nut-seat"].indexOf(design) >= 0) {
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
  static inline final PAD_WIDTH = 80.0;

  /** A hole's tool: a cylinder of `diameter` from `position` along `direction`, `depth` long, starting 1 mm outside the mouth. */
  static function holeTool(position:Array<Float>, direction:Array<Float>, diameter:Float, depth:Float):Part {
    var cylinder = Part.cylinderSpan(diameter / 2, -1, depth);
    var up = Math.abs(direction[2]) < 0.9 ? [0.0, 0, 1] : [1.0, 0, 0];
    var placed = PosedParts.moved(cylinder, machinekit.assembly.AxisBuilder.orient(position[0], position[1], position[2], up, direction));
    cylinder.close();
    return placed;
  }

  /** `body` with the drive's seat holes (tapped to their drilled depth, dowels reamed) cut in. */
  static function withSeatHoles(body:Part, axis:HobberDriveAxis, support:String):Part {
    return Solids.cut(body, [for (h in HobberDrives.seatHoles(axis, support)) holeTool(h.position, h.direction, h.diameter, h.depth)]);
  }

  /** `body` with clear holes for the nut block's two carriage bolts, `through` mm deep from the block's base end. */
  static function withBlockBolts(body:Part, axis:HobberDriveAxis, through:Float):Part {
    var nut = new machinekit.motion.BallNut(), block = HobberDrives.nutBlock(axis, nut);
    return Solids.cut(body, [for (h in HobberDrives.nutBlockHoles(axis, nut, block)) holeTool(h.position, h.direction, h.diameter, through)]);
  }

  static function padBox(b:Array<Float>):Part return box(b[0], b[3], b[1], b[4], b[2], b[5]);

  /** A fixed pad: its box with the mount's flange relief and the seat holes. */
  static function fixedPad(axis:HobberDriveAxis, thickness:Float):Part {
    var relieved = Solids.cut(padBox(HobberDrives.padBounds(axis, "fixed", PAD_WIDTH, thickness)), [padBox(HobberDrives.mountRelief(axis))]);
    return withSeatHoles(relieved, axis, "fixed");
  }

  /** How each made design is produced; null for the designs that are not part of the moving structure. */
  static function routeOf(design:String):Null<ManufacturingRoute> {
    function bar(thickness:Float, faces:Array<MachinedFace>):ManufacturingRoute return ManufacturingRoute.groundStock(GroundStock.bar(PAD_WIDTH, thickness), faces);
    return switch design {
      case "GE-X-carriage":
        ManufacturingRoute.milled(2, "fine", [
          {name: "block pads, underside", tolerance: 0.02, roughnessRa: 1.6},
          {name: "column sole, top", tolerance: 0.02, roughnessRa: 1.6}
        ], 490);
      case "GE-Z-saddle":
        ManufacturingRoute.milled(2, "fine", [
          {name: "block faces, back", tolerance: 0.02, roughnessRa: 1.6},
          {name: "swivel seat, front", tolerance: 0.02, roughnessRa: 1.6}
        ], 350);
      case "GE-X-drive-fixed" | "GE-X-drive-floating":
        bar(15, [{name: "seat holes", tolerance: 0.1, roughnessRa: null}]);
      case "GE-Z-drive-fixed" | "GE-Z-drive-floating":
        bar(40, [{name: "seat holes", tolerance: 0.1, roughnessRa: null}]);
      case "GE-Y-drive-fixed" | "GE-Y-drive-floating":
        ManufacturingRoute.milled(1, "fine", [{name: "seat face, bolts and dowels", tolerance: 0.05, roughnessRa: 3.2}], 160);
      case "GE-Y-nut-seat":
        ManufacturingRoute.groundStock(GroundStock.bar(60, 20), [{name: "nut block seat", tolerance: 0.02, roughnessRa: 1.6}]);
      default: null;
    };
  }

  override public function geometry(detail:ComponentDetail = Preview):Part return Solids.union(pieces());
  function pieces():Array<Part> {
    return switch designId {
      case "GE-X-carriage":
        // One milled part: a 20 mm deck between the block plane (260) and the column sole plane (280), open at the front over the
        // swivel stack and the head's swing (x 60-140, 300 wide; a front bar would meet the Y guides at A +-15 deg, Y and Z at -10); 25 mm deep ribs under the deck run
        // beside the rails (y +-120..140 and +-160..180), clear of the blocks (y 196..244) and the bed.
        [
          withBlockBolts(Solids.cut(box(60, 386, -245, 245, 260, 280), [box(55, 140, -150, 150, 259, 281)]), HobberDrives.x(), 40),
          box(140, 386, -140, -120, 235, 260),
          box(140, 386, 120, 140, 235, 260),
          box(140, 386, -180, -160, 235, 260),
          box(140, 386, 160, 180, 235, 260)
        ];
      case "GE-Z-saddle":
        // One milled plate between the swivel seat (x 135) and the block faces (x 148), with the nut block's bolts.
        [withBlockBolts(box(135, 148, -175, 175, -65, 170), HobberDrives.z(), 30)];
      case "GE-swivel-plate":
        [box(85, 100, -115, 115, -95, 95)];
      case "GE-Y-saddle":
        [
          radialPlate(
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
          ),
          box(45, 57, 90, 240, 55, 95),
          Solids.cut(box(-50, 57, 80, 100, -50, 50), [boreY(40, 79.9, 100.1, 0, 0)]),
          Solids.cut(box(-50, 35, 200, 220, -50, 50), [boreY(40, 199.9, 220.1, 0, 0)]),
          box(35, 57, 200, 220, 40, 55),
          Solids.cut(box(-45, 57, -115, -85, -45, 45), [box(-30, 30, -115.1, -84.9, -30, 30)])
        ];
      case "GE-X-drive-fixed":
        [fixedPad(HobberDrives.x(), 15)];
      case "GE-X-drive-floating":
        [withSeatHoles(padBox(HobberDrives.padBounds(HobberDrives.x(), "floating", PAD_WIDTH, 15)), HobberDrives.x(), "floating")];
      case "GE-Z-drive-fixed":
        [fixedPad(HobberDrives.z(), 40)];
      case "GE-Z-drive-floating":
        [withSeatHoles(padBox(HobberDrives.padBounds(HobberDrives.z(), "floating", PAD_WIDTH, 40)), HobberDrives.z(), "floating")];
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
        var y0 = support == "fixed" ? lo - 18 : -113.0, y1 = support == "fixed" ? hi + 18 : -57.0;
        [
          withSeatHoles(support == "fixed" ? Solids.cut(box(35, 45, y0, y1, 463, 548), [padBox(HobberDrives.mountRelief(HobberDrives.y()))]) : box(35, 45, y0, y1, 463, 548), HobberDrives.y(), support),
          box(35, 100, y0, y1, 463, 473),
          box(85, 100, y0, y1, 458, 473)
        ];
      case "GE-Y-nut-seat":
        // Ground bar, 20 thick, 42 x 60, on the Y saddle's top edge (x 45-57, z 458) under the nut block, 3 mm short of the swivel plate.
        [box(40, 82, -40, 20, 458, 478)];
      case "GE-chip-tray":
        // Removable stationary skirt: 5 mm radial face clearance, 15 mm housing
        // clearance and 2 mm sheet. The lower skirt covers the full table height.
        var lip = parameterNumber("tableFaceDiameter") / 2 + 5;
        var skirt = Math.max(parameterNumber("tableHousingDiameter") / 2 + 15, lip + 5);
        var top = parameterNumber("tableFaceHeight") - 3, bottom = parameterNumber("tableSeatHeight") + 2;
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
        [Solids.cut(box(33, 45, 30, 50, 60, 84), [boreX(2, 32.9, 45.1, 40, 72)])];
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
