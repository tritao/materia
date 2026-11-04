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
    // The adapter ring turns with the head: light alloy.
    if (design == "GE-swivel-adapter") setMaterial("aluminium 6061");
    addConnector("datum", Mount, F.identity());
    var route = routeOf(design);
    if (route != null) {
      addFacet(route);
      addFacet(route.intent());
    }
    if (["GE-X-carriage", "GE-Z-saddle", "GE-X-drive-pad", "GE-Z-drive-pad",
      "GE-Y-saddle", "GE-Y-drive-fixed", "GE-Y-drive-floating", "GE-Y-nut-seat", "GE-X-scale-seat", "GE-Z-scale-seat", "GE-Z-scale-arm"].indexOf(design) >= 0) {
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

  /** `body` with the drive's seat holes (tapped to their drilled depth, dowels reamed) cut in. */
  static function withSeatHoles(body:Part, axis:HobberDriveAxis, support:String):Part {
    return Solids.cut(body, [for (h in HobberDrives.seatHoles(axis, support)) holeTool(h.position, h.direction, h.diameter, h.depth)]);
  }

  /** Where the Z saddle's own frame sits in the work frame (z), mm: `FlexsplineHobber` places it at z 363. The drive's holes are in the work frame. */
  static inline final Z_SADDLE_ORIGIN_Z = HobberZLayout.SEAT_Z;
  /** The head screws' clearance holes run through the X carriage's rib and deck, and through the Y saddle's boss and plate, mm. */
  static inline final X_HEAD_HOLE_DEPTH = 34.3;
  static inline final Y_HEAD_HOLE_DEPTH = 14.3;

  /** `body` with clear holes for the nut block's two carriage bolts, `through` mm deep from the block's base end; `origin` is the part frame's origin in the work frame. */
  static function withBlockBolts(body:Part, axis:HobberDriveAxis, through:Float, ?origin:Array<Float>):Part {
    var nut = new machinekit.motion.BallNut(), block = HobberDrives.nutBlock(axis, nut), at = origin ?? [0.0, 0, 0];
    return Solids.cut(body, [for (h in HobberDrives.nutBlockHoles(axis, nut, block))
      holeTool([h.position[0] - at[0], h.position[1] - at[1], h.position[2] - at[2]], h.direction, h.diameter, through)]);
  }

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

  /** The head of the scale of `axis` rests on this face: a clearance hole for each of its two screws, `depth` mm into the carrier. */
  static function headHoles(axis:HobberScaleAxis, depth:Float, originZ:Float):Array<machinekit.component.ManufacturingRoute.RouteHole> return shifted(HobberScales.carrierHoles(axis, depth), originZ);

  /** The seat bar under a scale housing as a box (it is authored in the work frame, so no origin shift). */
  static function barBox(axis:HobberScaleAxis):Array<Float> {
    var b = axis.seatBar;
    if (b == null) throw "Scale " + axis.id + " has no seat bar";
    return [b[0], b[2], b[4], b[1], b[3], b[5]];
  }

  static function padBox(b:Array<Float>):Part return box(b[0], b[3], b[1], b[4], b[2], b[5]);

  // ---- A axis and spindle seat (FRAME_PLAN R1, R2) ------------------------------------------------------------------------------

  /** The adapter ring's screws into the eRob output flange: 12 clearance holes, the flange's pitch circle, in the ring's frame (x along the axis). */
  static function ringOutputHoles():Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var length = HobberSwivelLayout.OUTPUT_X - HobberSwivelLayout.PLATE_X, d = machinekit.motion.ErobActuatorData.OUTPUT_CLEARANCE_HOLE;
    return [for (i in 0...machinekit.motion.ErobActuatorData.OUTPUT_BOLT_COUNT) {
      var b = machinekit.motion.ErobActuatorPart.outputBolt(i);
      // The actuator's CAD x is the work frame's y, its y is -z.
      ({id: "O" + (i + 1), kind: "clearance", thread: "M4", diameter: d, nominal: 4, fit: null, depth: length,
        position: [HobberSwivelLayout.OUTPUT_X, b.x, -b.y], direction: [-1.0, 0, 0]} : machinekit.component.ManufacturingRoute.RouteHole);
    }].concat([for (i in 0...machinekit.motion.ErobActuatorData.OUTPUT_BOLT_COUNT) {
      // The screws go in from the plate end (the flange is tapped), their heads sunk so the swivel plate can sit on the ring.
      var b = machinekit.motion.ErobActuatorPart.outputBolt(i);
      ({id: "O" + (i + 1) + "c", kind: "bore", thread: null, diameter: 7.5, nominal: 7.5, fit: null, depth: 4.5,
        position: [HobberSwivelLayout.PLATE_X, b.x, -b.y], direction: [1.0, 0, 0]} : machinekit.component.ManufacturingRoute.RouteHole);
    }]);
  }

  static function platePositions():Array<{y:Float, z:Float}> {
    var r = HobberSwivelLayout.PLATE_PCD / 2, n = HobberSwivelLayout.PLATE_SCREWS;
    return [for (i in 0...n) {y: r * Math.cos(2 * Math.PI * i / n), z: r * Math.sin(2 * Math.PI * i / n)}];
  }

  /** The ring's M5 holes into its plate end (tapped), in the ring's frame. */
  static function ringPlateHoles():Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var drill = machinekit.standard.SocketHeadCapScrew.catalog().get("M5").tapDrill, n = 0;
    return [for (p in platePositions()) ({id: "P" + (++n), kind: "tapped", thread: "M5", diameter: drill, nominal: 5, fit: null, depth: 12.0,
      position: [HobberSwivelLayout.PLATE_X, p.y, p.z], direction: [1.0, 0, 0]} : machinekit.component.ManufacturingRoute.RouteHole)];
  }

  /** The swivel plate's clearance holes for those screws, in the plate's frame. */
  static function platePlateHoles():Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var n = 0;
    return [for (p in platePositions()) ({id: "A" + (++n), kind: "clearance", thread: "M5", diameter: 5.5, nominal: 5, fit: null, depth: 15.0,
      position: [HobberSwivelLayout.PLATE_X, p.y, p.z], direction: [-1.0, 0, 0]} : machinekit.component.ManufacturingRoute.RouteHole)];
  }

  /** The Z saddle's eRob features that are plain holes: the screw clearance holes and the output bore, in the saddle's frame (the pocket is a drawing note). */
  static function erobSaddleHoles():Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var web = HobberSwivelLayout.OUTPUT_X - machinekit.motion.ErobActuatorPart.SEAT_Z - HobberSwivelLayout.PLATE_FRONT, list:Array<machinekit.component.ManufacturingRoute.RouteHole> = [
      {id: "P1", kind: "bore", thread: null, diameter: machinekit.motion.ErobActuatorData.OUTPUT_PILOT_DIAMETER + machinekit.motion.ErobActuatorPart.PILOT_BORE_CLEARANCE,
        nominal: machinekit.motion.ErobActuatorData.OUTPUT_PILOT_DIAMETER + machinekit.motion.ErobActuatorPart.PILOT_BORE_CLEARANCE, fit: null,
        depth: web, position: [HobberSwivelLayout.PLATE_FRONT, 0, 0], direction: [1.0, 0, 0]}];
    var screw = machinekit.standard.SocketHeadCapScrew.metric("M4", 10);
    for (i in 0...machinekit.motion.ErobActuatorData.FIXED_BOLT_COUNT) {
      var b = machinekit.motion.ErobActuatorPart.fixedBolt(i);
      list.push({id: "E" + (i + 1), kind: "clearance", thread: "M4", diameter: screw.clearanceDiameter(), nominal: 4, fit: null, depth: web,
        position: [HobberSwivelLayout.PLATE_FRONT, b.x, -b.y], direction: [1.0, 0, 0]});
    }
    return list;
  }

  /** Spindle seat block: x from the bore's front side to the saddle plate, y along the cartridge, z across. */
  static inline final SEAT_X0 = -48.0;
  static inline final SEAT_X1 = 45.0;
  static inline final SEAT_Y0 = 70.0;
  static inline final SEAT_Y1 = 222.0;
  /** Half height on the +z side (the slit and clamp screws); the -z side is cut back to `SEAT_LOW` so the head clears the X rail at A +15 deg. */
  static inline final SEAT_HALF = 58.0;
  static inline final SEAT_LOW = -50.0;
  static inline final SEAT_BORE = 80.0;
  static inline final SEAT_SLIT = 2.0;
  static inline final SEAT_CLAMP_Z = 49.5;

  /** The seat's holes in its own frame (and the matching ones in the Y saddle): 4 M8 into its back face, 2 dowels, 3 clamp screws M6 across the slit. */
  static function seatHoles(inSaddle:Bool):Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var m8 = machinekit.standard.SocketHeadCapScrew.catalog().get("M8").tapDrill, m6 = machinekit.standard.SocketHeadCapScrew.catalog().get("M6").tapDrill;
    var list:Array<machinekit.component.ManufacturingRoute.RouteHole> = [], n = 0, back = SEAT_X1;
    for (y in [95.0, 205.0]) for (z in [-40.0, 49.0]) {
      n++;
      if (inSaddle) list.push({id: "S" + n, kind: "clearance", thread: "M8", diameter: 9.0, nominal: 8, fit: null, depth: 12.0,
        position: [57.0, y, z], direction: [-1.0, 0, 0]});
      else list.push({id: "T" + n, kind: "tapped", thread: "M8", diameter: m8, nominal: 8, fit: null, depth: 16.0,
        position: [back, y, z], direction: [-1.0, 0, 0]});
    }
    n = 0;
    for (z in [-35.0, 35.0]) {
      n++;
      list.push({id: "D" + n, kind: "dowel", thread: null, diameter: 8.0, nominal: 8.0, fit: "H7", depth: inSaddle ? 12.0 : 12.0,
        position: inSaddle ? [57.0, 150.0, z] : [back, 150.0, z], direction: [-1.0, 0, 0]});
    }
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

  /** The spindle seat's extent along the spindle axis (work y) and the y of its three clamp screws. */
  public static function spindleSeatRange():{y0:Float, y1:Float, clampYs:Array<Float>} return {y0: SEAT_Y0, y1: SEAT_Y1, clampYs: [100.0, 150.0, 200.0]};

  /** The split-clamp block that holds the Ø80 cartridge: bore, slit up the +z side and the holes of `seatHoles`. */
  static function spindleSeat():Part {
    var block = box(SEAT_X0, SEAT_X1, SEAT_Y0, SEAT_Y1, SEAT_LOW, SEAT_HALF);
    var cuts = [boreY(SEAT_BORE / 2, SEAT_Y0 - 0.1, SEAT_Y1 + 0.1, 0, 0), box(-SEAT_SLIT / 2, SEAT_SLIT / 2, SEAT_Y0 - 0.1, SEAT_Y1 + 0.1, 0, SEAT_HALF + 0.1)];
    return withHoles(Solids.cut(block, cuts), seatHoles(false));
  }

  /** The seat holes of both supports on the one pad, ids numbered per support (F for the fixed support, B for the floating one). */
  static function padRouteHoles(axis:HobberDriveAxis):Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var result = [];
    for (support in ["fixed", "floating"]) for (h in HobberDrives.seatRouteHoles(axis, support)) {
      h.id = (support == "fixed" ? "F" : "B") + h.id;
      result.push(h);
    }
    return result;
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
    var relieved = Solids.cut(padBox(HobberDrives.padBoundsBoth(axis, PAD_WIDTH, thickness)), cutters);
    for (c in cutters) c.close();
    var fixed = Solids.cut(relieved, [for (h in HobberDrives.seatHoles(axis, "fixed")) holeTool(h.position, h.direction, h.diameter, h.depth)]);
    return withSeatHoles(fixed, axis, "floating");
  }

  /** How each made design is produced; null for the designs that are not part of the moving structure. */
  static function routeOf(design:String):Null<ManufacturingRoute> {
    function bar(thickness:Float, faces:Array<MachinedFace>):ManufacturingRoute return ManufacturingRoute.groundStock(GroundStock.bar(PAD_WIDTH, thickness), faces);
    return switch design {
      case "GE-X-carriage":
        ManufacturingRoute.milled(2, "fine", [
          {name: "block pads, underside", tolerance: 0.02, roughnessRa: 1.6},
          {name: "column sole, top", tolerance: 0.02, roughnessRa: 1.6},
          {name: "scale head seat, rib underside", tolerance: HobberScales.x().head.mounting().headParallelism, roughnessRa: 1.6}
        ], 490);
      case "GE-Z-saddle":
        ManufacturingRoute.milled(2, "fine", [
          {name: "block faces, back", tolerance: 0.02, roughnessRa: 1.6},
          {name: "eRob seat: output bore, front face web and pocket", tolerance: 0.05, roughnessRa: 1.6}
        ], 350);
      case "GE-X-drive-pad":
        // One bar under both supports: the grinding makes both seats coplanar, and it stays the re-grindable height spacer.
        bar(15, [{name: "seat plane under both supports (ground together)", tolerance: 0.01, roughnessRa: 1.6}, {name: "seat holes", tolerance: 0.1, roughnessRa: null},
          {name: "groove for the nut block's travel (block + 2.5 mm each way)", tolerance: 0.5, roughnessRa: null}]);
      case "GE-Z-drive-pad":
        bar(40, [{name: "seat plane under both supports (ground together)", tolerance: 0.01, roughnessRa: 1.6}, {name: "seat holes", tolerance: 0.1, roughnessRa: null},
          {name: "groove for the nut block's travel (block + 2.5 mm each way)", tolerance: 0.5, roughnessRa: null},
          {name: "relief for the A housing envelope (swept over Z, 2.5 mm gap)", tolerance: 0.5, roughnessRa: null}]);
      case "GE-Y-drive-fixed" | "GE-Y-drive-floating":
        ManufacturingRoute.milled(1, "fine", [{name: "seat face, bolts and dowels", tolerance: 0.05, roughnessRa: 3.2}], 160);
      case "GE-Y-nut-seat":
        ManufacturingRoute.groundStock(GroundStock.bar(60, 20), [{name: "nut block seat", tolerance: 0.02, roughnessRa: 1.6}]);
      case "GE-X-scale-seat" | "GE-Z-scale-seat":
        // Ground bar: the housing's mounting face parallelism comes from the datasheet (0.1 over the length), the grinding gives 0.01.
        ManufacturingRoute.groundStock(GroundStock.bar(HobberScales.SEAT_BAR_WIDTH, HobberScales.SEAT_BAR),
          [{name: "housing seat, tapped holes", tolerance: HobberScales.x().housing.mounting().housingParallelism, roughnessRa: null}]);
      case "GE-Z-scale-arm":
        ManufacturingRoute.milled(1, "fine", [{name: "scale head seat, face", tolerance: HobberScales.z().head.mounting().headParallelism, roughnessRa: 1.6}], 230);
      case "GE-Y-saddle":
        ManufacturingRoute.milled(2, "fine", [
          {name: "block faces, back", tolerance: 0.02, roughnessRa: 1.6},
          {name: "scale head seat, boss", tolerance: HobberScales.y().head.mounting().headParallelism, roughnessRa: 1.6}
        ], 400);
      case "GE-swivel-plate":
        // Ground plate (the faces are as ground; the Y rails seat on the front face against an edge); the holes are drilled and tapped.
        ManufacturingRoute.groundStock(GroundStock.plate(15), [
          {name: "scale housing seat, front", tolerance: HobberScales.y().housing.mounting().housingParallelism, roughnessRa: 1.6},
          {name: "adapter ring seat, back", tolerance: 0.05, roughnessRa: 1.6}
        ]);
      case "GE-swivel-adapter":
        ManufacturingRoute.turned(2, "fine", [
          {name: "output flange face (bore and ends turned together)", tolerance: 0.02, roughnessRa: 1.6},
          {name: "plate end face, parallel to the flange face", tolerance: 0.02, roughnessRa: 1.6}
        ]);
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
    function bolts(axis:HobberDriveAxis, through:Float, at:Array<Float>) {
      var nut = new machinekit.motion.BallNut(), block = HobberDrives.nutBlock(axis, nut), n = 0;
      return [for (h in HobberDrives.nutBlockHoles(axis, nut, block)) ({
        id: "B" + (++n), kind: "clearance", thread: machinekit.motion.NutBlock.BASE_THREAD, diameter: h.diameter, nominal: h.diameter,
        fit: null, depth: through, position: [h.position[0] - at[0], h.position[1] - at[1], h.position[2] - at[2]], direction: h.direction
      } : machinekit.component.ManufacturingRoute.RouteHole)];
    }
    return switch designId {
      case "GE-X-carriage": bolts(HobberDrives.x(), 40, [0.0, 0, 0]).concat(headHoles(HobberScales.x(), X_HEAD_HOLE_DEPTH, 0));
      case "GE-Z-saddle": bolts(HobberDrives.z(), 30, [0.0, 0, Z_SADDLE_ORIGIN_Z]).concat(erobSaddleHoles());
      case "GE-Y-saddle": headHoles(HobberScales.y(), Y_HEAD_HOLE_DEPTH, Z_SADDLE_ORIGIN_Z).concat(seatHoles(true));
      case "GE-swivel-plate": shifted(HobberScales.seatHoles(HobberScales.y(), 12), Z_SADDLE_ORIGIN_Z).concat(platePlateHoles());
      case "GE-swivel-adapter": ringOutputHoles().concat(ringPlateHoles());
      case "GE-spindle-seat": seatHoles(false);
      case "GE-X-scale-seat": HobberScales.seatHoles(HobberScales.x(), 8);
      case "GE-Z-scale-seat": HobberScales.seatHoles(HobberScales.z(), 8);
      case "GE-Z-scale-arm": headHoles(HobberScales.z(), HobberScales.Z_ARM_THICKNESS, Z_SADDLE_ORIGIN_Z);
      case "GE-X-drive-pad": padRouteHoles(HobberDrives.x());
      case "GE-Z-drive-pad": padRouteHoles(HobberDrives.z());
      case "GE-Y-drive-fixed": HobberDrives.seatRouteHoles(HobberDrives.y(), "fixed");
      case "GE-Y-drive-floating": HobberDrives.seatRouteHoles(HobberDrives.y(), "floating");
      default: [];
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
          withHoles(withBlockBolts(Solids.cut(box(60, 386, -245, 245, 260, 280), [box(55, 140, -150, 150, 259, 281)]), HobberDrives.x(), 40),
            headHoles(HobberScales.x(), X_HEAD_HOLE_DEPTH, 0)),
          box(140, 386, -140, -120, 235, 260),
          box(140, 386, 120, 140, 235, 260),
          box(140, 386, -180, -160, 235, 260),
          // The outer rib on the scale's side stands under the head: its underside is the head's seat, as deep as the scale stack leaves.
          withHoles(box(140, 386, 178 - 9.5, 178 + 9.5, HobberScales.headFace(HobberScales.x())[2], 260), headHoles(HobberScales.x(), X_HEAD_HOLE_DEPTH, 0))
        ];
      case "GE-Z-saddle":
        // One milled plate between the swivel seat (x 135) and the block faces (x 148), with the nut block's bolts.
        // The eRob's pocket opens in the back face (x 148); the web in front of the housing carries its 16 screws and the output bore.
        [Solids.cut(withBlockBolts(box(135, 148, -175, 175, -65, 170), HobberDrives.z(), 30, [0.0, 0, Z_SADDLE_ORIGIN_Z]), [HobberSwivelLayout.housingCutout()])];
      case "GE-swivel-plate":
        // The Y scale housing's seat is the front face (x 85); its end screws are tapped into the plate. Ground plate, 15 thick.
        [withHoles(box(85, 100, -115, 115, -95, 95), shifted(HobberScales.seatHoles(HobberScales.y(), 12), Z_SADDLE_ORIGIN_Z).concat(platePlateHoles()))];
      case "GE-swivel-adapter":
        // Turned ring from the output flange (x 124) to the swivel plate (x 100): Ø110, Ø40 through, M4 clearance holes for the actuator and M5 tapped into the plate end.
        [withHoles(Solids.cut(boreX(HobberSwivelLayout.RING_OUTSIDE / 2, HobberSwivelLayout.PLATE_X, HobberSwivelLayout.OUTPUT_X, 0, 0),
          [boreX(HobberSwivelLayout.RING_INSIDE / 2, HobberSwivelLayout.PLATE_X - 1, HobberSwivelLayout.OUTPUT_X + 1, 0, 0)]), ringOutputHoles().concat(ringPlateHoles()))];
      case "GE-spindle-seat":
        [spindleSeat()];
      case "GE-X-scale-seat":
        [withHoles(padBox(barBox(HobberScales.x())), HobberScales.seatHoles(HobberScales.x(), 8))];
      case "GE-Z-scale-seat":
        [withHoles(padBox(barBox(HobberScales.z())), HobberScales.seatHoles(HobberScales.z(), 8))];
      case "GE-Z-scale-arm":
        // Milled arm from the Z saddle's back face (x 148) into the column channel, 10 thick across it; it carries the head.
        var z = HobberScales.z(), face = HobberScales.headFace(z), half = z.head.series.headLength / 2 + 2;
        var reach = z.headPose.x + z.head.series.headDepth / 2 + HobberScales.HEAD_CARRIER_MARGIN;
        var zc = face[2] - Z_SADDLE_ORIGIN_Z;
        var wall = HobberScales.zWallSign(), y0 = wall > 0 ? face[1] - HobberScales.Z_ARM_THICKNESS : face[1], y1 = wall > 0 ? face[1] : face[1] + HobberScales.Z_ARM_THICKNESS;
        [withHoles(box(148, reach, y0, y1, zc - half, zc + half), headHoles(z, HobberScales.Z_ARM_THICKNESS, Z_SADDLE_ORIGIN_Z))];
      case "GE-Y-saddle":
        // A 2.3 mm boss on the back face (x 57) is the head's seat; the head's screws go through it and the plate.
        var yHead = HobberScales.y(), yFace = HobberScales.headFace(yHead), yHeadHalf = yHead.head.series.headLength / 2 + 2;
        var yHoles = headHoles(yHead, Y_HEAD_HOLE_DEPTH, Z_SADDLE_ORIGIN_Z);
        [
          withHoles(radialPlate(
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
          ), yHoles),
          withHoles(box(57, yFace[0], yFace[1] - yHeadHalf, yFace[1] + yHeadHalf, -yHead.head.series.headDepth / 2 - 2, yHead.head.series.headDepth / 2 + 2), yHoles),
          // The plate continues under the spindle seat block (y 70-230) and its screws.
          withHoles(box(45, 57, 90, SEAT_Y1, SEAT_LOW + 2, 95), seatHoles(true)),
          withHoles(Solids.cut(box(-45, 57, -115, -85, -45, 45), [box(-30, 30, -115.1, -84.9, -30, 30)]), yHoles)
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
