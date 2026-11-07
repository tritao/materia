package machinekit.hobbing;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingFeature;
import materia.casting.CastingPatterns as C;
import materia.casting.InsertSpec;
import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.CastingFeaturePatterns as Patterns;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import machinekit.structural.InsertHoles;

typedef InsertLocation = {
  var id:String;
  var part:CastInsert;
  var pose:AssemblyFrame;
  var openings:Array<CastingOpening>;
  @:optional var clampHeight:Float;
}

/** Ground corner contact, in the X-carriage/work frame; column coordinates derive from it. */
typedef ColumnSolePad = {
  var id:String;
  var side:Int;
  var front:Bool;
  var x0:Float;
  var x1:Float;
  var y0:Float;
  var y1:Float;
  var boltXs:Array<Float>;
  var boltY:Float;
}

/** Machine design intent. Exterior openings are authored here, never inferred. */
class FoundationLayout {
  public static inline final TABLE_FACE = 228.0;
  public static inline final TABLE_SEAT = TABLE_FACE - 210.0;
  public static inline final PLINTH_HEIGHT = 800.0;
  public static inline final TABLE_SEAT_HEIGHT = 608.0;
  public static inline final TABLE_WELL_FLOOR_HEIGHT = 598.0;
  public static inline final WORK_WELL_TOP = 810.0;
  public static inline final WORK_WELL_WIDTH = 398.0;
  public static inline final WORK_WELL_DEPTH = 350.0;
  public static inline final WORK_WELL_CENTRE_X = -45.0;
  public static inline final DRIVE_CHANNEL_FRONT_CLEARANCE = 12.5;
  public static inline final DRIVE_CHANNEL_FLOOR = 720.0;
  public static inline final DRIVE_CHANNEL_WIDTH = 110.0;
  public static inline final CHIP_DRAIN_DEPTH = 80.0;
  public static inline final CHIP_DRAIN_WIDTH = 180.0;
  public static inline final CHIP_TRAY_WELL_CLEARANCE_MM = 5.0;
  public static inline final RAIL_SEAT_HEIGHT = 790.0;
  public static inline final DRIVE_SEAT_HEIGHT = 710.0;
  public static inline final LIFT_SEAT_HEIGHT = 770.0;
  public static inline final LIFT_CENTRE_Y = 310.0;
  public static inline final BED_RAIL_CENTRE = 220.0;
  public static inline final COLUMN_SOLE_THICKNESS = 20.0;
  public static inline final BED_RAIL_INSERT_THICKNESS = 20.0;
  public static function bedRailSeatTop():Float return TABLE_SEAT - TABLE_SEAT_HEIGHT
    + RAIL_SEAT_HEIGHT + BED_RAIL_INSERT_THICKNESS;
  public static function columnSoleTop():Float return HobberSaddle.xCarriageTop() + COLUMN_SOLE_THICKNESS;
  public static inline final COLUMN_RAIL_CENTRE = 145.0;

  /** Keep four ground rectangles on resin-backed corner lands; the front pair
   * clear the same source recess that clears the actuator. Dimensions are
   * derived from the casting, rail centres, M8 edge/pitch and running gap. */
  public static function columnSolePads():Array<ColumnSolePad> {
    var screw = machinekit.standard.SocketHeadCapScrew.catalog().get("M8");
    if (screw == null) throw "Column sole screw unavailable";
    var outer = (dimensions("column")[0] - COLUMN_SOLE_THICKNESS) / 2;
    var depth = dimensions("column")[1] - COLUMN_SOLE_THICKNESS;
    var pitch = machinekit.assembly.JointFasteners.SPACING_DIAMETERS * screw.diameter;
    var edge = machinekit.assembly.JointFasteners.EDGE_DIAMETERS * screw.diameter + 0.5;
    // Leave two screw diameters beyond the service-hole edge allowance
    // for the separately demand-sized cast-anchor pattern (layout allowance,
    // not a supplier limit). Exact source selection verifies its capacity.
    var anchorLand = 2*screw.diameter;
    var length = Math.ceil((2*(edge+anchorLand) + pitch)/10)*10;
    var centreX = HobberZLayout.SEAT_FACE_X + HobberZLayout.SPINE_FRONT_COLUMN_Y;
    var out:Array<ColumnSolePad> = [];
    for (front in [true,false]) for (side in [-1,1]) {
      var inner = front ? Math.ceil((columnRecessOuter(side) + HobberScales.RUNNING_CLEARANCE)/5)*5
        : 2*COLUMN_RAIL_CENTRE - outer;
      var cx = centreX + (front ? -1 : 1)*(depth-length)/2;
      var y0 = side < 0 ? -outer : inner;
      var y1 = side < 0 ? -inner : outer;
      // Rear right row lies between the scale rib and the outboard
      // source anchor. Keep its underside driver corridor outside the rib.
      var rearRight = Math.ceil((outer-2*edge+HobberScales.RUNNING_CLEARANCE)*2)/2;
      var boltY = side < 0 ? -150.0 : front ? 175.0 : rearRight;
      if (inner >= outer || boltY < y0 + screw.diameter || boltY > y1 - screw.diameter)
        throw "Column corner land cannot carry its derived bolt pattern";
      out.push({id:"sole"+(front?"Front":"Rear")+(side<0?"Left":"Right"),side:side,front:front,
        x0:cx-length/2,x1:cx+length/2,y0:y0,y1:y1,boltXs:[cx-pitch/2,cx+pitch/2],boltY:boltY});
    }
    return out;
  }

  static function columnRecessOuter(side:Int):Float {
    var driveSide = HobberZLayout.axisY() < 0 ? -1 : 1;
    var driveOuter = Math.abs(HobberZLayout.axisY()) + HobberDrives.PAD_WIDTH/2 + DRIVE_RECESS_CLEARANCE;
    return side == driveSide ? Math.max(driveOuter,COLUMN_HEAD_WIDTH/2) : COLUMN_HEAD_WIDTH/2;
  }
  public static inline final BED_RAIL_SEAT_WIDTH = 50.0;
  public static inline final BED_RAIL_BEAM_WIDTH = 110.0;
  public static inline final FOOT_FLOOR = 40.0;
  public static inline final FOOT_ROOF = 160.0;
  public static inline final FOOT_SUPPORT_TOP = 230.0;
  public static inline final FOOT_ACCESS_WIDTH = 140.0;
  public static inline final FOOT_BOSS_WIDTH = 190.0;
  public static inline final FOOT_STEM_CLEARANCE = 28.0;
  public static inline final FOOT_PLATE_THICKNESS = 15.0;
  public static inline final FOOT_PLATE_WIDTH = 110.0;
  public static inline final PLINTH_CORE_FLOOR = 85.0;
  public static inline final PLINTH_CORE_WEB = 35.0;
  public static inline final CABLE_DUCT_WIDTH = 30.0;
  public static inline final CABLE_DUCT_HEIGHT = 40.0;
  public static inline final CABLE_DUCT_CENTRE_Y = 300.0;
  public static inline final CABLE_DUCT_FLOOR = 500.0;
  public static inline final DRIVE_CAP_UNDERSIDE = 660.0;
  public static inline final DRIVE_RIB_HALF_WIDTH = 15.0;
  public static inline final FIXTURE_THREAD_MM = 6.0;
  public static inline final TABLE_INSERT_WIDTH = 280.0;
  public static inline final TABLE_INSERT_BORE = 40.0;
  public static inline final TABLE_INSERT_THICKNESS = 20.0;

  static function faces(names:Array<String>):Array<CastingOpening> return [for (name in names) C.face(name)];


  static function boxInsert(
    id:String,
    role:InsertRole,
    label:String,
    w:Float,
    d:Float,
    h:Float,
    pose:AssemblyFrame,
    openings:Array<String>,
    holes:String,
    scope:String = "bed"
  ):InsertLocation {
    var stock=CastInsert.boxValues(w,d,h);
    if (scope=="column" && role==ColumnMount)
      stock=CastInsert.manufacturedValues(materia.casting.InsertMaterial.C45Ground).setToken("body",stock.token("body"));
    return {
      id: id,
      part: new CastInsert(CastInsert.precastHoles(
        stock.setToken("role", role).setToken("label", label).setToken("partId", "HOBBER-" + scope + "-" + id).setToken(
          "finish",
          scope == "column" && role == ColumnMount ? AsSupplied : GroundAfterCasting
        ),
        holes
      )),
      pose: pose,
      openings: faces(openings)
    };
  }


  /** Source-declared M6 fixture drilling, using the same C45/8.8 hole policy
	 * as service threads. Offset only when a source service bore owns the centre.
	 * Tooling consumes these records; it does not create the steel thread. */
  public static function fixtureHoles(thickness:Float, ?existing:String, offsetX:Float = 0):String {
    var choice = InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(FIXTURE_THREAD_MM).mating());
    var holes = existing == null ? [] : InsertHoles.records(existing);
    var fixture = InsertHoles.records(InsertHoles.tappedBlind(choice, thickness))[0];
    fixture.id = "fixture";
    fixture.frame.x = offsetX;
    holes.push(fixture);
    return InsertHoles.fromNeutral(holes);
  }

  public static function enclosureSeats():Array<InsertLocation> {
    var out:Array<InsertLocation> = [];
    for (x in [-1, 1]) for (y in [-1, 1]) out.push(boxInsert(
      'enclosureSeat${x+1}${y+1}',
      EnclosureMount,
      "enclosure frame dry-side mounting pad",
      80,
      80,
      20,
      F.fromRotationMatrix(
        x * (HobberXLayout.front().bedLength / 2 - 100),
        y * 380,
        720,
        [1.0, 0, 0, 0, 0, y, 0, -y, 0]
      ),
      [y < 0 ? "body.y-min" : "body.y-max"],
      fixtureHoles(20)
    ));
    return out;
  }

  public static function trayLedges(depth:Float = machinekit.enclosure.TraySeat.DEPTH):Array<CastingFeature> {
    var d = dimensions("bed"), w = d[0], l = d[1], a = machinekit.enclosure.TraySeat.WIDTH;
    return [Patterns.box(
      "tray-front-ledge",
      InterfaceRecess,
      faces(["body.x-min", "body.y-min", "body.y-max", "body.z-max"]),
      a,
      l,
      depth,
      -w / 2 + a / 2,
      0,
      d[2] - depth,
      "tray"
    ),
    // The removable bridge owns the middle span. The two ledges stop
    // at its edges rather than claiming the same cavity twice.
    Patterns.box(
      "tray-rear-ledge-left",
      InterfaceRecess,
      faces(["body.x-max", "body.y-min", "body.z-max"]),
      a,
      (l - TRAY_REAR_BRIDGE_WIDTH) / 2,
      depth,
      w / 2 - a / 2,
      -(l + TRAY_REAR_BRIDGE_WIDTH) / 4,
      d[2] - depth,
      "tray"
    ), Patterns.box(
      "tray-rear-ledge-right",
      InterfaceRecess,
      faces(["body.x-max", "body.y-max", "body.z-max"]),
      a,
      (l - TRAY_REAR_BRIDGE_WIDTH) / 2,
      depth,
      w / 2 - a / 2,
      (l + TRAY_REAR_BRIDGE_WIDTH) / 4,
      d[2] - depth,
      "tray"
    ), Patterns.box(
      "tray-left-ledge",
      InterfaceRecess,
      faces(["body.y-min", "body.z-max"]),
      w - 2 * a,
      a,
      depth,
      0,
      -l / 2 + a / 2,
      d[2] - depth,
      "tray"
    ), Patterns.box(
      "tray-right-ledge",
      InterfaceRecess,
      faces(["body.y-max", "body.z-max"]),
      w - 2 * a,
      a,
      depth,
      0,
      l / 2 - a / 2,
      d[2] - depth,
      "tray"
    )];
  }

  /** Unverified exterior study choices; published bore/thread dimensions live in the catalogue variant. */
  public static inline final TRAY_REAR_BRIDGE_WIDTH = 150.0;
  public static inline final TRAY_SOCKET_OUTSIDE_DIAMETER = 8.0;
  public static inline final TRAY_SOCKET_RELIEF_DIAMETER = 6.2;
  public static inline final TRAY_SOCKET_TAIL_WIDTH = 11.0;
  public static inline final TRAY_SOCKET_TAIL_THICKNESS = 2.2;
  public static inline final TRAY_SOCKET_TAIL_WAVE = 1.4;
  public static inline final TRAY_SOCKET_TAIL_BLEND = 0.5;
  public static inline final TRAY_SOCKET_POSITION_TOLERANCE = 0.5;
  // Approved 5 mm mix design: a 10 mm steel-to-ledge land. The toe
  // clamps remain on the same finished plane; side/end toe overlap is
  // 11/9.5 mm respectively, with grip qualification still a physical input.
  public static inline final SOURCE_STEEL_COVER_MM = 10.0;
  public static inline final TRAY_SOCKET_EDGE_INSET = machinekit.enclosure.TraySeat.WIDTH
    + TRAY_SOCKET_OUTSIDE_DIAMETER / 2 + SOURCE_STEEL_COVER_MM;
  public static inline final TRAY_SOCKET_END_INSET:Float = machinekit.enclosure.TraySeat.WIDTH + (TRAY_SOCKET_TAIL_WIDTH
    > TRAY_SOCKET_OUTSIDE_DIAMETER ? TRAY_SOCKET_TAIL_WIDTH : TRAY_SOCKET_OUTSIDE_DIAMETER) / 2 + SOURCE_STEEL_COVER_MM;

  public static function traySocket():CastInsert return machinekit.structural.FixingSockets.pfeiferStudy(
    {
      outsideDiameter: TRAY_SOCKET_OUTSIDE_DIAMETER,
      reliefDiameter: TRAY_SOCKET_RELIEF_DIAMETER,
      tailWidth: TRAY_SOCKET_TAIL_WIDTH,
      tailThickness: TRAY_SOCKET_TAIL_THICKNESS,
      tailWave: TRAY_SOCKET_TAIL_WAVE,
      tailBlend: TRAY_SOCKET_TAIL_BLEND
    },
    TRAY_SOCKET_POSITION_TOLERANCE
  );

  public static function trayClampSeats():Array<InsertLocation> {
    var out:Array<InsertLocation> = [], d = dimensions("bed");
    function add(x:Float, y:Float, height:Float = 11):Void out.push({
      id: "trayClampSeat" + out.length,
      part: traySocket(),
      openings: faces(["body.z-max"]),
      pose: F.translation(x, y, d[2] - machinekit.structural.FixingSockets.LENGTH),
      clampHeight: height
    });
    for (side in [-1, 1]) for (x in [-1, 0, 1]) add(x * (d[0] / 2 - 100), side * (d[1] / 2 - TRAY_SOCKET_EDGE_INSET));
    for (side in [-1, 1]) for (y in [-150.0, 150.0]) add(side * (d[0] / 2 - TRAY_SOCKET_END_INSET), y);
    for (y in [-75.0, 75.0]) add(d[0] / 2 - TRAY_SOCKET_END_INSET, y, 17);
    return out;
  }

  public static function feet():Array<{x:Float, y:Float}> return [for (i in 0...3) {
    x:(i < 2 ? -1 : 1) * (HobberXLayout.front().bedLength / 2 - 120), y : i == 0 ? -330.0 : i == 1 ? 330.0 : 0.0
  }
  ];

  /** Assembly construction frame shared with bought-part mounting sources. */
  public static function worldPose(kind:String):AssemblyFrame return switch kind {
    case "bed":
      F.translation(HobberXLayout.front().bedCentre, 0, TABLE_SEAT - TABLE_SEAT_HEIGHT);
    case "column":
      F.fromRotationMatrix(
        HobberZLayout.SEAT_FACE_X + HobberZLayout.SPINE_FRONT_COLUMN_Y,
        0,
        columnSoleTop() - COLUMN_SOLE_THICKNESS / 2,
        [0.0, -1, 0, 1, 0, 0, 0, 0, 1]
      );
    default:
      throw "Unknown hobber foundation frame: " + kind;
  };

  public static function dimensions(kind:String):Array<Float> return switch kind {
    case "bed":
      [HobberXLayout.front().bedLength, 800, PLINTH_HEIGHT];
    case "column":
      [400.0, 200, 500];
    default:
      throw "Unknown hobber foundation kind";
  };

  static inline final COLUMN_SPINE_HALF_WIDTH = 60.0;
  public static inline final COLUMN_HEAD_WIDTH = 240.0;
  /** The recess on the drive's side opens this far past the widest drive support (the pads, 80 wide, centred on the screw), mm. */
  static inline final DRIVE_RECESS_CLEARANCE = 3.0;
  static inline final COLUMN_HEAD_DEPTH = 100.0;
  static inline final COLUMN_HEAD_CENTRE_Y = 60.0;


  public static function bodyValues(kind:String):ComponentValues {
    var d = dimensions(kind);
    var cores:Array<CastingFeature> = kind == "bed" ? HobberCastingPockets.plinth(d[0], d[1], d[2]) : [];
    if (kind == "bed") {
      var layout = HobberXLayout.front(), centre = layout.bedCentre;
      var footCentres = feet();
      for (i in 0...footCentres.length) {
        var p = footCentres[i], alongX = i == 2;
        var opening = alongX ? "body.x-max" : p.y < 0 ? "body.y-min" : "body.y-max";
        var edge = alongX ? d[0] / 2 : p.y < 0 ? -d[1] / 2 : d[1] / 2;
        cores = cores.concat(Patterns.foot(
          {access: 'foot$i-access', stem: 'foot$i-stem'},
          p.x,
          p.y,
          edge,
          alongX,
          FOOT_ACCESS_WIDTH,
          FOOT_ACCESS_WIDTH,
          FOOT_FLOOR,
          FOOT_ROOF,
          FOOT_STEM_CLEARANCE,
          faces([opening]),
          faces(["body.z-min"])
        ));
      }
      cores.push(Patterns.box(
        "work-well",
        ServiceOpening,
        faces(["body.z-max"]),
        WORK_WELL_WIDTH,
        WORK_WELL_DEPTH,
        WORK_WELL_TOP - TABLE_WELL_FLOOR_HEIGHT,
        WORK_WELL_CENTRE_X - centre,
        0,
        TABLE_WELL_FLOOR_HEIGHT
      ));
      // This drain starts at the front chamber: the part of the former
      // rectangular cut already inside that chamber is not another feature.
      var drainFront = -TABLE_INSERT_WIDTH / 2 - SOURCE_STEEL_COVER_MM - centre, drainRear = 90.0 - centre;
      cores.push(Patterns.box(
        "chip-drain",
        ServiceOpening,
        [C.feature("work-well"), C.feature("pocket-frontSocketEnd-leftRailInside-floor")],
        drainRear - drainFront,
        CHIP_DRAIN_WIDTH,
        CHIP_DRAIN_DEPTH,
        (drainRear + drainFront) / 2,
        0,
        TABLE_WELL_FLOOR_HEIGHT - CHIP_DRAIN_DEPTH
      ));
      var channelFront = layout.deckStart - DRIVE_CHANNEL_FRONT_CLEARANCE;
      cores.push(Patterns.box(
        "X-drive-channel",
        ServiceOpening,
        faces(["body.x-max", "body.z-max"]),
        layout.bedRear - channelFront,
        DRIVE_CHANNEL_WIDTH,
        90,
        (layout.bedRear + channelFront) / 2 - centre,
        0,
        DRIVE_CHANNEL_FLOOR
      ));
      // The coolant reservoir and downcomer occupy the front chamber;
      // only the wall opening removes additional casting material.
      var accessFront = -305.0 - centre, accessRear = -d[0] / 2 + 25;
      cores.push(Patterns.box(
        "coolant-front-access",
        ServiceOpening,
        faces(["body.x-min"]).concat([C.feature("pocket-frontWall-leftWebOutside-floor")]),
        accessRear - accessFront,
        240,
        320,
        (accessFront + accessRear) / 2,
        0,
        140
      ));
      cores.push(Patterns.box(
        "rotary-union-access",
        Clearance,
        faces(["body.z-min"]).concat([C.feature("chip-drain")]),
        40,
        40,
        TABLE_WELL_FLOOR_HEIGHT - CHIP_DRAIN_DEPTH,
        -centre,
        0,
        0
      ));
      cores = cores.concat(trayLedges());
      for (side in [-1, 1]) cores.push(Patterns.box(
        "tray-rear-bridge-seat-" + (side < 0 ? "left" : "right"),
        InterfaceRecess,
        faces(["body.x-max", "body.z-max"]).concat([C.feature("X-drive-channel")]),
        25,
        (TRAY_REAR_BRIDGE_WIDTH - DRIVE_CHANNEL_WIDTH) / 2,
        10,
        d[0] / 2 - 12.5,
        side * (TRAY_REAR_BRIDGE_WIDTH + DRIVE_CHANNEL_WIDTH) / 4,
        d[2] - 10,
        "tray-rear-bridge"
      ));
      // Preserve the reviewed cable centreline. The filled outer band now
      // needs a real longitudinal duct opening at both body end faces.
      var cable = Patterns.box(
        "cable-route",
        ServiceOpening,
        faces(["body.x-min", "body.x-max"]),
        d[0],
        CABLE_DUCT_WIDTH,
        CABLE_DUCT_HEIGHT,
        0,
        CABLE_DUCT_CENTRE_Y,
        CABLE_DUCT_FLOOR
      );
      cable.draft.axis = {x: 1, y: 0, z: 0};
      cores.push(cable);
      // The hob spindle's servo passes close to the side beam's top at the Z lower limit: the relief is derived from its swept envelope.
      cores = cores.concat(HobberSpindleRelief.features());
    } else {
      // The spine continues into the head opening. Three direct spans
      // describe its U-shaped recess without cutting it and retaining a boss.
      var headRear = COLUMN_HEAD_CENTRE_Y + COLUMN_HEAD_DEPTH / 2;
      // The Z screw stands off the A axis housing on one side (`HobberZOffset`), its pads against the spine's front: the recess on that
      // side stops at the spine's front face, like the centre one, and opens to `DRIVE_RECESS_CLEARANCE` past the pads' outer edge.
      var driveSide = HobberZLayout.axisY() < 0 ? -1 : 1;
      for (side in [-1, 1]) {
        var drive = side == driveSide;
        var outer = columnRecessOuter(side);
        var width = outer - COLUMN_SPINE_HALF_WIDTH;
        cores.push(Patterns.box(
          "head-recess-" + (side < 0 ? "left" : "right"),
          ServiceOpening,
          faces(["body.y-max", "body.z-min", "body.z-max"]),
          width,
          drive ? headRear - HobberZLayout.SPINE_FRONT_COLUMN_Y : COLUMN_HEAD_DEPTH,
          d[2],
          side * (outer + COLUMN_SPINE_HALF_WIDTH) / 2,
          drive ? (headRear + HobberZLayout.SPINE_FRONT_COLUMN_Y) / 2 : COLUMN_HEAD_CENTRE_Y,
          0
        ));
      }
      cores.push(Patterns.box(
        "head-recess-centre",
        ServiceOpening,
        faces(["body.y-max", "body.z-min", "body.z-max"]),
        2 * COLUMN_SPINE_HALF_WIDTH,
        headRear - HobberZLayout.SPINE_FRONT_COLUMN_Y,
        d[2],
        0,
        (headRear + HobberZLayout.SPINE_FRONT_COLUMN_Y) / 2,
        0
      ));
    }
    return new ComponentValues().setNumber("width", d[0]).setNumber("depth", d[1]).setNumber("height", d[2]).setToken(
      "features",
      C.encode(cores)
    );
  }

  public static function placements(kind:String):Array<CastInsertPlacement> return [for (row in inserts(kind)) {
    id:row.id, part:row.part, pose:row.pose, openings:row.openings
  }
  ];

  public static function inserts(kind:String):Array<InsertLocation> {
    dimensions(kind);
    var out:Array<InsertLocation> = [];
    function add(
      id:String,
      role:InsertRole,
      label:String,
      w:Float,
      d:Float,
      h:Float,
      x:Float,
      y:Float,
      z:Float,
      openings:Array<String>,
      holes:String
    ):Void out.push(boxInsert(
      id,
      role,
      label,
      w,
      d,
      h,
      F.translation(x, y, z),
      openings,
      holes,
      kind
    ));
    var none = InsertHoles.fromNeutral([]);
    if (kind == "bed") {
      var layout = HobberXLayout.front(), centre = layout.bedCentre;
      out = enclosureSeats().concat(trayClampSeats());
      add(
        "tablePlate",
        TableMount,
        "table seat",
        TABLE_INSERT_WIDTH,
        TABLE_INSERT_WIDTH,
        TABLE_INSERT_THICKNESS,
        -centre,
        0,
        TABLE_SEAT_HEIGHT - TABLE_INSERT_THICKNESS,
        [],
        fixtureHoles(
          TABLE_INSERT_THICKNESS,
          InsertHoles.through(TABLE_INSERT_BORE, TABLE_INSERT_THICKNESS),
          (TABLE_INSERT_WIDTH + TABLE_INSERT_BORE) / 4
        )
      );
      for (side in [-1, 1]) out.push(boxInsert(
        side < 0 ? "railLeft" : "railRight",
        GuideSeat,
        "preloaded column guide seat",
        BED_RAIL_SEAT_WIDTH,
        layout.railLength,
        BED_RAIL_INSERT_THICKNESS,
        F.fromRotationMatrix(
          layout.railStart + layout.railLength / 2 - centre,
          side * BED_RAIL_CENTRE,
          RAIL_SEAT_HEIGHT,
          [0.0, 1, 0, -1, 0, 0, 0, 0, 1]
        ),
        ["body.z-max"],
        none
      ));
      out.push(boxInsert(
        "driveSeat",
        DriveMount,
        "central X screw channel seat",
        100,
        layout.motorFace - layout.deckStart,
        20,
        F.fromRotationMatrix(
          (layout.motorFace + layout.deckStart) / 2 - centre,
          0,
          DRIVE_SEAT_HEIGHT,
          [0.0, 1, 0, -1, 0, 0, 0, 0, 1]
        ),
        [],
        fixtureHoles(20)
      ));
      for (side in [-1, 1]) add(
        side < 0 ? "liftLeft" : "liftRight",
        LiftingMount,
        "lifting point",
        30,
        30,
        30,
        -150 - centre,
        side * LIFT_CENTRE_Y,
        LIFT_SEAT_HEIGHT,
        ["body.z-max"],
        none
      );
    } else {
      for (pad in columnSolePads()) out.push(boxInsert(pad.id,ColumnMount,
        "column corner sole " + pad.id,pad.y1-pad.y0,pad.x1-pad.x0,COLUMN_SOLE_THICKNESS,
        F.fromRotationMatrix((pad.y0+pad.y1)/2,
          HobberZLayout.SEAT_FACE_X+HobberZLayout.SPINE_FRONT_COLUMN_Y-(pad.x0+pad.x1)/2,
          COLUMN_SOLE_THICKNESS/2,[1.0,0,0,0,-1,0,0,0,-1]),
        ["body.z-min"],fixtureHoles(COLUMN_SOLE_THICKNESS),"column"));
      for (side in [-1, 1]) out.push(boxInsert(
        side < 0 ? "railLeft" : "railRight",
        GuideSeat,
        "Z rail seat",
        30,
        470,
        20,
        F.fromRotationMatrix(side * COLUMN_RAIL_CENTRE, 90, 245, [1.0, 0, 0, 0, 0, 1, 0, -1, 0]),
        ["body.y-max"],
        none,
        "column"
      ));
      out.push(boxInsert(
        "driveSeat",
        DriveMount,
        "Z drive back seat",
        HobberZOffset.insertWidth(),
        460,
        20,
        F.fromRotationMatrix(HobberZLayout.axisY(), HobberZLayout.SEAT_COLUMN_Y, 250, [-1.0, 0, 0, 0, 0, 1, 0, 1, 0]),
        [],
        fixtureHoles(20),
        "column"
      ));
      for (side in [-1, 1]) add(
        side < 0 ? "liftLeft" : "liftRight",
        LiftingMount,
        "lifting point",
        25,
        25,
        20,
        side * 160,
        -40,
        490,
        ["body.z-max"],
        none
      );
    }
    return out;
  }
}
