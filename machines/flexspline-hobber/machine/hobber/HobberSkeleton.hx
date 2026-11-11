package hobber;

import machinekit.hobbing.*;
import machinekit.cover.WayCoverMount.MountStock;
import machinekit.cover.*;

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
import machinekit.motion.*;
import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import machinekit.motion.ErobActuatorPart;
import machinekit.component.BoltedFace;
import machinekit.component.BoltedFace.PinnedFace;
import cadkit.modeling.Vector;
import machinekit.assembly.AxisBuilder;
import machinekit.component.*;
import machinekit.motion.BallNut;
import machinekit.motion.ErobActuatorData as E;
import machinekit.motion.LinearRail;

typedef InsertLocation = {
  var id:String;
  var purpose:InsertRole;
  var part:CastInsert;
  var pose:AssemblyFrame;
  var openings:Array<CastingOpening>;
  @:optional var clampHeight:Float;
}

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

typedef ZOffsetResult = {
  /** Signed screw position across the machine (work y). */
  var y:Float;
  /** The least |y| that keeps every running gap and hole margin (before rounding up to `zOffsetStep`). */
  var minimum:Float;
  /** The largest |y| the drive seat's casting cover allows. */
  var maximum:Float;
  /** The obstacle that set `minimum`. */
  var governing:String;
  /** Gap at `y` to each obstacle (name, mm). */
  var gaps:Array<{name:String, gap:Float}>;
  /** Half the drive seat insert needs around the screw line: the seat holes' reach plus the casting cover. */
  var insertHalfWidth:Float;
}

typedef HobberPlacedRail = {
  var memberId:String;
  var insertId:String;
  var rail:LinearRail;
  var pose:AssemblyFrame;
  var contact:AssemblyFrame;
}

typedef HobberXDimensions = {
  railStart:Float,
  railLength:Float,
  screwStart:Float,
  screwLength:Float,
  /** Nut-part origin station from the screw start; X nut points motorward. */
  nutStation:Float,
  motorFace:Float,
  motorRear:Float,
  floatingCentre:Float,
  deckStart:Float,
  bedRear:Float,
  bedLength:Float,
  bedCentre:Float,
  footRear:Float,
  footSpan:Float
};

typedef HobberHeightStack = {
  floorZ:Float, bedBottomZ:Float, bedTopZ:Float, xRailPlaneZ:Float, xGuideOriginZ:Float,
  xCarriageBottomZ:Float, xCarriageTopZ:Float, columnSoleTopZ:Float,
  zRailStartZ:Float, zRailEndZ:Float, zRailSeatY:Float, zGuideOriginY:Float, zBlockFaceY:Float,
  hobCentreZ:Float, tableFaceZ:Float, tableSeatZ:Float,
  foundationBed:AssemblyFrame, foundationColumn:AssemblyFrame,
  swivel:AssemblyFrame, yModule:AssemblyFrame, spindle:AssemblyFrame
};

/**
 * The hobber's skeleton: the layout inputs every unit and part is placed from, the values derived from them (the height
 * stack, rail and pad positions) and the frames parts mount at. `FlexsplineHobber` takes one and hands it to each unit;
 * a part made from its recipe alone uses the reference machine's (`reference()`). Parts still reading `reference()`
 * themselves take the skeleton explicitly when they are recorded in their own frames.
 */
class HobberSkeleton {

  static final referenceCache = new machinekit.cache.Once<HobberSkeleton>();
  final heightsCache = new machinekit.cache.Once<HobberHeightStack>();
  /** The reference machine's skeleton. */
  public static function reference():HobberSkeleton return referenceCache.get(() -> new HobberSkeleton());

  function new() {
    nativeTableFace = 228.0;
    xNutBlockHeight = 80.0;
    nativeXScaleHoleY = 180.0;
    nativeZScaleHoleX = 200.0;
    xScaleHeadStation = 262.0;
    zScaleHeadOffset = 50.0;
    nativeTableSeat = nativeTableFace - 210.0;
    bedRailCentre = 220.0;
    columnRailCentre = 145.0;
    xMin = 25.0;
    xRetract = 110.0;
    topRetract = 145.0;
    overtravel = 5.0;
    endAllowance = 10.0;
    xBlockFront = 102.0;
    xBlockRear = 300.0;
    nutBlockFront = 232.5;
    columnRear = 386.0;
    xDeckFront = 60.0;
    bedFront = -300.0;
    footFront = -180.0;
    frontLift = 35.0;
    frontApproach = 250.0;
    zTravelMin = -10.0;
    zTravelMax = 190.0;
    zBlockSpacing = 110.0;
    nativeHobCentreZ = 363.0;
    zRailLength = 480.0;
    zScrewNativeX = 176.0;
    zNutOffsetZ = 55.0;
    zShaftTop = 730.0;
    zShaftLength = 420.0;
    zNutStation = zShaftTop - (nativeHobCentreZ + zNutOffsetZ);
    zFixedBearingZ = zShaftTop-machinekit.motion.HzMotionDrawingScrew.BK12_SUPPORT_CENTRE;
    zFloatingBearingZ = zShaftTop - zShaftLength + machinekit.motion.HzMotionDrawingScrew.BF12_SUPPORT_CENTRE;
    zCouplingLength = 40.0;
    zSeatFaceNativeX = 241.0;
    zSeatColumnNativeY = 25.0;
    zSpineFrontColumnY = 45.0;
    zScaleNativeY = 70.0;
    swivelNativeX = 124.0;
    swivelRange = Math.PI / 12;
    swivelRotation = [0.0, 0, -1, 1, 0, 0, 0, -1, 0];
    spindleRotation = [-1.0, 0, 0, 0, 0, -1, 0, -1, 0];
    spindleNativeY = 240.0;
    zRunningGap = 2.0;
    zScrewSide = 1.0;
    zOffsetStep = 1.0;
    zHoleMargin = 1.0;
    zMaxInsertWidth = 100.0;
    zPointMargin = 1e-3;
  }

  /** ALIGNMENT A1–A9: one derived stack in the machine frame. Part-native
   * dimensions are read from the made parts and bought guide catalogue. */
  public function heights():HobberHeightStack return heightsCache.get(() -> {
    var guide = machinekit.motion.LinearRailSystem.catalog().get("HGH25CA");
    if (guide == null) throw "Missing X guide";
    var rail = HobberFrame.PART_DATUM_Z + bedRailSeatTop();
    var bottom = rail + guide.blockHeight;
    var top = bottom + HobberXCarriage.X_CARRIAGE_THICKNESS;
    var zGuide = machinekit.motion.LinearRailSystem.catalog().get("HGH15CA");
    if (zGuide == null) throw "Missing Z guide";
    var zSeat = zSeatFaceNativeX + zSpineFrontColumnY - MineralCastColumn.RAIL_NATIVE_Y - MineralCastColumn.RAIL_INSERT_THICKNESS;
    var zRailStart = top + MineralCastColumn.COLUMN_SOLE_THICKNESS / 2 + MineralCastColumn.RAIL_CENTRE_Z - zRailLength / 2;
    return {
      floorZ:floorZ, bedBottomZ:HobberFrame.PART_DATUM_Z + nativeTableSeat - MineralCastBed.TABLE_SEAT_HEIGHT,
      bedTopZ:HobberFrame.PART_DATUM_Z + nativeTableSeat - MineralCastBed.TABLE_SEAT_HEIGHT + MineralCastBed.PLINTH_HEIGHT,
      xRailPlaneZ:rail, xGuideOriginZ:rail + guide.railHeight,
      xCarriageBottomZ:bottom, xCarriageTopZ:top,
      columnSoleTopZ:top + MineralCastColumn.COLUMN_SOLE_THICKNESS,
      zRailStartZ:zRailStart, zRailEndZ:zRailStart + zRailLength,
      zRailSeatY:zSeat, zGuideOriginY:zSeat - zGuide.railHeight, zBlockFaceY:zSeat - zGuide.blockHeight,
      hobCentreZ:HobberFrame.PART_DATUM_Z + nativeHobCentreZ,
      tableFaceZ:HobberFrame.PART_DATUM_Z + nativeTableFace,
      tableSeatZ:HobberFrame.PART_DATUM_Z + nativeTableSeat,
      foundationBed:HobberFrame.sourcePose(foundationDatum("bed")),
      foundationColumn:HobberFrame.sourcePose(F.fromRotationMatrix(
        zSeatFaceNativeX + zSpineFrontColumnY, 0,
        top - HobberFrame.PART_DATUM_Z + MineralCastColumn.COLUMN_SOLE_THICKNESS / 2,
        [0.0,-1,0,1,0,0,0,0,1])),
      swivel:HobberFrame.sourcePose(swivelDatum()),
      yModule:HobberFrame.sourcePose(modulePose()),
      spindle:HobberFrame.sourcePose(spindleDatum())
    };
  });

  public final nativeTableFace:Float;
  public final xNutBlockHeight:Float;
  public final nativeXScaleHoleY:Float;
  public final nativeZScaleHoleX:Float;
  public final xScaleHeadStation:Float;
  public final zScaleHeadOffset:Float;
  final xAxisCache = new machinekit.cache.Once<machinekit.motion.SupportedScrewDrive.ScrewDriveAxis>();

  public function xDriveAxis():machinekit.motion.SupportedScrewDrive.ScrewDriveAxis return xAxisCache.get(() -> {
    var source = xNominal();
    return {start:[source.screwStart, 0.0, heights().xCarriageBottomZ - HobberFrame.PART_DATUM_Z - xNutBlockHeight],
      direction:[-1.0,0,0], normal:[0.0,0,1]};
  });

  public final nativeTableSeat:Float;

  public final bedRailCentre:Float;

  public function bedRailSeatTop():Float return nativeTableSeat - MineralCastBed.TABLE_SEAT_HEIGHT
    + MineralCastBed.RAIL_SEAT_HEIGHT + MineralCastBed.BED_RAIL_INSERT_THICKNESS;

  public function columnSoleTop():Float return HobberXCarriage.xCarriageTop() + MineralCastColumn.COLUMN_SOLE_THICKNESS;

  public final columnRailCentre:Float;


  /** Keep four ground rectangles on resin-backed corner lands; the front pair
   * clear the same source recess that clears the actuator. Dimensions are
   * derived from the casting, rail centres, M8 edge/pitch and running gap. */
  public function columnSolePads():Array<ColumnSolePad> {
    var screw = machinekit.standard.SocketHeadCapScrew.catalog().get("M8");
    if (screw == null) throw "Column sole screw unavailable";
    var outer = (foundationDimensions("column")[0] - MineralCastColumn.COLUMN_SOLE_THICKNESS) / 2;
    var depth = foundationDimensions("column")[1] - MineralCastColumn.COLUMN_SOLE_THICKNESS;
    var pitch = machinekit.assembly.JointFasteners.SPACING_DIAMETERS * screw.diameter;
    var edge = machinekit.assembly.JointFasteners.EDGE_DIAMETERS * screw.diameter + 0.5;
    // Leave two screw diameters beyond the service-hole edge allowance
    // for the separately demand-sized cast-anchor pattern (layout allowance,
    // not a supplier limit). Exact source selection verifies its capacity.
    var anchorLand = 2*screw.diameter;
    var length = Math.ceil((2*(edge+anchorLand) + pitch)/10)*10;
    var centreX = zSeatFaceNativeX + zSpineFrontColumnY;
    var out:Array<ColumnSolePad> = [];
    for (front in [true,false]) for (side in [-1,1]) {
      var inner = front ? Math.ceil((columnRecessOuter(side) + HobberScales.RUNNING_CLEARANCE)/5)*5
        : 2*columnRailCentre - outer;
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


  function columnRecessOuter(side:Int):Float {
    var driveSide = zAxisY() < 0 ? -1 : 1;
    var driveOuter = Math.abs(zAxisY()) + HobberDrives.PAD_WIDTH/2 + MineralCastColumn.DRIVE_RECESS_CLEARANCE;
    return side == driveSide ? Math.max(driveOuter,MineralCastColumn.COLUMN_HEAD_WIDTH/2) : MineralCastColumn.COLUMN_HEAD_WIDTH/2;
  }


  function faces(names:Array<String>):Array<CastingOpening> return [for (name in names) C.face(name)];

  function boxInsert(
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
      purpose:role,
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
  public function fixtureHoles(thickness:Float, ?existing:String, offsetX:Float = 0):String {
    var choice = InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(MineralCastBed.FIXTURE_THREAD_MM).mating());
    var holes = existing == null ? [] : InsertHoles.records(existing);
    var fixture = InsertHoles.records(InsertHoles.tappedBlind(choice, thickness))[0];
    fixture.id = "fixture";
    fixture.frame.x = offsetX;
    holes.push(fixture);
    return InsertHoles.fromNeutral(holes);
  }


  public function enclosureSeats():Array<InsertLocation> {
    var out:Array<InsertLocation> = [];
    for (x in [-1, 1]) for (y in [-1, 1]) out.push(boxInsert(
      'enclosureSeat${x+1}${y+1}',
      EnclosureMount,
      "enclosure frame dry-side mounting pad",
      80,
      80,
      20,
      F.fromRotationMatrix(
        x * (xNominal().bedLength / 2 - 100),
        y * 380,
        720,
        [1.0, 0, 0, 0, 0, y, 0, -y, 0]
      ),
      [y < 0 ? "body.y-min" : "body.y-max"],
      fixtureHoles(20)
    ));
    return out;
  }


  public function trayLedges(depth:Float = machinekit.enclosure.TraySeat.DEPTH):Array<CastingFeature> {
    var d = foundationDimensions("bed"), w = d[0], l = d[1], a = machinekit.enclosure.TraySeat.WIDTH;
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
    Patterns.box(
      "tray-rear-ledge", InterfaceRecess,
      faces(["body.x-max", "body.y-min", "body.y-max", "body.z-max"]),
      a, l, depth, w / 2 - a / 2, 0, d[2] - depth, "tray"
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


  public function traySocket():CastInsert return machinekit.structural.FixingSockets.pfeiferStudy(
    {
      outsideDiameter: MineralCastBed.TRAY_SOCKET_OUTSIDE_DIAMETER,
      reliefDiameter: MineralCastBed.TRAY_SOCKET_RELIEF_DIAMETER,
      tailWidth: MineralCastBed.TRAY_SOCKET_TAIL_WIDTH,
      tailThickness: MineralCastBed.TRAY_SOCKET_TAIL_THICKNESS,
      tailWave: MineralCastBed.TRAY_SOCKET_TAIL_WAVE,
      tailBlend: MineralCastBed.TRAY_SOCKET_TAIL_BLEND
    },
    MineralCastBed.TRAY_SOCKET_POSITION_TOLERANCE
  );


  public function trayClampSeats():Array<InsertLocation> {
    var out:Array<InsertLocation> = [], d = foundationDimensions("bed");
    function add(x:Float, y:Float, height:Float = 11):Void out.push({
      id: "trayClampSeat" + out.length,
      purpose:InsertRole.TrayClampMount,
      part: traySocket(),
      openings: faces(["body.z-max"]),
      pose: F.translation(x, y, d[2] - machinekit.structural.FixingSockets.LENGTH),
      clampHeight: height
    });
    for (side in [-1, 1]) for (x in [-1, 0, 1]) add(x * (d[0] / 2 - 100), side * (d[1] / 2 - MineralCastBed.TRAY_SOCKET_EDGE_INSET));
    for (side in [-1, 1]) for (sign in [-1, 1]) add(side * (d[0] / 2 - MineralCastBed.TRAY_SOCKET_END_INSET),
      sign * (HobberChipCover.OUTER_HALF_WIDTH + machinekit.enclosure.TrayClamp.WIDTH / 2 + 70));
    return out;
  }


  public function feet():Array<{x:Float, y:Float}> return [for (i in 0...3) {
    x:(i < 2 ? -1 : 1) * (xNominal().bedLength / 2 - 120), y : i == 0 ? -330.0 : i == 1 ? 330.0 : 0.0
  }
  ];

  /** Assembly construction frame shared with bought-part mounting sources. */
  public function foundationDatum(kind:String):AssemblyFrame return switch kind {
    case "bed":
      F.translation(xNominal().bedCentre, 0, nativeTableSeat - MineralCastBed.TABLE_SEAT_HEIGHT);
    case "column":
      F.fromRotationMatrix(
        zSeatFaceNativeX + zSpineFrontColumnY,
        0,
        columnSoleTop() - MineralCastColumn.COLUMN_SOLE_THICKNESS / 2,
        [0.0, -1, 0, 1, 0, 0, 0, 0, 1]
      );
    default:
      throw "Unknown hobber foundation frame: " + kind;
  };

  public function foundationDimensions(kind:String):Array<Float> return switch kind {
    case "bed":
      [xNominal().bedLength, 800, MineralCastBed.PLINTH_HEIGHT];
    case "column":
      [400.0, 200, 500];
    default:
      throw "Unknown hobber foundation kind";
  };

  public function foundationBodyValues(kind:String):ComponentValues {
    var d = foundationDimensions(kind);
    var cores:Array<CastingFeature> = kind == "bed" ? HobberCastingPockets.plinth(d[0], d[1], d[2]) : [];
    if (kind == "bed") {
      var layout = xNominal(), centre = layout.bedCentre;
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
          MineralCastBed.FOOT_ACCESS_WIDTH,
          MineralCastBed.FOOT_ACCESS_WIDTH,
          MineralCastBed.FOOT_FLOOR,
          MineralCastBed.FOOT_ROOF,
          MineralCastBed.FOOT_STEM_CLEARANCE,
          faces([opening]),
          faces(["body.z-min"])
        ));
      }
      cores.push(Patterns.box(
        "work-well",
        ServiceOpening,
        faces(["body.z-max"]),
        MineralCastBed.WORK_WELL_WIDTH,
        MineralCastBed.WORK_WELL_DEPTH,
        MineralCastBed.WORK_WELL_TOP - MineralCastBed.TABLE_WELL_FLOOR_HEIGHT,
        MineralCastBed.WORK_WELL_CENTRE_X - centre,
        0,
        MineralCastBed.TABLE_WELL_FLOOR_HEIGHT
      ));
      // This drain starts at the front chamber: the part of the former
      // rectangular cut already inside that chamber is not another feature.
      var drainFront = -MineralCastBed.TABLE_INSERT_WIDTH / 2 - MineralCastBed.SOURCE_STEEL_COVER_MM - centre, drainRear = 90.0 - centre;
      cores.push(Patterns.box(
        "chip-drain",
        ServiceOpening,
        [C.feature("work-well"), C.feature("pocket-frontSocketEnd-leftRailInside-floor")],
        drainRear - drainFront,
        MineralCastBed.CHIP_DRAIN_WIDTH,
        MineralCastBed.CHIP_DRAIN_DEPTH,
        (drainRear + drainFront) / 2,
        0,
        MineralCastBed.TABLE_WELL_FLOOR_HEIGHT - MineralCastBed.CHIP_DRAIN_DEPTH
      ));
      var channelFront = layout.deckStart - MineralCastBed.DRIVE_CHANNEL_FRONT_CLEARANCE;
      cores.push(Patterns.box(
        "X-drive-channel",
        ServiceOpening,
        faces(["body.x-max", "body.z-max"]),
        layout.bedRear - channelFront,
        MineralCastBed.DRIVE_CHANNEL_WIDTH,
        90,
        (layout.bedRear + channelFront) / 2 - centre,
        0,
        MineralCastBed.DRIVE_CHANNEL_FLOOR
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
        MineralCastBed.TABLE_WELL_FLOOR_HEIGHT - MineralCastBed.CHIP_DRAIN_DEPTH,
        -centre,
        0,
        0
      ));
      cores = cores.concat(trayLedges());
      // Preserve the reviewed cable centreline. The filled outer band now
      // needs a real longitudinal duct opening at both body end faces.
      var cable = Patterns.box(
        "cable-route",
        ServiceOpening,
        faces(["body.x-min", "body.x-max"]),
        d[0],
        MineralCastBed.CABLE_DUCT_WIDTH,
        MineralCastBed.CABLE_DUCT_HEIGHT,
        0,
        MineralCastBed.CABLE_DUCT_CENTRE_Y,
        MineralCastBed.CABLE_DUCT_FLOOR
      );
      cable.draft.axis = {x: 1, y: 0, z: 0};
      cores.push(cable);
      // The hob spindle's servo passes close to the side beam's top at the Z lower limit: the relief is derived from its swept envelope.
      cores = cores.concat(HobberSpindleRelief.features());
    } else {
      // The spine continues into the head opening. Three direct spans
      // describe its U-shaped recess without cutting it and retaining a boss.
      var headRear = MineralCastColumn.COLUMN_HEAD_CENTRE_Y + MineralCastColumn.COLUMN_HEAD_DEPTH / 2;
      // The Z screw stands off the A axis housing on one side (the derived Z offset), its pads against the spine's front: the recess on that
      // side stops at the spine's front face, like the centre one, and opens to `MineralCastColumn.DRIVE_RECESS_CLEARANCE` past the pads' outer edge.
      var driveSide = zAxisY() < 0 ? -1 : 1;
      for (side in [-1, 1]) {
        var drive = side == driveSide;
        var outer = columnRecessOuter(side);
        var width = outer - MineralCastColumn.COLUMN_SPINE_HALF_WIDTH;
        cores.push(Patterns.box(
          "head-recess-" + (side < 0 ? "left" : "right"),
          ServiceOpening,
          faces(["body.y-max", "body.z-min", "body.z-max"]),
          width,
          drive ? headRear - zSpineFrontColumnY : MineralCastColumn.COLUMN_HEAD_DEPTH,
          d[2],
          side * (outer + MineralCastColumn.COLUMN_SPINE_HALF_WIDTH) / 2,
          drive ? (headRear + zSpineFrontColumnY) / 2 : MineralCastColumn.COLUMN_HEAD_CENTRE_Y,
          0
        ));
      }
      cores.push(Patterns.box(
        "head-recess-centre",
        ServiceOpening,
        faces(["body.y-max", "body.z-min", "body.z-max"]),
        2 * MineralCastColumn.COLUMN_SPINE_HALF_WIDTH,
        headRear - zSpineFrontColumnY,
        d[2],
        0,
        (headRear + zSpineFrontColumnY) / 2,
        0
      ));
    }
    return new ComponentValues().setNumber("width", d[0]).setNumber("depth", d[1]).setNumber("height", d[2]).setToken(
      "features",
      C.encode(cores)
    );
  }


  public function foundationPlacements(kind:String):Array<CastInsertPlacement> return [for (row in foundationInserts(kind)) {
    id:row.id, purpose:row.purpose, part:row.part, pose:row.pose, openings:row.openings
  }
  ];


  public function foundationInserts(kind:String):Array<InsertLocation> {
    foundationDimensions(kind);
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
      var layout = xNominal(), centre = layout.bedCentre;
      out = enclosureSeats().concat(trayClampSeats());
      add(
        "tablePlate",
        TableMount,
        "table seat",
        MineralCastBed.TABLE_INSERT_WIDTH,
        MineralCastBed.TABLE_INSERT_WIDTH,
        MineralCastBed.TABLE_INSERT_THICKNESS,
        -centre,
        0,
        MineralCastBed.TABLE_SEAT_HEIGHT - MineralCastBed.TABLE_INSERT_THICKNESS,
        [],
        fixtureHoles(
          MineralCastBed.TABLE_INSERT_THICKNESS,
          InsertHoles.through(MineralCastBed.TABLE_INSERT_BORE, MineralCastBed.TABLE_INSERT_THICKNESS),
          (MineralCastBed.TABLE_INSERT_WIDTH + MineralCastBed.TABLE_INSERT_BORE) / 4
        )
      );
      for (side in [-1, 1]) out.push(boxInsert(
        side < 0 ? "railLeft" : "railRight",
        GuideSeat,
        "preloaded column guide seat",
        MineralCastBed.BED_RAIL_SEAT_WIDTH,
        layout.railLength,
        MineralCastBed.BED_RAIL_INSERT_THICKNESS,
        F.fromRotationMatrix(
          layout.railStart + layout.railLength / 2 - centre,
          side * bedRailCentre,
          MineralCastBed.RAIL_SEAT_HEIGHT,
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
          MineralCastBed.DRIVE_SEAT_HEIGHT,
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
        side * MineralCastBed.LIFT_CENTRE_Y,
        MineralCastBed.LIFT_SEAT_HEIGHT,
        ["body.z-max"],
        none
      );
    } else {
      for (pad in columnSolePads()) out.push(boxInsert(pad.id,ColumnMount,
        "column corner sole " + pad.id,pad.y1-pad.y0,pad.x1-pad.x0,MineralCastColumn.COLUMN_SOLE_THICKNESS,
        F.fromRotationMatrix((pad.y0+pad.y1)/2,
          zSeatFaceNativeX+zSpineFrontColumnY-(pad.x0+pad.x1)/2,
          MineralCastColumn.COLUMN_SOLE_THICKNESS/2,[1.0,0,0,0,-1,0,0,0,-1]),
        ["body.z-min"],fixtureHoles(MineralCastColumn.COLUMN_SOLE_THICKNESS),"column"));
      for (side in [-1, 1]) out.push(boxInsert(
        side < 0 ? "railLeft" : "railRight",
        GuideSeat,
        "Z rail seat",
        MineralCastColumn.RAIL_SEAT_WIDTH,
        MineralCastColumn.RAIL_SEAT_LENGTH,
        MineralCastColumn.RAIL_INSERT_THICKNESS,
        F.fromRotationMatrix(side * columnRailCentre, MineralCastColumn.RAIL_NATIVE_Y, MineralCastColumn.RAIL_CENTRE_Z, [1.0, 0, 0, 0, 0, 1, 0, -1, 0]),
        ["body.y-max"],
        none,
        "column"
      ));
      out.push(boxInsert(
        "driveSeat",
        DriveMount,
        "Z drive back seat",
        zDriveInsertWidth(),
        460,
        20,
        F.fromRotationMatrix(zAxisY(), zSeatColumnNativeY, 250, [-1.0, 0, 0, 0, 0, 1, 0, 1, 0]),
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

  public final xMin:Float;

  public final xRetract:Float;
 // required 108.54, round up to 5 mm
  public final topRetract:Float;
 // required 142.33, evaluated alternative
  public final overtravel:Float;

  public final endAllowance:Float;

  public final xBlockFront:Float;

  public final xBlockRear:Float;

  /** Front face of the X nut block at joint zero; carriage interface datum. */
  public final nutBlockFront:Float;

  public final columnRear:Float;
  /** Front edge of the X carriage deck (part datum x), mm; the deck runs from here to `columnRear`. */
  public final xDeckFront:Float;

  /**
   * Where the X carriage's own frame sits in the part datum frame: on the deck's underside (the guide-block lands), centred
   * between the X rails and along the deck, axes as the datum frame (+X along X travel, +Z up).
   */
  public function xCarriageSeat():AssemblyFrame
    return F.translation((xDeckFront + columnRear) / 2, 0, heights().xCarriageBottomZ - HobberFrame.PART_DATUM_Z);

  public final bedFront:Float;

  public final footFront:Float;

  public final frontLift:Float;
 // max 20 mm band + 8 mm sleeve + 2 mm cap + 5 mm clearance
  public final frontApproach:Float;
 // robot transfer distance, not X-axis stroke
  final xCache = new machinekit.cache.Once<HobberXDimensions>();

  public function xNominal():HobberXDimensions {
    return xCache.get(() -> xDimensions(xRetract));
  }

  function up10(v:Float):Float return Math.ceil(v / 10) * 10;

  public function xDimensions(retract:Float):HobberXDimensions {
    var guide = LinearRailSystem.catalog().get("HGH25CA"),
      nut = new BallNut(), motor = ServoMotor.model("GENERIC-HV-SERVO-400W").rating;
    var railStart = xMin + xBlockFront - guide.blockLength / 2 - guide.railEndMargin - overtravel;
    var railLength = up10(retract + xBlockRear + guide.blockLength / 2 + guide.railEndMargin + overtravel - railStart);
    var start = up10(nutBlockFront
      + retract + nut.bodyLength + BallScrew.INPUT_JOURNAL_LENGTH + overtravel + endAllowance);
    var raceEnd = nutBlockFront
      + xMin - nut.flangeThickness - overtravel - endAllowance - BallScrew.JOURNAL_LENGTH;
    var carrierFront = nutBlockFront + xMin - overtravel - nut.flangeThickness - 20;
    var supportEnd = carrierFront - 12.5 - ScrewSupportUnit.bf12().length / 2 - endAllowance;
    var length = up10(start - Math.min(raceEnd, supportEnd));
    // The bought mount puts the motor face 34 mm from the screw start; the bed keeps the length it was cast for.
    var face = start + SupportedScrewDrive.m60dBk12().faceToStart(), rear = start + motor.shaftLength + motor.bodyLength, floating = start - length + machinekit.motion.HzMotionDrawingScrew.BF12_SUPPORT_CENTRE;
    var bedRear = up10(Math.max(
      retract + columnRear + overtravel + 35,
      Math.max(railStart + railLength + 25, rear + 20)
    ));
    return {
      railStart: railStart,
      railLength: railLength,
      screwStart: start,
      screwLength: length,
      nutStation: start - (nutBlockFront + machinekit.structural.GroundStock.atLeast("flat bar", NutBlock.MIN_THICKNESS) - nut.bodyLength),
      motorFace: face,
      motorRear: rear,
      floatingCentre: floating,
      deckStart: floating - 10,
      bedRear: bedRear,
      bedLength: bedRear - bedFront,
      bedCentre: (bedRear + bedFront) / 2,
      footRear: bedRear - 120,
      footSpan: bedRear - 120 - footFront
    };
  }

  public final zTravelMin:Float;

  public final zTravelMax:Float;

  public final zBlockSpacing:Float;

  /** Height of the work seat above the work frame's zero (z), mm: the hob axis, the mandrel's seat and the origin of the Z, Y and swivel parts. */
  public final nativeHobCentreZ:Float;
  public final zRailLength:Float;

  /** Screw axis, 25 mm (the BK12 and BF12 shaft height) in front of the 40 mm ground bar pads on the column's drive seat at 241. */
  public final zScrewNativeX:Float;

  public final zNutOffsetZ:Float;

  public final zShaftTop:Float;

  public final zShaftLength:Float;

  public final zNutStation:Float;

  /** BK12 centre, from the top of the standard 51 mm screw end. */
  public final zFixedBearingZ:Float;

  /** BF12 centre, from the screw tip on the standard 11 mm floating end. */
  public final zFloatingBearingZ:Float;

  /** Clamp coupling length: its screw half stays clear of the BK12 housing on the 51 mm screw end. */
  public final zCouplingLength:Float;

  public final zSeatFaceNativeX:Float;

  public final zSeatColumnNativeY:Float;

  public final zSpineFrontColumnY:Float;

  public final zScaleNativeY:Float;

  public static inline final Y_JOINT = "Ymodule/travel";

  // A short plain plate carries the refitted spindle seat and existing outboard arm.
  final yModuleSpecCache = new machinekit.cache.Once<machinekit.motion.ScrewLinearModuleSpec>();
  public function moduleSpec():machinekit.motion.ScrewLinearModuleSpec return yModuleSpecCache.get(() -> new machinekit.motion.ScrewLinearModuleSpec("HGH15CA",30,140,64,5,140,290));

  public function moduleLift():Float return 145;

  public function modulePose():materia.assembly.AssemblyRecord.AssemblyFrame {
    var s=moduleSpec(),back=HobberSpindleSeat.headFaceX()+s.carriageThickness+s.carriageBottom();
    return materia.assembly.AssemblyFrames.fromRotationMatrix(back,s.carriageCentre+60,nativeHobCentreZ+moduleLift(),[0.0,0,-1,-1,0,0,0,1,0]);
  }

  public function headInModule():materia.assembly.AssemblyRecord.AssemblyFrame return materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.inverse(modulePose()),materia.assembly.AssemblyFrames.translation(0,0,nativeHobCentreZ));

  function railClearance():Float {
    var rail=machinekit.motion.LinearRailSystem.catalog().get("HGH25CA");if(rail==null)throw "Missing X guide";
    var stroke=HobberScales.stroke("Y"),half=(stroke.upper-stroke.lower)/2+HobberScales.overtravel();
    return bedRailSeatTop()+rail.railHeight+machinekit.motion.ScrewLinearModuleSpec.GAP
      +half*Math.sin(swivelRange)-nativeHobCentreZ-zTravelMin+HobberScales.overtravel();
  }

  /** Conservative floor above either X rail through Y/Z overtravel and A.
   * This removes free lower corners, never changes an operating limit. */
  public function headClearance():cadkit.modeling.Part {
    var a=swivelRange,c=railClearance(),sn=Math.sin(a),cs=Math.cos(a);
    var raw=cadkit.modeling.Part.prism([new cadkit.modeling.Vector(-1000,(c+1000*sn)/cs),new cadkit.modeling.Vector(0,c/cs),new cadkit.modeling.Vector(1000,(c+1000*sn)/cs),new cadkit.modeling.Vector(1000,1000),new cadkit.modeling.Vector(-1000,1000)],-1000,1000);
    var out=machinekit.assembly.PosedParts.moved(raw,materia.assembly.AssemblyFrames.fromRotationMatrix(0,0,0,[0.0,0,1,1,0,0,0,1,0]));raw.close();return out;
  }

  final yReliefCache=new machinekit.cache.Once<cadkit.modeling.Part>();

  public function sweptDeckRelief():cadkit.modeling.Part {
    var raw=yReliefCache.get(()->{
      var drop=zTravelMax-zTravelMin+2*HobberScales.overtravel();
      var gap=machinekit.motion.ScrewLinearModuleSpec.GAP,skirt=new HobberChipCover(new machinekit.component.ComponentValues().setToken("role","Xskirt")).geometry();
      var b=machinekit.assembly.PosedParts.boxOf(skirt);skirt.close();
      // Sweep the stationary deck/skirt through the complete relative Z
      // passage. A single low-Z slab misses material near its lower face.
      var skirtBox=cadkit.modeling.Part.box(b.maxX-b.minX+2*gap,b.maxY-b.minY+2*gap,b.maxZ-b.minZ+2*gap+drop);
      var skirtKeep=skirtBox.translated(new cadkit.modeling.Vector((b.minX+b.maxX)/2,(b.minY+b.maxY)/2,b.minZ-gap-drop));skirtBox.close();
      var deck=machinekit.component.Solids.union([HobberXCarriage.xDeckKeepout(gap,drop),skirtKeep]);
      var low=nativeHobCentreZ+zTravelMin-HobberScales.overtravel();
      var local=machinekit.assembly.PosedParts.moved(deck,materia.assembly.AssemblyFrames.translation(0,0,-low));deck.close();
      var copies:Array<cadkit.modeling.Part>=[];
      for(i in 0...13) {
        var angle=-swivelRange+2*swivelRange*i/12,c=Math.cos(angle),s=Math.sin(angle);
        copies.push(machinekit.assembly.PosedParts.moved(local,materia.assembly.AssemblyFrames.fromRotationMatrix(0,0,0,[1.0,0,0,0,c,-s,0,s,c])));
      }
      local.close();return machinekit.component.Solids.union(copies);
    });
    return machinekit.assembly.PosedParts.moved(raw,headInModule());
  }

  /** Output flange face, x. */
  public final swivelNativeX:Float;

  /** Swing either side of the middle position. */
  public final swivelRange:Float;

  /** Rotation that takes the actuator's CAD +Z to the work frame's -x (the old swivel head used the same). */
  public final swivelRotation:Array<Float>;


  /** Rotation of the spindle cartridge and the hob's line in the work frame (the arbor, outboard support and hob use the same). */
  public final spindleRotation:Array<Float>;

  /** Where the cartridge's housing datum stands across the machine (work y), at X = 0 and the swivel's middle position. */
  public final spindleNativeY:Float;


  /** The spindle cartridge's housing frame in the work frame at X = 0, Y = 0, Z = 0 and the swivel's middle position. */
  public function spindleDatum():AssemblyFrame return F.fromRotationMatrix(0, spindleNativeY, nativeHobCentreZ, spindleRotation);


  /** The actuator's CAD frame in the work frame. */
  public function swivelDatum():AssemblyFrame return F.fromRotationMatrix(swivelNativeX, 0, nativeHobCentreZ, swivelRotation);


  public function outboardDatum():AssemblyFrame return F.fromRotationMatrix(0, -85, 0, spindleRotation);

  /** The running gap the housing keeps from every fixed or moving part of the drive, mm. */
  public final zRunningGap:Float;

  /** The side of the housing the screw stands on: +1 is +y. */
  public final zScrewSide:Float;

  /** The offset is rounded up to a whole millimetre. */
  public final zOffsetStep:Float;

  /** Clearance of the floating pad's seat holes from its relief, mm. */
  public final zHoleMargin:Float;

  /** Widest drive seat insert the casting wants (the width of the old central one), mm. */
  public final zMaxInsertWidth:Float;

  /** What a surface point's closed-form distance to the envelope may be off by (tolerance and rounding), mm. */
  final zPointMargin:Float;


  final zOffsetCache = new machinekit.cache.Once<ZOffsetResult>();


  public function zAxisY():Float return zOffset().y;


  public function zOffset():ZOffsetResult return zOffsetCache.get(() -> deriveZOffset());


  /** Width of the drive seat insert at the derived offset: the widest the casting allows, up to `zMaxInsertWidth`. */
  public function zDriveInsertWidth():Float {
    var r = zOffset();
    return Math.min(zMaxInsertWidth, 2 * (MineralCastColumn.COLUMN_HEAD_WIDTH / 2 - Math.abs(r.y)));
  }


  /** The housing envelope (body, rear boss, connectors) swept over the Z joint range `lo`..`hi`, grown by `grow`, in the work frame. */
  public function envelope(lo:Float, hi:Float, grow:Float):Part {
    var segments = envelopeSegments(grow);
    var pieces:Array<Part> = [];
    var zLow = nativeHobCentreZ + lo, zHigh = nativeHobCentreZ + hi;
    for (s in segments) {
      for (z in (hi > lo ? [zLow, zHigh] : [zLow])) {
        var c = Part.cylinderSpan(s.r, s.x0, s.x1);
        pieces.push(PosedParts.moved(c, AxisBuilder.orient(0, 0, z, [0.0, 0, 1], [1.0, 0, 0])));
        c.close();
      }
      if (hi > lo) {
        var raw = Part.box(s.x1 - s.x0, 2 * s.r, zHigh - zLow), moved = raw.translated(new Vector((s.x0 + s.x1) / 2, 0, zLow));
        raw.close();
        pieces.push(moved);
      }
    }
    return Solids.union(pieces);
  }


  /**
   * The distance from a point to `envelope(lo, hi, grow)`, in closed form: each segment is the set within its radius, across
   * y and z, of the line z = nativeHobCentreZ + lo .. nativeHobCentreZ + hi at y = 0, over its x span (a cylinder along x when lo = hi), so the
   * distance to it is the x overshoot combined with the radial overshoot beyond the radius.
   */
  public function envelopeDistance(lo:Float, hi:Float, grow:Float, x:Float, y:Float, z:Float):Float {
    var zLow = nativeHobCentreZ + lo, zHigh = nativeHobCentreZ + Math.max(lo, hi), best = Math.POSITIVE_INFINITY;
    for (s in envelopeSegments(grow)) {
      var dx = Math.max(0, Math.max(s.x0 - x, x - s.x1)), dz = z - Math.max(zLow, Math.min(zHigh, z));
      var radial = Math.max(0, Math.sqrt(y * y + dz * dz) - s.r);
      best = Math.min(best, Math.sqrt(dx * dx + radial * radial));
    }
    return best;
  }


  function envelopeSegments(grow:Float):Array<{x0:Float, x1:Float, r:Float}
> {
    var front = swivelNativeX + E.PILOT_HEIGHT;
    // Segments along work x: the Ø110 body from the front face, the Ø85 rear boss, the connectors (the boss's envelope to the overall length).
    var bodyEnd = swivelNativeX + E.BODY_LENGTH - E.REAR_SPIGOT_HEIGHT;
    var bossEnd = swivelNativeX + E.BODY_LENGTH;
    var connectorEnd = swivelNativeX + E.OVERALL_LENGTH;
    return [
      {x0: front - grow, x1: bodyEnd + grow, r: E.BODY_DIAMETER / 2 + grow},
      {x0: bodyEnd, x1: bossEnd + grow, r: E.REAR_SPIGOT_DIAMETER / 2 + grow},
      {x0: bossEnd, x1: connectorEnd + grow, r: E.REAR_SPIGOT_DIAMETER / 2 + grow}
    ];
  }

  /** The relief the floating pad needs: the housing's envelope over the Z travel grown by the running gap. */
  public function relief():Part {
    var s = HobberScales.stroke("Z");
    return envelope(s.lower, s.upper, zRunningGap);
  }


  function deriveZOffset():ZOffsetResult {
    var s = HobberScales.stroke("Z");
    var drive = HobberDrives.zAt(0.0);
    var nut = new BallNut(), poses = HobberDrives.poses(drive);
    var swept = envelope(s.lower, s.upper, 0), here = envelope(0, 0, 0), grown = relief();
    var fixedMount = PosedParts.moved(drive.drive.fixed.geometry(), poses.get("mount"));
    var floatingSupport = PosedParts.moved(drive.drive.floating.geometry(), poses.get("floating"));
    var nutPart = PosedParts.moved(nut.geometry(), poses.get("nut"));
    var block = HobberDrives.nutBlock(drive, nut), blockPart = PosedParts.moved(block.geometry(), HobberDrives.nutBlockPose(drive, nut));
    var tools:Array<Part> = [];
    var holes = HobberDrives.seatHoles(drive, "floating"), reach = 0.0;
    for (support in ["fixed", "floating"]) for (h in HobberDrives.seatHoles(drive, support)) reach = Math.max(reach, Math.abs(h.position[1]) + h.diameter / 2);
    for (h in holes) {
      var cylinder = Part.cylinderSpan(h.diameter / 2, -1, h.depth);
      var up = Math.abs(h.direction[2]) < 0.9 ? [0.0, 0, 1] : [1.0, 0, 0];
      tools.push(PosedParts.moved(cylinder, AxisBuilder.orient(h.position[0], h.position[1], h.position[2], up, h.direction)));
      cylinder.close();
    }
    // Obstacles: name, part at y = 0, the housing it is measured against.
    var obstacles = [
      {name: "BK12 mount (fixed)", part: fixedMount, against: swept, lo: s.lower, hi: s.upper},
      {name: "BF12 support (floating)", part: floatingSupport, against: swept, lo: s.lower, hi: s.upper},
      {name: "ball nut", part: nutPart, against: here, lo: 0.0, hi: 0.0},
      {name: "nut block", part: blockPart, against: here, lo: 0.0, hi: 0.0}
    ];
    function obstacleGap(o:{name:String, part:Part, against:Part, lo:Float, hi:Float}, a:Float):Float {
      var moved = o.part.translated(new Vector(0, zScrewSide * a, 0)), d = moved.distance(o.against);
      moved.close();
      return d;
    }
    function holesGap(a:Float):Float {
      var worst = 1e9;
      for (t in tools) {
        var moved = t.translated(new Vector(0, zScrewSide * a, 0)), d = moved.distance(grown);
        moved.close();
        worst = Math.min(worst, d);
      }
      return worst;
    }
    function gapsAt(a:Float):Array<{name:String, gap:Float}> {
      var out = [for (o in obstacles) {name: o.name, gap: obstacleGap(o, a)}];
      out.push({name: "floating pad seat holes to relief", gap: holesGap(a)});
      return out;
    }
    // A point of an obstacle bounds its gap from above by the point's distance to the housing's envelope (closed form), so a
    // step where some point is short of the need fails without the exact query, by at least that shortfall. The points are
    // the nodes of a coarse tessellation, which lie on the obstacle's surface, taken the first time an obstacle needs them.
    // `zPointMargin` keeps the nodes' tolerance and the closed form's rounding from deciding a step.
    var points:Array<Null<Array<Float>>> = [for (_ in obstacles) null];
    function pointGap(index:Int, a:Float):Float {
      var o = obstacles[index], nodes = points[index];
      if (nodes == null) {
        var mesh = o.part.shape.tessellate(1.0, 0.5);
        nodes = [for (i in 0...mesh.vertexCount * 3) mesh.vertices.getDouble(i * 8)];
        points[index] = nodes;
      }
      var best = Math.POSITIVE_INFINITY, shift = zScrewSide * a;
      for (i in 0...Std.int(nodes.length / 3)) best = Math.min(best,
        envelopeDistance(o.lo, o.hi, 0, nodes[3 * i], nodes[3 * i + 1] + shift, nodes[3 * i + 2]));
      return best + zPointMargin;
    }
    // Boxes bound each part, so their separation bounds the distance from below. A step an obstacle's boxes clear by a
    // millimetre beyond its need passes that obstacle without the exact query (the gap of a failing one is still exact).
    var boxes = [for (o in obstacles) {part: PosedParts.boxOf(o.part), against: PosedParts.boxOf(o.against)}];
    function boxGap(index:Int, a:Float):Float {
      var p = boxes[index].part, q = boxes[index].against, shift = zScrewSide * a;
      var dx = Math.max(0, Math.max(p.minX - q.maxX, q.minX - p.maxX));
      var dy = Math.max(0, Math.max(p.minY + shift - q.maxY, q.minY - p.maxY - shift));
      var dz = Math.max(0, Math.max(p.minZ - q.maxZ, q.minZ - p.maxZ));
      return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    /** How far the first failing gap at `a` falls short of its need (beyond the 1e-6 acceptance), or 0 when all pass. */
    function shortfall(a:Float):Float {
      for (index in 0...obstacles.length) {
        if (boxGap(index, a) >= zRunningGap + 1) continue;
        var bound = pointGap(index, a);
        if (bound < zRunningGap - 1e-6) return zRunningGap - 1e-6 - bound;
        var gap = obstacleGap(obstacles[index], a);
        if (gap < zRunningGap - 1e-6) return zRunningGap - 1e-6 - gap;
      }
      var gap = holesGap(a);
      return gap < zHoleMargin - 1e-6 ? zHoleMargin - 1e-6 - gap : 0;
    }
    // The gaps that can govern at `a`: an obstacle whose boxes already clear it by more than the least margin measured
    // cannot be the least, so its exact gap is not needed to name the governing one.
    function governingGaps(a:Float):Array<{name:String, gap:Float}> {
      var holes = holesGap(a), best = holes - zHoleMargin, exact:Array<Null<Float>> = [for (_ in obstacles) null];
      var order = [for (index in 0...obstacles.length) index];
      order.sort((i, j) -> Reflect.compare(boxGap(i, a), boxGap(j, a)));
      for (index in order) {
        if (boxGap(index, a) - zRunningGap > best) continue;
        var gap = obstacleGap(obstacles[index], a);
        exact[index] = gap;
        best = Math.min(best, gap - zRunningGap);
      }
      var out = [for (index in 0...obstacles.length) if (exact[index] != null) {name: obstacles[index].name, gap: (exact[index] : Float)}];
      out.push({name: "floating pad seat holes to relief", gap: holes});
      return out;
    }
    function governing(g:Array<{name:String, gap:Float}>):String {
      var name = "", worst = 1e9;
      for (item in g) {
        var need = item.name.indexOf("holes") >= 0 ? zHoleMargin : zRunningGap;
        if (item.gap - need < worst) {
          worst = item.gap - need;
          name = item.name;
        }
      }
      return name;
    }
    // The first passing step of the 1 mm scan. Moving a part by d changes its distance to another by at most d, so a
    // gap short of its need by more than k mm still fails k steps on: those steps are skipped, not assumed. A step
    // stops at its first failing gap; the reported gaps are measured in full below.
    var found:Null<Float> = null;
    var a = 0.0;
    while (a <= 130) {
      var deficit = shortfall(a);
      if (deficit <= 0) {
        found = a;
        break;
      }
      a += Math.max(1.0, Math.ceil(deficit - 1e-6));
    }
    var maximum = MineralCastColumn.COLUMN_HEAD_WIDTH / 2 - (reach + MineralCastBed.SOURCE_STEEL_COVER_MM);
    var below = found == null ? [] : governingGaps(Math.max(0.0, found - 1.0));
    var g = found == null ? [] : gapsAt(found);
    for (o in obstacles) o.part.close();
    for (t in tools) t.close();
    swept.close();
    here.close();
    grown.close();
    if (found == null) throw "Z screw offset: no |y| up to 130 mm keeps " + zRunningGap + " mm from the A axis housing";
    var y = zScrewSide * found;
    // The step is 1 mm and the scan runs in steps of 1 mm, so `found` is already rounded up.
    if (found > maximum + 1e-9)
      throw "Z screw offset: the housing needs |y| >= " + found + " mm (" + governing(below) + " at " + (found - 1) + " mm), the drive seat's casting cover allows at most "
        + maximum + " mm (seat holes reach " + reach + " mm from the screw line, cover " + MineralCastBed.SOURCE_STEEL_COVER_MM + " mm, head half-width "
        + MineralCastColumn.COLUMN_HEAD_WIDTH / 2 + " mm)";
    return {y: y, minimum: found, maximum: maximum, governing: governing(below), gaps: g, insertHalfWidth: reach + MineralCastBed.SOURCE_STEEL_COVER_MM};
  }

  public function rails(kind:String, rows:Array<CastInsertPlacement>):Array<HobberPlacedRail> {
    var rail = switch kind {
      case "bed":
        LinearRail.metric("HGH25CA", xNominal().railLength, LinearRail.MEASURED);
      case "column":
        LinearRail.metric("HGH15CA", zRailLength);
      default:
        throw "Unknown hobber foundation rail family: " + kind;
    };
    var frame = foundationDatum(kind), alongSign = kind == "bed" ? 1.0 : -1.0;
    return [for (side in [-1, 1]) {
      var suffix = side < 0 ? "Left" : "Right", id = "rail" + suffix;
      var matches = rows.filter(row -> row.id == id);
      if (matches.length != 1) throw "Bought rail needs one declared source seat: " + kind + "/" + id;
      var row = matches[0], size = row.part.boxDimensions(id);
      var face = F.compose(F.compose(frame, row.pose), F.translation(0, 0, size.height));
      var start = F.transformPoint(face, 0, -alongSign * rail.length / 2, 0);
      var normal = F.transformVector(face, 0, 0, 1), along = F.transformVector(face, 0, alongSign, 0);
      {
        memberId:(kind == "bed" ? "railX" : "railZ") + suffix, insertId : id, rail : rail, pose : AxisBuilder.orient(
          start.x + normal.x * rail.spec.railHeight,
          start.y + normal.y * rail.spec.railHeight,
          start.z + normal.z * rail.spec.railHeight,
          [normal.x, normal.y, normal.z],
          [along.x, along.y, along.z]
        ), contact : F.alongY(
          start.x,
          start.y,
          start.z,
          normal.x,
          normal.y,
          normal.z
        )
      };
    }
    ];
  }


  public function railFingerprint(placed:HobberPlacedRail):String {
    var rail = placed.rail;
    return haxe.crypto.Sha256.encode(materia.tooling.ToolingCodec.canonical({
      definition: machinekit.assembly.MachineAssembly.definitionKey(placed.memberId, rail),
      specification: rail.spec,
      length: rail.length,
      mountingHole: rail.mountingHole(),
      connectors: rail.mountingConnectors()
    }));
  }

  public final zBellowsRow="BELLOWS-PU-LOW-8x10";
  function zBellowsGuide():machinekit.motion.LinearRailProfile.LinearRailProfileSpec {
    var g=LinearRailSystem.catalog().get("HGH15CA");if(g==null)throw "Missing Z zBellowsGuide";return g;
  }
  final zBellowsRailCache=new machinekit.cache.Once<HobberSkeleton.HobberPlacedRail>();
  function zBellowsRail():HobberSkeleton.HobberPlacedRail return zBellowsRailCache.get(()->rails("column",foundationPlacements("column"))[0]);
  public function zBellowsSurfaceX():Float return zBellowsRail().contact.x;
  public function zBellowsLowerLimit():Float return zTravelMin-HobberScales.overtravel();
  public function zBellowsUpperLimit():Float return zTravelMax+HobberScales.overtravel();
  final zBellowsPlanCache = new machinekit.cache.Once<WayCoverPlan>();
  public function zBellowsPlan():WayCoverPlan return zBellowsPlanCache.get(() -> deriveZBellowsPlan());
  function deriveZBellowsPlan():WayCoverPlan {
    var row=WayCoverCatalog.get(zBellowsRow),g=zBellowsGuide();
    var width=Math.ceil((g.blockWidth+2*ScrewLinearModuleSpec.GAP)/10)*10;
    var height=g.railHeight+ScrewLinearModuleSpec.GAP+0.2;
    var space=Math.ceil((row.foldCompressed*Math.ceil((zBellowsUpperLimit()-zBellowsLowerLimit())/(row.ratio-1)/row.foldCompressed)+2*row.flangeThickness)/10)*10;
    return new WayCoverPlan(row,width,height,zBellowsUpperLimit()-zBellowsLowerLimit(),space,
      WayCoverPlan.boxCover(0,zBellowsSurfaceX()-HobberZSaddle.SWIVEL_PLATE_BACK-2*ScrewLinearModuleSpec.GAP,100000,true));
  }
  public function zBellowsMovingZ(upper:Bool):Float return upper
    ?nativeHobCentreZ+zBlockSpacing+zBellowsGuide().blockLength/2+ScrewLinearModuleSpec.GAP+zBellowsPlan().row.flangeThickness
    :HobberXCarriage.xCarriageTop()+zBellowsPlan().space-zBellowsLowerLimit();
  public function zBellowsFixedZ(upper:Bool):Float return upper?zBellowsMovingZ(true)+zBellowsUpperLimit()+zBellowsPlan().space:HobberXCarriage.xCarriageTop();
  public function zBellowsPose(side:Int,upper:Bool,fixed:Bool):AssemblyFrame return AxisBuilder.orient(zBellowsSurfaceX(),side*columnRailCentre,fixed?zBellowsFixedZ(upper):zBellowsMovingZ(upper),[-1.0,0,0],[0.0,0,(upper?1.0:-1.0)*(fixed?-1:1)]);
  public function zBellowsMovingMount():WayCoverMount {
    var p=zBellowsPlan(),half=p.fullPlateHalf(),offset=zBellowsSurfaceX()-HobberZSaddle.SWIVEL_PLATE_BACK;
    var leg=24.0;
    var stock:Array<MountStock>=[{x0:-half,x1:half,y0:offset-4,y1:offset,z0:0.0,z1:8.0}];
    for(side in [-1,1])stock.push({x0:side<0?-half:p.width/2,x1:side<0?-p.width/2:half,y0:offset-4,y1:offset,z0:-leg,z1:0.0});
    var cuts:Array<WayCoverMount.MountCut>=[{x0:-p.width/2,x1:p.width/2,y0:-1.0,y1:p.shape.hostMax+1}];
    zBellowsTrimEnds(cuts,half,p.shape.hostMax);
    return new WayCoverMount("HOBBER-Z-BELLOWS-MOVING-MOUNT",half,p.shape.hostMax,8,[],cuts,0,
      {stock:stock,bolted:[BoltedFace.clamped("saddle",BoltedFace.frameOf({x:0.0,y:offset,z:-leg/2-ScrewLinearModuleSpec.GAP},{x:0.0,y:-1,z:0},{x:1.0,y:0,z:0}),
        [{x:-26.0,y:0.0},{x:26.0,y:0.0}],4.5,4,"M4",null,false,true)],pinned:[]});
  }
  public function zBellowsFixedUpperMount():WayCoverMount {
    var p=zBellowsPlan(),half=p.fullPlateHalf(),railHalf=zBellowsGuide().railWidth/2;
    var stock:Array<MountStock>=[];
    for(side in [-1,1]) {
      stock.push({x0:side<0?-17.0:railHalf,x1:side<0?-railHalf:17.0,y0:0.0,y1:8.0,z0:-28.0,z1:0.0});
      // Continue the inner feet down inside the protected opening, clear of
      // the nested skins. Their two rail-seat contact strips exceed625 mm2.
      stock.push({x0:side<0?-p.width/2+3:railHalf,x1:side<0?-railHalf:p.width/2-3,y0:0.0,y1:8.0,z0:-1.0,z1:20.0});
    }
    var screw=machinekit.standard.SocketHeadCapScrew.catalog().get("M3"),x=railHalf+1.5*screw.diameter;
    var cuts:Array<WayCoverMount.MountCut>=[{x0:-railHalf-ScrewLinearModuleSpec.GAP,x1:railHalf+ScrewLinearModuleSpec.GAP,y0:-1.0,y1:p.height}];
    zBellowsTrimEnds(cuts,half,p.shape.hostMax);
    return new WayCoverMount("HOBBER-Z-BELLOWS-UPPER-MOUNT",half,p.shape.hostMax,8,[],
      cuts,0,
      {stock:stock,bolted:[BoltedFace.clamped("column",BoltedFace.frameOf({x:0.0,y:0.0,z:-18.0},{x:0.0,y:1,z:0},{x:1.0,y:0,z:0}),
        [for(xx in [-x,x])for(zz in [-4.0,4.0]){x:xx,y:zz}],3.4,8,"M3",null,false,true)],pinned:[]});
  }
  /** Only the end plate is relieved. The M4 legs behind it retain their
   * full width and edge distances. The side cuts end at their shoulder. */
  function zBellowsTrimEnds(cuts:Array<WayCoverMount.MountCut>,half:Float,height:Float):Void {
    var d=machinekit.standard.SocketHeadCapScrew.catalog().get("M3").diameter;
    for(side in [-1,1]) {
      cuts.push({x0:side<0?-half-1:half-ScrewLinearModuleSpec.GAP,x1:side<0?-half+ScrewLinearModuleSpec.GAP:half+1,y0:-1.0,y1:height});
      cuts.push({x0:side<0?-half-1:half-1.5*d,x1:side<0?-half+1.5*d:half+1,y0:-1.0,y1:d+0.2});
    }
  }
  public function zBellowsSaddleReceivers():Array<BoltedFace> {
    var p=zBellowsPlan(),mount=zBellowsMovingMount(),face=mount.fasteningFaces().filter(f->f.id=="saddle")[0],out=[];
    for(side in [-1,1])for(upper in [false,true]) {
      var at=F.compose(zBellowsPose(side,upper,false),face.frame);at=F.compose(F.translation(0,0,-nativeHobCentreZ),at);
      out.push(BoltedFace.drilledToSuit("bellows-"+side+"-"+upper,F.compose(at,F.fromRotationMatrix(0,0,0,[1.0,0,0,0,-1,0,0,0,-1])),2*mount.halfWidth,24,12,13,BoltedFace.ThreadMaterial.Steel,"M4"));
    }
    return out;
  }
  public function zBellowsDeckReceivers():Array<BoltedFace> {
    var p=zBellowsPlan(),out=[];
    for(side in [-1,1]) {
      var frame=F.compose(F.compose(zBellowsPose(side,false,true),F.translation(0,p.shape.hostMax/2,0)),F.fromRotationMatrix(0,0,0,[1.0,0,0,0,-1,0,0,0,-1]));
      out.push(BoltedFace.drilledToSuit("z-bellows-"+side,frame,2*p.fullPlateHalf(),p.shape.hostMax,8,HobberXCarriage.X_CARRIAGE_THICKNESS,BoltedFace.ThreadMaterial.Steel,"M3"));
    }
    return out;
  }

  public final floorZ:Float = 0.0;
  public final electricalPanelX:Float = -650.0;
  public final electricalPanelY:Float = 100.0;

  /**
   * The rack stands free on the floor with no mate, so this pose is used as given: in the machine frame, whose floor (the feet's
   * soles) is z 0. Mated members are solved from the bed instead, which is why the work-frame floor height does not apply here.
   */
  public function electricalPanelPose():AssemblyFrame return F.translation(electricalPanelX, electricalPanelY, floorZ);

  public function electricalDevicePose(offsetX:Float, heightFromFloor:Float):AssemblyFrame {
    var panel = electricalPanelPose();
    // Device local +Z points out of the plate, towards work -Y; local +Y points up.
    return F.fromRotationMatrix(
      panel.x + offsetX,
      panel.y - 10,
      panel.z + heightFromFloor,
      [1, 0, 0, 0, 0, -1, 0, 1, 0]
    );
  }
}
