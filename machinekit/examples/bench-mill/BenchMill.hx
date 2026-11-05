import machinekit.assembly.MachineAssembly;
import machinekit.assembly.Transmission;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentValues;
import machinekit.component.BomItem.BomCut;
import machinekit.component.Procurement.ProcurementCodec;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.milling.MillCasting;
import machinekit.milling.MillCasting.MillBase;
import machinekit.milling.MillCasting.ColumnBoss;
import machinekit.milling.MillCasting.MillColumn;
import machinekit.milling.MillCasting.MillSaddle;
import machinekit.milling.MillCasting.MillTable;
import machinekit.milling.MillCasting.MillHead;
import machinekit.cover.WayCover;
import machinekit.cover.WayCoverCatalog;
import machinekit.cover.WayCoverMount;
import machinekit.cover.WayCoverSection;
import machinekit.cover.WayCoverMount.MountBore;
import machinekit.cover.WayCoverMount.MountCut;
import machinekit.cover.WayCoverPlan;
import machinekit.cover.WayWiperCatalog;
import machinekit.design.DesignRequirement;
import machinekit.milling.SpindleCartridge;
import machinekit.milling.SpindleCartridge.Er20Holder;
import machinekit.milling.SpindleMotor;
import machinekit.motion.BallScrew;
import machinekit.motion.BallNut;
import machinekit.motion.ScrewSupportUnit;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.CableChain;
import machinekit.motion.ServoMotor;
import machinekit.motion.MotorFlange;
import machinekit.motion.ShaftBrakePart;
import machinekit.motion.MotorDriver;
import machinekit.power.ThreePhaseRectifier;
import machinekit.motion.ShaftCoupling;
import machinekit.motion.ShaftEncoder;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingPulley;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import toolpathkit.tool.Tool;
import toolpathkit.setup.Fixture;
import CncRouter.EndMill;
import CncRouter.RouterPlate;
import CncRouter.ToeClamp;

/** Machine coordinates, mm. */
typedef MillAxisSpec = {
  var id:String;
  var lower:Float;
  var upper:Float;
  var initial:Float;
}

/** Installed parts of one axis; references are shared with the mechanical assembly. */
typedef MillAxisDrive = {
  var spec:MillAxisSpec;
  var screw:BallScrew;
  var motor:ServoMotor;
  var coupling:ShaftCoupling;
  var brake:Null<ShaftBrakePart>;
}

/** Table X and saddle Y move opposite the tool's work coordinates; the head moves along Z.
 * All mates are solved from the zero poses, as on the router. The rails, screw supports and
 * current motor parts decide room, speed and acceleration every time the machine is compiled.
 */
class BenchMill extends MachineAssembly implements machinekit.cover.WayCover.CoverMembers {
  public static inline var RAIL:String = "HGH15CA";
  public static inline var STOCK_WIDTH:Float = 60;
  public static inline var STOCK_DEPTH:Float = 60;
  public static inline var STOCK_HEIGHT:Float = 20;
  public static inline var STEP_TICK_HZ:Int = 40000;
  /** Assumed assembly gap between carriage blocks, in mm. */
  public static inline var BLOCK_GAP:Float = 4;
  public final vise:Null<machinekit.milling.PneumaticVise>;
  public final base:MillBase;
  public final column:MillColumn;
  public final saddle:MillSaddle;
  public final table:MillTable;
  public final head = new MillHead();
  public final spindle = new SpindleCartridge();
  public final holder = new Er20Holder();
  public final tool = new EndMill(Er20Holder.BORE_DIAMETER, 22, 35);
  public final guideBlock:LinearRailBlock = LinearRailBlock.metric(RAIL);
  public final specs:Array<MillAxisSpec> = [
    {id: "x", lower: 0, upper: 250, initial: 125},
    {id: "y", lower: 0, upper: 150, initial: 75},
    {id: "z", lower: -250, upper: 0, initial: 0}
  ];
  public final axisDrives:Array<MillAxisDrive> = [];
  public final tableTop:Float;
  public final gaugeZero:AssemblyFrame;
  final zeroPoses = new Map<String, AssemblyFrame>();
  final overtravel = new Map<String, Float>();
  final axisDirections = new Map<String, Array<Float>>();
  /** The six installed way covers, by id. */
  public final wayCovers = new Map<String, WayCover>();
  final powerCableCuts:Array<BomCut> = [];
  final encoderCableCuts:Array<BomCut> = [];

  public function new(withVise:Bool = false, includeWorkpiece:Bool = true) {
    super();
    base = new MillBase();
    column = new MillColumn(zBosses());
    saddle = new MillSaddle();
    table = new MillTable();
    vise = withVise ? new machinekit.milling.PneumaticVise(STOCK_DEPTH, STOCK_WIDTH) : null;
    var railSpec = guideBlock.spec;
    var halfSpan = (railSpec.blockLength + BLOCK_GAP) / 2;
    // The saddle's two blocks on a Y rail stand under its two X rails. The saddle is bolted to them from its top face, through
    // pockets, and the blocks' four holes (26 mm apart) then fall either side of each X rail: the inner pockets between the
    // rails, the outer ones between the rail and the casting's edge, so a driver reaches every screw.
    var yHalfSpan = saddle.railSpacing / 2;
    var up = [0.0, 0, 1], alongX = [1.0, 0, 0], alongY = [0.0, 1, 0];
    var baseTop = base.height;
    var saddleBottom = baseTop + railSpec.blockHeight;
    var saddleTop = saddleBottom + saddle.height;
    var tableBottom = saddleTop + railSpec.blockHeight;
    tableTop = tableBottom + table.height;
    var columnSeat = base.connector("column").frame;
    var columnPose = AssemblyFrames.translation(columnSeat.x, columnSeat.y, columnSeat.z);
    var columnFront = columnPose.y - column.depth / 2;
    var headBack = columnFront - railSpec.blockHeight;
    var headY = headBack - head.depth / 2;
    var spindleY = headY + head.connector("spindle").frame.y;
    gaugeZero = AssemblyFrames.translation(0, spindleY, tableTop + tool.stickout - specs[2].lower);
    var saddleY = spindleY + specs[1].upper / 2;
    var tableX = specs[0].upper / 2;
    place("base", base, AssemblyFrames.translation(0, 0, 0));
    attach("column", column, columnPose, "base");
    var yRailStart = -base.depth / 2 + 5;
    var yRailLength = columnFront - 5 - yRailStart;
    var yRails:Array<LinearRail> = [];
    for (side in [-1, 1]) {
      var name = side < 0 ? "Left" : "Right";
      var guide = LinearRail.metric(RAIL, yRailLength);
      yRails.push(guide);
      attach(
        'railY$name',
        guide,
        orient(side * base.railSpacing / 2, yRailStart, baseTop + railSpec.railHeight, up, alongY),
        "base"
      );
    }
    var yBlocks = [for (side in [-1, 1]) for (span in [-yHalfSpan, yHalfSpan]) orient(
      side * base.railSpacing / 2,
      saddleY + span,
      baseTop + railSpec.railHeight,
      up,
      alongY
    )];
    slide(specs[1], "railYLeft", yRails[0], "blockYLeft", guideBlock, yBlocks[0], [0.0, -1, 0], yBlocks);
    attach("saddle", saddle, AssemblyFrames.translation(0, saddleY, saddleBottom), "blockYLeft");
    attach("blockYLeftRear", guideBlock, yBlocks[1], "saddle");
    attach("blockYRight", guideBlock, yBlocks[2], "saddle");
    attach("blockYRightRear", guideBlock, yBlocks[3], "saddle");

    var xRailLength = saddle.width - 10;
    var xRails:Array<LinearRail> = [];
    for (side in [-1, 1]) {
      var name = side < 0 ? "Front" : "Back";
      var guide = LinearRail.metric(RAIL, xRailLength);
      xRails.push(guide);
      attach(
        'railX$name',
        guide,
        orient(-xRailLength / 2, saddleY + side * saddle.railSpacing / 2, saddleTop + railSpec.railHeight, up, alongX),
        "saddle"
      );
    }
    var xBlocks = [for (side in [-1, 1]) for (span in [-halfSpan, halfSpan]) orient(
      tableX + span,
      saddleY + side * saddle.railSpacing / 2,
      saddleTop + railSpec.railHeight,
      up,
      alongX
    )];
    slide(specs[0], "railXFront", xRails[0], "blockXFront", guideBlock, xBlocks[0], [-1.0, 0, 0], xBlocks);
    attach("table", table, AssemblyFrames.translation(tableX, saddleY, tableBottom), "blockXFront");
    attach("blockXFrontRear", guideBlock, xBlocks[1], "table");
    attach("blockXBack", guideBlock, xBlocks[2], "table");
    attach("blockXBackRear", guideBlock, xBlocks[3], "table");

    var headZ = gaugeZero.z + spindle.connector("mount").frame.z;
    var zRailStart = columnPose.z + railSpec.railEndMargin;
    var zRailLength = column.height - 2 * railSpec.railEndMargin;
    var zBlock = headZ + head.height / 2;
    var zRails:Array<LinearRail> = [];
    for (side in [-1, 1]) {
      var name = side < 0 ? "Left" : "Right";
      var guide = LinearRail.metric(RAIL, zRailLength);
      zRails.push(guide);
      attach(
        'railZ$name',
        guide,
        orient(side * column.railSpacing / 2, columnFront - railSpec.railHeight, zRailStart, [0.0, -1, 0], [0.0, 0, 1]),
        "column"
      );
    }
    var zBlocks = [for (side in [-1, 1]) for (span in [-halfSpan, halfSpan]) orient(
      side * column.railSpacing / 2,
      columnFront - railSpec.railHeight,
      zBlock + span,
      [0.0, -1, 0],
      [0.0, 0, 1]
    )];
    slide(specs[2], "railZLeft", zRails[0], "blockZLeft", guideBlock, zBlocks[0], [0.0, 0, 1], zBlocks);
    attach("head", head, AssemblyFrames.translation(0, headY, headZ), "blockZLeft");
    attach("blockZLeftTop", guideBlock, zBlocks[1], "head");
    attach("blockZRight", guideBlock, zBlocks[2], "head");
    attach("blockZRightTop", guideBlock, zBlocks[3], "head");
    // The at-speed switch returns real feedback to the CNC program.
    addComponent("spindle", spindle);
    zeroPoses.set("spindle", gaugeZero);
    connect("head", "spindle");
    addMateOnAxis(
      "spindle-turn",
      "continuous",
      "head",
      "to-spindle",
      "spindle",
      "attach-spindle",
      {x: 0, y: 0, z: 1},
      0,
      {lower: null, upper: null, velocity: SpindleCartridge.MAX_RPM * 2 * Math.PI / 60, effort: null}
    );
    addAtSpeed("spindle-at-speed", "spindle-turn");
    attach("holder", holder, gaugeZero, "spindle");
    attach("tool", tool, AssemblyFrames.translation(0, spindleY, gaugeZero.z - tool.stickout), "spindle");
    var installedVise = vise;
    if (installedVise == null) {
      if (includeWorkpiece) {
        attach(
          "stock",
          new RouterPlate(null, STOCK_WIDTH, STOCK_DEPTH, STOCK_HEIGHT, "aluminium 6061", "Bearing block blank", null, null, true),
          AssemblyFrames.translation(tableX, saddleY, tableTop),
          "table"
        );
        // Both clamps hold the back corners, clear of the bearing seat and fastening recesses.
        for (side in [-1, 1]) attach(
          side < 0 ? "clampLeft" : "clampRight",
          new ToeClamp(STOCK_HEIGHT),
          AssemblyFrames.fromRotationMatrix(
            tableX + side * (STOCK_WIDTH / 2 - 8),
            saddleY + STOCK_DEPTH / 2,
            tableTop,
            [0.0, -1, 0, 1.0, 0, 0, 0.0, 0, 1]
          ),
          "table"
        );
      }
    } else {
      // Mount off centre only as far as needed to bring the entire vise beside
      // the retracted head at the X stop. Keep the complete blank machinable.
      var halfTravel = (specs[0].upper - specs[0].lower) / 2;
      var loadingRadius = head.width
        / 2 + installedVise.body.width / 2 + robotkit.collision.CollisionClearance.MARGIN * 1000;
      var fixtureOffset = Math.max(0.0, loadingRadius - halfTravel);
      if (fixtureOffset + STOCK_WIDTH / 2 > halfTravel || fixtureOffset + installedVise.body.width / 2
        > table.width / 2) throw "The vise cannot load beside the head while retaining full blank travel";
      var visePose = AssemblyFrames.translation(tableX + fixtureOffset, saddleY, tableTop);
      include("vise", installedVise, visePose);
      zeroPoses.set("vise/body", visePose);
      connect("table", "vise/body");
      addMate("vise-mount", "fixed", "table", "to-vise/body", "vise/body", "attach-vise/body");
      if (includeWorkpiece) attach(
        "stock",
        new RouterPlate(null, STOCK_WIDTH, STOCK_DEPTH, STOCK_HEIGHT, "aluminium 6061", "Bearing block blank", null, null, true),
        AssemblyFrames.compose(zeroPose("vise/body"), installedVise.seat),
        "vise/body"
      );
    }

    exposeConnector("gaugeLine", "spindle", "gaugeLine");
    exposeConnector("toolTip", "tool", "tip");
    attach(
      "powerSupply",
      new ThreePhaseRectifier(
        new machinekit.power.AcMains(ThreePhase, new machinekit.power.AcRmsVoltage(400), 50),
        new machinekit.power.DcCurrent(20),
        4,
        240,
        120,
        100,
        8000
      ),
      AssemblyFrames.translation(-190, columnPose.y, 0),
      "base"
    );
    exposePort("mains", "powerSupply", "mains");
    var xScrewLength = table.width
      + specs[0].upper - specs[0].lower + 2 * (BallScrew.INPUT_JOURNAL_LENGTH + ScrewSupportUnit.bk12().length);
    driveScrew(
      specs[0],
      "saddle",
      "table",
      [-xScrewLength / 2, saddleY, tableBottom - railSpec.blockHeight / 2],
      [1.0, 0, 0],
      [0.0, 0, 1],
      xScrewLength,
      [tableX, saddleY, tableBottom - railSpec.blockHeight / 2]
    );
    var yNut = new BallNut();
    var yNutFace = yNut.bodyLength / 2 + yNut.flangeThickness;
    driveScrew(
      specs[1],
      "base",
      "saddle",
      [0.0, yRailStart, saddleBottom + saddle.screwHeight],
      [0.0, 1, 0],
      [0.0, 0, 1],
      yRailLength + Y_SCREW_EXTENSION,
      [0.0, saddleY + saddle.depth / 2 - yNutFace, saddleBottom + saddle.screwHeight]
    );
    var zScrewTop = columnPose.z + column.height + railSpec.railEndMargin;
    var zScrewX = column.width / 2 + new BallNut().flangeDiameter / 2 + 2;
    driveScrew(
      specs[2],
      "column",
      "head",
      [zScrewX, (headBack + columnFront) / 2, zScrewTop],
      [0.0, 0, -1],
      [1.0, 0, 0],
      column.height - Z_SCREW_SHORTENING,
      [zScrewX, (headBack + columnFront) / 2, headZ + head.height / 2]
    );
    var pulley = new TimingPulley(HTD5M, 20, 20, 15);
    var motorPulley = new TimingPulley(HTD5M, 20, 20, 15);
    var belt = TimingBelt.twoPulley(HTD5M,
      20, 20, 150, 15, 5000); // Assumed glass-cord EA per mm width, pending vendor data.
    var beltZ = gaugeZero.z + spindle.connector("pulley").frame.z;
    attach("spindlePulley", pulley, AssemblyFrames.translation(0, spindleY, beltZ), "spindle");
    var spindleDrive = new SpindleMotor(true);
    var motorFaceZ = beltZ + motorPulley.thickness;
    attach("spindleMotor", spindleDrive, orient(150, spindleY, motorFaceZ, [0.0, 1, 0], [0.0, 0, -1]), "head");
    var shelfBottom = motorFaceZ - 8;
    attach(
      "spindleMotorShelf",
      new MillMotorPlate("MILL-SPINDLE-MOTOR-SHELF", spindleDrive.rating.bodyDiameter + 20, 8, motorPulley.outsideDiameter + 2),
      AssemblyFrames.translation(150, spindleY, shelfBottom),
      "head"
    );
    attach(
      "spindleMotorRiser",
      new MillDriveBox("MILL-SPINDLE-MOTOR-RISER", 20, 100, shelfBottom - headZ - head.height),
      AssemblyFrames.translation(head.width / 2, spindleY + 5, headZ + head.height),
      "head"
    );
    addComponent("spindleMotorPulley", motorPulley);
    zeroPoses.set("spindleMotorPulley", AssemblyFrames.translation(150, spindleY, beltZ));
    connect("spindleMotor", "spindleMotorPulley");
    addMateOnAxis(
      "spindle-motor-turn",
      "continuous",
      "spindleMotor",
      "to-spindleMotorPulley",
      "spindleMotorPulley",
      "attach-spindleMotorPulley",
      {x: 0, y: 0, z: 1}
    );
    attach(
      "spindleMotorShaft",
      spindleDrive.shaft(),
      orient(150, spindleY, motorFaceZ, [0.0, 1, 0], [0.0, 0, -1]),
      "spindleMotorPulley"
    );
    attach("spindleBelt", belt, AssemblyFrames.translation(0, spindleY, beltZ + pulley.thickness / 2), "head");
    addBeltPath({
      belt: "spindleBelt",
      wraps: [
        {instanceId: "spindlePulley", connectorName: "axis"},
        {instanceId: "spindleMotorPulley", connectorName: "axis"}
      ]
    });
    var motorRatio = addTransmission(
      "spindle-belt",
      "spindle-turn",
      "spindle-motor-turn",
      Transmission.BeltReduction("spindleBelt", "spindlePulley", "spindleMotorPulley"),
      machinekit.assembly.Sense.Same
    );
    var spindleDriver = "spindleMotorDriver";
    attach(
      spindleDriver,
      new MotorDriver("GENERIC-400V-SERVO-AMP", new machinekit.power.PhaseRmsCurrent(20)),
      AssemblyFrames.translation(-190, base.depth / 2 - 30, 0),
      "base"
    );
    connectPorts("spindle-power", "powerSupply", "power4", spindleDriver, "power");
    powerCableCuts.push(cableCut(zeroPose("spindleMotor"), zeroPose(spindleDriver), 0));
    declareServices();
    installCovers();
    // The rail and motor screws follow from the bolted faces the parts declare, not from a list here.
    deriveFasteners();
    addMotor(
      "spindleMotor",
      "spindle-motor-turn",
      "spindleMotor",
      spindleDriver,
      0.5,
      null,
      {
        speedChannel: "spindle.speed",
        directionChannel: "spindle.direction",
        radiansPerSpeedUnit: motorRatio * 2 * Math.PI / 60
      }
    );
  }


  /** How far the Y screw's rear bearing moves back into the column's bay to leave the rear cover its stack, mm. */
  public static inline var Y_SCREW_EXTENSION:Float = 12.5;
  /**
   * How much shorter the Z screw is to clear the Y rear cover, mm. Its lower 180 mm lay below the head's travel and carried nothing but the
   * floating bearing, which stood at the corner of the base where the Y rear cover's stack goes, and its cover wall would have stood at the
   * table's height.
   */
  public static inline var Z_SCREW_SHORTENING:Float = 180;
  /** Clearance a cover's opening keeps round what it protects, mm. */
  public static inline var COVER_CLEARANCE:Float = 3;
  /** How far a cover end wall or carrier stands out beyond the flange plate it holds, so the screws keep their edge distance, mm. */
  public static inline var COVER_MARGIN:Float = 4;
  /** Thickness of the cover end walls and carriers, mm. */
  public static inline var COVER_PLATE:Float = 14;
  /** The cover row the mill buys (every end has less than 100 mm to fold into). */
  public static inline var COVER_ROW:String = "BELLOWS-LAMELLA-8x8";

  /** The installed way cover `id`. */
  public function coverOf(id:String):WayCover {
    var cover = wayCovers.get(id);
    if (cover == null) throw 'Bench mill has no way cover "$id"';
    return cover;
  }

  /** The cover skins, flanges and their mounting plates, all fitted after the enclosed hardware. */
  public function coverMembers():Array<String> return [for (entry in components())
    if (Std.isOfType(entry.component, WayCoverMount) || Std.isOfType(entry.component, WayCoverSection)) entry.id];

  /** A rail is unbolted with its cover off; a cover's own host remains present for its fastener checks. */
  public function coverInstallationOrder():Array<{member:String, fastened:Array<String>}> {
    var members = coverMembers();
    var ground = [for (entry in components()) if (members.indexOf(entry.id) < 0) entry.id];
    var derived = derivedFasteners();
    if (derived == null) throw "The mill derived no fasteners";
    return [for (id in members) {
      var hosts = [for (joint in derived.joints) if (joint.clampedMember == id) joint.receivingMember];
      {member: id, fastened: [for (other in ground) if (hosts.indexOf(other) < 0) other]};
    }];
  }

  function dot(a:Array<Float>, b:Array<Float>):Float return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

  /**
   * One way cover and its mounting plates: an end wall for the fixed flange (and a carrier for the moving one where the moving member
   * has no face of its own), the cover from `WayCover.installOn`, all at the zero pose. `origin` is the fixed flange's mounting plane on
   * the surface the box cover stands on, `along` runs from it to the moving flange, whose plane at joint value 0 is `moving0`. The
   * fixed and moving ends' space is measured at the end stop where they are closest.
   */
  function coverFrom(id:String, row:String, wallParent:String, movingParent:String, leader:MillAxisSpec, origin:Array<Float>, moving0:Array<Float>,
      along:Array<Float>, up:Array<Float>, width:Float, height:Float, shape:WayCoverPlan.CoverShape, wallBores:Array<MountBore>, wallCuts:Array<MountCut>,
      wallBelow:Float, carrierBores:Null<Array<MountBore>>):WayCover {
    var axis = axisDirection(leader.id);
    var folded = dot(axis, along) > 0 ? leader.lower : leader.upper;
    var closest = [for (k in 0...3) moving0[k] + axis[k] * folded];
    var space = dot([for (k in 0...3) closest[k] - origin[k]], along);
    var plan = new WayCoverPlan(WayCoverCatalog.get(row), width, height, leader.upper - leader.lower, space, shape);
    var wall = "wall" + id.substr(5);
    attach(wall, new WayCoverMount("MILL-COVER-WALL-" + id.substr(5).toUpperCase(), plan.fullPlateHalf() + COVER_MARGIN, plan.plateHigh() + COVER_MARGIN, COVER_PLATE, wallBores, wallCuts,
      wallBelow), orient(origin[0], origin[1], origin[2], up, along), wallParent);
    var host = movingParent;
    if (carrierBores != null) {
      host = "carrier" + id.substr(5);
      attach(host, new WayCoverMount("MILL-COVER-CARRIER-" + id.substr(5).toUpperCase(), plan.plateHalf() + COVER_MARGIN, plan.plateHigh() + COVER_MARGIN, COVER_PLATE, carrierBores, null, wallBelow),
        orient(moving0[0], moving0[1], moving0[2], up, [-along[0], -along[1], -along[2]]), movingParent);
    }
    var cover = WayCover.installOn(this, this, {
      id: id, row: row, fixedParent: wall, movingParent: host, leader: {id: leader.id, lower: leader.lower, upper: leader.upper, initial: leader.initial}, origin: origin, along: along, up: up, width: width, height: height,
      space: space, shape: shape
    });
    wayCovers.set(id, cover);
    return cover;
  }

  /**
   * Chip protection, built into the mill: a box cover on each end of each axis, all of one lamella bellows (`COVER_ROW`).
   * Y has a cover either side of the saddle from an end wall behind the bearing to the saddle's face; X either side of the table from an end
   * wall on the saddle's bridge to the table's end face; Z above and below the nut, from a wall at the bearing to a carrier on the head.
   * The cover walls are the surfaces they stand on: the base top for Y, the saddle top for X, the column's side face for Z.
   */
  function installCovers():Void {
    var bk = ScrewSupportUnit.bk12(), bf = ScrewSupportUnit.bf12();
    var up = [0.0, 0, 1], baseTop = base.height, rail = guideBlock.spec;
    var xSpec = specs[0], ySpec = specs[1], zSpec = specs[2];
    var saddlePose = zeroPose("saddle"), tablePose = zeroPose("table");
    var nut = new BallNut();
    // The rails stand through a wall as it stands on the base: a slot clears each, a little wider than the rail.
    var railSlots:Array<MountCut> = [for (side in [-1, 1]) {
      x0: side * base.railSpacing / 2 - rail.railWidth / 2 - 3.5,
      x1: side * base.railSpacing / 2 + rail.railWidth / 2 + 3.5,
      y0: -1.0,
      y1: rail.railHeight + 4
    }];
    var screwYUp = zeroPose("screwY").z - baseTop;
    var yWidth = 2 * (base.railSpacing / 2 + rail.blockWidth / 2 + COVER_CLEARANCE);
    // The faces on the saddle the moving flanges are bolted to span the saddle's height.
    var saddleHost = WayCoverPlan.boxCover(saddlePose.z - baseTop, saddlePose.z + saddle.height - baseTop, saddle.width / 2);
    // Y front: the wall stands in front of the front bearing's stand and the coupling passes through it; the cover encloses the bearing.
    var frontWall = zeroPose("screwYFixed").y - Math.max(bk.length, standLength(bk)) / 2 - 7;
    var yCoupling = axisDrives[1].coupling.outerDiameter + 6;
    coverFrom("coverYFront", COVER_ROW, "base", "saddle", ySpec, [0.0, frontWall, baseTop], [0.0, saddlePose.y - saddle.depth / 2, baseTop], [0.0, 1, 0],
      up, yWidth, screwYUp + 22, saddleHost, [{x: 0.0, y: screwYUp, d: yCoupling}], railSlots, 0, null);
    // Y rear: the wall stands in front of the rear bearing, which has moved back into the column's bay; the screw passes through it.
    var rearWall = zeroPose("screwYFloating").y - Math.max(bf.length, standLength(bf)) / 2 - 2 - COVER_PLATE;
    var bracketTop = screwYUp + nut.flangeDiameter / 2 + 11;
    coverFrom("coverYRear", COVER_ROW, "base", "saddle", ySpec, [0.0, rearWall, baseTop], [0.0, saddlePose.y + saddle.depth / 2, baseTop], [0.0, -1, 0],
      up, yWidth, bracketTop + COVER_CLEARANCE, saddleHost, [{x: 0.0, y: screwYUp, d: 24.0}], railSlots, 0, null);
    // X: the covers stand on the saddle's top, between the rails' outer edges and the saddle's sides, and their walls on the bridges that
    // carry the screw's bearings; the nesting grows toward the wall so the skins at the table stay narrower than the table.
    var floor = saddlePose.z + saddle.height, bridgeTop = zeroPose("screwXNearBridge").z + 10;
    var screwXUp = zeroPose("screwX").z - floor;
    var xWidth = 2 * (saddle.railSpacing / 2 + rail.railWidth / 2 + 4);
    var tableHost = WayCoverPlan.boxCover(tablePose.z - floor, tablePose.z + table.height - floor, table.depth / 2 - 3, true);
    var leftWall = zeroPose("screwXFixed").x - bk.length / 2 - 7;
    var xCoupling = axisDrives[0].coupling.outerDiameter + 6;
    coverFrom("coverXLeft", COVER_ROW, "screwXNearBridge", "table", xSpec, [leftWall, saddlePose.y, floor], [tablePose.x - table.width / 2, saddlePose.y, floor],
      [1.0, 0, 0], up, xWidth, screwXUp + 19, tableHost, [{x: 0.0, y: screwXUp, d: xCoupling}], [], floor - bridgeTop, null);
    var rightWall = Math.max(zeroPose("screwXFloating").x + bf.length / 2, zeroPose("screwX").x + axisDrives[0].screw.totalLength) + 2;
    coverFrom("coverXRight", COVER_ROW, "screwXFarBridge", "table", xSpec, [rightWall, saddlePose.y, floor], [tablePose.x + table.width / 2, saddlePose.y, floor],
      [-1.0, 0, 0], up, xWidth, screwXUp + 19, tableHost, [], [], floor - bridgeTop, null);
    // Z: the covers stand on the column's side face, open toward it, and run along the screw above and below the nut; the walls are on the
    // column, at the bearings, and the carriers on the head, above the nut and below its bracket.
    var side = column.width / 2 + 1, screwZ = zeroPose("screwZ"), carrier = COVER_PLATE;
    var screwZUp = screwZ.x - side;
    var zWidth = Math.max(bk.width, bf.width) + 2 * COVER_CLEARANCE;
    var zHeight = Math.max(bk.height, bf.height) + COVER_CLEARANCE;
    var topWall = Math.max(zeroPose("screwZFixed").z + bk.length / 2, zeroPose("screwZStand0").z + standLength(bk) / 2) + 1;
    var zCoupling = axisDrives[2].coupling.outerDiameter + 6;
    var nutTop = zeroPose("screwZNut").z;
    var bracketBottom = zeroPose("screwZNutBracket").z - 8;
    var zeroUp = [1.0, 0, 0];
    coverFrom("coverZTop", COVER_ROW, "column", "head", zSpec, [side, screwZ.y, topWall], [side, screwZ.y, nutTop + carrier], [0.0, 0, -1], zeroUp,
      zWidth, zHeight, WayCoverPlan.boxCover(), [{x: 0.0, y: screwZUp, d: zCoupling}], [], 1, [{x: 0.0, y: screwZUp, d: 24.0}]);
    var bottomWall = Math.min(zeroPose("screwZFloating").z - bf.length / 2, screwZ.z - axisDrives[2].screw.totalLength) - 4;
    // The carrier under the bracket clears the nut flange's bolts, whose nuts and washers stand on the bracket's far side.
    var boltReach = 0.0;
    for (point in nut.boltPattern()) boltReach = Math.max(boltReach, Math.sqrt(point.x * point.x + point.y * point.y));
    coverFrom("coverZBottom", COVER_ROW, "column", "head", zSpec, [side, screwZ.y, bottomWall], [side, screwZ.y, bracketBottom - carrier], [0.0, 0, 1], zeroUp,
      zWidth, zHeight, WayCoverPlan.boxCover(), [], [], 1, [{x: 0.0, y: screwZUp, d: 2 * (boltReach + 8)}]);
    // The Y covers keep chips and coolant off the rails and the screw; what gets past their wipers runs along the base's gutters to the drain
    // holes at the front, which open into the hollow base over the chip tray.
    var gutterTop = base.height - MillCasting.BODY_PAD;
    var gutterFall = (MillCasting.GUTTER_REAR - MillCasting.GUTTER_FRONT) * MillCasting.GUTTER_FALL;
    var drains:Array<ChannelDrain> = [for (side in [-1.0, 1.0]) {
      high: [side * MillCasting.GUTTER_CENTRE, MillCasting.GUTTER_REAR, gutterTop],
      low: [side * MillCasting.GUTTER_CENTRE, MillCasting.GUTTER_FRONT, gutterTop - gutterFall],
      drain: "base",
      diameter: MillCasting.DRAIN_DIAMETER,
      outletZ: base.height - 25
    }];
    var yMembers = coverOf("coverYFront").members().concat(coverOf("coverYRear").members());
    addDesignRequirement(GuardedChannel("yCoverGutters", "base", yMembers, drains));
    // Wipers on the twelve guide blocks, an end scraper at each end.
    addBomItem(WayWiperCatalog.bomItem("WIPER-HG15-SCRAPER", 24));
  }

  /** Where in the column the Z supports' bosses are cast: under the feet of the supports `driveScrew` places on the screw. */
  static function zBosses():Array<ColumnBoss> {
    var casting = new MillColumn(), rail = LinearRailBlock.metric(RAIL).spec;
    var near = ScrewSupportUnit.bk12(), far = ScrewSupportUnit.bf12();
    // The screw runs from the column's top plus the rail's end margin down its height; the supports' feet look at the column from the
    // middle of the gap between its front and the head's back.
    var top = casting.height + rail.railEndMargin, y = -casting.depth / 2 - rail.blockHeight / 2;
    return [for (end in 0...2) {
      var unit = end == 0 ? near : far;
      var offset = end == 0 ? BallScrew.INPUT_JOURNAL_LENGTH - near.length / 2 : casting.height - Z_SCREW_SHORTENING - BallScrew.JOURNAL_LENGTH / 2;
      {y: y, z: top - offset, width: standWidth(unit), length: standLength(unit), back: Z_BOSS_BACK};
    }];
  }

  /** The x of a boss's back face, mm: it is let 15 mm into the 20 mm wall and its face stands 1 mm proud. */
  static inline var Z_BOSS_BACK:Float = 75;

  /** A stand's footprint under a support: the foot's hole pattern with room for the screws' edge distance (`JointFasteners.EDGE_DIAMETERS`). */
  static function standWidth(unit:ScrewSupportUnit):Float return Math.max(unit.width, unit.spec.pitch + 2 * 8);
  static function standLength(unit:ScrewSupportUnit):Float return Math.max(unit.length, unit.spec.longitudinalPitch + 2 * 8);

  /** The workpiece, its clamps and the vise: the operator takes them off the table to reach the carriage screws. */
  public function workholdingMembers():Array<String> return [for (entry in components()) if (entry.id == "stock" || entry.id == "clampLeft"
    || entry.id == "clampRight" || StringTools.startsWith(entry.id, "vise/")) entry.id];

  /** Conservative routing allowance until the actual cable paths are designed. */
  function cableCut(motor:AssemblyFrame, driver:AssemblyFrame, extra:Float):BomCut {
    var travel = 0.0;
    for (axis in specs) travel += axis.upper - axis.lower;
    var distance = Math.abs(motor.x - driver.x) + Math.abs(motor.y - driver.y) + Math.abs(motor.z - driver.z);
    return {length:Math.ceil((distance + travel + extra + 500) / 100) / 10, count:1};
  }

  function declareServices():Void {
    for (axis in specs) {
      var chain = new CableChain(new ComponentValues().setNumber("travel", axis.upper - axis.lower));
      var metres = Math.ceil(chain.developedLength()) / 1000;
      var item = chain.bom;
      item.measure = {value:metres, unit:"m"};
      item.cuts = [{length:metres, count:1}];
      addBomItem(item);
    }
    var powerLength = 0.0, encoderLength = 0.0;
    for (cut in powerCableCuts) powerLength += cut.length * cut.count;
    for (cut in encoderCableCuts) encoderLength += cut.length * cut.count;
    addBomItem({partNumber:"MILL-POWER-CABLE-4C-400V-ASSUMED", description:"Assumed continuous-flex motor power cable",
      fingerprint:machinekit.component.BomRevision.fingerprint("flex-power|400V|4-core|continuous-flex", 1),
      quantity:1, material:null, procurement:Buy(null, null, null), maturity:"Assumed",
      requirements:[ProcurementCodec.requirement("ratedVoltage", "V", 400),
        ProcurementCodec.requirement("cores", "1", null, null, 4), ProcurementCodec.choice("service", "continuous flex")],
      measure:{value:powerLength, unit:"m"}, cuts:powerCableCuts});
    addBomItem({partNumber:"MILL-ENCODER-CABLE-ASSUMED", description:"Assumed shielded continuous-flex encoder cable",
      fingerprint:machinekit.component.BomRevision.fingerprint("encoder|shielded|continuous-flex", 1),
      quantity:1, material:null, procurement:Buy(null, null, null), maturity:"Assumed",
      requirements:[ProcurementCodec.choice("signal", "incremental encoder"),
        ProcurementCodec.choice("shield", "shielded"), ProcurementCodec.choice("service", "continuous flex")],
      measure:{value:encoderLength, unit:"m"}, cuts:encoderCableCuts});
    addBomItem({partNumber:"MILL-LUBE-15-OUT-ASSUMED", description:"Assumed manual way-oil pump and distribution set",
      fingerprint:machinekit.component.BomRevision.fingerprint("manual-way-oil|15-outlets|ISO-VG-68", 1),
      quantity:1, material:null, procurement:Buy(null, null, null), maturity:"Assumed",
      requirements:[ProcurementCodec.requirement("outlets", "1", 15),
        ProcurementCodec.choice("lubricant", "ISO VG 68 way oil"), ProcurementCodec.choice("operation", "manual")]});
    addBomItem({partNumber:"MILL-WAY-OIL-ISO-VG68", description:"ISO VG 68 way oil, initial fill",
      fingerprint:machinekit.component.BomRevision.fingerprint("ISO-3448|viscosity-grade-68", 1),
      quantity:1, material:null, procurement:Buy(null, null, "ISO 3448"), maturity:"Assumed",
      requirements:[ProcurementCodec.requirement("viscosityGrade", "1", null, null, 68)],
      measure:{value:1, unit:"l"}});
  }

  function driveScrew(
    spec:MillAxisSpec,
    parent:String,
    carriage:String,
    start:Array<Float>,
    along:Array<Float>,
    up:Array<Float>,
    length:Float,
    nutCenter:Array<Float>
  ):Void {
    var id = "screw" + spec.id.toUpperCase();
    var motorId = "motor" + spec.id.toUpperCase();
    var motor = ServoMotor.model(spec.id == "z" ? "GENERIC-HV-SERVO-750W" : "GENERIC-HV-SERVO-400W");
    var screw = new BallScrew(length), nut = new BallNut();
    var brake = spec.id == "z" ? new ShaftBrakePart(motor.rating.shaftDiameter, "shell") : null;
    var plateThickness = 8.0;
    var brakeEnd = brake == null ? 0.0 : plateThickness + brake.depth;
    var couplingLength = brake == null ? null : 2 * (motor.rating.shaftLength - brakeEnd - 1);
    var coupling = new ShaftCoupling(motor.rating.shaftDiameter, BallScrew.JOURNAL_DIAMETER, null, couplingLength);
    if (coupling.length / 2 < Math.max(coupling.boreA,
      coupling.boreB) * 0.75) throw "Shaft brake leaves insufficient coupling engagement";
    axisDrives.push({spec: spec, screw: screw, motor: motor, coupling: coupling, brake: brake});
    var shaftLength = motor.rating.shaftLength;
    // The shaft seats through side A to the sleeve midpoint; side B seats on the screw.
    if (shaftLength < coupling.length / 2) throw "Motor shaft is too short for its coupling";
    var motorOffset = shaftLength;
    var motorPose = orient(
      start[0] - along[0] * motorOffset,
      start[1] - along[1] * motorOffset,
      start[2] - along[2] * motorOffset,
      up,
      along
    );
    var couplingPose = orient(
      start[0] - along[0] * coupling.length / 2,
      start[1] - along[1] * coupling.length / 2,
      start[2] - along[2] * coupling.length / 2,
      up,
      along
    );
    attach(motorId, motor, motorPose, parent);
    addComponent(id, screw);
    zeroPoses.set(id, orient(start[0], start[1], start[2], up, along));
    connect(motorId, id);

    attach(id + "Coupling", coupling, couplingPose, id);
    attach(motorId + "Shaft", motor.shaft(), motorPose, id);
    if (brake != null) {
      var brakePose = orient(
        motorPose.x + along[0] * plateThickness,
        motorPose.y + along[1] * plateThickness,
        motorPose.z + along[2] * plateThickness,
        up,
        along
      );
      for (role in ["front", "shell", "back"]) for (sector in 0...ShaftBrakePart.sectorCount(brake.bore, role)) {
        var part = role == "shell" && sector == 0 ? brake : new ShaftBrakePart(brake.bore, role, sector);
        attach(motorId + 'Brake-$role-$sector', part, brakePose, parent);
      }
      for (role in ["hub",
        "disc"]) attach(motorId + 'Brake-$role', new ShaftBrakePart(brake.bore, role), brakePose, id);
    }
    var nutOffset = nut.bodyLength / 2;
    attach(
      id + "Nut",
      nut,
      orient(
        nutCenter[0] - along[0] * nutOffset,
        nutCenter[1] - along[1] * nutOffset,
        nutCenter[2] - along[2] * nutOffset,
        up,
        along
      ),
      carriage
    );
    var faceDistance = nut.bodyLength / 2 + nut.flangeThickness;
    attach(
      id + "NutBracket",
      new MillNutBracket(nut, spec.id, spec.id == "x" ? tableTop - table.height - nutCenter[2] : 0),
      orient(
        nutCenter[0] + along[0] * faceDistance,
        nutCenter[1] + along[1] * faceDistance,
        nutCenter[2] + along[2] * faceDistance,
        up,
        along
      ),
      carriage
    );
    var near = ScrewSupportUnit.bk12(), far = ScrewSupportUnit.bf12();
    for (end in 0...2) {
      var support = end == 0 ? near : far;
      var offset = end == 0 ? BallScrew.INPUT_JOURNAL_LENGTH - near.length / 2 : length - BallScrew.JOURNAL_LENGTH / 2;
      attach(
        id + (end == 0 ? "Fixed" : "Floating"),
        support,
        orient(start[0] + along[0] * offset, start[1] + along[1] * offset, start[2] + along[2] * offset, up, along),
        parent
      );
    }
    var couplingStart = shaftLength - coupling.length / 2;
    var flange = motor.mounting();
    if (flange == null || flange.holes.length != 4) throw "Axis motor needs its declared four-hole flange";
    var plateBore = Math.max(flange.pilotDiameter,
      couplingStart < plateThickness ? coupling.outerDiameter : motor.rating.shaftDiameter);
    var motorPlate = new MillMotorPlate("MILL-" + spec.id.toUpperCase() + "-MOTOR-PLATE", motor.rating.bodyDiameter + 20, plateThickness, plateBore, flange);
    if (spec.id == "y") {
      var footZ = start[2] + near.connector("mount").frame.y;
      for (end in 0...2) {
        var unit = end == 0 ? near : far;
        var offset = end
          == 0 ? BallScrew.INPUT_JOURNAL_LENGTH - near.length / 2 : length - BallScrew.JOURNAL_LENGTH / 2;
        attach(
          id + 'Stand$end',
          new MillDriveBox(end == 0 ? "MILL-Y-STAND-FIXED" : "MILL-Y-STAND-FLOATING", standWidth(unit), standLength(unit), footZ - base.height, true),
          AssemblyFrames.translation(start[0], start[1] + offset, base.height),
          parent
        );
      }
      var plateBottom = start[2] - motorPlate.width / 2;
      var front = -base.depth / 2;
      attach(
        motorId + "Bridge",
        new MillDriveBox("MILL-Y-MOTOR-BRIDGE", motorPlate.width, front - motorPose.y, plateBottom - base.height + 10),
        AssemblyFrames.translation(0, (front + motorPose.y) / 2, base.height - 10),
        parent
      );
    } else if (spec.id == "x") {
      var footZ = start[2] + near.connector("mount").frame.y;
      for (side in [-1, 1]) {
        // The bridge ends against the motor plate's inner face: nothing stands behind the plate, where the flange screws' heads
        // and their drivers are.
        var reach = Math.abs(motorPose.x) - motorPlate.thickness - saddle.width / 2;
        attach(
          id + (side < 0 ? "NearBridge" : "FarBridge"),
          new MillDriveBox(side < 0 ? "MILL-X-BRIDGE-NEAR" : "MILL-X-BRIDGE-FAR", reach, 80, 10, true),
          AssemblyFrames.translation(side * (saddle.width / 2 + reach / 2), start[1], footZ - 10),
          parent
        );
      }
    } else {
      // Each Z support's foot is bolted to a steel boss cast into the column's side wall (`zBosses`): the boss fills a pocket in the
      // wall and stands 1 mm proud of it, which is where the support's foot plane is. Cast in, it needs no screws of its own.
      var columnPose = zeroPose(parent);
      for (end in 0...2) {
        var unit = end == 0 ? near : far;
        var offset = end
          == 0 ? BallScrew.INPUT_JOURNAL_LENGTH - near.length / 2 : length - BallScrew.JOURNAL_LENGTH / 2;
        var footX = start[0] + unit.connector("mount").frame.y;
        var boss = column.bosses[end];
        if (Math.abs(footX - (column.width / 2 + 1)) > 1e-6 || Math.abs(start[2] - offset - (columnPose.z + boss.z)) > 1e-6
          || Math.abs(start[1] - (columnPose.y + boss.y)) > 1e-6) throw "The column's Z support bosses are not under the supports' feet";
        attach(
          id + 'Stand$end',
          new MillDriveBox(end == 0 ? "MILL-Z-BOSS-FIXED" : "MILL-Z-BOSS-FLOATING", boss.width, boss.length, footX - boss.back, true),
          orient(boss.back, start[1], start[2] - offset, [0.0, 0, 1], [1.0, 0, 0]),
          parent
        );
      }
    }
    if (spec.id == "z") {
      var top = zeroPose(parent).z + column.height;
      attach(
        motorId + "Post",
        new MillDriveBox("MILL-Z-MOTOR-POST", 20, 20, motorPose.z - motorPlate.thickness - top),
        AssemblyFrames.translation(column.width / 2 - 10, start[1] + motorPlate.width / 2 - 10, top),
        parent
      );
    }
    attach(motorId + "Plate", motorPlate, motorPose, parent);
    var ratio = addTransmission(id + "-lead", spec.id, id + "-turn", Transmission.LeadScrew(id, id + "Nut"), Opposite);
    // Motor zero agrees with the table/head's initial displacement.
    addMateOnAxis(
      id + "-turn",
      "continuous",
      motorId,
      'to-$id',
      id,
      'attach-$id',
      {x: along[0], y: along[1], z: along[2]},
      ratio * spec.initial
    );
    supportScrew(id + "-lead", near.support, far.support);
    var driver = motorId + "Driver";
    var index = spec.id == "x" ? 1 : spec.id == "y" ? 2 : 3;
    attach(
      driver,
      new MotorDriver("GENERIC-400V-SERVO-AMP", new machinekit.power.PhaseRmsCurrent(20)),
      AssemblyFrames.translation(-190 + index * 65, base.depth / 2 - 30, 0),
      "base"
    );
    powerCableCuts.push(cableCut(motorPose, zeroPose(driver), 0));
    encoderCableCuts.push(cableCut(motorPose, zeroPose(driver), motor.rating.bodyLength));
    connectPorts(driver + "-power", "powerSupply", 'power$index', driver, "power");
    addMotor(motorId, id + "-turn", motorId, driver);
    if (brake != null) bindMotorBrake(motorId, motorId + "Brake-shell-0", "zBrake.release", "zBrake.released", 250, 30);
    var encoderId = motorId + "Encoder";
    attach(
      encoderId,
      new ShaftEncoder(motor.rating.encoderCounts, true),
      orient(
        motorPose.x - along[0] * motor.rating.bodyLength,
        motorPose.y - along[1] * motor.rating.bodyLength,
        motorPose.z - along[2] * motor.rating.bodyLength,
        up,
        [-along[0], -along[1], -along[2]]
      ),
      motorId
    );
    addEncoder(encoderId, id + "-turn", encoderId, motorId);
  }

  public function workOffset():Array<Float> {
    var installedVise = vise;
    if (installedVise != null) {
      var datum = AssemblyFrames.compose(zeroPose("vise/body"), installedVise.datum);
      return [datum.x - gaugeZero.x, datum.y - gaugeZero.y, datum.z + STOCK_HEIGHT - gaugeZero.z];
    }
    var stock = zeroPose("stock");
    return [
      stock.x - STOCK_WIDTH / 2 - gaugeZero.x,
      stock.y - STOCK_DEPTH / 2 - gaugeZero.y,
      stock.z + STOCK_HEIGHT - gaugeZero.z
    ];
  }
  /** CAM forbidden boxes come from the installed clamps in stock work coordinates. */
  public function fixtures():Array<Fixture> {
    var stock = zeroPose("stock");
    var origin = AssemblyFrames.translation(
      stock.x - STOCK_WIDTH / 2,
      stock.y - STOCK_DEPTH / 2,
      stock.z + STOCK_HEIGHT
    );
    var inverse = AssemblyFrames.inverse(origin);
    var result:Array<Fixture> = [];
    var model = new cadkit.modeling.AssemblyModel("mm");
    addTo(model, "");
    var state = new cadkit.modeling.AssemblyState(model.definition("mill-fixtures"));
    for (spec in specs) state.setJoint(spec.id, 0);
    for (entry in components()) if (entry.id == "clampLeft" || entry.id == "clampRight"
      || entry.id == "vise/fixedJaw" || entry.id == "vise/movingJaw" || entry.id == "vise/endStop") {
      var part = entry.component.geometry();
      var bounds = part.shape.bounds(), minimum = bounds.get_min(), maximum = bounds.get_max();
      var pose = AssemblyFrames.compose(inverse, state.worldPose(entry.id));
      var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
      var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
      for (x in [minimum.get_x(), maximum.get_x()]) for (y in [minimum.get_y(), maximum.get_y()]) for (z in [
        minimum.get_z(),
        maximum.get_z()
      ]) {
        var point = AssemblyFrames.transformPoint(pose, x, y, z);
        var values = [point.x, point.y, point.z];
        for (axis in 0...3) {
          low[axis] = Math.min(low[axis], values[axis]);
          high[axis] = Math.max(high[axis], values[axis]);
        }
      }
      part.close();
      result.push(new Fixture(
        entry.id,
        low[0] / 1000,
        high[0] / 1000,
        low[1] / 1000,
        high[1] / 1000,
        low[2] / 1000,
        high[2] / 1000
      ));
    }
    return result;
  }

  public function tools():Array<Tool> return [Tool.shaped(
    1,
    tool.stickout / 1000,
    tool.cutter().withHolder(Er20Holder.DIAMETER / 1000, Er20Holder.LENGTH / 1000)
  )];
  public function axisOvertravel(id:String):Float {
    var room = overtravel.get(id);
    if (room == null) throw 'Bench mill has no axis "$id"';
    return room;
  }

  function slide(
    spec:MillAxisSpec,
    parent:String,
    guide:LinearRail,
    id:String,
    block:LinearRailBlock,
    pose:AssemblyFrame,
    axis:Array<Float>,
    blockPoses:Array<AssemblyFrame>
  ):Void {
    var inverse = AssemblyFrames.inverse(zeroPose(parent));
    var reach = guide.spec.railEndMargin + block.spec.blockLength / 2;
    var room = guide.length;
    for (support in blockPoses) {
      var first = AssemblyFrames.transformPoint(
        inverse,
        support.x + axis[0] * spec.lower,
        support.y + axis[1] * spec.lower,
        support.z + axis[2] * spec.lower
      ).z;
      var last = AssemblyFrames.transformPoint(
        inverse,
        support.x + axis[0] * spec.upper,
        support.y + axis[1] * spec.upper,
        support.z + axis[2] * spec.upper
      ).z;
      room = Math.min(room, Math.min(Math.min(first, last) - reach, guide.length - reach - Math.max(first, last)));
    }
    if (!(room > 0)) throw 'Bench mill ${spec.id} needs room beyond travel on its rail';
    overtravel.set(spec.id, room);
    axisDirections.set(spec.id, axis.copy());
    addComponent(id, block);
    zeroPoses.set(id, pose);
    connect(parent, id);
    addMateOnAxis(
      spec.id,
      "prismatic",
      parent,
      'to-$id',
      id,
      'attach-$id',
      {x: axis[0], y: axis[1], z: axis[2]},
      spec.initial,
      {lower: spec.lower, upper: spec.upper, velocity: null, effort: null, overtravel: room}
    );
  }

  /** Frame whose local +Y points along `up` and local +Z along `along`. */
  static function orient(x:Float, y:Float, z:Float, up:Array<Float>, along:Array<Float>):AssemblyFrame {
    var xx = up[1] * along[2] - up[2] * along[1];
    var xy = up[2] * along[0] - up[0] * along[2];
    var xz = up[0] * along[1] - up[1] * along[0];
    return AssemblyFrames.fromRotationMatrix(x, y, z, [xx, up[0], along[0], xy, up[1], along[1], xz, up[2], along[2]]);
  }

  /** A fixed root member at its world pose. */
  function place(id:String, component:MachineComponent, pose:AssemblyFrame):Void {
    addComponent(id, component, pose);
    zeroPoses.set(id, pose);
  }

  /** A member fixed to `parent`, at its world pose with every axis at zero. */
  public function attach(id:String, component:MachineComponent, pose:AssemblyFrame, parent:String):Void {
    addComponent(id, component);
    zeroPoses.set(id, pose);
    connect(parent, id);
    addMate('$id-mount', "fixed", parent, 'to-$id', id, 'attach-$id');
  }

  /** A member hung on `parent` at its world pose with every axis at zero, without a mate: the caller adds the joint (`WayCover.installOn`). */
  public function hang(id:String, component:MachineComponent, pose:AssemblyFrame, parent:String):Void {
    addComponent(id, component);
    zeroPoses.set(id, pose);
    connect(parent, id);
  }

  /** The world direction axis `id` moves its carriage as its coordinate grows, a unit vector. */
  public function axisDirection(id:String):Array<Float> {
    var direction = axisDirections.get(id);
    if (direction == null) throw 'Bench mill has no axis "$id"';
    return direction.copy();
  }

  function component(id:String):MachineComponent {
    for (entry in components()) if (entry.id == id) return entry.component;
    throw 'Bench mill has no member "$id" yet';
  }

  /**
	 * Connectors meeting at the child's origin with world-aligned axes, so joint axes are world
	 * directions. Names carry the member ids, so members that share geometry can share one
	 * definition holding all of their connectors.
	 */
  function connect(parent:String, child:String):Void {
    var childPose = zeroPose(child);
    var meeting = AssemblyFrames.translation(childPose.x, childPose.y, childPose.z);
    addMemberConnector(parent, 'to-$child', AssemblyFrames.compose(AssemblyFrames.inverse(zeroPose(parent)), meeting));
    addMemberConnector(child, 'attach-$child', AssemblyFrames.compose(AssemblyFrames.inverse(childPose), meeting));
  }

  function zeroPose(id:String):AssemblyFrame {
    var pose = zeroPoses.get(id);
    if (pose == null) throw 'Bench mill has no member "$id" yet';
    return pose;
  }
}

/** Designed flat bracket bodies; source placement derives from the supported shaft and casting. */
class MillDriveBox extends MachineComponent {
  final width:Float;
  final depth:Float;
  final height:Float;
  /** Whether a screw support stands on the top face (drilled and tapped to suit its foot). */
  final footed:Bool;
  /** A plate this thin is tapped through. */
  static inline var TAPPED_THROUGH:Float = 12;
  /** The deepest thread tapped in a stand, mm. */
  static inline var TAP_DEPTH:Float = 20;
  public function new(drawing:String, width:Float, depth:Float, height:Float, footed:Bool = false) {
    super('MILL-BRACKET-$width-$depth-$height' + (footed ? "-footed" : ""), "Mill drive support bracket", "steel", true);
    declareDrawingNumber(drawing);
    addFacet(new machinekit.component.ProcurementIntent(machinekit.component.Procurement.Make("CNC machining")));
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.width = width;
    this.depth = depth;
    this.height = height;
    this.footed = footed;
  }
  override public function revisionFingerprint():Null<String> return machinekit.component.BomRevision.fingerprint(
    [drawingNumber, materialSpec(), machinekit.component.ComponentNumberKey.encode(width),
      machinekit.component.ComponentNumberKey.encode(depth), machinekit.component.ComponentNumberKey.encode(height),
      footed ? "footed" : "plain"].join("|"), 1);
  /** The top face a screw support's foot is bolted to: drilled and tapped to suit the support, the whole top face. */
  override public function fasteningFaces():Array<machinekit.component.BoltedFace> return footed ? [machinekit.component.BoltedFace.drilledToSuit(
    "foot",
    machinekit.component.BoltedFace.frameOf({x: 0, y: 0, z: height}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0}),
    width,
    depth,
    Math.min(height, TAP_DEPTH),
    height,
    machinekit.component.BoltedFace.ThreadMaterial.Steel,
    null,
    height <= TAPPED_THROUGH
  )] : [];
  override public function machinable():Bool return footed;
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return machined(Solids.named(Part.box(width, depth, height), "bracket"));
  }
}

private class MillMotorPlate extends MachineComponent {
  public final width:Float;
  public final thickness:Float;
  final bore:Float;
  public final flange:Null<MotorFlange>;
  public function new(drawing:String, width:Float, thickness:Float, bore:Float, ?flange:MotorFlange) {
    super('MILL-MOTOR-PLATE-$width-$thickness-$bore', flange == null ? "Mill motor mounting plate"
      : flange.holeDiameter == 5.5 ? "Mill motor mounting plate with M5 tapped holes"
      : "Mill motor mounting plate with M6 tapped holes", "steel", true);
    declareDrawingNumber(drawing);
    addFacet(new machinekit.component.ProcurementIntent(machinekit.component.Procurement.Make("CNC machining")));
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.width = width;
    this.thickness = thickness;
    this.bore = bore;
    this.flange = flange;
  }
  override public function revisionFingerprint():Null<String> {
    var inputs = [drawingNumber, materialSpec(), machinekit.component.ComponentNumberKey.encode(width),
      machinekit.component.ComponentNumberKey.encode(thickness), machinekit.component.ComponentNumberKey.encode(bore)];
    if (flange != null) {
      inputs.push(machinekit.component.ComponentNumberKey.encode(flange.holeDiameter));
      for (hole in flange.holes) inputs.push(machinekit.component.ComponentNumberKey.encode(hole.x) + ","
        + machinekit.component.ComponentNumberKey.encode(hole.y));
    }
    return machinekit.component.BomRevision.fingerprint(inputs.join("|"), 1);
  }
  /** The tapped face the motor flange bolts to: its holes are the flange's, tapped through the plate. */
  override public function fasteningFaces():Array<machinekit.component.BoltedFace> {
    if (flange == null) return [];
    var size = flange.holeDiameter == 5.5 ? "M5" : flange.holeDiameter == 6.6 ? "M6" : null;
    if (size == null) throw "Axis motor flange has no reviewed tapped hole";
    return [machinekit.component.BoltedFace.tapped(
      "motor",
      machinekit.component.BoltedFace.frameOf({x: 0, y: 0, z: 0}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0}),
      flange.holes.copy(),
      size,
      thickness,
      thickness,
      true
    )];
  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var body = Part.box(width, width, thickness).translated(new Vector(0, 0, 0));
    var holes = [Part.cylinderSpan(bore / 2 + (flange == null ? 1 : 0.1), -1, thickness + 1)];
    if (flange == null) {
      for (x in [-1, 1]) for (y in [-1, 1]) holes.push(Part.cylinderSpan(
        3.3, -1, thickness + 1, x * (width / 2 - 10), y * (width / 2 - 10)));
    } else {
      var threadDiameter = flange.holeDiameter == 5.5 ? 5.0 : flange.holeDiameter == 6.6 ? 6.0 : 0.0;
      if (threadDiameter == 0) throw "Axis motor flange has no reviewed tapped hole";
      for (point in flange.holes) holes.push(Part.cylinderSpan(threadDiameter / 2, -1, thickness + 1, point.x, point.y));
    }
    return Solids.cut(Solids.named(body, "plate"), holes);
  }
}

/** Nut flange adapter; each axis has a machined bearing face on its carriage. */
private class MillNutBracket extends MachineComponent {
  final nut:BallNut;
  final axis:String;
  final mountRise:Float;
  public function new(nut:BallNut, axis:String, mountRise:Float) {
    super('MILL-NUT-BRACKET-${nut.designation}-$axis-$mountRise', "Ball-nut flange adapter", "steel", true);
    declareDrawingNumber("MILL-NUT-BRACKET-" + axis.toUpperCase());
    addFacet(new machinekit.component.ProcurementIntent(machinekit.component.Procurement.Make("CNC machining")));
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.nut = nut;
    this.axis = axis;
    this.mountRise = mountRise;
  }
  override public function revisionFingerprint():Null<String> return machinekit.component.BomRevision.fingerprint(
    [drawingNumber, materialSpec(), axis, machinekit.component.ComponentNumberKey.encode(mountRise),
      machinekit.motion.BallNut.recipeType().key(nut.values())].join("|"), 1);
  /**
	 * The plate the nut flange's back face seats on: clearance holes for the flange bolts, which pass through it and are held by
	 * a washer and nut on the far side.
	 */
  override public function fasteningFaces():Array<machinekit.component.BoltedFace> return [machinekit.component.BoltedFace.through(
    "nut",
    machinekit.component.BoltedFace.frameOf({x: 0, y: 0, z: 0}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0}),
    [for (p in nut.boltPattern()) {x: p.x, y: p.y}],
    5.5,
    8,
    nut.mountScrew
  )];
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var radius = nut.flangeDiameter / 2 + (axis == "y" ? 11 : 1);
    var plate = Solids.named(Part.cylinderSpan(radius, 0, 8), "flange-seat");
    if (axis == "z") plate = Solids.union([
      plate,
      Solids.named(Part.box(10, 10, 8).translated(new Vector(-radius + 6, -radius + 4, 0)), "head-seat")
    ]);
    if (axis == "x") for (side in [-1, 1]) plate = Solids.union([
      plate,
      Solids.named(
        Part.box(16, mountRise, 8).translated(new Vector(side * (radius + 4), mountRise / 2, 0)),
        side < 0 ? "table-seat-front" : "table-seat-back"
      )
    ]);
    var tools = [Solids.named(Part.cylinderSpan(nut.screwDiameter / 2 + 1, -1, 9), "screw-bore")];
    for (point in nut.boltPattern()) tools.push(Part.cylinderSpan(2.75, -1, 9, point.x, point.y));
    if (axis == "y") for (side in [-1, 1]) tools.push(Part.cylinderSpan(3.3, -1, 9, side * (radius - 5), 0));
    if (axis == "x") for (side in [-1, 1]) tools.push(Part.cylinderAlongY(
      3.3,
      -1,
      mountRise + 1,
      side * (radius + 4),
      4
    ));
    return Solids.cut(plate, tools);
  }
}
