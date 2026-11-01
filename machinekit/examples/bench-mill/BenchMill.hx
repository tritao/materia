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
import machinekit.milling.MillCasting.MillBase;
import machinekit.milling.MillCasting.MillColumn;
import machinekit.milling.MillCasting.MillSaddle;
import machinekit.milling.MillCasting.MillTable;
import machinekit.milling.MillCasting.MillHead;
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
import machinekit.standard.SocketHeadCapScrew;
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
class BenchMill extends MachineAssembly {
  public static inline var RAIL:String = "HGH15CA";
  public static inline var STOCK_WIDTH:Float = 60;
  public static inline var STOCK_DEPTH:Float = 60;
  public static inline var STOCK_HEIGHT:Float = 20;
  public static inline var STEP_TICK_HZ:Int = 40000;
  /** Assumed assembly gap between carriage blocks, in mm. */
  public static inline var BLOCK_GAP:Float = 4;
  public final vise:Null<machinekit.milling.PneumaticVise>;
  public final base = new MillBase();
  public final column = new MillColumn();
  public final saddle = new MillSaddle();
  public final table = new MillTable();
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
  final powerCableCuts:Array<BomCut> = [];
  final encoderCableCuts:Array<BomCut> = [];

  public function new(withVise:Bool = false, includeWorkpiece:Bool = true) {
    super();
    vise = withVise ? new machinekit.milling.PneumaticVise(STOCK_DEPTH, STOCK_WIDTH) : null;
    var railSpec = guideBlock.spec;
    var halfSpan = (railSpec.blockLength + BLOCK_GAP) / 2;
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
      declareRailFasteners(guide);
      attach(
        'railY$name',
        guide,
        orient(side * base.railSpacing / 2, yRailStart, baseTop + railSpec.railHeight, up, alongY),
        "base"
      );
    }
    var yBlocks = [for (side in [-1, 1]) for (span in [-halfSpan, halfSpan]) orient(
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
      declareRailFasteners(guide);
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
      declareRailFasteners(guide);
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
      yRailLength,
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
      column.height,
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

  /** Each drive reads its lead and bearing boundary from its screw, nut and support parts. */
  function declareRailFasteners(rail:LinearRail):Void {
    var screw = rail.spec.railMountScrew;
    if (screw == null || screw.size == null || !Math.isFinite(screw.length)
      || screw.length <= 0) throw "Rail must declare its mounting screw size and length";
    addBomItem(SocketHeadCapScrew.metric(screw.size, screw.length).bom, rail.holePositions.length);
  }

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
    var motorScrew = flange.holeDiameter == 5.5 ? "M5" : flange.holeDiameter == 6.6 ? "M6" : null;
    if (motorScrew == null) throw "Axis motor flange has no reviewed mounting screw";
    addBomItem(SocketHeadCapScrew.metric(motorScrew, 12).bom, flange.holes.length);
    if (spec.id == "y") {
      var footZ = start[2] + near.connector("mount").frame.y;
      for (end in 0...2) {
        var unit = end == 0 ? near : far;
        var offset = end
          == 0 ? BallScrew.INPUT_JOURNAL_LENGTH - near.length / 2 : length - BallScrew.JOURNAL_LENGTH / 2;
        attach(
          id + 'Stand$end',
          new MillDriveBox(end == 0 ? "MILL-Y-STAND-FIXED" : "MILL-Y-STAND-FLOATING", unit.width, unit.length, footZ - base.height),
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
        var reach = Math.abs(motorPose.x) + motorPlate.thickness + 4 - saddle.width / 2;
        var slot = side < 0 ? motorPose.x + saddle.width / 2 + reach / 2 + motorPlate.thickness / 2 : null;
        attach(
          id + (side < 0 ? "NearBridge" : "FarBridge"),
          new MillDriveBox(side < 0 ? "MILL-X-BRIDGE-NEAR" : "MILL-X-BRIDGE-FAR", reach, 80, 10, slot, motorPlate.thickness),
          AssemblyFrames.translation(side * (saddle.width / 2 + reach / 2), start[1], footZ - 10),
          parent
        );
      }
    } else {
      for (end in 0...2) {
        var unit = end == 0 ? near : far;
        var offset = end
          == 0 ? BallScrew.INPUT_JOURNAL_LENGTH - near.length / 2 : length - BallScrew.JOURNAL_LENGTH / 2;
        var footX = start[0] + unit.connector("mount").frame.y;
        attach(
          id + 'Stand$end',
          new MillDriveBox(end == 0 ? "MILL-Z-STAND-FIXED" : "MILL-Z-STAND-FLOATING", footX - column.width / 2, unit.width, unit.length),
          AssemblyFrames.translation((footX + column.width / 2) / 2, start[1], start[2] - offset - unit.length / 2),
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
  function attach(id:String, component:MachineComponent, pose:AssemblyFrame, parent:String):Void {
    addComponent(id, component);
    zeroPoses.set(id, pose);
    connect(parent, id);
    addMate('$id-mount', "fixed", parent, 'to-$id', id, 'attach-$id');
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
  final slot:Null<Float>;
  final slotWidth:Float;
  public function new(drawing:String, width:Float, depth:Float, height:Float, ?slot:Float, slotWidth:Float = 0) {
    super('MILL-BRACKET-$width-$depth-$height-$slot-$slotWidth', "Mill drive support bracket", "steel", true);
    declareDrawingNumber(drawing);
    addFacet(new machinekit.component.ProcurementIntent(machinekit.component.Procurement.Make("CNC machining")));
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.width = width;
    this.depth = depth;
    this.height = height;
    this.slot = slot;
    this.slotWidth = slotWidth;
  }
  override public function revisionFingerprint():Null<String> return machinekit.component.BomRevision.fingerprint(
    [drawingNumber, materialSpec(), machinekit.component.ComponentNumberKey.encode(width),
      machinekit.component.ComponentNumberKey.encode(depth), machinekit.component.ComponentNumberKey.encode(height),
      slot == null ? "none" : machinekit.component.ComponentNumberKey.encode(slot),
      machinekit.component.ComponentNumberKey.encode(slotWidth)].join("|"), 1);
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var body = Solids.named(Part.box(width, depth, height), "bracket");
    if (slot == null) return body;
    return Solids.cut(
      body,
      [Solids.named(Part.box(slotWidth, depth + 2, height + 2).translated(new Vector(slot, 0, -1)), "motor-plate-seat")]
    );
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
