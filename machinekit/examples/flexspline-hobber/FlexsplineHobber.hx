import cadkit.modeling.AssemblyState;
import gearkit.*;
import machinekit.assembly.*;
import machinekit.assembly.AxisBuilder;
import machinekit.component.*;
import machinekit.control.BissMaster;
import machinekit.design.DesignRequirement;
import machinekit.motion.*;
import machinekit.hobbing.*;
import machinekit.power.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

typedef HobberTravel = {id:String, lower:Float, upper:Float, initial:Float};

/** Six independent physical axes. There is deliberately no B/C mechanical gear coupling.
    CAD and prismatic coordinates mm; rotary coordinates rad. Nested joint IDs are explicit. */
class FlexsplineHobber extends MachineAssembly {
  /** Work-frame +X (radial) -> machine +Y; work-frame -Y (hob) -> machine +X. */
  public static final MACHINE_FROM_WORK = F.fromRotationMatrix(0, 0, 637, [0.0, -1, 0, 1, 0, 0, 0, 0, 1]);
  /** Collapsed sleeve to cup bore, diametral. The H5 bore (58.050/+0.013) against a sleeve OD of 58.040 -0.005/0 gives
      0.010-0.028 mm, mean 0.019: enough lead-in for the robot and for the oil to close in the 0.5-1.5 MPa contact window
      (workholding/loads/README 2). The previous 0.050 mm could not hold the cup at 10 MPa oil. */
  public static final LOADING_CLEARANCE = 0.02;
  /** Pressed length beyond the tooth band, from the open edge upward (workholding/fem/README case 1). */
  public static final PRESSED_MARGIN = 8.0;
  public static final C = "table/C";
  public static final B = "spindle/B";
  /** The servo's own joint: the belt couples it to B, and the hob drive is bound on it. */
  public static final B_MOTOR = "spindle/motor-turn";
  public static final A = "swivel/A";
  /**
   * ASSUMED holding torque of the A brake, N m at the output flange. ZeroErr has not published it (asked 2026-11-03); this is the
   * actuator's rated output torque (87 N m, ratio 100), a placeholder for a spring-applied holding brake of this class. Replace it from
   * the maker's answer; `HobberFrameChecks.swivelLoad` reports the demand against it.
   */
  public static final A_BRAKE_HOLDING = 87.0;
  /** ASSUMED acceleration limit of A, rad/s^2: the old torque-motor head used the same; not a vendor value. */
  public static final A_ACCELERATION:Float = Math.PI / 2;
  public final spec:GearSpec;
  public final hob:Hob;
  public final mandrel:ExpandingMandrel;
  public final bed:MineralCastBed;
  public final column:MineralCastColumn;
  public final table:machinekit.assembly.MachineAssembly;
  public final tableSpec:machinekit.motion.RotaryAxisSpec;
  public final spindle:HobSpindle;
  public final swivel:ErobActuator;
  public final travel:Array<HobberTravel>;
  public final workOrigin:AssemblyFrame;
  public final precisionMounts:Array<{name:String, parent:String}> = [];
  public final mountContacts:Array<{parent:String, child:String}> = [];
  final zero:Map<String, AssemblyFrame> = [];
  var driverNumber = 0;
  var bissChannel = 0;
  public function new(
    ?spec:GearSpec,
    gashes:Int = 12,
    detailedHob:Bool = false,
    ?rotaryAxis:machinekit.motion.PrecisionRotaryAxis
  ) {
    super();
    this.spec = spec == null ? new GearSpec(0.3, 200, 8, new Involute(20 * Math.PI / 180)) : spec;
    if (this.spec.internal) throw "external hobbing machine requires external gear";
    var profile = this.spec.profile();
    var hobTool = Hob.fromDesign(new HobDesign(ConjugateRack.of(profile),
      1, 32, gashes), this.spec, 40, 16, false, detailedHob);
    hob = new Hob(hobTool.values().setNumber("faceKeyWidth", 5.2));
    // The bed and the column size independently (each its own anchors, lifting trials and casting): side by side.
    function sizedBed():machinekit.structural.MineralFoundation return new MineralCastBed();
    function sizedColumn():machinekit.structural.MineralFoundation return new MineralCastColumn();
    var foundations = machinekit.cache.Parallel.all([sizedBed, sizedColumn]);
    bed = cast foundations[0];
    column = cast foundations[1];
    group("Bed and column");
    includeAt("bed", bed, FoundationLayout.worldPose("bed"));
    var footCentres = FoundationLayout.feet();
    for (i in 0...footCentres.length) {
      var p = footCentres[i], x = p.x + HobberXLayout.front().bedCentre, y = p.y;
      var roof = -590 + FoundationLayout.FOOT_ROOF, plateBottom = roof - FoundationLayout.FOOT_PLATE_THICKNESS;
      var plate = machinekit.structural.LevelingFoot.of(Plate(
        FoundationLayout.FOOT_PLATE_WIDTH,
        FoundationLayout.FOOT_PLATE_WIDTH,
        FoundationLayout.FOOT_PLATE_THICKNESS,
        26
      ));
      fixAt("footSeat" + i, plate, F.translation(x, y, plateBottom), "bed/casting", F.alongY(x, y, roof, 0, 0, 1));
      var footThread = machinekit.standard.FastenerThreads.metric(24, 3);
      var nut = machinekit.structural.LevelingFoot.of(Nut(Hexagon(36), 19, footThread));
      fixAt(
        "footNut" + i,
        nut,
        F.translation(x, y, plateBottom - 19),
        "footSeat" + i,
        F.alongY(x, y, plateBottom, 0, 0, 1)
      );
      fixAt(
        "footLocknut" + i,
        nut,
        F.translation(x, y, plateBottom - 38),
        "footNut" + i,
        F.alongY(x, y, plateBottom - 19, 0, 0, 1)
      );
      var foot = machinekit.structural.LevelingFoot.of(Foot(
        100,
        15,
        footThread,
        15,
        47 + FoundationLayout.FOOT_ROOF - 2,
        47 + FoundationLayout.FOOT_ROOF - FoundationLayout.FOOT_PLATE_THICKNESS - 19,
        Nominal,
        NoCollar
      ));
      fixAt("foot" + i, foot, F.translation(x, y, -637), "footNut" + i);
    }
    group("C table and work");
    var installed:machinekit.motion.PrecisionRotaryAxis = rotaryAxis
      == null ? new machinekit.motion.PurchasedRotaryTable() : rotaryAxis;
    table = installed.assembly();
    tableSpec = installed.specification();
    var tableBase = FoundationLayout.TABLE_FACE - tableSpec.number("height"), seat = FoundationLayout.TABLE_SEAT;
    if (tableBase
      < seat) throw "Rotary-axis height exceeds reserved table well; resize foundation seat before installation";
    var tableParent = "bed/tablePlate";
    if (tableBase > seat) {
      fixAt(
        "tableAdapter",
        new machinekit.motion.PurchasedRotaryPart(new ComponentValues().setToken("variant", "adapter").setNumber(
          "adapterHeight",
          tableBase - seat
        ).setNumber(
          "adapterOutside",
          tableSpec.number("housingDiameter")
        )),
        F.translation(0, 0, seat),
        tableParent
      );
      precisionMounts.push({name: "tableAdapter-mount", parent: tableParent});
      tableParent = "tableAdapter";
    }
    includeAt("table", table, F.translation(0, 0, tableBase));
    fixed("table-mount", tableParent, "top", "table/" + tableSpec.mountPart, "mount");
    if (tableParent == "bed/tablePlate") precisionMounts.push({name: "table-mount", parent: tableParent});
    for (name in table.portNames()) {
      var port = table.port(name, "table");
      exposePort("table." + name, port.instanceId, port.portName);
    }
    var diameter = 2 * (this.spec.rootRadius() - 0.6), workSeat = HobberZLayout.SEAT_Z - FoundationLayout.TABLE_FACE;
    mandrel = new ExpandingMandrel(new ComponentValues().setNumber("outside", diameter - LOADING_CLEARANCE).setNumber(
      "radialFitGap",
      LOADING_CLEARANCE / 2
    ).setNumber(
      "pressedMargin",
      PRESSED_MARGIN
    ).setNumber(
      "workSeat",
      workSeat
    ).setNumber(
      "height",
      workSeat + Math.max(16, this.spec.faceWidth + PRESSED_MARGIN) + 2
    ).setNumber(
      "sleeveStart",
      workSeat
    ).setNumber(
      "sleeveLength",
      Math.max(16, this.spec.faceWidth + PRESSED_MARGIN)
    ).setNumber(
      "stopDiameter",
      diameter + 0.6
    ).setBoolean(
      "separateSleeve",
      true
    ).setNumber(
      "baseDiameter",
      Math.max(tableSpec.number("facePcd") + 30, diameter + 10)
    ).setNumber(
      "baseHeight",
      25
    ).setNumber(
      "baseBore",
      40
    ).setNumber(
      "baseBoreDepth",
      20
    ).setNumber(
      "basePilotDiameter",
      tableSpec.number("centeringDiameter")
    ).setNumber(
      "basePilotDepth",
      tableSpec.number("centeringDiameter") > 0 ? 3 : 0
    ).setNumber(
      "baseBoltPcd",
      tableSpec.number("facePcd")
    ));
    fixAt(
      "mandrel",
      mandrel,
      F.translation(0, 0, FoundationLayout.TABLE_FACE),
      "table/" + tableSpec.workPart,
      F.alongY(0, 20, FoundationLayout.TABLE_FACE, 0, 0, 1)
    );
    fixAt(
      "mandrelSleeve",
      new MandrelSleeve(new ComponentValues().setNumber("outside", diameter - LOADING_CLEARANCE).setNumber(
        "wall",
        mandrel.parameterNumber("sleeveWall")
      ).setNumber(
        "height",
        mandrel.parameterNumber("sleeveLength")
      )),
      F.translation(0, 0, FoundationLayout.TABLE_FACE + mandrel.parameterNumber("sleeveStart")),
      "mandrel"
    );
    workOrigin = F.compose(MACHINE_FROM_WORK, F.translation(0, 0, HobberZLayout.SEAT_Z));
    var blank = new FlexsplineCupBlank(new ComponentValues().setNumber("outside", 2 * this.spec.tipRadius()).setNumber(
      "bore",
      diameter
    ).setNumber(
      "bodyOutside",
      2 * this.spec.rootRadius()
    ).setNumber(
      "bandWidth",
      this.spec.faceWidth
    ));
    fixAt("blank", blank, F.translation(0, 0, HobberZLayout.SEAT_Z), "mandrel");
    var minX = Math.max(25, mandrel.parameterNumber("outside") / 2 + hob.design.outsideDiameter / 2 + 0.2);
    var cutDistance = this.spec.pitchRadius()
      + hob.design.pitchDiameter / 2 + this.spec.profileShift * this.spec.module;
    // Construction frames are at X=0; the joint coordinate supplies radial infeed once.
    var centre = 0.0;
    if (cutDistance < minX
      || cutDistance > 100 || this.spec.faceWidth > 20) throw "gear/fixture exceeds hobber capacity";
    travel = [
      {id: "X", lower: minX, upper: HobberXLayout.RETRACT, initial: cutDistance},
      {id: "Y", lower: -10, upper: 10, initial: 0},
      {id: "Z", lower: -10, upper: 190, initial: 0}
    ];
    group("X axis");
    for (placed in HobberRailPlacement.placements("bed", FoundationLayout.placements("bed"))) {
      var insert = "bed/" + placed.insertId;
      fixAt(placed.memberId, placed.rail, placed.pose, insert, placed.contact);
      precisionMounts.push({name: placed.memberId + "-mount", parent: insert});
    }
    // The prismatic joint is the guide block on its rail; the carriage is fixed to that block and carries the others.
    for (side in [-1, 1]) for (row in [HobberXLayout.BLOCK_FRONT, HobberXLayout.BLOCK_REAR]) {
      var id = "blockX" + (side < 0 ? "Left" : "Right") + (row == HobberXLayout.BLOCK_FRONT ? "" : "Front");
      var block = LinearRailBlock.metric("HGH25CA", LinearRailBlock.SUPPLIER);
      var pose = AxisBuilder.orient(centre + row, side * FoundationLayout.BED_RAIL_CENTRE, HobberSaddle.xGuideOriginZ(), [0.0, 0, 1], [1.0, 0, 0]);
      var top = F.alongY(centre + row, side * FoundationLayout.BED_RAIL_CENTRE, HobberSaddle.xCarriageBottom(), 0, 0, 1);
      if (id == "blockXLeft") {
        addComponent(id, block);
        zero.set(id, pose);
        moving("X", Prismatic, "railXLeft", id, F.translation(0, 0, HobberSaddle.xGuideOriginZ()), [1.0, 0, 0], cutDistance, minX, HobberXLayout.RETRACT);
        fixAt("Xbridge", new HobberSaddle("GE-X-carriage"), F.translation(centre, 0, 0), id, top);
      } else fixAt(id, block, pose, "Xbridge", top);
    }
    for (role in HobberCarriageRib.ROLES) {
      var rib = new HobberCarriageRib(new ComponentValues().setToken("role", role));
      var b = rib.band();
      fixAt("Xrib-" + role, rib, F.identity(), "Xbridge",
        F.alongY(263, (b.low + b.high) / 2, HobberSaddle.xCarriageBottom(), 0, 0, 1));
    }
    group("Bed and column");
    includeAt("column", column, FoundationLayout.worldPose("column"));
    fixExisting("column-mount", "Xbridge", "column/basePlate", F.alongY(centre + 326, 0, HobberSaddle.xCarriageTop(), 0, 0, 1));
    group("Z axis");
    for (placed in HobberRailPlacement.placements("column", FoundationLayout.placements("column"))) {
      var insert = "column/" + placed.insertId;
      fixAt(placed.memberId, placed.rail, placed.pose, insert, placed.contact);
      precisionMounts.push({name: placed.memberId + "-mount", parent: insert});
    }
    for (side in [-1, 1]) for (row in [0, 110]) {
      var id = "blockZ" + (side < 0 ? "Left" : "Right") + (row == 0 ? "" : "Upper");
      var block = LinearRailBlock.metric("HGH15CA", LinearRailBlock.SUPPLIER);
      var pose = AxisBuilder.orient(centre + 161, side * 145, HobberZLayout.SEAT_Z + row, [-1.0, 0, 0], [0.0, 0, 1]);
      var face = F.alongY(centre + 148, side * 145, HobberZLayout.SEAT_Z + row, -1, 0, 0);
      if (id == "blockZLeft") {
        addComponent(id, block);
        zero.set(id, pose);
        moving("Z", Prismatic, "railZLeft", id, F.translation(centre + 161, 0, HobberZLayout.SEAT_Z), [0.0, 0, 1], 0, -10, 190);
        fixAt("Zplate", new HobberSaddle("GE-Z-saddle"), F.translation(centre, 0, HobberZLayout.SEAT_Z), id, face);
      } else fixAt(id, block, pose, "Zplate", face);
    }
    // A axis (FRAME_PLAN R1): the eRob 110 actuator, its housing in a pocket cut into the Z saddle from behind, a turned adapter ring on its
    // output flange and the ground swivel plate on the ring. The actuator's own brake holds the head; its holding torque is an assumption.
    group("A swivel");
    swivel = new ErobActuator(100, 20, true, "A", new ComponentValues().setNumber("brakeHoldingTorque", A_BRAKE_HOLDING), HobberSwivelLayout.RANGE, A_ACCELERATION);
    includeAt("swivel", swivel, HobberSwivelLayout.pose());
    fixExisting("swivel-mount", "Zplate", "swivel/housing", F.alongY(HobberSwivelLayout.OUTPUT_X + 6, 0, HobberZLayout.SEAT_Z, 1, 0, 0));
    var e0 = HobberSwivelLayout.SPINDLE_ROTATION;
    fixAt(
      "swivelAdapter",
      new HobberSaddle("GE-swivel-adapter"),
      F.translation(centre, 0, HobberZLayout.SEAT_Z),
      "swivel/output",
      F.alongY(HobberSwivelLayout.OUTPUT_X, 0, HobberZLayout.SEAT_Z, -1, 0, 0)
    );
    fixAt(
      "swivelPlate",
      new HobberSaddle("GE-swivel-plate"),
      F.translation(centre, 0, HobberZLayout.SEAT_Z),
      "swivelAdapter",
      F.alongY(HobberSwivelLayout.PLATE_X, 0, HobberZLayout.SEAT_Z, -1, 0, 0)
    );
    group("Y axis");
    for (side in [-1, 1]) fixAt(
      side < 0 ? "railYLeft" : "railYRight",
      LinearRail.metric("HGH15CA", 220),
      AxisBuilder.orient(centre + 70, 110, HobberZLayout.SEAT_Z + side * 60, [-1.0, 0, 0], [0.0, -1, 0]),
      "swivelPlate",
      F.alongY(centre + 85, 110, HobberZLayout.SEAT_Z + side * 60, -1, 0, 0)
    );
    for (side in [-1, 1]) for (row in [-40, 40]) {
      var id = "blockY" + (side < 0 ? "Left" : "Right") + (row < 0 ? "" : "Front");
      var block = LinearRailBlock.metric("HGH15CA", LinearRailBlock.SUPPLIER);
      var pose = AxisBuilder.orient(centre + 70, row, HobberZLayout.SEAT_Z + side * 60, [-1.0, 0, 0], [0.0, -1, 0]);
      var face = F.alongY(centre + 57, row, HobberZLayout.SEAT_Z + side * 60, -1, 0, 0);
      if (id == "blockYLeft") {
        addComponent(id, block);
        zero.set(id, pose);
        moving("Y", Prismatic, "railYLeft", id, F.translation(centre + 70, 0, HobberZLayout.SEAT_Z), [0.0, -1, 0], 0, -10, 10);
        fixAt("Yplate", new HobberSaddle("GE-Y-saddle"), F.translation(centre, 0, HobberZLayout.SEAT_Z), id, face);
      } else fixAt(id, block, pose, "Yplate", face);
    }
    group("B spindle");
    spindle = new HobSpindle();
    includeAt("spindle", spindle, F.fromRotationMatrix(centre, HobberSwivelLayout.SPINDLE_Y, HobberZLayout.SEAT_Z, e0));
    // One milled split-clamp block holds the Ø80 cartridge (FRAME_PLAN R2): bore with a slit up the +z side, three clamp screws across it,
    // four M8 and two dowels into the Y saddle's plate.
    fixAt("spindleSeat", new HobberSaddle("GE-spindle-seat"), F.translation(centre, 0, HobberZLayout.SEAT_Z), "Yplate",
      F.alongY(centre + 45, 150, HobberZLayout.SEAT_Z, 1, 0, 0));
    fixExisting("spindle-mount", "spindleSeat", "spindle/housing", F.alongY(centre + 40, 150, HobberZLayout.SEAT_Z, 1, 0, 0));
    // The belt-drive servo stands above the cartridge on a milled cradle bolted to the top of the spindle seat block (the stiffest part
    // near it); its flange is on the cradle wall.
    fixAt("motorCradle", new HobberSaddle("GE-motor-cradle"), F.translation(centre, 0, HobberZLayout.SEAT_Z), "spindleSeat",
      F.alongY(centre, 190, HobberZLayout.SEAT_Z + 58, 0, 0, 1));
    fixExisting("motor-mount", "motorCradle", "spindle/motor",
      F.alongY(centre - HobSpindle.MOTOR_OFFSET, HobberSwivelLayout.SPINDLE_Y - HobSpindle.MOTOR_FACE_Z, HobberZLayout.SEAT_Z + HobSpindle.motorHeight(), 0, 1, 0));
    fixAt("hob", hob, F.fromRotationMatrix(centre, 20, HobberZLayout.SEAT_Z, e0), "spindle/arbor");
    fixAt("outboardCarrier", new HobberSaddle("GE-outboard-carrier"), F.translation(centre, 0, HobberZLayout.SEAT_Z), "Yplate",
      F.alongY(centre + 45, -100, HobberZLayout.SEAT_Z, 1, 0, 0));
    var supportPose = F.compose(F.translation(centre, 0, HobberZLayout.SEAT_Z), HobberOutboardLayout.bodyPose());
    fixAt("outboard", new OutboardSupport(), supportPose, "outboardCarrier",
      F.alongY(centre + 30, -100, HobberZLayout.SEAT_Z, 1, 0, 0));
    for (role in ["sleeve", "gland", "cap"]) fixAt("outboard" + (role == "sleeve" ? "Sleeve" : role == "gland" ? "Gland" : "Cap"),
      new OutboardBearingPart(new ComponentValues().setToken("role", role)),
      supportPose, role == "cap" ? "outboard" : role == "gland" ? "outboardSleeve" : "outboard");
    for (rear in [false, true]) {
      var id = rear ? "outboardBearingRear" : "outboardBearingFront";
      var pose = F.compose(supportPose, F.fromRotationMatrix(0, 0, rear ? HobberOutboardLayout.PAIR_TO : HobberOutboardLayout.PAIR_FROM,
        rear ? [1.0, 0, 0, 0, -1, 0, 0, 0, -1] : [1.0, 0, 0, 0, 1, 0, 0, 0, 1]));
      fixAt(id, new SpindleBearing(), pose, "outboardSleeve");
      mountContacts.push({parent: "spindle/arbor", child: id});
    }
    fixAt("outboardInnerSpacer", new OutboardBearingPart(new ComponentValues().setToken("role", "innerSpacer")),
      supportPose, "spindle/arbor");
    fixAt("outboardLocknut", new machinekit.standard.FinePitchNut(),
      F.compose(supportPose, F.translation(0, 0, HobberOutboardLayout.PAIR_TO + HobberOutboardLayout.SPACER_LENGTH)), "spindle/arbor");
    var supply = new ThreePhaseRectifier(
      new AcMains(ThreePhase, new AcRmsVoltage(400), 50),
      new DcCurrent(40),
      6,
      240,
      120,
      100,
      15000
    );
    // The rack is a free-standing floor unit with its own base plate on the floor, not part of the machine's structure (it was
    // mated to the bed's lifting seat 650 mm away); the wiring to the machine is by ports, not by a mate.
    group("Electrical");
    addComponent("electricalPanel", new HobberSaddle("electrical-panel"), HobberElectricalLayout.panelPose());
    zero.set("electricalPanel", HobberElectricalLayout.panelPose());
    fixAt("powerSupply", supply, HobberElectricalLayout.devicePose(0, 130), "electricalPanel");
    exposePort("mains", "powerSupply", "mains");
    // The 48 V bus for the A actuator, from the same mains: it takes back what the actuator regenerates in a brake chopper and resistor.
    fixAt("supply48", new ThreePhaseDcSupply(new AcMains(ThreePhase, new AcRmsVoltage(400), 50), 48, 20, 1, 125, 125, 120, 300),
      HobberElectricalLayout.devicePose(-105, 700), "electricalPanel");
    exposePort("mains48", "supply48", "mains");
    // The three sealed scales are read by a BiSS-C master module of the controller.
    fixAt("bissMaster", new BissMaster(4), HobberElectricalLayout.devicePose(110, 700), "electricalPanel");
    // Drive seats: one ground bar under both supports of X and Z (the nut block travels above it; grinding makes both seats coplanar and
    // it stays the re-grindable height spacer), two brackets on the swivel plate for Y.
    var xDrive = HobberDrives.x(), yDrive = HobberDrives.y(), zDrive = HobberDrives.z();
    var xb = HobberDrives.padBoundsBoth(xDrive, HobberDrives.PAD_WIDTH, 15), zb = HobberDrives.padBoundsBoth(zDrive, HobberDrives.PAD_WIDTH, 40);
    group("X axis/Drive");
    fixAt("XdrivePad", new HobberSaddle("GE-X-drive-pad"), F.identity(), "bed/driveSeat", F.alongY((xb[0] + xb[3]) / 2, 0, 140, 0, 0, 1));
    precisionMounts.push({name: "XdrivePad-mount", parent: "bed/driveSeat"});
    group("Z axis/Drive");
    fixAt("ZdrivePad", new HobberSaddle("GE-Z-drive-pad"), F.identity(), "column/driveSeat",
      F.alongY(HobberZLayout.SEAT_FACE_X, 0, (zb[2] + zb[5]) / 2, -1, 0, 0));
    precisionMounts.push({name: "ZdrivePad-mount", parent: "column/driveSeat"});
    group("Y axis/Drive");
    for (support in ["fixed", "floating"]) {
      var name = support == "fixed" ? "Fixed" : "Floating";
      var yHoles = HobberDrives.seatHoles(yDrive, support), yLo = 1e9, yHi = -1e9;
      for (h in yHoles) {
        yLo = Math.min(yLo, h.position[1]);
        yHi = Math.max(yHi, h.position[1]);
      }
      fixAt("Ydrive" + name, new HobberSaddle("GE-Y-drive-" + support), F.identity(), "swivelPlate",
        F.alongY(92.5, support == "fixed" ? (yLo + yHi) / 2 : -85, 458, 0, 0, 1));
    }
    fixAt("YnutSeat", new HobberSaddle("GE-Y-nut-seat"), F.identity(), "Yplate", F.alongY(60, -10, 458, 0, 0, 1));
    drive(xDrive, "XdrivePad", "XdrivePad", "Xbridge");
    drive(zDrive, "ZdrivePad", "ZdrivePad", "Zplate");
    drive(yDrive, "YdriveFixed", "YdriveFloating", "YnutSeat");
    // Sealed scales (FRAME_PLAN H4): X on a ground bar on the bed beam, Z on a bar on the column channel's wall with its head on
    // an arm from the Z saddle, Y on the swivel plate with its head on a boss of the Y saddle.
    var scaleX = HobberScales.x(), scaleY = HobberScales.y(), scaleZ = HobberScales.z();
    group("X axis/Scale");
    fixAt("XscaleSeat", new HobberSaddle("GE-X-scale-seat"), F.identity(), "bed/casting", scaleBarContact(scaleX));
    group("Z axis/Scale");
    fixAt("ZscaleSeat", new HobberSaddle("GE-Z-scale-seat"), F.identity(), "column/casting", scaleBarContact(scaleZ));
    var armFace = HobberScales.headFace(scaleZ);
    fixAt("ZscaleArm", new HobberSaddle("GE-Z-scale-arm"), F.translation(0, 0, HobberZLayout.SEAT_Z), "Zplate",
      F.alongY(148, armFace[1] - HobberScales.Z_ARM_THICKNESS / 2, armFace[2], 1, 0, 0));
    scale(scaleX, "XscaleSeat", "Xrib-scale");
    scale(scaleY, "swivelPlate", "Yplate");
    scale(scaleZ, "ZscaleSeat", "ZscaleArm");
    group("Chip protection and enclosure");
    fixAt(
      "chipTray",
      new HobberSaddle(
        "GE-chip-tray",
        new ComponentValues().setNumber("tableFaceDiameter", tableSpec.number("faceDiameter")).setNumber(
          "tableHousingDiameter",
          tableSpec.number("housingDiameter")
        ).setNumber(
          "tableFaceHeight",
          FoundationLayout.TABLE_FACE
        ).setNumber(
          "tableSeatHeight",
          tableBase
        )
      ),
      F.identity(),
      "bed/tablePlate"
    );
    fixAt(
      "coolantPort",
      new HobberSaddle("GE-coolant"),
      F.translation(centre, 0, HobberZLayout.SEAT_Z),
      "Yplate",
      F.alongY(centre + 45, 64.35, 435, 1, 0, 0)
    );
    // Enclosure envelope reserved; panels omitted for the current open inspection layout.
    bind("table-drive", C, "table/" + tableSpec.drivePart);
    // A: the actuator's own amplifier (EtherCAT, 48 V), no separate driver.
    connectPorts("swivel-power", "supply48", "power1", "swivel/housing", "power");
    addMotor("swivel-drive", A, "swivel/housing", "swivel/housing", 1);
    exposePort("swivel.bus", "swivel/housing", "bus");
    bind("hob-drive", B_MOTOR, "spindle/motor", true, spindle.ratio);
    // Feedback belongs to the outer drives; the cartridge encoders are also exposed as sensors.
    addEncoder("table-feedback", C, "table/" + tableSpec.encoderPart, "table-drive");
    // The spindle's own encoder on the cartridge's rear shaft is the B master of the electronic gearbox; the servo's absolute encoder
    // stays on its motor joint for the drive's loop.
    addEncoder("hob-angle", B, "spindle/encoder");
    addEncoder("hob-motor", B_MOTOR, "spindle/motorEncoder", "hob-drive");
    bindMotorBrake("swivel-drive", "swivel/housing", "swivel.brakeRelease", "swivel.brakeReleased", 1000, 30);
    machinekit.structural.FoundationIntegrity.check(bed).throwIfErrors();
    machinekit.structural.FoundationIntegrity.check(column).throwIfErrors();
    machinekit.structural.FoundationIntegrity.checkMounts(
      this,
      precisionMounts,
      [for (mount in precisionMounts) mount.name => mount.name]
    ).throwIfErrors();
    // P4: fixed lapped X sheets, the carriage skirt, and Z channel shields.
    // All feet are on existing steel parts; the castings are unchanged.
    for (r in HobberChipCover.installations()) withGroup(r.role.substr(0, 1) + " axis/Protection", () ->
      fixAt(r.id, new HobberChipCover(new ComponentValues().setToken("role",r.role)), F.identity(),r.parent,r.seat));
    withGroup("X axis/Protection", () -> {
      for (p in HobberChipCover.plugs()) fixAt(p.id,new machinekit.standard.PanelHolePlug(),F.translation(p.x,p.y,p.z),p.parent,
        F.alongY(p.x,p.y,p.z,0,0,-1));
    });
    // The screws, washers, nuts and the holes they need follow from the bolted faces the parts declare (BOM_PLAN F1).
    deriveFasteners();
    // Replicated pad drilling follows the carriage joints, never a second bolt list.
    var deck:Null<MachineComponent> = null;
    for (entry in components()) if (entry.id == "Xbridge") deck = entry.component;
    if (deck == null) throw "Missing X carriage for replicated pads";
    var guide = LinearRailBlock.metric("HGH25CA");
    var film = HobberCarriageStockCheck.padLayer().nominalMm;
    for (side in [-1, 1]) for (row in [HobberXLayout.BLOCK_FRONT, HobberXLayout.BLOCK_REAR]) {
      var y = side * FoundationLayout.BED_RAIL_CENTRE;
      var holes:Array<materia.casting.InsertHole> = [];
      for (hole in deck.derivedRouteHoles()) {
        if (hole.kind != "clearance" || Math.abs(hole.position[0] - row) > guide.spec.blockLength / 2
          || Math.abs(hole.position[1] - y) > guide.spec.blockWidth / 2) continue;
        var record = machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.through(hole.diameter, film))[0];
        record.id = hole.id;
        record.frame = F.fromRotationMatrix(Math.round((hole.position[0] - row) * 1e6) / 1e6,
          Math.round((hole.position[1] - y) * 1e6) / 1e6, film,
          [1, 0, 0, 0, -1, 0, 0, 0, -1]);
        holes.push(record);
      }
      if (holes.length != 4) throw "X replicated pad must inherit four joint-derived clearances";
      holes.sort((a, b) -> a.frame.x == b.frame.x
        ? (a.frame.y < b.frame.y ? -1 : a.frame.y > b.frame.y ? 1 : 0)
        : (a.frame.x < b.frame.x ? -1 : 1));
      for (i in 0...holes.length) holes[i].id = "clearance-" + (i + 1);
      var pad = new machinekit.structural.LinearMountReplication(new ComponentValues()
        .setNumber("width", guide.spec.blockLength).setNumber("length", guide.spec.blockWidth)
        .setNumber("thickness", film).setToken("holes", machinekit.structural.InsertHoles.fromNeutral(holes))
        .setToken("reference", "ASSUMED compound/process properties; four pads replicated against one master to 0.01 mm; "
          + HobberCarriageStockCheck.report()));
      var padId = "Xpad-" + (side < 0 ? "L" : "R") + (row == HobberXLayout.BLOCK_FRONT ? "1" : "2");
      fixAt(padId, pad,
        F.translation(row, y, HobberSaddle.xCarriageBottom()), "Xbridge", F.alongY(row, y, HobberSaddle.xCarriageBottom() + film, 0, 0, 1));
      var blockId = "blockX" + (side < 0 ? "Left" : "Right") + (row == HobberXLayout.BLOCK_FRONT ? "" : "Front");
      var oldContacts = mountContacts.filter(c -> (c.parent == "Xbridge" && c.child == blockId)
        || (c.child == "Xbridge" && c.parent == blockId));
      for (contact in oldContacts) mountContacts.remove(contact);
      mountContacts.push({parent: blockId, child: padId});
    }
    // The loads the joints carry come from the finished parts (masses), so they are checked after the derived holes are cut.
    checkJointLoads(HobberJointLoads.of(this));
    // Chip protection and service (FRAME_PLAN, "Chip protection and service"): wipers on every guide block, and the removals a technician needs.
    addBomItem(machinekit.cover.WayWiperCatalog.bomItem("WIPER-HG25-SCRAPER", 8));
    addBomItem(machinekit.cover.WayWiperCatalog.bomItem("WIPER-HG15-SCRAPER", 16));
    declareServices();
    exposeConnector("work", "mandrel", "work");
    exposeConnector("hobCentre", "hob", "centre");
  }

  /** The assembly's world direction of a work-frame direction (the work frame is `MACHINE_FROM_WORK` turned). */
  static function worldDir(x:Float, y:Float, z:Float):Array<Float> return [-y, x, z];

  /** Every member below `root` in the mate tree, `root` not included. */
  function below(root:String):Array<String> {
    var parents = mateParents(), out = [];
    for (c in components()) {
      var at:Null<String> = c.id, guard = 0;
      while (at != null && at != root && guard++ < 200) at = parents.get(at);
      if (at == root && c.id != root) out.push(c.id);
    }
    return out;
  }

  /** The direction (assembly world frame) of `local` in member `id`'s frame at the rest pose, as a unit vector. */
  function axisOf(id:String, x:Float, y:Float, z:Float):Array<Float> {
    var p = solvedPoses().get(id), v = F.transformVector(p, x, y, z);
    return [v.x, v.y, v.z];
  }

  /** Clearance added to every service path past the length that frees the part, mm. */
  public static final SERVICE_CLEARANCE = 5.0;

  function startsWith(id:String, prefix:String):Bool return StringTools.startsWith(id, prefix);

  /** The members of the B cartridge: the housing with the shaft, bearings, encoder, pulley and arbor, and the hob on it (not the servo, its pulley or the belt). */
  function cartridgeMembers():Array<String> {
    var motorGroup = ["spindle/motor", "spindle/motorShaft", "spindle/motorPulley", "spindle/motorEncoder", "spindle/belt"];
    return [for (c in components()) if (startsWith(c.id, "spindle/") && motorGroup.indexOf(c.id) < 0) c.id].concat(["hob"]);
  }

  /**
   * The service removals the machine declares (`ServiceChecks` sweeps each against everything that stays; screws that hold a removed or prerequisite
   * member are taken away with it). A servo comes off its mount along its own axis, away from the screw or pulley: the shaft stub out of the coupling,
   * then the body free of the flange pilot. The B servo leaves with its belt and pulley off. The B belt slides off both pulleys along their axis. The spindle
   * cartridge, with the hob and arbor, leaves forward out of its split clamp (the front register cannot pass the bore), with the outboard support and belt off.
   * The head stays on the eRob, which leaves with its Z saddle; the Z nut block stays on the screw.
   */
  function declareServices():Void {
    for (r in serviceRemovals()) addDesignRequirement(r);
  }

  public static function outboardMembers():Array<String> return ["outboardCarrier", "outboard", "outboardSleeve", "outboardGland", "outboardCap", "outboardBearingFront", "outboardBearingRear"];

  /** The declared removals. */
  public function serviceRemovals():Array<DesignRequirement> {
    var away = (d:Array<Float>) -> [for (c in d) -c];
    var out:Array<DesignRequirement> = [];
    for (axis in ["X", "Y", "Z"]) {
      var back = away(axisOf(axis + "motor", 0, 0, 1));
      out.push(ServiceRemoval(axis.toLowerCase() + "MotorWithdrawal", [axis + "motor", axis + "motor-shaft"],
        [{direction: back, distance: 30 + 10 + SERVICE_CLEARANCE}], [], []));
    }
    var shaft = spindle.motor.rating.shaftLength;
    out.push(ServiceRemoval("bMotorWithdrawal", ["spindle/motor", "spindle/motorShaft", "spindle/motorEncoder"],
      [{direction: away(axisOf("spindle/motor", 0, 0, 1)), distance: shaft + 10 + SERVICE_CLEARANCE}], ["spindle/belt", "spindle/motorPulley"], []));
    out.push(ServiceRemoval("bBeltRemoval", ["spindle/belt"],
      [{direction: away(axisOf("spindle/housing", 0, 0, 1)), distance: 2 * HobSpindle.BELT_WIDTH + SERVICE_CLEARANCE}], [], []));
    out.push(ServiceRemoval("outboardRetainersWithdrawal", ["outboardLocknut", "outboardInnerSpacer"],
      [{direction: axisOf("spindle/housing", 0, 0, 1), distance: 16.0}], [], [{joint: "X", value: HobberXLayout.RETRACT}]));
    out.push(ServiceRemoval("outboardWithdrawal", outboardMembers(),
      [{direction: axisOf("spindle/housing", 0, 0, 1), distance: 55.0}], ["outboardLocknut", "outboardInnerSpacer", "YscaleHead"], [{joint: "X", value: HobberXLayout.RETRACT}]));
    out.push(ServiceRemoval("spindleCartridgeWithdrawal", cartridgeMembers(),
      [{direction: axisOf("spindle/housing", 0, 0, 1), distance: 180 + 60}], ["spindle/belt", "outboardLocknut", "outboardInnerSpacer", "YscaleHead"].concat(outboardMembers()), [{joint: "X", value: HobberXLayout.RETRACT}]));
    // The flange is motor-side. Release the nut flange, motor and BK12;
    // the BF12 stays and the barrel slides through the stationary nut block.
    var x = HobberDrives.x();
    var rearward = HobberXLayout.front().bedRear - (x.axis.start[0] - x.length) + SERVICE_CLEARANCE;
    out.push(ServiceRemoval("xScrewWithdrawal", ["Xscrew", "Xcoupling", "Xnut"],
      [{direction: worldDir(1,0,0), distance: rearward}],
      ["Xmotor", "Xmotor-shaft", "Xfixed", "XfloatingRing"], []));
    var stack = below("Xbridge").concat(["Xbridge"]).filter(id -> id != "Xnut");
    out.push(ServiceRemoval("xChipGuardWithdrawal",HobberChipCover.fixedMembers(),
      [{direction:worldDir(0,0,1),distance:100.0}],stack,[]));
    var head = below("swivelAdapter").concat(["swivelAdapter"]).filter(id -> id != "Ymotor" && id != "Ymotor-shaft");
    var retract = [{joint:"X",value:HobberXLayout.RETRACT}];
    // Free the under-deck foot, lift it above the deck, then move the
    // C-profile clear of the head. Left/right shields leave to their own side.
    for (side in [-1,1]) out.push(ServiceRemoval(side<0?"zLeftGuardWithdrawal":"zRightGuardWithdrawal",
      [side<0?"ZcoverLeft":"ZcoverRight"],
      [{direction:worldDir(-1,0,0),distance:19.0}, {direction:worldDir(0,0,1),distance:24.0},
       {direction:worldDir(1,0,0),distance:4.0}, {direction:worldDir(0,side,0),distance:200.0}],side>0?["Ymotor","Ymotor-shaft"]:[],retract));
    // Keep the head on the actuator: the adapter output screws are buried
    // behind the swivel plate. Release the saddle from its blocks and nut
    // block, and support the disconnected scale head on the fixed scale.
    out.push(ServiceRemoval("erobWithSaddle",["Zplate","swivel/housing","swivel/output","ZscaleArm"].concat(head),
      [{direction:worldDir(-1,0,0),distance:60.0}],HobberChipCover.zMembers().concat(["Ymotor","Ymotor-shaft"]),retract));
    return out;
  }

  /** The pairs of members the hobbing process brings together: the hob in the cup's band. */
  public function processContacts():Array<{a:String, b:String}> return [{a: "blank", b: "hob"}, {a: "mandrel", b: "hob"}, {a: "mandrelSleeve", b: "hob"}];

  function includeAt(id:String, a:MachineAssembly, pose:AssemblyFrame):Void {
    include(id, a, F.compose(MACHINE_FROM_WORK, pose));
    var positions = a.solvedPoses();
    for (m in a.components()) {
      var p = positions.get(m.id);
      if (p == null) throw "missing package pose";
      zero.set(id + "/" + m.id, F.compose(pose, p));
    }
  }
  function fixed(id:String, parent:String, pc:String, child:String, cc:String):Void addMate(
    id,
    Fixed,
    parent,
    pc,
    child,
    cc
  );
  function fixAt(id:String, p:MachineComponent, pose:AssemblyFrame, parent:String, ?seat:AssemblyFrame):Void {
    addComponent(id, p);
    zero.set(id, pose);
    fixExisting(id + "-mount", parent, id, seat);
  }
  function fixExisting(id:String, parent:String, child:String, ?seat:AssemblyFrame):Void {
    var p = required(parent), c = required(child), frame = seat == null ? F.alongY(c.x, c.y, c.z, 0, 0, 1) : seat;
    if (seat != null) mountContacts.push({parent: parent, child: child});
    addMemberConnector(parent, id + "Parent", F.compose(F.inverse(p), frame));
    addMemberConnector(child, id + "Child", F.compose(F.inverse(c), frame));
    addMate(id, Fixed, parent, id + "Parent", child, id + "Child");
  }
  function required(id:String):AssemblyFrame {
    var p = zero.get(id);
    if (p == null) throw 'missing zero pose $id';
    return p;
  }
  function moving(
    id:String,
    type:AssemblyJointType,
    parent:String,
    child:String,
    origin:AssemblyFrame,
    d:Array<Float>,
    initial:Float,
    lower:Float,
    upper:Float
  ):Void {
    var frame = F.alongY(origin.x, origin.y, origin.z, d[0], d[1], d[2]);
    addMemberConnector(parent, id + "Parent", F.compose(F.inverse(required(parent)), frame));
    addMemberConnector(child, id + "Child", F.compose(F.inverse(required(child)), frame));
    addMateOnAxis(
      id,
      type,
      parent,
      id + "Parent",
      child,
      id + "Child",
      {x: 0, y: 1, z: 0},
      initial,
      {lower: lower, upper: upper, velocity: null, effort: null, acceleration: 500, overtravel: 5}
    );
  }
  /**
   * One axis's drive from `HobberDrives`: the mount on its fixed pad, servo and coupling, the screw turning in them, the BF12
   * and its ring on the floating pad, and the nut on a nut block bolted to `blockHost`. The seat holes are cut into the pads.
   */
  function drive(hd:HobberDriveAxis, fixedPad:String, floatingPad:String, blockHost:String):Void {
    group(hd.id + " axis/Drive");
    var id = hd.id, dr = hd.drive, at = HobberDrives.poses(hd), d = hd.axis.direction, s = hd.axis.start;
    var motor = dr.motor, screw = new HzMotionDrawingScrew(hd.length, dr.endProfile), nut = new BallNut();
    fixAt(id + "fixed", dr.fixed, at.get("mount"), fixedPad);
    fixAt(id + "motor", motor, at.get("motor"), id + "fixed");
    addComponent(id + "screw", screw);
    zero.set(id + "screw", at.get("screw"));
    var f = F.alongY(s[0], s[1], s[2], d[0], d[1], d[2]);
    // The screw turns in the mount's bearings: the mount is the joint's parent (the motor is bolted to it).
    addMemberConnector(id + "fixed", id + "turnParent", F.compose(F.inverse(required(id + "fixed")), f));
    addMemberConnector(id + "screw", id + "turnChild", F.compose(F.inverse(required(id + "screw")), f));
    var block = HobberDrives.nutBlock(hd, nut), blockPose = HobberDrives.nutBlockPose(hd, nut);
    fixAt(id + "nutBlock", block, blockPose, blockHost, F.compose(blockPose, F.translation(0, -block.centreHeight, block.thickness / 2)));
    fixAt(id + "nut", nut, at.get("nut"), id + "nutBlock", blockPose);
    var ratio = addTransmission(id + "lead", id, id + "screw-turn", Transmission.LeadScrew(id + "screw", id + "nut"), id == "Z" ? Opposite : id == "X" ? Opposite : Same);
    var initial = travel.filter(t -> t.id == id)[0].initial;
    addMate(id + "screw-turn", Continuous, id + "fixed", id + "turnParent", id + "screw", id + "turnChild", ratio * initial);
    fixAt(id + "floating", dr.floating, at.get("floating"), floatingPad);
    fixAt(id + "floatingRing", dr.ring, at.get("ring"), id + "floating");
    supportScrew(id + "lead", ScrewSupport.Fixed, dr.floating.support);
    fixAt(id + "coupling", dr.coupling, at.get("coupling"), id + "screw");
    fixAt(id + "motor-shaft", motor.shaft(), at.get("motor"), id + "coupling");
    mountContacts.push({parent: fixedPad, child: id + "fixed"});
    mountContacts.push({parent: floatingPad, child: id + "floating"});
    // Position feedback comes from the sealed scales (`scale`).
    bind(id + "drive", id + "screw-turn", id + "motor");
    // The braked Z servo holds its axis when the power drops: the controller releases the brake and reads back that it let go.
    if (dr.motor.motorBrakeSpec() != null) bindMotorBrake(id + "drive", id + "motor", id + ".brakeRelease", id + ".brakeReleased", 250, 30);
  }
  /** Where a scale's seat bar rests on its host: the far face of the bar from the housing, normal toward the host. */
  function scaleBarContact(a:HobberScaleAxis):AssemblyFrame {
    var face = HobberScales.housingFace(a), t = a.toSeat, bar = HobberScales.SEAT_BAR;
    var b = a.seatBar;
    if (b == null) throw "scale has no seat bar";
    // The bar's contact face is one bar thickness behind the housing's mounting face; the bar is 20 wide across the travel.
    return F.alongY(face[0] + t[0] * bar, face[1] + t[1] * bar, face[2] + t[2] * bar, t[0], t[1], t[2]);
  }
  /** One axis's sealed scale: the housing on its seat, the head on its carrier, and the head as the axis's position encoder. */
  function scale(a:HobberScaleAxis, seatHost:String, headHost:String):Void {
    group(a.id + " axis/Scale");
    var housing = a.id + "scale", head = a.id + "scaleHead";
    fixAt(housing, a.housing, a.housingPose, seatHost, F.compose(a.housingPose, a.housing.connector("mount").frame));
    fixAt(head, a.head, a.headPose, headHost, F.compose(a.headPose, a.head.connector("mount").frame));
    addEncoder(a.id + "position", a.id, head);
    connectPorts(a.id + "-biss", head, "biss", "bissMaster", "ch" + (bissChannel++ + 1));
  }
  function bind(id:String, joint:String, motor:String, process:Bool = false, ratio:Float = 1.0):Void {
    driverNumber++;
    var driver = id + "-driver";
    withGroup("Electrical", () -> fixAt(
      driver,
      new MotorDriver(
        "GENERIC-400V-SERVO-AMP",
        new PhaseRmsCurrent(id == "table-drive" ? tableSpec.number("peakCurrent") : 20)
      ),
      HobberElectricalLayout.devicePose(
        -140 + ((driverNumber - 1) % 3) * 140,
        300 + Std.int((driverNumber - 1) / 3) * 250
      ),
      "electricalPanel"
    ));
    connectPorts(id + "-power", "powerSupply", "power" + driverNumber, driver, "power");
    addMotor(id, joint, motor, driver, 1, null, process ? {
      speedChannel:"hob.speed", directionChannel:"hob.direction", radiansPerSpeedUnit:ratio * 2 * Math.PI / 60
    }
    :null);
  }
  public function hobToWork(state:AssemblyState):AssemblyFrame {
    var work = F.compose(state.worldPose("mandrel"), F.translation(0, 0, mandrel.parameterNumber("workSeat")));
    var cutter = F.compose(state.worldPose("hob"), F.translation(0, 0, hob.parameterNumber("height") / 2));
    return F.compose(F.inverse(work), cutter);
  }
}
