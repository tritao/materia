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
class FlexsplineHobber extends MachineAssembly implements machinekit.cover.WayCover.CoverMembers {
  public final zBellows:Array<machinekit.cover.WayCover>=[];
  public final zBellowsMounts:Array<String>=[];
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
  public static final Y = HobberYLayout.JOINT;
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
  public final yModule:ScrewLinearModule;
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
      group("Bed and column/Leveling feet/Foot " + (i + 1));
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
      {id: "Y", lower: -15, upper: 15, initial: 0},
      {id: "Z", lower: HobberZLayout.TRAVEL_MIN, upper: HobberZLayout.TRAVEL_MAX, initial: 0}
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
    var firstSolePad = FoundationLayout.columnSolePads()[0];
    fixExisting("column-mount", "Xbridge", "column/"+firstSolePad.id,
      F.alongY(centre+(firstSolePad.x0+firstSolePad.x1)/2,(firstSolePad.y0+firstSolePad.y1)/2,
        HobberSaddle.xCarriageTop(),0,0,1));
    group("Z axis");
    for (placed in HobberRailPlacement.placements("column", FoundationLayout.placements("column"))) {
      var insert = "column/" + placed.insertId;
      fixAt(placed.memberId, placed.rail, placed.pose, insert, placed.contact);
      precisionMounts.push({name: placed.memberId + "-mount", parent: insert});
    }
    for (side in [-1, 1]) for (row in [0.0, HobberZLayout.BLOCK_SPACING]) {
      var id = "blockZ" + (side < 0 ? "Left" : "Right") + (row == 0 ? "" : "Upper");
      var block = LinearRailBlock.metric("HGH15CA", LinearRailBlock.SUPPLIER);
      var pose = AxisBuilder.orient(centre + 161, side * 145, HobberZLayout.SEAT_Z + row, [-1.0, 0, 0], [0.0, 0, 1]);
      var face = F.alongY(centre + 148, side * 145, HobberZLayout.SEAT_Z + row, -1, 0, 0);
      if (id == "blockZLeft") {
        addComponent(id, block);
        zero.set(id, pose);
        moving("Z", Prismatic, "railZLeft", id, F.translation(centre + 161, 0, HobberZLayout.SEAT_Z), [0.0, 0, 1], 0, HobberZLayout.TRAVEL_MIN, HobberZLayout.TRAVEL_MAX);
        fixAt("Zplate", new HobberSaddle("GE-Z-saddle"), F.translation(centre, 0, HobberZLayout.SEAT_Z), id, face);
      } else fixAt(id, block, pose, "Zplate", face);
    }
    // A axis: eRob housing in the Z saddle pocket, with the ground Y module base directly on its output flange. The actuator's own brake holds the head; its holding torque is an assumption.
    group("A swivel");
    swivel = new ErobActuator(100, 20, true, "A", new ComponentValues().setNumber("brakeHoldingTorque", A_BRAKE_HOLDING), HobberSwivelLayout.RANGE, A_ACCELERATION);
    includeAt("swivel", swivel, HobberSwivelLayout.pose());
    fixExisting("swivel-mount", "Zplate", "swivel/housing", F.alongY(HobberSwivelLayout.OUTPUT_X + 6, 0, HobberZLayout.SEAT_Z, 1, 0, 0));
    var e0 = HobberSwivelLayout.SPINDLE_ROTATION;
    group("Y axis");
    yModule=new ScrewLinearModule(HobberYLayout.moduleSpec(),new HobberYPart(new ComponentValues().setToken("role","base")),new HobberYPart(new ComponentValues().setToken("role","carriage")));
    yModule.takeServiceRemoval("carriage");
    includeAt("Ymodule",yModule,HobberYLayout.modulePose());
    fixExisting("Ymodule-mount","swivel/output","Ymodule/base",F.alongY(HobberSwivelLayout.OUTPUT_X,0,HobberZLayout.SEAT_Z,-1,0,0));
    group("B spindle");
    spindle = new HobSpindle();
    includeAt("spindle", spindle, F.fromRotationMatrix(centre, HobberSwivelLayout.SPINDLE_Y, HobberZLayout.SEAT_Z, e0));
    // One milled split-clamp block holds the Ø80 cartridge (FRAME_PLAN R2): bore with a slit up the +z side, three clamp screws across it,
    // four M8 and two dowels into the Y saddle's plate.
    fixAt("spindleSeat", new HobberSaddle("GE-spindle-seat"), F.translation(centre, 0, HobberZLayout.SEAT_Z), "Ymodule/carriage",
      F.alongY(centre + 45, 150, HobberZLayout.SEAT_Z, 1, 0, 0));
    fixExisting("spindle-mount", "spindleSeat", "spindle/housing", F.alongY(centre + 40, 150, HobberZLayout.SEAT_Z, 1, 0, 0));
    // The belt-drive servo stands above the cartridge on a milled cradle bolted to the top of the spindle seat block (the stiffest part
    // near it); its flange is on the cradle wall.
    fixAt("motorCradle", new HobberSaddle("GE-motor-cradle"), F.translation(centre, 0, HobberZLayout.SEAT_Z), "spindleSeat",
      F.alongY(centre, 190, HobberZLayout.SEAT_Z + 58, 0, 0, 1));
    fixExisting("motor-mount", "motorCradle", "spindle/motor",
      F.alongY(centre - HobSpindle.MOTOR_OFFSET, HobberSwivelLayout.SPINDLE_Y - HobSpindle.MOTOR_FACE_Z, HobberZLayout.SEAT_Z + HobSpindle.motorHeight(), 0, 1, 0));
    fixAt("hob", hob, F.fromRotationMatrix(centre, 20, HobberZLayout.SEAT_Z, e0), "spindle/arbor");
    group("B spindle/Outboard support");
    fixAt("outboardCarrier", new HobberSaddle("GE-outboard-carrier"), F.translation(centre, 0, HobberZLayout.SEAT_Z), "Ymodule/carriage",
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
      HobberElectricalLayout.devicePose(-105, HobberSaddle.ELECTRICAL_HEIGHT - 120), "electricalPanel");
    exposePort("mains48", "supply48", "mains");
    // The three sealed scales are read by a BiSS-C master module of the controller.
    fixAt("bissMaster", new BissMaster(4), HobberElectricalLayout.devicePose(110, HobberSaddle.ELECTRICAL_HEIGHT - 120), "electricalPanel");
    // Four short ground bars, matched in thickness per axis; open nut channel.
    for(axis in [HobberDrives.x(),HobberDrives.z()]) {
      group(axis.id+" axis/Drive");var insert=axis.id=="X"?"bed/driveSeat":"column/driveSeat";
      for(support in ["fixed","floating"]) {
        var id=HobberDrives.barMember(axis.id,support),face=HobberPadBolts.face(axis,support);
        fixAt(id,new HobberSaddle(HobberDrives.barDesign(axis.id,support)),F.identity(),insert,face.frame);
        precisionMounts.push({name:id+"-mount",parent:insert});
      }
    }
    drive(HobberDrives.x(),HobberDrives.barMember("X","fixed"),HobberDrives.barMember("X","floating"),"Xbridge");
    drive(HobberDrives.z(),HobberDrives.barMember("Z","fixed"),HobberDrives.barMember("Z","floating"),"Zplate");
    bind("Ydrive","Ymodule/screw-turn","Ymodule/motor");
    // Sealed scales (FRAME_PLAN H4): X on a ground bar on the bed beam, Z on a bar on the column channel's wall with its head on
    // an arm from the Z saddle, Y on the swivel plate with its head on a boss of the Y saddle.
    var scaleX = HobberScales.x(), scaleZ = HobberScales.z();
    group("X axis/Scale");
    fixAt("XscaleSeat", new HobberSaddle("GE-X-scale-seat"), F.identity(), "bed/casting", scaleBarContact(scaleX));
    group("Z axis/Scale");
    fixAt("ZscaleSeat", new HobberSaddle("GE-Z-scale-seat"), F.identity(), "column/casting", scaleBarContact(scaleZ));
    var armFace = HobberScales.headFace(scaleZ);
    fixAt("ZscaleArm", new HobberSaddle("GE-Z-scale-arm"), F.translation(0, 0, HobberZLayout.SEAT_Z), "Zplate",
      F.alongY(148, armFace[1] - HobberScales.Z_ARM_THICKNESS / 2, armFace[2], 1, 0, 0));
    scale(scaleX, "XscaleSeat", "Xrib-scale");
    connectPorts("Y-biss","Ymodule/scaleHead","biss","bissMaster","ch"+(bissChannel++ + 1));
    scale(scaleZ, "ZscaleSeat", "ZscaleArm");
    // The chip tray around the table is part of the work zone; the coolant nozzle belongs to the hob it feeds.
    withGroup("C table and work", () -> {
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
    });
    withGroup("B spindle", () -> {
      fixAt(
        "coolantPort",
        new HobberSaddle("GE-coolant"),
        F.translation(centre, 0, HobberZLayout.SEAT_Z),
        "Ymodule/carriage",
        F.alongY(centre + 45, 64.35, 435, 1, 0, 0)
      );
    });
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
    installZBellows();
    // The screws, washers, nuts and the holes they need follow from the bolted faces the parts declare (BOM_PLAN F1).
    deriveFasteners();
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
    for (axis in ["X", "Z"]) {
      var back = away(axisOf(HobberDrives.member(axis,"motor"), 0, 0, 1));
      out.push(ServiceRemoval(axis.toLowerCase() + "MotorWithdrawal", [HobberDrives.member(axis,"motor"), HobberDrives.member(axis,"motorShaft")],
        [{direction: back, distance: 30 + 10 + SERVICE_CLEARANCE}], [], []));
    }
    var nutGroup=below("Ymodule/nutCarrier").concat(["Ymodule/nutCarrier"]);
    var carriageBundle=below("Ymodule/carriage").concat(["Ymodule/carriage"]).filter(id->!nutGroup.contains(id)&&!StringTools.startsWith(id,"Ymodule/block")&&id!="Ymodule/scaleHead");
    out.push(ServiceRemoval("yCarriageWithdrawal",carriageBundle,[{direction:worldDir(-1,0,0),distance:HobberYLayout.moduleSpec().carriageThickness+40}],
      ["Ymodule/scaleHead"],[{joint:"X",value:HobberXLayout.RETRACT}]));
    var shaft = spindle.motor.rating.shaftLength;
    out.push(ServiceRemoval("bMotorWithdrawal", ["spindle/motor", "spindle/motorShaft", "spindle/motorEncoder"],
      [{direction: away(axisOf("spindle/motor", 0, 0, 1)), distance: shaft + 10 + SERVICE_CLEARANCE}], ["spindle/belt", "spindle/motorPulley"], []));
    out.push(ServiceRemoval("bBeltRemoval", ["spindle/belt"],
      [{direction: away(axisOf("spindle/housing", 0, 0, 1)), distance: 2 * HobSpindle.BELT_WIDTH + SERVICE_CLEARANCE}], [], []));
    out.push(ServiceRemoval("outboardRetainersWithdrawal", ["outboardLocknut", "outboardInnerSpacer"],
      [{direction: axisOf("spindle/housing", 0, 0, 1), distance: 16.0}], [], [{joint: "X", value: HobberXLayout.RETRACT}]));
    out.push(ServiceRemoval("outboardWithdrawal", outboardMembers(),
      [{direction: axisOf("spindle/housing", 0, 0, 1), distance: 55.0}], ["outboardLocknut", "outboardInnerSpacer", "Ymodule/scaleHead"], [{joint: "X", value: HobberXLayout.RETRACT}]));
    out.push(ServiceRemoval("spindleCartridgeWithdrawal", cartridgeMembers(),
      [{direction: axisOf("spindle/housing", 0, 0, 1), distance: 180 + 60}], ["spindle/belt", "outboardLocknut", "outboardInnerSpacer", "Ymodule/scaleHead"].concat(outboardMembers()), [{joint: "X", value: HobberXLayout.RETRACT}]));
    // The flange is motor-side. Release the nut flange, motor and BK12;
    // the BF12 stays and the barrel slides through the stationary nut block.
    var x = HobberDrives.x();
    var rearward = HobberXLayout.front().bedRear - (x.axis.start[0] - x.length) + SERVICE_CLEARANCE;
    out.push(ServiceRemoval("xScrewWithdrawal", ["Xscrew", "Xcoupling", "Xnut"],
      [{direction: worldDir(1,0,0), distance: rearward}],
      ["Xmotor", "Xmotor-shaft", "Xfixed", "XfloatingRing"], []));
    var bayPark = travel.filter(t -> t.id == "X")[0].lower;
    var baySlide = 386 + bayPark - (115 + HobberChipCover.nutBounds().maxX + 3) + 3;
    out.push(ServiceRemoval("xMotorBayLid", ["XrearCover", "XrearPlugLeft", "XrearPlugRight"],
      [{direction:worldDir(0,0,1),distance:3.0}, {direction:worldDir(1,0,0),distance:baySlide},
       {direction:worldDir(0,0,1),distance:100.0}], [], [{joint:"X",value:bayPark}]));
    var stack = below("Xbridge").concat(["Xbridge"]).filter(id -> id != "Xnut");
    out.push(ServiceRemoval("xChipGuardWithdrawal",HobberChipCover.fixedMembers(),
      [{direction:worldDir(0,0,1),distance:100.0}],stack,[]));
    var head = below("Ymodule/base").concat(["Ymodule/base"]).filter(id -> id != "Ymodule/motor" && id != "Ymodule/motorShaft");
    var retract = [{joint:"X",value:HobberXLayout.RETRACT}];
    // Keep the head on the actuator: the adapter output screws are buried
    // behind the swivel plate. Release the saddle from its blocks and nut
    // block, and support the disconnected scale head on the fixed scale.
    out.push(ServiceRemoval("erobWithSaddle",["Zplate","swivel/housing","swivel/output","ZscaleArm"].concat(zBellowsMounts.filter(id->id.indexOf("Moving")>=0)).concat(head),
      [{direction:worldDir(-1,0,0),distance:60.0}],["Ymodule/motor","Ymodule/motorShaft"],retract));
    return out;
  }

  /** The pairs of members the hobbing process brings together: the hob in the cup's band. */
  public function processContacts():Array<{a:String, b:String}> return [{a: "blank", b: "hob"}, {a: "mandrel", b: "hob"}, {a: "mandrelSleeve", b: "hob"}].concat([for(pair in yModule.stopContacts()){a:"Ymodule/"+pair.a,b:"Ymodule/"+pair.b}]).concat(zBellowsInternalContacts());

  /** Thin nested cloth is internal to one bought cover, with its own radial
   * clearance rather than the machine's 2 mm running gap. Only grant those
   * internal pairs after actual-solid zero-overlap checks at both overtravel
   * limits and mid travel. External guard clearances remain machine checks. */
  function zBellowsInternalContacts():Array<{a:String,b:String}> {
    var out:Array<{a:String,b:String}>=[],poses=solvedPoses(),entries=components();
    PosedParts.scope(cache->{
      for(cover in zBellows) {
        var ids=cover.members();
        for(q in [HobberZBellowsLayout.lowerLimit(),0.0,HobberZBellowsLayout.upperLimit()]) {
          var shapes:Array<cadkit.modeling.Part>=[];
          for(i in 0...ids.length) {
            var id=ids[i],entry=entries.filter(m->m.id==id)[0],pose=poses.get(id);
            if(pose==null)throw "Missing Z bellows pose: "+id;
            var ratio=i==0?0.0:i==ids.length-1?1.0:i*1.0/cover.plan.levels;
            shapes.push(cache.posed(entry.component,F.compose(F.translation(0,0,ratio*q),pose)));
          }
          for(i in 0...shapes.length)for(j in i+1...shapes.length) {
            var volume=PosedParts.commonVolume(shapes[i],PosedParts.boxOf(shapes[i]),shapes[j],PosedParts.boxOf(shapes[j]));
            if(volume>1e-5)throw 'Z bellows internal overlap: ${ids[i]} / ${ids[j]} at $q: $volume mm3';
          }
          PosedParts.closeAll(shapes);
        }
        for(i in 0...ids.length)for(j in i+1...ids.length)out.push({a:ids[i],b:ids[j]});
      }
    });
    return out;
  }

  /** Bellows flanges and loose skins use the shared coupled-cover builder. */
  function installZBellows():Void {
    var p=HobberZBellowsLayout.plan();
    for(side in [-1,1])for(upper in [false,true]) {
      var id="Zbellows"+(side<0?"Left":"Right")+(upper?"Upper":"Lower"),moving=id+"MovingMount",fixed=id+"FixedMount";
      group("Z axis/Protection/"+id+"/Mounts");
      var movingPose=HobberZBellowsLayout.pose(side,upper,false);
      fixAt(moving,HobberZBellowsLayout.movingMount(),movingPose,"Zplate");zBellowsMounts.push(moving);
      if(upper) {
        var mount=HobberZBellowsLayout.fixedUpperMount(),pose=HobberZBellowsLayout.pose(side,true,true);
        var face=mount.fasteningFaces().filter(f->f.id=="column")[0];
        var contact=F.compose(pose,face.frame);
        var rail=HobberRailPlacement.placements("column",FoundationLayout.placements("column")).filter(r->side*r.contact.y>0)[0];
        fixAt(fixed,mount,pose,"column/"+rail.insertId,contact);zBellowsMounts.push(fixed);
      }
      var origin=HobberZBellowsLayout.pose(side,upper,true),cover=machinekit.cover.WayCover.installOn(this,this,{
        id:id,row:p.row.id,fixedParent:upper?fixed:"Xbridge",movingParent:moving,
        leader:{id:"Z",lower:HobberZBellowsLayout.lowerLimit(),upper:HobberZBellowsLayout.upperLimit(),initial:0.0},
        origin:[origin.x,origin.y,origin.z],along:[0.0,0,upper?-1.0:1.0],up:[-1.0,0,0],width:p.width,height:p.height,space:p.space,shape:p.shape});
      zBellows.push(cover);
      var last=cover.skinMembers[cover.skinMembers.length-1];
      addMemberConnector(last,"end-guide",F.compose(F.inverse(required(last)),movingPose));
      addMemberConnector(cover.movingMember,"end-guide",F.compose(F.inverse(required(cover.movingMember)),movingPose));
      addConstraintOnAxis(id+"-end-guide","prismatic",last,"end-guide",cover.movingMember,"end-guide",{x:0.0,y:0,z:1});
    }
  }
  public function axisDirection(id:String):Array<Float> {if(id!="Z")throw "Unknown cover axis: "+id;return [0.0,0,1];}
  public function attach(id:String,component:MachineComponent,pose:AssemblyFrame,parent:String):Void {
    if(Std.isOfType(component,machinekit.cover.WayCoverSection)) {
      group("Z axis/Protection/"+id.substr(0,id.lastIndexOf("-"))+"/"+(StringTools.endsWith(id,"-fixed")?"Fixed end":"Moving end"));
      component=new HobberZBellowsPart(component.values());
    }
    fixAt(id,component,pose,parent);
  }
  public function hang(id:String,component:MachineComponent,pose:AssemblyFrame,parent:String):Void {
    if(Std.isOfType(component,machinekit.cover.WayCoverSection)) {
      group("Z axis/Protection/"+id.substr(0,id.lastIndexOf("-"))+"/Folds");
      component=new HobberZBellowsPart(component.values());
    }
    addComponent(id,component);zero.set(id,pose);
    var meeting=F.translation(pose.x,pose.y,pose.z);
    addMemberConnector(parent,'to-$id',F.compose(F.inverse(required(parent)),meeting));
    addMemberConnector(id,'attach-$id',F.compose(F.inverse(pose),meeting));
  }

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
    fixAt(HobberDrives.member(id,"mount"), dr.fixed, at.get("mount"), fixedPad);
    fixAt(HobberDrives.member(id,"motor"), motor, at.get("motor"), HobberDrives.member(id,"mount"));
    addComponent(HobberDrives.member(id,"screw"), screw);
    zero.set(HobberDrives.member(id,"screw"), at.get("screw"));
    var f = F.alongY(s[0], s[1], s[2], d[0], d[1], d[2]);
    // The screw turns in the mount's bearings: the mount is the joint's parent (the motor is bolted to it).
    addMemberConnector(HobberDrives.member(id,"mount"), id + "turnParent", F.compose(F.inverse(required(HobberDrives.member(id,"mount"))), f));
    addMemberConnector(HobberDrives.member(id,"screw"), id + "turnChild", F.compose(F.inverse(required(HobberDrives.member(id,"screw"))), f));
    var block = HobberDrives.nutBlock(hd, nut), blockPose = HobberDrives.nutBlockPose(hd, nut);
    fixAt(HobberDrives.member(id,"nutCarrier"), block, blockPose, blockHost, F.compose(blockPose, F.translation(0, -block.centreHeight, block.thickness / 2)));
    fixAt(HobberDrives.member(id,"nut"), nut, at.get("nut"), HobberDrives.member(id,"nutCarrier"), blockPose);
    var ratio = addTransmission(HobberDrives.member(id,"lead"), id, HobberDrives.member(id,"screw-turn"), Transmission.LeadScrew(HobberDrives.member(id,"screw"), HobberDrives.member(id,"nut")), id == "Z" ? Opposite : id == "X" ? Opposite : Same);
    var initial = travel.filter(t -> t.id == id)[0].initial;
    addMate(HobberDrives.member(id,"screw-turn"), Continuous, HobberDrives.member(id,"mount"), id + "turnParent", HobberDrives.member(id,"screw"), id + "turnChild", ratio * initial);
    fixAt(HobberDrives.member(id,"floating"), dr.floating, at.get("floating"), floatingPad);
    fixAt(HobberDrives.member(id,"ring"), dr.ring, at.get("ring"), HobberDrives.member(id,"floating"));
    supportScrew(HobberDrives.member(id,"lead"), ScrewSupport.Fixed, dr.floating.support);
    fixAt(HobberDrives.member(id,"coupling"), dr.coupling, at.get("coupling"), HobberDrives.member(id,"screw"));
    fixAt(HobberDrives.member(id,"motorShaft"), motor.shaft(), at.get("motor"), HobberDrives.member(id,"coupling"));
    mountContacts.push({parent: fixedPad, child: HobberDrives.member(id,"mount")});
    mountContacts.push({parent: floatingPad, child: HobberDrives.member(id,"floating")});
    // Position feedback comes from the sealed scales (`scale`).
    bind(id + "drive", HobberDrives.member(id,"screw-turn"), HobberDrives.member(id,"motor"));
    // The braked Z servo holds its axis when the power drops: the controller releases the brake and reads back that it let go.
    if (dr.motor.motorBrakeSpec() != null) bindMotorBrake(id + "drive", HobberDrives.member(id,"motor"), id + ".brakeRelease", id + ".brakeReleased", 250, 30);
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
    var housing = HobberScales.member(a.id), head = HobberScales.member(a.id,true);
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
