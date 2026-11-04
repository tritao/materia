import cadkit.modeling.AssemblyState;
import gearkit.*;
import machinekit.assembly.*;
import machinekit.assembly.AxisBuilder;
import machinekit.component.*;
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
  public static final A = "swivel/A";
  public final spec:GearSpec;
  public final hob:Hob;
  public final mandrel:ExpandingMandrel;
  public final bed:MineralCastBed;
  public final column:MineralCastColumn;
  public final table:machinekit.assembly.MachineAssembly;
  public final tableSpec:machinekit.motion.RotaryAxisSpec;
  public final spindle:HobSpindle;
  public final swivel:SwivelHead;
  public final travel:Array<HobberTravel>;
  public final workOrigin:AssemblyFrame;
  public final precisionMounts:Array<{name:String, parent:String}> = [];
  public final mountContacts:Array<{parent:String, child:String}> = [];
  final zero:Map<String, AssemblyFrame> = [];
  var driverNumber = 0;
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
    hob = Hob.fromDesign(new HobDesign(ConjugateRack.of(profile),
      1, 32, gashes), this.spec, 40, 16, false, detailedHob);
    bed = new MineralCastBed();
    column = new MineralCastColumn();
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
    for (placed in HobberRailPlacement.placements("bed", FoundationLayout.placements("bed"))) {
      var insert = "bed/" + placed.insertId;
      fixAt(placed.memberId, placed.rail, placed.pose, insert, placed.contact);
      precisionMounts.push({name: placed.memberId + "-mount", parent: insert});
    }
    // The prismatic joint is the guide block on its rail; the carriage is fixed to that block and carries the others.
    for (side in [-1, 1]) for (row in [HobberXLayout.BLOCK_FRONT, HobberXLayout.BLOCK_REAR]) {
      var id = "blockX" + (side < 0 ? "Left" : "Right") + (row == HobberXLayout.BLOCK_FRONT ? "" : "Front");
      var block = LinearRailBlock.metric("HGH25CA", LinearRailBlock.SUPPLIER);
      var pose = AxisBuilder.orient(centre + row, side * FoundationLayout.BED_RAIL_CENTRE, 242, [0.0, 0, 1], [1.0, 0, 0]);
      var top = F.alongY(centre + row, side * FoundationLayout.BED_RAIL_CENTRE, 260, 0, 0, 1);
      if (id == "blockXLeft") {
        addComponent(id, block);
        zero.set(id, pose);
        moving("X", Prismatic, "railXLeft", id, F.translation(0, 0, 242), [1.0, 0, 0], cutDistance, minX, HobberXLayout.RETRACT);
        fixAt("Xbridge", new HobberSaddle("GE-X-carriage"), F.translation(centre, 0, 0), id, top);
      } else fixAt(id, block, pose, "Xbridge", top);
    }
    includeAt("column", column, FoundationLayout.worldPose("column"));
    fixExisting("column-mount", "Xbridge", "column/basePlate", F.alongY(centre + 326, 0, 280, 0, 0, 1));
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
    var swivelMotor = new TorqueMotor(new ComponentValues().setNumber("height", 20).setNumber(
      "ratedSpeed",
      Math.PI / 12
    ).setNumber(
      "maxSpeed",
      Math.PI / 6
    ).setNumber(
      "torqueConstant",
      2
    ).setNumber(
      "backEmfConstant",
      2 / Math.sqrt(3)
    ));
    swivel = new SwivelHead(swivelMotor, null, Math.PI / 12);
    includeAt("swivel", swivel, F.fromRotationMatrix(centre + 135, 0, HobberZLayout.SEAT_Z, [0.0, 0, -1, 1, 0, 0, 0, -1, 0]));
    fixExisting("swivel-mount", "Zplate", "swivel/housing", F.alongY(centre + 135, 0, HobberZLayout.SEAT_Z, 1, 0, 0));
    var e0 = [-1.0, 0, 0, 0, 0, -1, 0, -1, 0];
    fixAt(
      "swivelPlate",
      new HobberSaddle("GE-swivel-plate"),
      F.translation(centre, 0, HobberZLayout.SEAT_Z),
      "swivel/head",
      F.alongY(centre + 100, 0, HobberZLayout.SEAT_Z, -1, 0, 0)
    );
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
    spindle = new HobSpindle(
      null,
      new HobArbor(new ComponentValues().setNumber("height", 175).setNumber("hobCentreOffset", 60).setNumber(
        "supportDiameter",
        15
      ).setNumber(
        "supportLength",
        22
      ))
    );
    includeAt("spindle", spindle, F.fromRotationMatrix(centre, 240, HobberZLayout.SEAT_Z, e0));
    fixExisting("spindle-mount", "Yplate", "spindle/housing", F.alongY(centre + 40, 210, HobberZLayout.SEAT_Z, 1, 0, 0));
    fixAt("hob", hob, F.fromRotationMatrix(centre, 20, HobberZLayout.SEAT_Z, e0), "spindle/arbor");
    fixAt(
      "outboard",
      new OutboardSupport(new ComponentValues().setNumber("height", 30).setNumber("seatLength", 9)),
      F.fromRotationMatrix(centre, -85, HobberZLayout.SEAT_Z, e0),
      "Yplate",
      F.alongY(centre + 30, -100, HobberZLayout.SEAT_Z, 1, 0, 0)
    );
    fixAt("outboardBearing", new FloatingRadialBearing(), F.fromRotationMatrix(centre, -95.5, HobberZLayout.SEAT_Z, e0), "outboard");
    mountContacts.push({parent: "outboard", child: "outboardBearing"});
    mountContacts.push({parent: "spindle/arbor", child: "outboardBearing"});
    var supply = new ThreePhaseRectifier(
      new AcMains(ThreePhase, new AcRmsVoltage(400), 50),
      new DcCurrent(40),
      6,
      240,
      120,
      100,
      15000
    );
    fixAt("electricalPanel", new HobberSaddle("electrical-panel"), HobberElectricalLayout.panelPose(), "bed/liftLeft");
    fixAt("powerSupply", supply, HobberElectricalLayout.devicePose(0, 130), "electricalPanel");
    exposePort("mains", "powerSupply", "mains");
    // Drive seats: pads under the two supports of X and Z (the nut block travels between them), brackets on the swivel plate for Y.
    var xDrive = HobberDrives.x(), yDrive = HobberDrives.y(), zDrive = HobberDrives.z();
    for (support in ["fixed", "floating"]) {
      var name = support == "fixed" ? "Fixed" : "Floating";
      var xb = HobberDrives.padBounds(xDrive, support, 80, 15), zb = HobberDrives.padBounds(zDrive, support, 80, 40);
      fixAt("Xdrive" + name, new HobberSaddle("GE-X-drive-" + support), F.identity(), "bed/driveSeat",
        F.alongY((xb[0] + xb[3]) / 2, 0, 140, 0, 0, 1));
      precisionMounts.push({name: "Xdrive" + name + "-mount", parent: "bed/driveSeat"});
      fixAt("Zdrive" + name, new HobberSaddle("GE-Z-drive-" + support), F.identity(), "column/driveSeat",
        F.alongY(HobberZLayout.SEAT_FACE_X, 0, (zb[2] + zb[5]) / 2, -1, 0, 0));
      precisionMounts.push({name: "Zdrive" + name + "-mount", parent: "column/driveSeat"});
      var yHoles = HobberDrives.seatHoles(yDrive, support), yLo = 1e9, yHi = -1e9;
      for (h in yHoles) {
        yLo = Math.min(yLo, h.position[1]);
        yHi = Math.max(yHi, h.position[1]);
      }
      fixAt("Ydrive" + name, new HobberSaddle("GE-Y-drive-" + support), F.identity(), "swivelPlate",
        F.alongY(92.5, support == "fixed" ? (yLo + yHi) / 2 : -85, 458, 0, 0, 1));
    }
    fixAt("YnutSeat", new HobberSaddle("GE-Y-nut-seat"), F.identity(), "Yplate", F.alongY(60, -10, 458, 0, 0, 1));
    drive(xDrive, "XdriveFixed", "XdriveFloating", "Xbridge");
    drive(zDrive, "ZdriveFixed", "ZdriveFloating", "Zplate");
    drive(yDrive, "YdriveFixed", "YdriveFloating", "YnutSeat");
    // Sealed scales (FRAME_PLAN H4): X on a ground bar on the bed beam, Z on a bar on the column channel's wall with its head on
    // an arm from the Z saddle, Y on the swivel plate with its head on a boss of the Y saddle.
    var scaleX = HobberScales.x(), scaleY = HobberScales.y(), scaleZ = HobberScales.z();
    fixAt("XscaleSeat", new HobberSaddle("GE-X-scale-seat"), F.identity(), "bed/casting", scaleBarContact(scaleX));
    fixAt("ZscaleSeat", new HobberSaddle("GE-Z-scale-seat"), F.identity(), "column/casting", scaleBarContact(scaleZ));
    var armFace = HobberScales.headFace(scaleZ);
    fixAt("ZscaleArm", new HobberSaddle("GE-Z-scale-arm"), F.translation(0, 0, HobberZLayout.SEAT_Z), "Zplate",
      F.alongY(148, armFace[1] - HobberScales.Z_ARM_THICKNESS / 2, armFace[2], 1, 0, 0));
    scale(scaleX, "XscaleSeat", "Xbridge");
    scale(scaleY, "swivelPlate", "Yplate");
    scale(scaleZ, "ZscaleSeat", "ZscaleArm");
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
      F.alongY(centre + 45, 40, 435, 1, 0, 0)
    );
    // Enclosure envelope reserved; panels omitted for the current open inspection layout.
    bind("table-drive", C, "table/" + tableSpec.drivePart);
    bind("swivel-drive", A, "swivel/motor");
    bind("hob-drive", B, "spindle/motor", true);
    // Feedback belongs to the outer drives; the cartridge encoders are also exposed as sensors.
    addEncoder("table-feedback", C, "table/" + tableSpec.encoderPart, "table-drive");
    addEncoder("hob-master", B, "spindle/encoder", "hob-drive");
    bindMotorBrake("swivel-drive", "swivel/clamp", "swivel.clampRelease", "swivel.clampReleased", 1000, 30);
    machinekit.structural.FoundationIntegrity.check(bed).throwIfErrors();
    machinekit.structural.FoundationIntegrity.check(column).throwIfErrors();
    machinekit.structural.FoundationIntegrity.checkMounts(
      this,
      precisionMounts,
      [for (mount in precisionMounts) mount.name => mount.name]
    ).throwIfErrors();
    exposeConnector("work", "mandrel", "work");
    exposeConnector("hobCentre", "hob", "centre");
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
    var housing = a.id + "scale", head = a.id + "scaleHead";
    fixAt(housing, a.housing, a.housingPose, seatHost, F.compose(a.housingPose, a.housing.connector("mount").frame));
    fixAt(head, a.head, a.headPose, headHost, F.compose(a.headPose, a.head.connector("mount").frame));
    addEncoder(a.id + "position", a.id, head);
  }
  function bind(id:String, joint:String, motor:String, process:Bool = false):Void {
    driverNumber++;
    var driver = id + "-driver";
    fixAt(
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
    );
    connectPorts(id + "-power", "powerSupply", "power" + driverNumber, driver, "power");
    addMotor(id, joint, motor, driver, 1, null, process ? {
      speedChannel:"hob.speed", directionChannel:"hob.direction", radiansPerSpeedUnit:2 * Math.PI / 60
    }
    :null);
  }
  public function hobToWork(state:AssemblyState):AssemblyFrame {
    var work = F.compose(state.worldPose("mandrel"), F.translation(0, 0, mandrel.parameterNumber("workSeat")));
    var cutter = F.compose(state.worldPose("hob"), F.translation(0, 0, hob.parameterNumber("height") / 2));
    return F.compose(F.inverse(work), cutter);
  }
}
