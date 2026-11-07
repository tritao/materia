import machinekit.assembly.RackMount;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.AssemblyIntegrity;
import machinekit.assembly.AssemblyIntegrity.IntegrityReport;
import machinekit.assembly.AxisBuilder;
import machinekit.assembly.AxisBuilder.AxisSpec;
import machinekit.assembly.Sense;
import machinekit.assembly.Transmission;
import machinekit.gantry.GantryParts.GantryPlate;
import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.LeadshineServo;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewNut;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.MotorDriver;
import machinekit.motion.NemaStepper;
import machinekit.structural.SeatedPlate;
import machinekit.transmission.Rack;
import machinekit.transmission.SpurGear;
import materia.assembly.AssemblyFrames;

/** `AssemblyIntegrity` on small assemblies with one defect each, and on their sound versions. */
class AssemblyIntegrityTests {
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw 'Assembly integrity: $message';
  }

  static function codes(report:IntegrityReport):Array<String> return [for (item in report.errors()) item.code];

  static function expectClean(report:IntegrityReport, label:String):Void check(
    report.errors().length == 0,
    '$label should be sound, found: ${[for (item in report.errors()) item.message].join("; ")}'
  );

  static function expectOnly(report:IntegrityReport, wanted:Array<String>, label:String):Void {
    var found = codes(report);
    for (code in wanted) check(found.indexOf(code) >= 0, '$label should report $code, found $found');
    for (code in found) check(
      wanted.indexOf(code) >= 0,
      '$label should not report $code: ${[for (item in report.errors()) item.message].join("; ")}'
    );
  }

  /** A plate on a plate, `lift` mm above (negative: sunk into) the lower one's top. */
  static function stack(lift:Float, baseZ:Float = 0):AxisBuilder {
    var builder = new AxisBuilder();
    builder.place("base", new GantryPlate("TEST-INTEGRITY-BASE", "Integrity base", 100, 100, 10), AssemblyFrames.translation(0, 0, baseZ));
    builder.attach(
      "top",
      new GantryPlate("TEST-INTEGRITY-TOP", "Integrity top", 50, 50, 10),
      AssemblyFrames.translation(0, 0, baseZ + 10 + lift),
      "base"
    );
    return builder;
  }

  static function beltSupports():Void {
    function loop(lift:Float):AxisBuilder {
      var builder = new AxisBuilder();
      builder.place("base", new GantryPlate("TEST-BELT-BASE", "Base", 100, 100, 10), AssemblyFrames.identity());
      for (side in ["left", "right"]) {
        builder.attach(side, new GantryPlate("TEST-BELT-" + side, "Pulley contact", 10, 10, 10),
          AssemblyFrames.translation(side == "left" ? -20 : 20, 0, 10), "base");
        builder.addMemberConnector(side, "wrap", AssemblyFrames.identity());
      }
      builder.attach("belt", new GantryPlate("TEST-BELT-ENVELOPE", "Closed loop envelope", 50, 10, 2),
        AssemblyFrames.translation(0, 0, 20 + lift), "base");
      builder.addBeltPath({belt:"belt", wraps:[
        {instanceId:"left", connectorName:"wrap"}, {instanceId:"right", connectorName:"wrap"}
      ]});
      return builder;
    }
    expectClean(AssemblyIntegrity.check(loop(0), {floorZ:0}),
      "a stationary belt envelope is supported by its declared pulley contacts");
    expectOnly(AssemblyIntegrity.check(loop(2), {floorZ:0}),
      ["integrity.support.gap", "integrity.support.ground"],
      "a declared belt path cannot conceal missing physical support");
  }

  public static function run():Void {
    for (id in ["machine/bed/rail-seat-bolt1", "guide00", "column/lifting-point2"])
      check(machinekit.assembly.OccurrenceNames.problems(id).length == 0, 'readable occurrence $id');
    for (id in ["bed/anchor-deadbeef-bolt", "anchor-ABCDEF12", "seat-1.25", "seat-.5", "seat-2."])
      check(machinekit.assembly.OccurrenceNames.problems(id).length > 0, 'unreadable occurrence $id rejected');
    var namingRejected = false;
    try machinekit.assembly.OccurrenceNames.requireReadable(["sound-role", "anchor-deadbeef"])
    catch (_:Dynamic) namingRejected = true;
    check(namingRejected, "export naming check rejects unreadable paths");
    var badNames = new AxisBuilder();
    badNames.place("anchor-deadbeef", new GantryPlate("TEST-NAMING-PLATE", "Naming test", 20, 20, 5),
      AssemblyFrames.identity());
    expectOnly(AssemblyIntegrity.check(badNames), ["assembly.occurrence-name"],
      "assembly integrity rejects hashed occurrence ids");
    beltSupports();
    expectClean(AssemblyIntegrity.check(stack(0), {floorZ: 0}), "a plate standing on a plate on the floor");
    expectOnly(
      AssemblyIntegrity.check(stack(5)),
      ["integrity.support.gap", "integrity.support.ground"],
      "a plate fixed 5 mm above its base"
    );
    expectOnly(AssemblyIntegrity.check(stack(-2)), ["integrity.overlap"], "a plate sunk 2 mm into its base");
    expectOnly(
      AssemblyIntegrity.check(stack(0, 5), {floorZ: 0}),
      ["integrity.support.ground"],
      "a stack 5 mm above the floor"
    );
    expectClean(AssemblyIntegrity.check(stack(0, 5)), "a stack with no floor stated");
    sweeps();
    seated();
    racks();
    screws();
    strictChecks();
    journals();
    Sys.println("assembly integrity: support, interference, sweep and drive checks pass");
  }

  /** The new checks are off unless asked for; `strict` turns on the running gap, contact area and flange cut-out. */
  static function strictChecks():Void {
    var strict:IntegrityOptions = {strict: true};
    // A face that only rubs: a wall flush with the side of the block that runs past it. No volume is shared, so the
    // overlap check passes; the running gap does not.
    function rubbing(clear:Float):AxisBuilder {
      var builder = new AxisBuilder();
      var up = [0.0, 0, 1], along = [1.0, 0, 0];
      var block = LinearRailBlock.metric("MGN12C"), rail = LinearRail.metric("MGN12C", 300);
      // The rail's frame is on its running surface; its foot stands on the base.
      var top = 10 + rail.spec.railHeight;
      builder.place("base", new GantryPlate("TEST-STRICT-BASE", "Strict base", 400, 120, 10), AssemblyFrames.translation(150, 0, 0));
      builder.attach("rail", rail, AxisBuilder.orient(0, 0, top, up, along), "base");
      builder.slide(
        {id: "x", lower: 0, upper: 100, initial: 0},
        "rail",
        "block",
        block,
        AxisBuilder.orient(50, 0, top, up, along),
        {x: 1.0, y: 0.0, z: 0.0}
      );
      var wallY = block.spec.blockWidth / 2 + clear + 5;
      builder.attach(
        "wall",
        new GantryPlate("TEST-STRICT-WALL", "Strict wall", 300, 10, 40),
        AssemblyFrames.translation(150, wallY, 10),
        "base"
      );
      return builder;
    }
    expectClean(AssemblyIntegrity.check(rubbing(0)), "a block running along a flush wall, new checks off");
    expectOnly(AssemblyIntegrity.check(rubbing(0), strict), ["integrity.gap"], "a block rubbing a wall");
    expectOnly(
      AssemblyIntegrity.check(rubbing(0.5), strict),
      ["integrity.gap"],
      "a block running 0.5 mm from a wall"
    );
    expectClean(AssemblyIntegrity.check(rubbing(5), strict), "a block running 5 mm from a wall");
    expectClean(
      AssemblyIntegrity.check(rubbing(0.5), {runningGap: 0.25}),
      "a block 0.5 mm from a wall against a 0.25 mm running gap"
    );

    // A part carried by a 20 x 20 tab on a large plate.
    function tabbed(tab:Float):AxisBuilder {
      var builder = new AxisBuilder();
      builder.place("base", new GantryPlate("TEST-STRICT-BASE2", "Strict base", 200, 200, 10), AssemblyFrames.translation(0, 0, 0));
      builder.attach("tab", new GantryPlate("TEST-STRICT-TAB", "Strict tab", tab, tab, 40), AssemblyFrames.translation(0, 0, 10), "base");
      builder.attach("load", new GantryPlate("TEST-STRICT-LOAD", "Strict load", 120, 120, 10), AssemblyFrames.translation(0, 0, 50), "tab");
      return builder;
    }
    expectClean(AssemblyIntegrity.check(tabbed(20)), "a part on a 20 mm tab, new checks off");
    expectOnly(AssemblyIntegrity.check(tabbed(20), strict), ["integrity.contact.area"], "a part on a 20 x 20 mm tab");
    expectClean(AssemblyIntegrity.check(tabbed(60), strict), "a part on a 60 x 60 mm tab");

    // A servo on a plate: cut for its pilot and bolts, drilled for the shaft only, or no plate under its flange.
    var motor = LeadshineServo.model("ELM2H-0400LA60E");
    function motorOn(plate:MachineComponent):AxisBuilder {
      var builder = new AxisBuilder();
      builder.place("plate", plate, AssemblyFrames.translation(0, 0, 0));
      builder.attach("motor", motor, AssemblyFrames.translation(0, 0, 0), "plate");
      return builder;
    }
    var cut = new GantryPlate("TEST-STRICT-MOTOR-PLATE", "Strict motor plate", 100, 100, 8, motor, AssemblyFrames.translation(0, 0, 0));
    expectClean(AssemblyIntegrity.check(motorOn(cut), strict), "a servo on a plate cut for it");
    var shaftOnly = AssemblyIntegrity.check(motorOn(new ShaftHolePlate()), strict);
    check(codes(shaftOnly).indexOf("integrity.mount.cutout") >= 0, 'a plate with a shaft hole only must lack the cut-out, found ${codes(shaftOnly)}');
    var plain = AssemblyIntegrity.check(motorOn(new GantryPlate("TEST-STRICT-UNCUT", "Strict uncut plate", 100, 100, 8)), strict);
    check(codes(plain).indexOf("integrity.mount.cutout") >= 0, 'an uncut plate must lack the cut-out, found ${codes(plain)}');
  }

  /**
   * A shaft turns in a bearing of body A. A cap in A (it carries a journal, too) stands 0.5 mm from the shaft's end: the shaft
   * runs in its own mount, which is fine. A motor fixed in an unrelated body B (it carries a journal, too) stands 1 mm from the
   * shaft's side: that is not the bearing the shaft turns in, so it is still a rubbing contact (`integrity.gap`). The sweep
   * turns the shaft, so the shaft and the parts it stands near move relative to each other.
   */
  static function journals():Void {
    function build(withMotor:Bool):AxisBuilder {
      var builder = new AxisBuilder();
      var up = [0.0, 0, 1], along = [1.0, 0, 0];
      builder.place("base", new GantryPlate("TEST-JOURNAL-BASE", "Journal base", 90, 30, 10), AssemblyFrames.translation(25, 0, 0));
      builder.attach("bearing", new JournalBox("BEARING", 20, 20, 20, true), AssemblyFrames.translation(0, 0, 10), "base");
      builder.hang("shaft", new JournalBox("SHAFT", 10, 10, 40, false), AxisBuilder.orient(10, 0, 20, up, along), "bearing");
      builder.addMateOnAxis("turn", "continuous", "bearing", "to-shaft", "shaft", "attach-shaft", {x: 1, y: 0, z: 0}, 0);
      builder.attach("cap", new JournalBox("CAP", 10, 30, 30, true), AssemblyFrames.translation(55.5, 0, 10), "base");
      if (withMotor) {
        builder.place("mountB", new GantryPlate("TEST-JOURNAL-BASE-B", "Journal base B", 90, 30, 10), AssemblyFrames.translation(25, 35, 0));
        builder.attach("motor", new JournalBox("MOTOR", 30, 34, 30, true), AssemblyFrames.translation(35, 23, 10), "mountB");
      }
      return builder;
    }
    var options:IntegrityOptions = {runningGap: 2.0, sweepStates: [["turn" => Math.PI / 2]]};
    expectClean(AssemblyIntegrity.check(build(false), options), "a shaft turning 0.5 mm from a cap of its own bearing's body");
    expectOnly(
      AssemblyIntegrity.check(build(true), options),
      ["integrity.gap"],
      "a shaft turning 1 mm from a motor fixed in an unrelated body"
    );
  }

  /** A block on a rail meets a stop only at the far end of its travel: the rest state is sound, the sweep is not. */
  static function sweeps():Void {
    function slider(withStop:Bool):AxisBuilder {
      var builder = new AxisBuilder();
      var up = [0.0, 0, 1], along = [1.0, 0, 0];
      builder.place("rail", LinearRail.metric("MGN12C", 300), AxisBuilder.orient(0, 0, 0, up, along));
      builder.slide(
        {id: "x", lower: 0, upper: 100, initial: 0},
        "rail",
        "block",
        LinearRailBlock.metric("MGN12C"),
        AxisBuilder.orient(50, 0, 0, up, along),
        {x: 1.0, y: 0.0, z: 0.0}
      );
      if (withStop) builder.attach(
        "stop",
        new GantryPlate("TEST-INTEGRITY-STOP", "Integrity stop", 20, 30, 20),
        AssemblyFrames.translation(170, 0, 0),
        "rail"
      );
      return builder;
    }
    expectClean(AssemblyIntegrity.check(slider(false)), "a block on a rail");
    expectOnly(
      AssemblyIntegrity.check(slider(true)),
      ["integrity.sweep"],
      "a block that runs into a stop at the end of its travel"
    );
  }

  /** A rail mounted on a plate's seat and a block on the rail: poses, the axis's direction and support all follow from the seats. */
  static function seated():Void {
    var builder = new AxisBuilder();
    var rail = LinearRail.metric("MGN12C", 300);
    // The seat is where the rail's base sits: on the plate's top, +Y (a connector's axis) up, +Z along the rail.
    var plate = new SeatedPlate(
      "TEST-SEATED-PLATE",
      400,
      100,
      10,
      "Integrity base",
      [{name: "rail", holes: [], frame: AxisBuilder.orient(-150, 0, 10, [0.0, 0, 1], [1.0, 0, 0])}]
    );
    builder.place("base", plate, AssemblyFrames.translation(0, 0, 0));
    builder.mount("rail", rail, "base", "base", "rail");
    builder.slideOn({id: "x",
      lower: 0, upper: 200, initial: 50}, "rail", "block", LinearRailBlock.metric("MGN12C"), 50);
    var direction = builder.axisDirection("x");
    check(
      Math.abs(direction[0] - 1) < 1e-12 && Math.abs(direction[1]) < 1e-12 && Math.abs(direction[2]) < 1e-12,
      'the axis runs along the rail, got $direction'
    );
    var model = new AssemblyModel("mm");
    builder.addTo(model, "");
    var state = new AssemblyState(model.definition("seated"));
    function describe(pose:materia.assembly.AssemblyRecord
      .AssemblyFrame):String return '(${pose.x}, ${pose.y}, ${pose.z} | ${pose.qx}, ${pose.qy}, ${pose.qz}, ${pose.qw})';
    for (x in [0.0, 200.0]) {
      state.setJoint("x", x);
      state.forwardKinematics();
      var block = state.worldPose("block"), railPose = state.worldPose("rail");
      // The block rides on the rail's running surface with the rail's orientation.
      check(
        Math.abs(block.x - (-100 + x)) < 1e-9 && Math.abs(block.y) < 1e-9
        && Math.abs(block.z - (10 + rail.spec.railHeight)) < 1e-9 && Math.abs(block.qx - railPose.qx) < 1e-9
        && Math.abs(block.qy - railPose.qy) < 1e-9 && Math.abs(block.qz - railPose.qz) < 1e-9,
        'the block rides the rail: at x = $x the block is at ${describe(block)}, the rail at ${describe(railPose)}'
      );
    }
    expectClean(AssemblyIntegrity.check(builder, {floorZ: 0}), "a block on a rail seated on a plate");
  }

  /**
	 * A carriage on a rail whose motor's pinion rolls on a fixed rack. `driveRack` derives the rack's place and the
	 * coupling's sense: rolling on the rack's pitch line, a pinion turning positively about +Y (its shaft's direction)
	 * carries its centre along +X, by one pitch radius per radian. The same drive with the sense reversed by hand must be
	 * found by the drive check.
	 */
  static function racks():Void {
    function axis(reversed:Bool):AxisBuilder {
      var builder = new AxisBuilder();
      var spec:AxisSpec = {id: "x", lower: 0.0, upper: 100.0, initial: 25.0};
      var along = [1.0, 0, 0], up = [0.0, 0, 1];
      builder.place("rail", LinearRail.metric("MGN12C", 300), AxisBuilder.orient(0, 0, -20, up, along));
      builder.slide(
        spec,
        "rail",
        "carriage",
        LinearRailBlock.metric("MGN12C"),
        AxisBuilder.orient(60, 0, -20, up, along),
        {x: 1.0, y: 0.0, z: 0.0}
      );
      var motor = NemaStepper.frame(17);
      builder.attach(
        "motor",
        motor,
        AxisBuilder.orient(60, 30 - motor.variant.shaftLength, 10, up, [0, 1, 0]),
        "carriage"
      );
      var pinion = new SpurGear(1, 20, 8), rack = new Rack(1, 100, 8);
      var pose = AxisBuilder.orient(60, 30, 10, up, [0, 1, 0]);
      var driver = (motorId:String) -> {builder.attach(
        "driver",
        new MotorDriver(
          "GENERIC-TMC2209",
          new machinekit.power.PhaseRmsCurrent(1.5),
          16,
          new machinekit.power.DcVoltage(24)
        ),
        AssemblyFrames.translation(0, -100, 0),
        "rail"
      );
      return "driver";
      };
      var fixtureTop = 10 - pinion.pitchDiameter / 2 - 1.25 * rack.moduleSize - rack.barHeight;
      builder.place(
        "rackFixture",
        new machinekit.gantry.GantryParts.GantryPlate(
          "TEST-RACK-FIXTURE",
          "anchored rack test fixture",
          rack.length + 2 * Math.PI * rack.moduleSize,
          8,
          10
        ),
        AssemblyFrames.translation(110, 34, fixtureTop - 10)
      );
      var rackMount = new RackMount(
        "rackFixture",
        AxisBuilder.orient(0, 0, 10, up, along),
        "rackFixture",
        [{first: "$rack", second: "rackFixture"}]
      );
      if (!reversed) builder.driveRack(spec, "pinion", "motor", pinion, pose, rackMount, "rack", rack, driver);
      else {
        var placement = builder.rackPlacement(spec, pinion, pose, rackMount, rack);
        builder.attach("rack", rack, placement.pose, "rackFixture");
        builder.hang("pinion", pinion, pose, "motor");
        var ratio = builder.addTransmission(
          "pinion-rack",
          "x",
          "pinion-turn",
          RackAndPinion("pinion", "rack"),
          placement.sense == Same ? Opposite : Same
        );
        builder.addMateOnAxis(
          "pinion-turn",
          "continuous",
          "motor",
          "to-pinion",
          "pinion",
          "attach-pinion",
          {x: 0, y: 1, z: 0},
          ratio * spec.initial
        );
        builder.addMotor("motor", "pinion-turn", "motor", driver("motor"));
      }
      return builder;
    }
    var derived = axis(false);
    var model = new AssemblyModel("mm");
    derived.addTo(model, "");
    var state = new AssemblyState(model.definition("rack"));
    function turnAt(x:Float):Float {
      state.setJoint("x", x);
      state.forwardKinematics();
      return state.joint("pinion-turn");
    }
    function pinionAngle(x:Float):Float {
      state.setJoint("x", x);
      state.forwardKinematics();
      var pose = state.worldPose("pinion");
      // The pinion's local +X, seen along +Y (its shaft): its angle about +Y.
      var axisX = AssemblyFrames.transformVector(pose, 1, 0, 0);
      return Math.atan2(axisX.x, axisX.z);
    }
    // Module 1, 20 teeth: a 10 mm pitch radius.
    var turn = turnAt(40) - turnAt(30);
    check(Math.abs(turn - 1) < 1e-9, 'the pinion turns +1 rad about +Y for 10 mm along +X, got $turn');
    var posed = pinionAngle(40) - pinionAngle(30);
    check(Math.abs(posed - 1) < 1e-9, 'the pinion\'s pose turns with its joint: +1 rad about +Y for 10 mm, got $posed');
    var drives = [for (item in AssemblyIntegrity.check(derived).errors()) if (StringTools.startsWith(
      item.code,
      "integrity.drive"
    )) item];
    check(drives.length == 0, 'the derived rack drive rolls, found: ${[for (item in drives) item.message].join("; ")}');
    var reversed = [for (item in AssemblyIntegrity.check(axis(true)).errors()) item.code];
    check(reversed.indexOf("integrity.drive.slip") >= 0, 'a hand-reversed rack sense is a slip, found $reversed');
  }

  /**
	 * A nut on a slide and a right-hand screw beside it. `screwSense` derives the coupling's sense from the thread and the
	 * slide's direction; the same drive with the sense reversed by hand must be found by the drive check.
	 */
  static function screws():Void {
    function axis(reversed:Bool):AxisBuilder {
      var builder = new AxisBuilder();
      var spec:AxisSpec = {id: "z", lower: 0.0, upper: 50.0, initial: 10.0};
      var up = [0.0, 1, 0], vertical = [0.0, 0, 1];
      builder.place("rail", LinearRail.metric("MGN12C", 200), AxisBuilder.orient(0, 0, 0, up, vertical));
      builder.slide(
        spec,
        "rail",
        "carriage",
        LinearRailBlock.metric("MGN12C"),
        AxisBuilder.orient(0, 0, 60, up, vertical),
        {x: 0.0, y: 0.0, z: 1.0}
      );
      var thread = new LeadScrewThread(MetricTrapezoidal, 8, 2, 4);
      builder.hang("screw", new LeadScrew(thread, 150), AxisBuilder.orient(40, 30, -10, [1.0, 0, 0], vertical), "rail");
      builder.attach(
        "screwNut",
        new LeadScrewNut(thread, 4, false),
        AxisBuilder.orient(40, 30, 55, [1.0, 0, 0], vertical),
        "carriage"
      );
      var sense = builder.screwSense("z", vertical);
      var ratio = builder.addTransmission(
        "screw-lead",
        "z",
        "screw-turn",
        LeadScrew("screw", "screwNut"),
        reversed ? (sense == Same ? Opposite : Same) : sense
      );
      builder.addMateOnAxis(
        "screw-turn",
        "continuous",
        "rail",
        "to-screw",
        "screw",
        "attach-screw",
        {x: 0, y: 0, z: 1},
        ratio * spec.initial
      );
      return builder;
    }
    var derived = [for (item in AssemblyIntegrity.check(axis(false)).errors()) if (StringTools.startsWith(
      item.code,
      "integrity.drive"
    )) item];
    check(
      derived.length == 0,
      'the derived screw sense follows the thread, found: ${[for (item in derived) item.message].join("; ")}'
    );
    var reversed = [for (item in AssemblyIntegrity.check(axis(true)).errors()) item.code];
    check(reversed.indexOf("integrity.drive.slip") >= 0, 'a hand-reversed screw sense is a slip, found $reversed');
  }
}

/** A plate drilled for a motor shaft and nothing else: no pilot bore, no bolt holes. */
class ShaftHolePlate extends MachineComponent {
  public function new() super("TEST-SHAFT-HOLE-PLATE", "Plate with a shaft hole only", "aluminium 6061", true);

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part return Solids.building([], owned -> {
    var block = Part.box(100, 100, 8);
    owned.push(block);
    var hole = Part.cylinderSpan(7, -1, 9);
    owned.push(hole);
    return Solids.cut(block, [hole]);
  });
}

/** A box of steel that may carry a journal (a bearing or a motor), for the journal rule's test. */
class JournalBox extends MachineComponent {
  final w:Float;
  final d:Float;
  final h:Float;
  final journal:Bool;

  public function new(name:String, w:Float, d:Float, h:Float, journal:Bool) {
    super('TEST-JOURNAL-$name-${w}x${d}x$h', 'Journal test box $name', "steel", true);
    this.w = w;
    this.d = d;
    this.h = h;
    this.journal = journal;
  }

  override public function hasGeometry():Bool return true;

  override public function carriesJournal():Bool return journal;

  override public function geometry(detail:ComponentDetail = Preview):Part return Solids.named(Part.box(w, d, h), "box");
}

/** Standalone run. */
function main():Void AssemblyIntegrityTests.run();
