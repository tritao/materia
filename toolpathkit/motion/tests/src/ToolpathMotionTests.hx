import toolpathkit.motion.ToolpathMotion;
import toolpathkit.motion.MachineBinding;
import toolpathkit.motion.ToolpathLowering;
import toolpathkit.motion.ToolpathMotionBinding;
import toolpathkit.path.MoveKind;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.setup.Setup;
import toolpathkit.setup.TravelEnvelope;
import toolpathkit.tool.ToolLibrary;
import motionkit.program.MotionOp;
import motionkit.path.CornerBlender;
import motionkit.path.GeometricPath;
import motionkit.path.PathPoint;
import motionkit.path.ArcSegment;
import machinekit.assembly.LinearAxis;
import motionkit.robot.MachineKitRobotCompiler;
import haxe.Int64;

class ToolpathMotionTests {
  static function acceptsBinding(binding:ToolpathMotionBinding):Void {}
  static function pathProgram(ops:Array<ToolpathOp>):ToolpathProgram
    return new ToolpathProgram(ops, new ToolLibrary(),
      [new Setup("1", new Point3(0, 0, 0))]);

  /**
    A printer's travels run at their feed and round their corners; moves of different speeds join in one rounded path
    whose corner arc goes at the slower of the two; the corners it still stops at are counted by reason.
  **/
  static function printerLowering(origin:Provenance):Void {
    function line(from:Array<Float>, to:Array<Float>, kind:MoveKind, speed:Float, tolerance:Float):ToolpathOp
      return ToolpathOp.Move(kind, PathGeometry.Line(new Point3(from[0], from[1], 0), new Point3(to[0], to[1], 0)), speed, tolerance, origin);
    function primitives(result:toolpathkit.motion.ToolpathLoweringResult):Array<motionkit.path.PosePrimitive> {
      var found:Array<motionkit.path.PosePrimitive> = [];
      var lowered = result.program;
      if (lowered == null) throw "nothing lowered";
      for (op in lowered.ops) switch op {
        case MotionOp.FollowPath(path, _, _, _): for (primitive in path.primitives) found.push(primitive);
        case _:
      }
      return found;
    }
    var printer = new MachineBinding("work", "x", "y", "z", 0.25, new toolpathkit.motion.ToolpathChannels(""));
    printer.setRapidsAtFeed(true);
    // A cut at 40 mm/s, a corner of 90 degrees, a travel at 150 mm/s: one path of line, arc, line.
    var rounded = ToolpathMotion.lower(pathProgram([line([0, 0], [0.02, 0], Cut, 0.04, 0.0001),
      line([0.02, 0], [0.02, 0.02], Rapid, 0.15, 0.0001)]), printer);
    var pieces = primitives(rounded);
    if (pieces.length != 3) throw 'a corner between moves of two speeds is rounded, got ${pieces.length} pieces';
    if (Math.abs(pieces[0].speedLimit() - 0.04) > 1e-12 || Math.abs(pieces[2].speedLimit() - 0.15) > 1e-12)
      throw "each move keeps its own speed";
    if (Math.abs(pieces[1].speedLimit() - 0.04) > 1e-12) throw "the corner arc goes at the slower of the two moves";
    for (_ in rounded.stops) throw "a rounded corner is not a stop";
    // Without the flag a travel is a rapid: the machine's rapid speed, and an exact stop.
    var mill = ToolpathMotion.lower(pathProgram([line([0, 0], [0.02, 0], Cut, 0.04, 0.0001),
      line([0.02, 0], [0.02, 0.02], Rapid, 0.15, 0.0001)]), new MachineBinding("work", "x", "y", "z", 0.25, new toolpathkit.motion.ToolpathChannels("")));
    var millPieces = primitives(mill);
    if (millPieces.length != 2 || Math.abs(millPieces[1].speedLimit() - 0.25) > 1e-12 || mill.stops.get("no blend tolerance") != 1)
      throw "a rapid keeps the machine's rapid speed and stops at its corner";
    // A turn of 160 degrees is above the blend limit; a zero tolerance blends nothing.
    var turn = 160.0 * Math.PI / 180.0;
    var sharp = ToolpathMotion.lower(pathProgram([line([0, 0], [0.02, 0], Cut, 0.04, 0.0001),
      line([0.02, 0], [0.02 + 0.02 * Math.cos(turn), 0.02 * Math.sin(turn)], Cut, 0.04, 0.0001),
      line([0.02 + 0.02 * Math.cos(turn), 0.02 * Math.sin(turn)], [0.0, 0.05], Cut, 0.04, 0.0)]), printer);
    if (sharp.stops.get("turn angle exceeds blend limit") != 1 || sharp.stops.get("no blend tolerance") != 1)
      throw 'stops are counted by reason without the angle in it, got ${[for (reason in sharp.stops.keys()) reason + " " + sharp.stops.get(reason)]}';
  }

  /**
    A printer binding lowers the printing operations: beads drive the extruder along their path (what is laid is what
    is fed, a corner included), the melt rate caps a bead's speed, a retraction is an extruder-only move, a heater
    that is waited for is a barrier, and a heater or fan that is not is an event on the path.
  **/
  static function extruderLowering(origin:Provenance):Void {
    var machine = new MachineBinding("work", "x", "y", "z", 0.25, new toolpathkit.motion.ToolpathChannels(""));
    machine.setRapidsAtFeed(true);
    var printer = new toolpathkit.motion.PrinterBinding("e", 0.00175, 0.00365, 1.0e-8);
    printer.bind(3);
    machine.setPrinter(printer);
    var area = 8.0e-8;
    function line(from:Array<Float>, to:Array<Float>, kind:MoveKind, speed:Float):ToolpathOp
      return ToolpathOp.Move(kind, PathGeometry.Line(new Point3(from[0], from[1], 0), new Point3(to[0], to[1], 0)), speed, 0.0001, origin);
    var result = ToolpathMotion.lower(pathProgram([
      ToolpathOp.Heat("bed", 60.0, true, origin),
      ToolpathOp.Heat("hotend", 200.0, false, origin),
      ToolpathOp.Fan("part", 0.5, origin),
      ToolpathOp.Deposit(area, 4.5e-4, 2.0e-4, origin),
      line([0, 0], [0.02, 0], Cut, 0.2),
      line([0.02, 0], [0.02, 0.02], Cut, 0.2),
      ToolpathOp.DepositOff(origin),
      line([0.02, 0.02], [0.0, 0.02], Rapid, 0.15),
      ToolpathOp.FilamentMove(-0.001, 0.03, origin)]), machine);
    var motion = result.program;
    if (motion == null) throw "a print lowers to a program";
    var kinds:Array<String> = [];
    var drive:Null<motionkit.path.DistanceDrive> = null, capped = 0.0, eventChannels:Array<String> = [];
    for (op in motion.ops) switch op {
      case SetOutput(channel, _): kinds.push("set " + channel);
      case WaitInput(channel, _, _): kinds.push("wait " + channel);
      case MoveJ(JointOffset(joint, delta), options, _):
        kinds.push("offset");
        if (joint != 3 || Math.abs(delta - (-0.001 / 0.00365)) > 1e-9 || Math.abs(options.maxVelocity - 0.03 / 0.00365) > 1e-9)
          throw "a retraction turns the extruder by the filament over the gear radius, at the speed asked";
      case FollowPath(path, _, _, events):
        kinds.push("path");
        drive = path.drive;
        for (primitive in path.primitives) if (primitive.speedLimit() < 0.14) capped = primitive.speedLimit();
        for (event in events) eventChannels.push(event.channel);
      case _:
    }
    if (kinds.join(",") != "set heater.bed.target,wait heater.bed.at_temperature,path,offset" && kinds.join(",") != "set heater.bed.target,wait heater.bed.at_temperature,path,set heater.hotend.target,offset")
      throw 'the print lowers to a barrier for the bed, a path and a retraction, got ${kinds.join(",")}';
    if (drive == null || drive.joint != 3) throw "the beads drive the extruder joint";
    // 0.04 m of bead; the 90 degree corner is rounded and the arc feeds what the trimmed ends would have.
    var expected = 0.04 * area / printer.filamentArea() / 0.00365;
    if (Math.abs(drive.total() - expected) > 1e-9 * expected) throw 'the extruder is fed what the beads lay: ${drive.total()} against $expected';
    if (Math.abs(capped - 1.0e-8 / area) > 1e-12) throw 'the melt rate caps a bead at ${1.0e-8 / area} m/s, got $capped';
    // Each is required: the fan is an event on the path, and the hotend's target is one too or is set before it.
    if (eventChannels.indexOf("fan.part") < 0) throw 'a fan is an event on the path, got $eventChannels';
    if (eventChannels.indexOf("heater.hotend.target") < 0 && kinds.indexOf("set heater.hotend.target") < 0)
      throw 'a heater not waited for is set, not waited for, got ${kinds.join(",")} and $eventChannels';
    // Without a printer binding the printing operations are refused.
    var refused = false;
    try ToolpathMotion.lower(pathProgram([ToolpathOp.Heat("bed", 60.0, true, origin)]), new MachineBinding("work", "x", "y", "z", 0.25, new toolpathkit.motion.ToolpathChannels("")))
    catch (_:Dynamic) refused = true;
    if (!refused) throw "a mill has no heaters";
  }

  static function main():Void {
    PressureAdvanceLimitsTests.main();
    if (Sys.getEnv("TOOLPATH_VIRTUAL_CNC_ONLY") == "1") {
      new ToolpathProcessTests().testVirtualCncProgram();
      Sys.println('Virtual CNC tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    RapidClearanceTests.main();
    acceptsBinding(null);
    var origin = Provenance.cam(7, "face:2");
    var machine = new MachineBinding("work", "x", "y", "z", 0.2, new toolpathkit.motion.ToolpathChannels(""));
    var result = ToolpathMotion.lower(pathProgram([
      ToolpathOp.Spindle(Clockwise, 12000, origin),
      ToolpathOp.Move(Cut,
        PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
        0.01, 0, origin),
      ToolpathOp.Coolant(false, false, origin),
      ToolpathOp.End(origin)
    ]), machine);
    if (result.program == null || result.diagnostics.length != 0)
      throw "lowering failed";
    var pathIndex = -1;
    for (index in 0...result.program.ops.length) switch result.program.ops[index] {
      case MotionOp.FollowPath(_, _, _, _): pathIndex = index;
      case _:
    }
    if (pathIndex < 0 || result.sourceMap.provenanceAt(pathIndex, 0.005) != origin ||
        result.sourceMap.entriesFor(origin).length < 3)
      throw "path provenance lost";
    machine.setTravel(new TravelEnvelope(new Point3(-0.01, -0.01, -0.01),
      new Point3(0.005, 0.01, 0.01)));
    var rejected = false;
    try ToolpathMotion.lower(pathProgram([
      ToolpathOp.Move(Cut,
        PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
        0.01, 0, origin)
    ]), machine) catch (error:Dynamic) {
      rejected = Std.string(error).indexOf("X travel") >= 0;
    }
    if (!rejected) throw "machine travel was not checked";
    machine.setTravel(null);
    // Printing waits for a printer binding; its markers take no motion.
    var markers = ToolpathMotion.lower(pathProgram([ToolpathOp.Layer(0, 0.0002, origin),
      ToolpathOp.Feature(Perimeter, origin), ToolpathOp.PrintObject("a", true, origin),
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)), 0.01, 0, origin)]), machine);
    if (markers.program == null || markers.diagnostics.length != 0) throw "printing markers must not change lowering";
    for (printing in [ToolpathOp.Deposit(8e-8, 4.5e-4, 2e-4, origin), ToolpathOp.DepositOff(origin),
        ToolpathOp.FilamentMove(-0.001, 0.03, origin), ToolpathOp.Heat("hotend", 210, true, origin),
        ToolpathOp.Fan("part", 1.0, origin)]) {
      rejected = false;
      try ToolpathMotion.lower(pathProgram([printing]), machine)
      catch (error:Dynamic) rejected = Std.string(error).indexOf("need a printer binding") >= 0;
      if (!rejected) throw "printing operations must be refused until a printer binding lowers them";
    }
    var probeOps = [ToolpathOp.Move(Cut,
      PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
      0.01, 0.0, origin)];
    function endX(workOrigin:Point3):Float {
      var placed = new ToolpathProgram(probeOps, new ToolLibrary(),
        [new Setup("1", workOrigin)]);
      var lowered = ToolpathMotion.lower(placed, machine);
      var motion:motionkit.program.MotionProgram = cast lowered.program;
      if (motion == null) throw "probed setup produced no motion";
      for (op in motion.ops) switch op {
        case MotionOp.FollowPath(path, _, _, _):
          return path.poseAt(path.length()).x;
        case _:
      }
      throw "probed setup produced no path";
    }
    if (Math.abs(endX(new Point3(0.003, 0, 0)) -
        endX(new Point3(0, 0, 0)) - 0.003) > 1e-9)
      throw "probed origin must move the lowered path by the probe difference";
    var blueprint = MachineKitRobotCompiler.compileGantry(
      new machinekit.gantry.Gantry(new machinekit.gantry.GantrySpec(200, 200, 200)),
      0.1, 0.4);
    var robotBinding = new ToolpathMotionBinding(
      new MachineBinding("work", "x", "y", "z", 0.08, new toolpathkit.motion.ToolpathChannels("")), blueprint);
    var compiled = robotBinding.compile(pathProgram([
      ToolpathOp.Move(Cut,
        PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
        0.01, 0, Provenance.cam(8))
    ]), [for (_ in blueprint.model.joints) 0.0], Int64.ofInt(1));
    if (compiled.blocks.length == 0) throw "robot binding produced no plans";
    compiled.dispose();
    // A rapid down that carries straight on as a slower plunge is one motion; the exact
    // corner after it is where the machine stops.
    var entryOps = [
      ToolpathOp.Move(Rapid, PathGeometry.Line(new Point3(0, 0, 0.012), new Point3(0, 0, 0.006)),
        0.0, 0.0, Provenance.cam(9)),
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0, 0, 0.006), new Point3(0, 0, 0.002)),
        0.002, 0.0, Provenance.cam(10)),
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0, 0, 0.002), new Point3(0.01, 0, 0.002)),
        0.01, 0.0, Provenance.cam(11))
    ];
    var entry:motionkit.program.MotionProgram = cast ToolpathMotion.lower(pathProgram(entryOps),
      robotBinding.machine).program;
    if (entry == null || entry.ops.length != 1) throw "moves without a barrier must lower to one path";
    switch entry.ops[0] {
      case MotionOp.FollowPath(path, _, feed, _):
        var speeds = [for (primitive in path.primitives) primitive.speedLimit()];
        if (speeds.join(",") != "0.08,0.002,0.01" || feed != 0.08)
          throw 'each move keeps its own speed, got $speeds under $feed';
      case _: throw "entry path missing";
    }
    var entryStart = [for (_ in blueprint.model.joints) 0.0];
    var zAxis = blueprint.axes[2];
    for (slot in 0...zAxis.jointIds.length)
      entryStart[[for (joint in blueprint.model.joints) joint.id].indexOf(zAxis.jointIds[slot])] =
        zAxis.jointScales[slot] * 0.012;
    var entryPlans = robotBinding.compile(pathProgram(entryOps), entryStart, Int64.ofInt(1));
    var planCount = 0;
    for (block in entryPlans.blocks) planCount += block.plans.length;
    entryPlans.dispose();
    if (planCount != 2) throw 'the plunge must follow the rapid without stopping, got $planCount plans';
    var corners = CornerBlender.blendPerCorner(GeometricPath.lines([
      new PathPoint(0.0, 0.0), new PathPoint(0.01, 0.0),
      new PathPoint(0.01, 0.01), new PathPoint(0.02, 0.01)
    ]), [0.0001, 0.0005], Math.PI * 5.0 / 6.0);
    if (corners.path.primitives.length != 5)
      throw "both per-corner tolerances must blend";
    var first:ArcSegment = cast corners.path.primitives[1];
    var second:ArcSegment = cast corners.path.primitives[3];
    if (first.radius >= second.radius)
      throw "smaller corner tolerance must create a tighter fillet";
    var eventResult = ToolpathMotion.lower(pathProgram([
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0, 0, 0),
        new Point3(0.01, 0, 0)), 0.01, 0.0002, origin),
      ToolpathOp.Coolant(true, false, origin),
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0.01, 0, 0),
        new Point3(0.01, 0.01, 0)), 0.01, 0.0004, origin)
    ]), new MachineBinding("work", "x", "y", "z", 0.2, new toolpathkit.motion.ToolpathChannels("")));
    var eventProgram:motionkit.program.MotionProgram = cast eventResult.program;
    if (eventProgram == null || eventProgram.ops.length != 1)
      throw "coolant added a motion stop";
    switch eventProgram.ops[0] {
      case MotionOp.FollowPath(_, _, _, events):
        if (events.length != 2 || events[0].channel != "coolant.mist" ||
            events[0].distance <= 0.0)
          throw "coolant must be a position-tied path event";
      case _: throw "blended path missing";
    }
    for (scope in ["first", "second"]) {
      var channels = new toolpathkit.motion.ToolpathChannels(scope);
      var lowered = ToolpathMotion.lower(pathProgram([
        ToolpathOp.Spindle(Clockwise, 3000, origin)
      ]), new MachineBinding("work", "x", "y", "z", 0.2, channels));
      var program = lowered.program;
      if (program == null || program.ops.length != 3) throw "Scoped spindle did not lower";
      switch program.ops[0] {
        case SetOutput(channel, _) if (channel == scope + "/spindle.direction"):
        case _: throw "Spindle direction escaped its machine scope";
      }
      switch program.ops[1] {
        case SetOutput(channel, _) if (channel == scope + "/spindle.speed"):
        case _: throw "Spindle speed escaped its machine scope";
      }
      switch program.ops[2] {
        case WaitInput(channel, _, _) if (channel == scope + "/spindle.at_speed"):
        case _: throw "Spindle readiness escaped its machine scope";
      }
    }
    printerLowering(origin);
    extruderLowering(origin);
    Sys.println("ToolpathKit Motion tests passed (18 assertions)");
    Sys.println('Toolpath accuracy tests passed (${ToolpathAccuracyTests.run()} assertions)');
    MachiningRunTests.run();
    ToolpathScenarioTests.main();
  }
}
