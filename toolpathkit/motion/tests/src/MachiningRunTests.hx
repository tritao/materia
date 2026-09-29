import camkit.CamContour;
import camkit.CamJob;
import haxe.Int64;
import machinekit.assembly.LinearAxis;
import motionkit.event.EventValue;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.robot.MachineKitRobotCompiler;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.SessionState;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationHarness;
import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessEventValue;
import robotkit.world.SimulatedRobot;
import toolpathkit.motion.MachineBinding;
import toolpathkit.motion.MachiningRecipe;
import toolpathkit.motion.MachiningRun;
import toolpathkit.motion.ToolpathMotion;
import toolpathkit.motion.ToolpathMotionBinding;
import toolpathkit.path.Point3;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.setup.Setup;
import toolpathkit.setup.SetupStock;
import toolpathkit.tool.Tool;
import toolpathkit.tool.ToolLibrary;

class MachiningRunTests {
  public static function run():Void {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(
      new LinearAxis(23, 10, 200), new LinearAxis(23, 10, 200),
      new LinearAxis(23, 10, 200), 0.02, 0.08);
    for (channel in ["spindle.speed", "spindle.direction"])
      blueprint.runtime.channels.push(new ProcessChannelDeclaration(channel,
        ProcessEventValue.Analog(0.0)));
    for (channel in ["coolant.mist", "coolant.flood"])
      blueprint.runtime.channels.push(new ProcessChannelDeclaration(channel,
        ProcessEventValue.Digital(false)));
    var simulationHarness = new SimulationHarness(0.01);

    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("cam-pocket", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var binding = new MachineBinding("work", "x", "y", "z", 0.02);
    var robotBinding = new ToolpathMotionBinding(binding, blueprint);
    var contour = new CamContour([
      new Point3(0.004, 0.004, 0.01), new Point3(0.012, 0.004, 0.01),
      new Point3(0.012, 0.012, 0.01), new Point3(0.004, 0.012, 0.01)
    ]);
    var setup = new Setup("1", new Point3(0, 0, 0),
      new SetupStock(0.0, 0.02, 0.0, 0.02, 0.01, 0.008, 0.014));
    var cam = new CamJob(0.014, 12000).pocket(contour,
      new Tool(3, 0.0, 0.002), 0.009, 0.01, 0.001).finish(setup);
    var baseline = ToolpathMotion.lower(cam, binding);
    var controlledOps = cam.ops.copy();
    var firstMove = -1;
    for (index in 0...controlledOps.length) if (firstMove < 0)
      switch controlledOps[index] {
        case ToolpathOp.Move(_, _, _, _, _): firstMove = index;
        case _:
      }
    if (firstMove < 0) throw "CAM pocket has no motion";
    controlledOps.insert(firstMove + 1,
      ToolpathOp.Coolant(true, false, Provenance.cam(90)));
    controlledOps.insert(controlledOps.length - 2,
      ToolpathOp.Coolant(false, false, Provenance.cam(90)));
    var lowered = ToolpathMotion.lower(new ToolpathProgram(controlledOps,
      cam.tools, cam.setups), binding);
    var program:MotionProgram = cast lowered.program;
    if (program == null) throw "CAM pocket did not lower";
    var baselineProgram:MotionProgram = cast baseline.program;
    if (program.ops.length != baselineProgram.ops.length)
      throw "coolant changes added exact stops to the CAM pocket";
    var hasCoolantEvent = false;
    for (op in program.ops) switch op {
      case MotionOp.FollowPath(_, _, _, events):
        for (event in events)
          if (event.channel == "coolant.mist") hasCoolantEvent = true;
      case _:
    }
    if (!hasCoolantEvent) throw "CAM pocket lost coolant path events";
    var target = -1, targetLength = 0.0;
    for (index in 0...program.ops.length) switch program.ops[index] {
      case MotionOp.FollowPath(path, _, _, _):
        if (path.length() > targetLength && path.length() > 0.003) {
          target = index; targetLength = path.length();
        }
      case _:
    }
    if (target < 0) throw "CAM pocket needs a restartable cutting path";
    var motion = new ManipulatorMotion(robot, robotBinding.compiler,
      function(channel) return channel == "spindle.at_speed" ||
        StringTools.startsWith(channel, "cnc.tool_change.") ?
        EventValue.Digital(true) : null,
      function() return runtime.pollEvents());
    var run = new MachiningRun(new MachiningRecipe(0.01), program, motion);
    run.start();
    var tick = 0, heldAt = -1.0;
    for (_ in 0...5000) {
      motion.update(0.01);
      simulationHarness.step(Int64.ofInt(tick++));
      var progress = motion.progress();
      if (progress.op == target && progress.pathDistance > targetLength * 0.25) {
        heldAt = progress.pathDistance;
        run.feedHold();
        break;
      }
      if (!motion.running) break;
    }
    if (heldAt < 0.0) throw 'CAM pocket never reached restart path: ${motion.failure}';
    for (_ in 0...1000) {
      motion.update(0.01);
      simulationHarness.step(Int64.ofInt(tick++));
      if (motion.sessionState() == SessionState.Held) break;
    }
    if (motion.sessionState() != SessionState.Held)
      throw "machining feed hold did not reach rest";
    var continuation = run.prepareRestart(target, heldAt);
    var slicedLength = 0.0;
    for (op in continuation.ops) switch op {
      case MotionOp.FollowPath(path, _, _, _):
        slicedLength = path.length(); break;
      case _:
    }
    if (slicedLength <= 0.0 || slicedLength >= targetLength)
      throw "mid-path restart did not slice the pocket path";
    var settled = false;
    for (_ in 0...1000) {
      motion.update(0.01);
      simulationHarness.step(Int64.ofInt(tick++));
      var snapshot = robot.snapshot();
      if (motion.sessionState() == SessionState.Idle &&
          !snapshot.trajectoryActive && snapshot.trajectoryQueueDepth == 0 &&
          snapshot.safety == 0) {
        settled = true;
        break;
      }
    }
    if (!settled) throw "held pocket did not finish controlled stop";
    run.resumePrepared(continuation);
    for (_ in 0...10000) {
      motion.update(0.01);
      simulationHarness.step(Int64.ofInt(tick++));
      if (!motion.running) break;
    }
    if (!motion.completed || motion.failure != null)
      throw 'restarted CAM pocket failed: ${motion.failure}';
    var position = robot.snapshot().positions;
    var start = new Point3(position.get(0), position.get(1), position.get(2));
    var finish = new Point3(start.x + 0.01, start.y, start.z);
    var faultPath = ToolpathMotion.lower(new ToolpathProgram([
      ToolpathOp.Spindle(Clockwise, 12000, Provenance.cam(91)),
      ToolpathOp.Coolant(true, false, Provenance.cam(91)),
      ToolpathOp.Move(Cut, PathGeometry.Line(start, finish),
        0.01, 0.0, Provenance.cam(91))
    ], new ToolLibrary(), [setup]), binding);
    var faultRun = new MachiningRun(new MachiningRecipe(0.01),
      cast faultPath.program, motion);
    faultRun.start();
    var active = false;
    for (_ in 0...500) {
      motion.update(0.01);
      simulationHarness.step(Int64.ofInt(tick++));
      if (motion.progress().pathDistance > 0.002) {
        active = true;
        break;
      }
      if (!motion.running) break;
    }
    if (!active) throw 'fault trial did not start cutting: ${motion.failure}';
    faultRun.spindleFault();
    if (!motion.session.isStopping())
      throw "spindle fault did not request a controlled stop";
    settled = false;
    for (_ in 0...1000) {
      motion.update(0.01);
      simulationHarness.step(Int64.ofInt(tick++));
      var snapshot = robot.snapshot();
      if (motion.sessionState() == SessionState.Idle &&
          !snapshot.trajectoryActive && snapshot.trajectoryQueueDepth == 0 &&
          snapshot.safety == 0) {
        settled = true;
        break;
      }
    }
    if (!settled) throw "spindle fault did not reach safe rest";
    var shutdown = faultRun.safeShutdown();
    if (shutdown.ops.length != 4)
      throw "spindle fault shutdown needs spindle and coolant off";
    simulationHarness.dispose();
    Sys.println("Machining run tests passed (10 assertions)");
  }
}
