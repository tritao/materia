import haxe.Int64;
import haxe.io.Bytes;
import toolpathkit.tool.Tool;
import cnckit.CncCompiler;
import machinekit.assembly.LinearAxis;
import cadkit.modeling.AssemblyModel;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import materia.assembly.AssemblyFrames;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import motionkit.AxisTarget;
import motionkit.Feed;
import motionkit.MotionOptions;
import motionkit.Pose;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.event.ChannelDeclaration;
import motionkit.event.ChannelKind;
import motionkit.event.EventValue;
import motionkit.event.HoldPolicy;
import motionkit.event.PathEvent;
import motionkit.event.TimedEvent;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import motionkit.robot.ManipulatorKinematics;
import motionkit.robot.OpwKinematics;
import motionkit.robot.AxisKinematics;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.StartTolerances;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.MachineKitRobotCompiler;
import motionkit.robot.MotionSystem;
import motionkit.robot.SessionState;
import motionkit.robot.StopDisposition;
import motionkit.robot.MotionSystemBlueprint;
import motionkit.path.ArcSegment;
import motionkit.path.CornerBlender;
import motionkit.path.GeometricPath;
import motionkit.path.LineSegment;
import motionkit.path.PathPoint;
import motionkit.path.NativeJointPath;
import motionkit.path.PathTimeLaw;
import motionkit.path.PathTimeLaw.PathTimeStage;
import motionkit.path.PosePath;
import motionkit.path.PoseLine;
import motionkit.path.PoseArc;
import motionkit.path.PoseWaypoint;
import motionkit.path.OrientationPolicy;
import processkit.motion.ToolpathPosePath;
import processkit.path.Toolpath;
import processkit.path.ToolpathPoint;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import motionkit.planner.PathPlanningOptions;
import motionkit.planner.JointPathSamples;
import motionkit.planner.PathTimingBackend;
import motionkit.planner.PathTimingLimits;
import motionkit.planner.SimplePathTiming;
import motionkit.planner.ToppraPathTiming;
import motionkit.planner.BindingConstraint.BindingConstraintKind;
import motionkit.program.Blend;
import motionkit.program.InputPredicate;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.trajectory.MotionLimits;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.ExecutionPlan;
import trajectorykit.validation.ValidationGuarantee;
import motionkit.trajectory.PlanLimitError;
import motionkit.trajectory.ValidationLimits;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.Frame;
import robotkit.model.RobotModel;
import robotkit.model.Actuator;
import robotkit.model.Transmission;
import robotkit.model.JointCoupling;
import robotkit.manipulation.Manipulator;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.Simulation;
import robotkit.runtime.VirtualDeviceOptions;
import robotkit.runtime.VirtualActuatorOptions;
import robotkit.runtime.RobotRuntimeError;
import robotkit.runtime.RobotRuntimeCompiler;
import RobotKitRuntime;
import robotkit.recording.RecordingRobot;
import robotkit.recording.ReplayRobot;
import robotkit.recording.RobotRecording;
import robotkit.simulation.SimulatedRobot;
import robotkit.core.RobotCommand;
import robotkit.core.Robot;
import robotkit.core.RobotCapabilities;
import robotkit.core.RobotDescription;
import robotkit.core.RobotFault;
import robotkit.core.RobotId;
import robotkit.core.RobotSnapshot;
import robotkit.core.RobotStatus;
import robotkit.runtime.RuntimeRobotAdapter;
import robotkit.core.SensorFrame;
import robotkit.core.StopMode;
import robotkit.execution.ExecutionPlanSubmission;
import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;
import robotkit.execution.TrajectorySegment;

import MotionKitTestSupport.WristBranchSolver;
import MotionKitTestSupport.PlanarSolver;
import MotionKitTestSupport.SessionTransitionRig;
import MotionKitTestSupport.LaggingRobot;
import MotionKitTestSupport.TrialRig;

class ToolpathProcessTests extends ToolpathTestSupport {
  public function testPhysicalAssemblyCncBinding():Void {
    var gantry = new machinekit.gantry.Gantry(new machinekit.gantry.GantrySpec(80, 80, 80));
    var assembly = new AssemblyModel();
    gantry.addTo(assembly, "");
    var definition = assembly.definition("physical-gantry");
    var vertices = Bytes.alloc(4 * 24);
    var support = [0.0, 0.0, 0.0, 10.0, 0.0, 0.0,
      0.0, 10.0, 0.0, 0.0, 0.0, 10.0];
    for (index in 0...support.length) vertices.setDouble(index * 8, support[index]);
    var parts = AssemblyPhysicalPartView.fromSceneArtifact({metresPerUnit: 0.001, parts: [
      for (component in definition.definitions) {
        id: component.id, name: component.id, red: 0.5, green: 0.5, blue: 0.5,
        materialId: "steel", materialDensity: 7850.0, volume: 1000.0,
        centerOfMass: [0.0, 0.0, 0.0],
        inertia: [1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0],
        vertexCount: 4, indexCount: 0, vertices: vertices,
        normals: Bytes.alloc(0), indices: Bytes.alloc(0), faceRanges: []
      }
    ]});
    var physical = AssemblySimulationBridge.toRobotModel(definition, parts);
    var blueprint = MachineKitRobotCompiler.compileGantry(gantry, 0.1, 0.4);
    check(blueprint.model.joints.length == blueprint.model.couplings.length + 3 && blueprint.model.actuators.length == 4,
      "physical gantry includes three leaders and four authored motors");
    var gantryCouplings = gantry.definition().couplings;
    if (gantryCouplings == null) throw "Gantry definition has no drive couplings";
    check(blueprint.model.couplings.length == gantryCouplings.length,
      "physical gantry keeps every part-derived drive coupling");
    var hasTenMillimetreVertex = false, linksValid = physical.linkHulls.length > 0;
    for (hull in physical.linkHulls) {
      if (hull.link < 0 || hull.link >= blueprint.model.links.length) linksValid = false;
      for (value in hull.vertices)
        if (Math.abs(value - 0.01) < 1e-12) hasTenMillimetreVertex = true;
    }
    check(linksValid && hasTenMillimetreVertex,
      "physical assembly passes upstream hulls on their links in SI units");
    var cnc = new MotionCncRig("work", "x", "y", "z", 0.08);
    var result = ToolpathTestSupport.compileCnc(ToolpathTestSupport.cncBinding(cnc, blueprint), cnc,
      "G21 G90 G17\nS12000 M3\nG0 X10 Y10\nF600 G1 X20\nG3 X10 Y20 I-10 J0\nM5\nM2\n",
      [for (_ in blueprint.model.joints) 0.0], Int64.ofInt(990));
    check(result.blocks.length > 0, "physical gantry CNC compiles through ProgramCompiler");
    var plans = result.blocks[result.blocks.length - 1].plans;
    var end = plans[plans.length - 1].evaluate(plans[plans.length - 1].durationSeconds).positions;
    var kinematics = new AxisKinematics(blueprint);
    near(kinematics.forward(end).x, 0.01, "physical gantry arc ends at X", 1e-5);
    near(kinematics.forward(end).y, 0.02, "physical gantry arc ends at Y", 1e-5);
    var endState = plans[plans.length - 1].evaluate(plans[plans.length - 1].durationSeconds);
    var endSpeed = 0.0;
    for (speed in endState.velocities) endSpeed = Math.max(endSpeed, Math.abs(speed));
    check(endSpeed <= 1e-6, 'physical gantry ends at rest (speed=$endSpeed)');
    var lastSegments = plans[plans.length - 1].segments();
    var terminal = lastSegments[lastSegments.length - 1];
    var terminalSeconds = Int64.toFloat(terminal.durationNs) * 1e-9;
    var terminalSpeed = 0.0;
    for (coefficients in terminal.coefficients) {
      var speed = 0.0;
      for (degree in 1...coefficients.length)
        speed += degree * coefficients[degree] * Math.pow(terminalSeconds, degree - 1);
      terminalSpeed = Math.max(terminalSpeed, Math.abs(speed));
    }
    check(terminalSpeed <= 1e-6,
      'physical gantry terminal polynomial ends at rest (speed=$terminalSpeed)');
    var maximumCouplingResidual = 0.0;
    for (block in result.blocks) for (plan in block.plans) for (segment in plan.segments())
      for (coupling in blueprint.model.couplings) {
        var leader = -1, follower = -1;
        for (joint in 0...blueprint.model.joints.length) {
          if (blueprint.model.joints[joint].id == coupling.leader) leader = joint;
          if (blueprint.model.joints[joint].id == coupling.follower) follower = joint;
        }
        for (degree in 0...segment.coefficients[leader].length)
          maximumCouplingResidual = Math.max(maximumCouplingResidual,
            Math.abs(segment.coefficients[follower][degree] -
              coupling.ratio * segment.coefficients[leader][degree] -
              (degree == 0 ? coupling.offset : 0.0)));
      }
    check(maximumCouplingResidual <= 1e-6,
      'physical gantry path preserves coupling polynomial (residual=$maximumCouplingResidual)');
    for (block in result.blocks) for (plan in block.plans) {
      var segments = plan.segments();
      for (first in [0, 75, 150, 225]) if (first < segments.length) {
        var state = plan.evaluate(Int64.toFloat(segments[first].timeFromStartNs) * 1e-9);
        for (coupling in blueprint.model.couplings) {
          var leader = -1, follower = -1;
          for (joint in 0...blueprint.model.joints.length) {
            if (blueprint.model.joints[joint].id == coupling.leader) leader = joint;
            if (blueprint.model.joints[joint].id == coupling.follower) follower = joint;
          }
          check(Math.abs(state.accelerations[follower] -
            coupling.ratio * state.accelerations[leader]) < 1e-6,
            'physical gantry chunk acceleration follows coupling at $first');
        }
      }
    }
    result.dispose();
    for (channel in ["spindle.speed", "spindle.direction"])
      blueprint.runtime.channels.push(new ProcessChannelDeclaration(channel,
        ProcessEventValue.Analog(0.0)));
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobotAtPose(blueprint.runtime,
      [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], null, null, null, null, null,
      null, null, null, [for (hull in physical.linkHulls)
        {link: hull.link, vertices: hull.vertices}]);
    var robot = new SimulatedRobot("physical-cnc", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    homeGantryFixture(blueprint, runtime, simulationHarness, robot);
    var state = runtime.snapshot();
    var binding = new toolpathkit.motion.ToolpathMotionBinding(cnc.binding, blueprint,
      state.modelRevision, state.calibrationRevision);
    var motion = new ManipulatorMotion(robot, binding.compiler,
      function(channel) return channel == "spindle.at_speed" ?
        EventValue.Digital(true) : null,
      function() return runtime.pollEvents());
    motion.run(ToolpathTestSupport.cncProgram(cnc,
      "G21 G90 G17\nS12000 M3\nG0 X10 Y10\nF600 G1 X20\nG3 X10 Y20 I-10 J0\nM5\nM2\n"));
    for (tick in 0...3000) {
      motion.update(0.01);
      simulationHarness.step(Int64.ofInt(tick));
      if (!motion.running) break;
    }
    check(motion.completed, 'physical assembly CNC completes: ${motion.failure}');
    var finalPose = kinematics.forward(robot.snapshot().positions.toArray());
    near(finalPose.x, 0.01, "physical assembly CNC executes X", 2e-4);
    near(finalPose.y, 0.02, "physical assembly CNC executes Y", 2e-4);
    simulationHarness.dispose();
  }

  public function testCncProgramBinding():Void {
    var blueprint = MachineKitRobotCompiler.compileGantry(
      new machinekit.gantry.Gantry(new machinekit.gantry.GantrySpec(200, 200, 200)),
      0.1, 0.4);
    var cnc = new MotionCncRig("work", "x", "y", "z", 0.08);
    var binding = ToolpathTestSupport.cncBinding(cnc, blueprint);
    check(cnc.binding.travel != null,
      "CNC binding derives a machine travel envelope");
    var travelError = "";
    try ToolpathTestSupport.compileCnc(binding, cnc, "G21 G0 X500\nM2\n", [for (_ in blueprint.model.joints) 0.0],
      Int64.ofInt(899))
    catch (error:Dynamic) travelError = Std.string(error);
    check(travelError.indexOf("G-code line 1") >= 0 &&
      travelError.indexOf("X travel") >= 0,
      'bound CNC travel error names the G-code line and axis: $travelError');
    var result = ToolpathTestSupport.compileCnc(binding, cnc, "G21 G90 G17\nS12000 M3\nG0 X10 Y10\n" +
      "F600 G1 X20\nG3 X10 Y20 I-10 J0\nM5\nM2\n",
      [for (_ in blueprint.model.joints) 0.0], Int64.ofInt(900));
    check(result.blocks.length > 0, "CNC ProgramCompiler emits execution blocks");
    var last = result.blocks[result.blocks.length - 1].plans;
    check(last.length > 0, "CNC arc is lowered into an execution plan");
    var plan = last[last.length - 1];
    var end = plan.evaluate(plan.durationSeconds).positions;
    near(end[0], 0.01, "CNC arc ends at X", 1e-5);
    near(end[1], 0.02, "CNC arc ends at Y", 1e-5);
    result.dispose();
    cnc.controller.toolLibrary.set(new Tool(2, 0.0, 0.002));
    var compensated = ToolpathTestSupport.compileCnc(binding, cnc, "G21 G90 F600 G41 D2 G1 X10\n" +
      "G1 X20\nG1 X20 Y10\nG40 G1 X20 Y20\nM2\n",
      [for (_ in blueprint.model.joints) 0.0], Int64.ofInt(925));
    check(compensated.blocks.length > 0,
      "compensated contour lowers through MotionKit");
    compensated.dispose();
    for (arc in ["G17 G2 X5 Y5 Z5 I5 J0",
        "G18 G3 X5 Y5 Z5 I5 K0", "G19 G2 X5 Y5 Z5 J5 K0"]) {
      var helix = ToolpathTestSupport.compileCnc(binding, cnc, 'G21 G90 F600 $arc\nM2\n',
        [for (_ in blueprint.model.joints) 0.0], Int64.ofInt(950));
      var block = helix.blocks[helix.blocks.length - 1];
      var finalPlan = block.plans[block.plans.length - 1];
      var finalPose = binding.solver.forward(
        finalPlan.evaluate(finalPlan.durationSeconds).positions);
      near(finalPose.x, 0.005, '$arc ends at X', 1e-5);
      near(finalPose.y, 0.005, '$arc ends at Y', 1e-5);
      near(finalPose.z, 0.005, '$arc ends at Z', 1e-5);
      helix.dispose();
    }
  }

  public function testVirtualCncProgram():Void {
    var deterministic = cncTrial(false);
    var repeated = cncTrial(false);
    check(deterministic.length == repeated.length,
      "CNC simulated run has deterministic sample count");
    for (index in 0...deterministic.length)
      near(deterministic[index], repeated[index],
        'CNC deterministic sample $index', 1e-9);
    var device = cncTrial(true);
    check(device.length > 0, "CNC program runs through virtual steppers");
    var disconnected = cncTrial(true, true);
    check(disconnected.length > 0, "CNC link-loss trial recorded device positions");
  }

}
