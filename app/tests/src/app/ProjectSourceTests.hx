package app;

import haxeon.ui.Path;

import RobotKitRuntime;
import haxeon.ui.FontCollection;
import haxeon.ui.LayoutFrame;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiContext;
import haxeon.ui.theme.Theme;
import app.MateriaProjectRunner;
import app.Main.ReferenceEditorApp;
import app.ProjectDocumentSession;
import RobotArm;
import BenchMill;
import BenchMillPreview;
import TendingControllers;
import TendingCell.TendingMissionChecks;
import machinekit.robotics.DualArmGripperTool;
import machinekit.robotics.ArmGripperTool;
import machinekit.robot.RobotScene;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import machinekit.pneumatic.AirSupply;
import app.MatePickTool;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
import app.ApplicationSimulation;
import robotkit.world.RobotWorld;
import motionkit.robot.JointMotionPolicy;
import robotkit.core.Robot;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalData;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyMateKind;
import haxeon.ui.properties.PropertyBinding;
import haxeon.ui.properties.PropertyValue;
import haxeon.ui.properties.PropertyEditResult;
import haxe.Json;
import haxe.Int64;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.SimulationSpace;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.model.RobotModel;
import robotkit.model.Actuator;
import robotkit.model.Transmission;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import robotkit.model.JointCoupling;
import robotkit.model.SteadyLoads;
import robotkit.execution.TrajectorySegment;
import robotkit.execution.ProcessTimedEvent;
import robotkit.execution.ProcessEventValue;
import motionkit.event.EventValue;
import robotkit.execution.ExecutionPlanSubmission;
import robotkit.device.DeviceBinding;
import robotkit.device.DeviceLayout;
import motionkit.trajectory.PlanDiagnostic;
import haxeon.platform.NativeKitRuntime;
import processkit.moonraker.MoonrakerStatus;
import processkit.testing.FakeMoonrakerServer;
import machinekit.printing.KlipperConfig;
import sys.FileSystem;
import sys.io.File;
import haxe.io.Bytes;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactMissionStep;
import cnckit.controller.RobotInterfaceAlarm;
import cnckit.controller.RobotInterfaceChannels;

/** Save and reopen a generated project without persisting its mesh buffers. */
@:access(app.MissionPlayer)
@:access(motionkit.robot.ManipulatorMotion)
@:access(motionkit.robot.PlanExecutor)
@:access(processkit.WeldingPlanRunner)
@:access(app.ApplicationSimulation)
@:access(app.Main.ReferenceEditorApp)
@:access(app.CellSignals)
@:access(robotkit.runtime.RuntimeRobotAdapter)
class ProjectSourceTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
  }

  static function checkTendingAlarmClassification():Void {
    function timeoutAlarm(channel:String):MissionAlarmCode {
      var step:SceneArtifactMissionStep = {kind: "waitFor", waitFor: {name: channel, value: true, timeout: 0.1}};
      return MissionAlarmClassifier.classify(step, "controller input timed out", false);
    }
    var pick:SceneArtifactMissionStep = {kind: "pick"};
    var place:SceneArtifactMissionStep = {kind: "place"};
    check(MissionAlarmClassifier.classify(pick, "Joint path selection failed", false) == MissionAlarmCode.MotionFailure &&
      MissionAlarmClassifier.classify(place, "Joint path selection failed", false) == MissionAlarmCode.MotionFailure,
      "planning failures are motion alarms even during a pick or a loaded carry");
    check(MissionAlarmClassifier.classify(pick, "the tool did not confirm a grasp", false) == MissionAlarmCode.MissedGrasp,
      "unconfirmed grasp feedback retains the missed-grasp alarm");
    check(timeoutAlarm(RobotInterfaceChannels.DOOR_OPEN) == MissionAlarmCode.DoorJam &&
      timeoutAlarm(RobotInterfaceChannels.DOOR_CLOSED) == MissionAlarmCode.DoorJam,
      "door switch timeouts map to the stable door-jam alarm");
    check(timeoutAlarm(RobotInterfaceChannels.CLAMPED) == MissionAlarmCode.ViseFailure &&
      timeoutAlarm(RobotInterfaceChannels.PART_PRESENT) == MissionAlarmCode.ClampWithoutPart,
      "vise-actuator and stock-presence timeouts map to distinct stable alarms");
    check(timeoutAlarm(RobotInterfaceChannels.CNC_READY) == MissionAlarmCode.CncAlarm &&
      timeoutAlarm(RobotInterfaceChannels.IN_CYCLE) == MissionAlarmCode.CncAlarm &&
      timeoutAlarm(RobotInterfaceChannels.CYCLE_COMPLETE) == MissionAlarmCode.CncAlarm &&
      timeoutAlarm(RobotInterfaceChannels.ALARM) == MissionAlarmCode.CncAlarm,
      "cycle and CNC alarm timeouts map to the stable CNC alarm");
    check(MissionAlarmClassifier.fromInterfaceAlarm(RobotInterfaceAlarm.DoorOpenTimeout, true) == MissionAlarmCode.DoorJam &&
      MissionAlarmClassifier.fromInterfaceAlarm(RobotInterfaceAlarm.DoorCloseTimeout, true) == MissionAlarmCode.DoorJam &&
      MissionAlarmClassifier.fromInterfaceAlarm(RobotInterfaceAlarm.DoorOpenedDuringCycle, true) == MissionAlarmCode.DoorJam,
      "physical door interlock alarm codes map to the stable door-jam alarm");
    check(MissionAlarmClassifier.fromInterfaceAlarm(RobotInterfaceAlarm.ClampInterlock, false) == MissionAlarmCode.ClampWithoutPart &&
      MissionAlarmClassifier.fromInterfaceAlarm(RobotInterfaceAlarm.ClampTimeout, false) == MissionAlarmCode.ClampWithoutPart &&
      MissionAlarmClassifier.fromInterfaceAlarm(RobotInterfaceAlarm.ClampTimeout, true) == MissionAlarmCode.ViseFailure &&
      MissionAlarmClassifier.fromInterfaceAlarm(RobotInterfaceAlarm.UnclampTimeout, true) == MissionAlarmCode.ViseFailure,
      "physical vise alarm codes distinguish missing stock from actuator faults");
    check(MissionAlarmClassifier.fromInterfaceAlarm(RobotInterfaceAlarm.CycleInterlock, false) == MissionAlarmCode.CncAlarm &&
      MissionAlarmClassifier.fromInterfaceAlarm(RobotInterfaceAlarm.DoorOpenInterlock, false) == MissionAlarmCode.CncAlarm &&
      MissionAlarmClassifier.fromInterfaceAlarm(RobotInterfaceAlarm.ToolChangeRequired, false) == MissionAlarmCode.CncAlarm,
      "cycle and tool-change alarms map to the stable CNC alarm");
  }

  static function singleHandlingChannel(mission:MissionPlayer):String {
    var channels:Array<String> = [];
    for (channel in mission.handlingTools.keys()) channels.push(channel);
    if (channels.length != 1) throw 'Expected one handling tool, found ${channels.length}';
    return channels[0];
  }

  static function handlingRunner(mission:MissionPlayer, channel:String):motionkit.robot.HandlingPlanRunner {
    var runner = mission.handlingRunners.get(channel);
    if (runner == null) throw 'Handling runner "$channel" is unavailable';
    return runner;
  }

  static function checkCoupling(backend:Int):Void {
    var model = new RobotModel("coupled assembly probe");
    var base = model.addLink(new Link("base"));
    var sourceLink = model.addLink(new Link("source"));
    var targetLink = model.addLink(new Link("target"));
    var source = model.addJoint(new Joint("source", JointType.Revolute, base, sourceLink));
    var target = model.addJoint(new Joint("target", JointType.Revolute, base, targetLink));
    source.limits = new JointLimits(-2, 2);
    target.limits = new JointLimits(-2, 2);
    model.addCoupling(new JointCoupling("gears", source.id, target.id, -2.0, 0.0));
    var simulationHarness = new SimulationHarness(0.01, 1, backend);

    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()));
    runtime.submitPosition(0, 0.3, 1);
    for (index in 0...200) simulationHarness.step(Int64.ofInt(index));
    var q = runtime.snapshot().q;
    check(Math.abs(q.get(0)) > 0.1 &&
      Math.abs(q.get(1) + 2.0 * q.get(0)) < (backend == 0 ? 1e-9 : 0.02),
      'joint coupling tracks on backend $backend: ${q.get(0)}, ${q.get(1)}');
    simulationHarness.dispose();
  }

  static function checkLinkCollision(backend:Int, enabled:Bool):Void {
    var model = new RobotModel("generated part collision probe");
    model.addLink(new Link("part"));
    model.collisionApproximation = robotkit.model.CollisionApproximation.None;
    var simulationHarness = new SimulationHarness(0.01, 1, backend);

    var simulation = simulationHarness.simulation;
    simulation.addRobotAtPose(RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()), [0.0, 0.0, 0.0],
      [0.0, 0.0, 0.0, 1.0], null, [enabled ? [0.5, 0.5, 0.5] : null]);
    var box = simulationHarness.spawnBox([0.0, 0.0, 1.25], [0.1, 0.1, 0.1], true, 1.0);
    for (index in 0...200) simulationHarness.step(Int64.ofInt(index));
    var height = simulationHarness.objectPose(box).position[2];
    check(enabled ? height > 0.5 : (backend == 1 ? height < 0.25 : height < -1.0),
      'link collision ${enabled ? "on" : "off"} on backend $backend: $height');
    simulationHarness.dispose();
  }

  static function checkHullCollision(backend:Int, enabled:Bool):Void {
    var model = new RobotModel("convex link collision probe");
    model.addLink(new Link("part"));
    model.collisionApproximation = robotkit.model.CollisionApproximation.None;
    var vertices:Array<Float> = [];
    for (index in 0...8) {
      vertices.push((index & 1) == 0 ? -0.5 : 0.5);
      vertices.push((index & 2) == 0 ? -0.5 : 0.5);
      vertices.push((index & 4) == 0 ? -0.5 : 0.5);
    }
    if (enabled) model.links[0].collisionShapes.push(new robotkit.model.CollisionShape(collisionkit.CollisionGeometry.Convex(vertices)));
    var simulationHarness = new SimulationHarness(0.01, 1, backend);

    var simulation = simulationHarness.simulation;
    simulation.addRobotAtPose(RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()), [0.0, 0.0, 0.0],
      [0.0, 0.0, 0.0, 1.0]);
    var box = simulationHarness.spawnBox([0.0, 0.0, 1.25], [0.1, 0.1, 0.1], true, 1.0);
    for (index in 0...200) simulationHarness.step(Int64.ofInt(index));
    var height = simulationHarness.objectPose(box).position[2];
    check(enabled ? height > 0.5 : (backend == 1 ? height < 0.25 : height < -1.0),
      'hull link collision ${enabled ? "on" : "off"} on backend $backend: $height');
    simulationHarness.dispose();
  }

  static function checkWedgeContact():Void {
    var model = new RobotModel("wedge hull probe");
    model.addLink(new Link("wedge"));
    model.collisionApproximation = robotkit.model.CollisionApproximation.None;
    var wedge = [-0.5, -0.5, -0.5, 0.5, -0.5, -0.5,
      -0.5, -0.5, 0.5, -0.5, 0.5, -0.5,
      0.5, 0.5, -0.5, -0.5, 0.5, 0.5];
    model.links[0].collisionShapes.push(new robotkit.model.CollisionShape(collisionkit.CollisionGeometry.Convex(wedge)));
    for (backend in [ApplicationSimulation.DETERMINISTIC, ApplicationSimulation.MUJOCO]) {
      var simulationHarness = new SimulationHarness(0.01, 1, backend);

      var simulation = simulationHarness.simulation;
      simulation.addRobotAtPose(RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()), [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]);
      var box = simulationHarness.spawnBox([0.35, 0.0, 1.25], [0.1, 0.1, 0.1], true, 1.0);
      for (index in 0...200) simulationHarness.step(Int64.ofInt(index));
      var height = simulationHarness.objectPose(box).position[2];
      check(backend == ApplicationSimulation.DETERMINISTIC ? height > 0.5 : height < 0.5,
        'wedge contact differs from its bounding box on backend $backend: $height');
      simulationHarness.dispose();
    }
  }

  static function checkMachinePartHullCollision(vertices:Array<Float>, scale:Float):Void {
    check(vertices != null && vertices.length >= 12 && vertices.length % 3 == 0,
      "MachineKit part provides a collision hull");
    var count = Std.int(vertices.length / 3);
    var centerX = 0.0, centerY = 0.0, bottom = Math.POSITIVE_INFINITY;
    for (index in 0...count) {
      centerX += vertices[index * 3];
      centerY += vertices[index * 3 + 1];
      bottom = Math.min(bottom, vertices[index * 3 + 2]);
    }
    centerX /= count;
    centerY /= count;
    var hull:Array<Float> = [];
    var top = Math.NEGATIVE_INFINITY;
    for (index in 0...count) {
      hull.push((vertices[index * 3] - centerX) * scale);
      hull.push((vertices[index * 3 + 1] - centerY) * scale);
      var z = 1.0 + (vertices[index * 3 + 2] - bottom) * scale;
      hull.push(z);
      top = Math.max(top, z);
    }
    var model = new RobotModel("MachineKit part hull probe");
    model.addLink(new Link("part"));
    model.collisionApproximation = robotkit.model.CollisionApproximation.None;
    for (backend in [ApplicationSimulation.DETERMINISTIC, ApplicationSimulation.MUJOCO])
      for (enabled in [true, false]) {
        model.links[0].collisionShapes = enabled ? [new robotkit.model.CollisionShape(collisionkit.CollisionGeometry.Convex(hull))] : [];
        var simulationHarness = new SimulationHarness(0.01, 1, backend);

        var simulation = simulationHarness.simulation;
        simulation.addRobotAtPose(RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()), [0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 1.0]);
        var box = simulationHarness.spawnBox([0.0, 0.0, top + 0.5], [0.05, 0.05, 0.05], true, 1.0);
        for (index in 0...200) simulationHarness.step(Int64.ofInt(index));
        var height = simulationHarness.objectPose(box).position[2];
        check(enabled ? height > 0.8 : (backend == ApplicationSimulation.MUJOCO ? height < 0.25 : height < 0.0),
          'MachineKit part collision ${enabled ? "on" : "off"} on backend $backend: $height');
        simulationHarness.dispose();
      }
  }

  static function rotateVector(x:Float, y:Float, z:Float, q:Array<Float>):Array<Float> {
    var tx = 2 * (q[1] * z - q[2] * y);
    var ty = 2 * (q[2] * x - q[0] * z);
    var tz = 2 * (q[0] * y - q[1] * x);
    return [x + q[3] * tx + q[1] * tz - q[2] * ty,
      y + q[3] * ty + q[2] * tx - q[0] * tz,
      z + q[3] * tz + q[0] * ty - q[1] * tx];
  }

  /** The arm's hierarchy shows its rigid bodies, not its 20-deep joint tree. */
  static function checkArmHierarchy(session:ProjectDocumentSession):Void {
    var tree = new EditorSceneTree(session.scene, session.projectAssemblyDefinition, session.generatedLabels());
    var top = [for (index in 0...tree.childCount("scene")) tree.childKeyAt("scene", index)];
    check(top.join(",") == "project:pedestal,project:table,project:padPick,project:padPlace,project:workpiece",
      "the hierarchy starts at the arm's base body and the cell's fixed bodies (" + top.join(",") + ")");
    var chain = ["pedestal", "turret", "upperArm", "forearm", "wristBody", "hand", "toolFlange"];
    var key = "project:pedestal", rows = 1, depth = 1;
    function bodyBelow(parent:String, id:String):Bool {
      for (index in 0...tree.childCount(parent)) if (tree.childKeyAt(parent, index) == "project:" + id) return true;
      return false;
    }
    for (index in 1...chain.length) {
      check(bodyBelow(key, chain[index]), 'body ${chain[index]} hangs below ${chain[index - 1]} by its moving joint');
      key = "project:" + chain[index];
    }
    // Rows shown with the default expansion: every body, and each body's collapsed Parts node.
    function count(parent:String, level:Int):Void {
      for (index in 0...tree.childCount(parent)) {
        var child = tree.childKeyAt(parent, index);
        rows++;
        depth = Std.int(Math.max(depth, level));
        if (tree.initiallyExpanded(child)) count(child, level + 1);
      }
    }
    count("project:pedestal", 2);
    // Seven bodies chain seven levels deep; the last body's own Parts node is the eighth.
    check(rows == 14 && depth == 8, 'the arm shows 7 bodies and 7 collapsed Parts nodes, got $rows rows, depth $depth');
    var pedestalParts = ["baseFlange", "joint1", "gearbox1", "driver1", "driver2", "driver3", "driver4", "driver5", "driver6", "powerSupply"];
    check(tree.childCount("parts:pedestal") == pedestalParts.length && tree.childCount("parts:toolFlange") == 7,
      "a body's Parts node holds the parts fixed to its root");
    for (id in pedestalParts) {
      var found = false;
      for (index in 0...tree.childCount("parts:pedestal"))
        if (tree.childKeyAt("parts:pedestal", index) == "project:" + id) found = true;
      check(found, 'pedestal fixed parts include $id');
    }
    check(tree.isGroup("parts:toolFlange") && !tree.isGroup("project:turret"), "Parts nodes are not selectable objects");
    check(!tree.initiallyExpanded("parts:toolFlange"), "Parts nodes start collapsed");
    session.scene.select("project:tool/cup");
    tree.revision();
    check(tree.initiallyExpanded("parts:toolFlange") && !tree.initiallyExpanded("parts:pedestal"),
      "selecting a part opens the Parts node that holds it");
    var before = tree.revision();
    session.scene.select("project:turret");
    check(tree.revision() != before && !tree.initiallyExpanded("parts:toolFlange"),
      "selecting elsewhere closes it again and changes the model revision");
  }

  /**
   * The IK drag the viewport starts when a jointed part is pressed: the suction cup follows a target
   * pulled 3 cm, the drag commits as one undoable edit, a far target is reported and cancelled, and a
   * part with no joint above it cannot be dragged.
   */
  static function checkArmDrag(session:ProjectDocumentSession, metresPerUnit:Float):Void {
    var definition = session.projectAssemblyDefinition;
    var before = session.projectAssemblyState;
    if (definition == null || before == null) throw "robot arm has no assembly state";
    var cup = new AssemblyState(definition, before).worldPose("tool/cup");
    var start = [cup.x * metresPerUnit, cup.y * metresPerUnit, cup.z * metresPerUnit];
    var drag = session.beginAssemblyDrag("project:tool/cup", start);
    check(drag != null, "pressing the suction cup starts an IK drag");
    for (step in 1...16) {
      drag.update(start[0] + 0.002 * step, start[1], start[2]);
      check(drag.following(), 'the cup follows each small pull (${drag.message()})');
    }
    var held = drag.grabbedPoint();
    check(Math.abs(held[0] - (start[0] + 0.03)) < 1e-5 && Math.abs(held[1] - start[1]) < 1e-5 &&
      Math.abs(held[2] - start[2]) < 1e-5, 'the cup ends 3 cm along x ($held)');
    check(drag.commit(), "releasing records the dragged pose");
    var moved = new AssemblyState(definition, session.projectAssemblyState).worldPose("tool/cup");
    check(Math.abs(moved.x * metresPerUnit - (start[0] + 0.03)) < 1e-5, "the committed state holds the dragged pose");
    session.document.undo();
    var restored = new AssemblyState(definition, session.projectAssemblyState).worldPose("tool/cup");
    check(Math.abs(restored.x - cup.x) < 1e-9 && Math.abs(restored.z - cup.z) < 1e-9, "undo puts the cup back");

    var far = session.beginAssemblyDrag("project:tool/cup", start);
    far.update(start[0] + 5.0, start[1], start[2]);
    check(!far.following() && far.message().indexOf("reach") >= 0, 'a target 5 m away is reported (${far.message()})');
    far.cancel();
    var untouched = new AssemblyState(definition, session.projectAssemblyState).worldPose("tool/cup");
    check(Math.abs(untouched.x - cup.x) < 1e-9, "cancelling leaves the state as it was");

    var driven = new Map<String, Bool>();
    for (joint in definition.joints) if (joint.role == materia.assembly.AssemblyDefinition.AssemblyJointRole.Tree)
      driven.set(joint.child, true);
    var root = [for (occurrence in definition.occurrences) if (!driven.exists(occurrence.id)) occurrence.id][0];
    check(session.beginAssemblyDrag("project:" + root, start) == null, 'the fixed root part "$root" cannot be dragged');
  }

  /** A free part leaves the assembly robot; a joined part cannot be freed. */
  static function checkFreeParts(definition:AssemblyDefinition, physical:AssemblyPhysicalData):Void {
    // A free part is not simulated as part of the assembly; a part that is joined cannot be freed.
    var whole = AssemblySimulationBridge.toRobotModel(definition, physical);
    var freed = AssemblySimulationBridge.toRobotModel(definition, physical, null, ["workpiece"]);
    check(whole.partLinks.exists("workpiece") && !freed.partLinks.exists("workpiece") &&
      freed.model.links[0].mass < whole.model.links[0].mass && freed.model.joints.length == whole.model.joints.length,
      "a free part leaves the assembly robot without its mass or its link");
    var message = "";
    try AssemblySimulationBridge.toRobotModel(definition, physical, null, ["turret"]) catch (error:Dynamic) message = Std.string(error);
    check(message.indexOf("cannot be joined") >= 0, "a joined part cannot be freed: " + message);
  }

  /**
   * The mobile base opens as a wheeled robot on both backends: a commanded twist rolls its chassis and
   * every part on it over the floor, the wheels spin their own ways, and wheel odometry agrees.
   */
  /**
   * The runtime's speed and torque limits on each driven joint come from its drive, not from numbers typed in with
   * the joint: the motor's maximum speed over the gearbox ratio, and its peak torque through the gearbox at its
   * efficiency. Returns what each joint compiled to, as speed then torque.
   */
  static function checkLimitsFromDrives(session:ProjectDocumentSession, driven:Array<String>, label:String):Map<String, Array<Float>> {
    var definition = session.projectAssemblyDefinition, physical = session.projectPhysical;
    if (definition == null || physical == null) throw '$label has no assembly to build a robot from';
    var model = AssemblySimulationBridge.toRobotModel(definition, physical, session.projectAssemblyState, null, null, session.mobileBase).model;
    var compiled = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    var found = new Map<String, Array<Float>>();
    for (id in driven) {
      var index = -1;
      for (candidate in 0...model.joints.length) if (model.joints[candidate].id == id) index = candidate;
      var motors:Array<robotkit.model.Actuator> = [];
      for (actuator in model.actuators) switch actuator.transmission {
        case SimpleTransmission(joint, _, _): if (joint == id) motors.push(actuator);
      }
      check(index >= 0 && motors.length == 1, '$label drives joint $id with exactly one motor');
      var motor = motors[0];
      var ratio = switch motor.transmission { case SimpleTransmission(_, ratio, _): Math.abs(ratio); };
      var speed = motor.requireRate() / ratio, torque = motor.requireEffort() * ratio * motor.efficiency;
      var effective = model.joints[index].limits;
      check(effective.velocity != null && effective.effort != null &&
        Math.abs(effective.requireVelocity() - speed) <= 1e-9 * speed && Math.abs(effective.requireEffort() - torque) <= 1e-9 * torque,
        '$label compiled model preserves the drive limits used by mission planning and jogging on $id');
      check(Math.abs(compiled.joints[index].requireRate() - speed) <= 1e-9 * speed && Math.abs(compiled.joints[index].requireEffort() - torque) <= 1e-9 * torque,
        '$label joint $id is limited to ${compiled.joints[index].requireRate()} rad/s and ${compiled.joints[index].requireEffort()} N m, its drive gives $speed and $torque');
      check(motor.drive != null && ratio > 1.0, '$label joint $id has a drive and a gearbox');
      found.set(id, [compiled.joints[index].requireRate(), compiled.joints[index].requireEffort()]);
    }
    return found;
  }

  /**
   * The CoreXY plotter's two motors follow both axes: each pulley is the sum of its couplings' terms, each
   * axis is bound by both motors, and the shared simulation (MuJoCo, with fixed tendons for the sums) builds,
   * steps and keeps the machine at rest.
   */
  /**
   * The CoreXY printer: its motors follow two axes each (belts), its bed follows z through three screws, its extruder
   * through a gear pair; the shared simulation (MuJoCo) builds it, and it homes on its switches in the order the
   * assembly states: the bed down first, then X and Y.
   */
  static function checkPrinter(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/printer/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var definition = session.projectAssemblyDefinition, physical = session.projectPhysical;
    if (definition == null || physical == null) throw "the printer has no assembly to build a robot from";
    var model = AssemblySimulationBridge.toRobotModel(definition, physical, session.projectAssemblyState).model;
    check(model.actuators.length == 6, 'the printer has six steppers, got ${model.actuators.length}');
    var leaders = new Map<String, Int>();
    for (coupling in model.couplings) leaders.set(coupling.follower, (leaders.exists(coupling.follower) ? leaders.get(coupling.follower) : 0) + 1);
    check(leaders.get("pulleyA-turn") == 2 && leaders.get("pulleyB-turn") == 2 && leaders.get("idlerAStart-turn") == 1,
      "each belt motor's pulley follows two axes, a gantry idler one");
    var homes = [for (contact in model.switches) if (contact.role == "home") contact.joint];
    homes.sort(Reflect.compare);
    check(homes.join(",") == "x,y,z", 'the printer has home switches on x, y and z, got $homes');
    var compiled = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    check(compiled.couplings.length == model.couplings.length, "the runtime blueprint carries every coupling");
    // The controller's 40 kHz step tick, as the router's.
    var planning = DeviceBinding.planningModel(model, 40000);
    // The extruder's motor turns backwards as it pushes (one gear mesh): the board's cold-extrusion guard counts its
    // steps that way, so a cold push faults and a retraction does not.
    var deviceModel = AssemblySimulationBridge.toRobotModel(definition, physical, session.projectAssemblyState).model;
    deviceModel.switches.resize(0);
    var extruder = PrintProgramPlayer.extruderOnDevice(DeviceBinding.bind(deviceModel, robotkit.device.DeviceLayout.forActuators(deviceModel), 40000), "e");
    check(extruder.feed == -1, 'the extruder motor steps down as it pushes filament: feed ${extruder.feed}');
    // The axes at the motors' full rate (AM5b item 7), as the print player plans them.
    var xLimits = planning.coupledLimits("x", null, true), yLimits = planning.coupledLimits("y", null, true),
      zLimits = planning.coupledLimits("z", null, true);
    check(xLimits.requireAcceleration() > yLimits.requireAcceleration() && yLimits.requireAcceleration() > 5,
      'the carriage accelerates harder than the gantry it rides: ${xLimits.requireAcceleration()} against ${yLimits.requireAcceleration()}');
    Sys.println('printer: axes x ${Math.round(xLimits.requireVelocity() * 1e4) / 10} mm/s ${Math.round(xLimits.requireAcceleration() * 100) / 100} m/s², ' +
      'y ${Math.round(yLimits.requireVelocity() * 1e4) / 10} mm/s ${Math.round(yLimits.requireAcceleration() * 100) / 100} m/s², ' +
      'z ${Math.round(zLimits.requireVelocity() * 1e4) / 10} mm/s ${Math.round(zLimits.requireAcceleration() * 100) / 100} m/s²');
    var world = new RobotWorld();
    var simulation = new ApplicationSimulation(world);
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    check(simulation.rebuild(session.sensors, session.scene, session), "the printer builds in the shared simulation: " + simulation.error);
    simulation.step();

    // Homing on the deterministic simulation (MuJoCo joins in AM8): every axis finds its switch and is referenced, the bed first.
    var axes = [for (joint in planning.joints) if (joint.type == JointType.Prismatic) {
      var limits = planning.coupledLimits(joint.id, new SteadyLoads());
      new motionkit.axis.MotionAxisBlueprint(joint.id, [joint.id], joint.limits.lower, joint.limits.upper,
        limits.requireVelocity(), limits.requireAcceleration(), joint.id == "z" ? joint.limits.upper : joint.limits.lower);
    }];
    var blueprint = motionkit.robot.MotionSystemBlueprint.fromRobotModel(planning, axes);
    var harness = new SimulationHarness(0.01, 1, SimulationSpace.DETERMINISTIC);
    try {
      var runtime = harness.simulation.addRobot(blueprint.runtime);
      var robot = new robotkit.simulation.SimulatedRobot("printer", runtime, planning.name,
        [for (link in planning.links) link.name], [for (joint in planning.joints) joint.name]);
      var motion = new motionkit.robot.MotionSystem(robot, blueprint);
      motion.configureRuntimeHoming(runtime, () -> {}, harness.simulation.homingSides(0));
      harness.step(Int64.ofInt(1));
      motion.home();
      var ticks = 0, firstReferenced = "";
      while (motion.homingStatus() != "Complete" && ticks++ < 60000) {
        harness.step(Int64.ofInt(ticks + 1));
        motion.update(0.01);
        if (firstReferenced == "") for (index in 0...planning.joints.length)
          if (planning.joints[index].type == JointType.Prismatic && runtime.isReferenced(index)) firstReferenced = planning.joints[index].id;
      }
      check(motion.homingStatus() == "Complete", "The printer homes: " + motion.homingStatus());
      check(firstReferenced == "z", 'the bed is the first axis referenced, got "$firstReferenced"');
      var snapshot = runtime.snapshot();
      for (index in 0...planning.joints.length) if (planning.joints[index].type == JointType.Prismatic) {
        check(runtime.isReferenced(index), 'printer axis ${planning.joints[index].id} is referenced');
        var home = planning.joints[index].id == "z" ? planning.joints[index].limits.upper : planning.joints[index].limits.lower;
        check(Math.abs(snapshot.q.get(index) - home) < 0.0002, 'printer axis ${planning.joints[index].id} returns to its reference, ' +
          'at ${snapshot.q.get(index)} against ${home}');
      }
      Sys.println('printer: homed in ${ticks * 0.01} s');
      harness.dispose();
    } catch (error:Dynamic) { harness.dispose(); throw error; }
    printSmallTab(session, simulation, world, model);
    simulation.dispose();
    session.dispose();
  }

  /**
   * What a printed stock holds in the box `[x0, x1] x [y0, y1]` grown by `margin`: the volume, the extent of the material
   * from the exact ends of the X and Y rays and the Z rays' intervals, and the volume in the ring `margin` .. 3 mm
   * outside (where only the skirt may be). Metres, in the stock's frame shifted to its preview centre: the stock's
   * lattice and the box are both given in the frame of machine coordinates, so the shift is added back.
   */
  static function measurePrinted(stock:stockkit.Stock, x0:Float, x1:Float, y0:Float, y1:Float, margin:Float,
      ?shift:Array<Float>):{volume:Float, low:Array<Float>, high:Array<Float>, outside:Float} {
    var lattice = stock.lattice;
    var offset = shift == null ? [0.0, 0.0, 0.0] : shift;
    var volume = 0.0, outside = 0.0, area = lattice.spacing * lattice.spacing;
    var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
    var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
    var rays = stock.rays(0, 0, lattice.countX, lattice.countY);
    for (j in 0...lattice.countY) for (i in 0...lattice.countX) {
      var intervals = rays[j * lattice.countX + i];
      if (intervals.length == 0) continue;
      var x = lattice.x(i) + offset[0], y = lattice.y(j) + offset[1];
      var inside = x >= x0 - margin && x <= x1 + margin && y >= y0 - margin && y <= y1 + margin;
      var length = 0.0;
      for (interval in intervals) length += interval.hi - interval.lo;
      if (inside) {
        volume += length * area;
        low[2] = Math.min(low[2], intervals[0].lo + offset[2]);
        high[2] = Math.max(high[2], intervals[intervals.length - 1].hi + offset[2]);
      } else if (x >= x0 - 1.5e-3 && x <= x1 + 1.5e-3 && y >= y0 - 1.5e-3 && y <= y1 + 1.5e-3) outside += length * area;
    }
    // The extent in x and y from the rays along them, whose interval ends are exact.
    var xRays = stock.rays(0, 0, lattice.countY, lattice.countZ, stockkit.StockAxis.X);
    for (k in 0...lattice.countZ) for (j in 0...lattice.countY) {
      var intervals = xRays[k * lattice.countY + j];
      var y = lattice.y(j) + offset[1];
      if (intervals.length == 0 || y < y0 - margin || y > y1 + margin) continue;
      for (interval in intervals) {
        var lo = interval.lo + offset[0], hi = interval.hi + offset[0];
        if (hi < x0 - margin || lo > x1 + margin) continue;
        low[0] = Math.min(low[0], lo); high[0] = Math.max(high[0], hi);
      }
    }
    var yRays = stock.rays(0, 0, lattice.countX, lattice.countZ, stockkit.StockAxis.Y);
    for (k in 0...lattice.countZ) for (i in 0...lattice.countX) {
      var intervals = yRays[k * lattice.countX + i];
      var x = lattice.x(i) + offset[0];
      if (intervals.length == 0 || x < x0 - margin || x > x1 + margin) continue;
      for (interval in intervals) {
        var lo = interval.lo + offset[1], hi = interval.hi + offset[1];
        if (hi < y0 - margin || lo > y1 + margin) continue;
        low[1] = Math.min(low[1], lo); high[1] = Math.max(high[1], hi);
      }
    }
    return {volume: volume, low: low, high: high, outside: outside};
  }

  /**
   * The printer prints a calibration cube (20 mm square with a 6 mm hole through it, off centre by (2, -1) mm) and the
   * stock it laid is measured: the plastic against the filament the program feeds (and, for a solid cube, against the
   * CAD part's volume), the box, the hole's diameter at mid-height along x and along y through the hole's own centre,
   * and nothing laid beside the cube but the skirt. `project` is the example directory (`cube`: 100 layers at the
   * slicer's default infill, slow; `cube-short`: 4 mm, solid, the regular check), `height` its height in millimetres.
   * Runs on the deterministic backend and by itself (`PROJECT_SOURCE_ONLY=printer-cube-short` or `printer-cube`).
   */
  static function checkPrinterCube(root:String, project:String, height:Float, solid:Bool):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/printer/" + project + "/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var world = new RobotWorld();
    var simulation = new ApplicationSimulation(world);
    simulation.setBackend(ApplicationSimulation.DETERMINISTIC);
    check(simulation.rebuild(session.sensors, session.scene, session), "the cube printer builds: " + simulation.error);
    var player = simulation.printPlayer(), job = session.printJob;
    if (player == null || job == null) throw "the cube printer has no print player";
    player.showPrint = false;
    // Half a tenth of a millimetre between rays keeps a bead of 0.4 mm to eight and the hole's diameter to a tenth.
    player.stockSpacing = 0.00005;
    var dt = simulation.timestep, ticks = 0, limit = Math.round(3600.0 / dt);
    var wall = Sys.time();
    // The physical carriage (the x joint's link), not the device's view of it: the simulation must move the machine.
    var definition = session.projectAssemblyDefinition, physical = session.projectPhysical;
    if (definition == null || physical == null) throw "the cube printer has no assembly";
    var model = AssemblySimulationBridge.toRobotModel(definition, physical, session.projectAssemblyState).model;
    var carriageLink = model.links.indexOf([for (joint in model.joints) if (joint.id == "x") joint][0].child);
    var carriageLow = Math.POSITIVE_INFINITY, carriageHigh = Math.NEGATIVE_INFINITY;
    while (player.passes == 0 && player.failure == null && ticks++ < limit) {
      simulation.step();
      if (ticks % 50 == 0) {
        var at = simulation.physicalLinkPosition(0, carriageLink)[0];
        carriageLow = Math.min(carriageLow, at); carriageHigh = Math.max(carriageHigh, at);
      }
      if (ticks % 10000 == 0) Sys.println('printer $project: ${ticks * dt} s, layer ${player.layer} of ${player.layerCount}, ${Math.round((Sys.time() - wall) * 10) / 10} s wall');
    }
    check(player.failure == null, "the printer prints the cube: " + player.failure);
    check(player.passes == 1, 'the cube is done in ${ticks * dt} s (limit ${limit * dt} s)');
    // A device drives the motor pulleys; on a backend whose couplings are ideal it drives the carriage too (with springs
    // the belts carry it), so the physical carriage crosses the cube, not only the device's reconstruction of it.
    check(carriageHigh - carriageLow > 0.015, 'the physical carriage crosses the cube: ${(carriageHigh - carriageLow) * 1000} mm');
    var laid = player.stock;
    if (laid == null) throw "the cube print laid no material";
    var cube = measurePrinted(laid.stock, 0.100, 0.120, 0.100, 0.120, 0.0005, laid.center);
    var tall = height * 1e-3;
    var fed = programFilament(job.source) * 1e-3 * Math.PI * job.filamentDiameter * job.filamentDiameter / 4.0;
    var cad = (20.0 * 20.0 * height - Math.PI * 3.0 * 3.0 * height) * 1e-9;
    check(Math.abs(laid.deposited - fed) <= 0.02 * fed, 'the print laid ${laid.deposited * 1e9} mm^3 in ${laid.beads} beads for ${fed * 1e9} mm^3 of filament');
    check(cube.volume <= laid.deposited && cube.volume >= 0.9 * laid.deposited, 'the cube holds ${cube.volume * 1e9} mm^3 of the ${laid.deposited * 1e9} laid');
    if (solid) check(Math.abs(cube.volume - cad) <= 0.03 * cad, 'the solid cube holds ${cube.volume * 1e9} mm^3, its CAD volume is ${cad * 1e9}');
    else check(cube.volume < cad && cube.volume > 0.25 * cad, 'the sparse cube holds ${cube.volume * 1e9} mm^3 of its solid CAD volume ${cad * 1e9}');
    for (axis in 0...3) {
      var wanted = [[0.100, 0.120], [0.100, 0.120], [0.0, tall]][axis];
      check(Math.abs(cube.low[axis] - wanted[0]) <= 0.0002 && Math.abs(cube.high[axis] - wanted[1]) <= 0.0002,
        'the cube spans ${cube.low[axis] * 1000}..${cube.high[axis] * 1000} mm on axis $axis, CAD ${wanted[0] * 1000}..${wanted[1] * 1000}');
    }
    check(cube.outside <= 1e-12, 'nothing is laid next to the cube but the skirt: ${cube.outside * 1e9} mm^3');
    // The hole: every wall point that the X and the Y rays crossing it meet, over the middle half of the part's height
    // (a ray's interval ends are exact, so each is a point of the printed wall), fitted with a circle by least squares.
    // Its centre is the part's centre (0.110, 0.110) plus the hole's offset (2, -1) mm.
    var holeX = 0.112, holeY = 0.109, holeRadius = 0.003;
    var lattice = laid.stock.lattice;
    var holeWall:Array<Array<Float>> = [];
    function wallOn(across:Array<stockkit.StockInterval>, offset:Float, centre:Float, other:Float, alongX:Bool):Void {
      for (index in 0...across.length - 1) {
        var from = across[index].hi + offset, to = across[index + 1].lo + offset;
        if (from < centre && to > centre) {
          holeWall.push(alongX ? [from, other] : [other, from]);
          holeWall.push(alongX ? [to, other] : [other, to]);
        }
      }
    }
    var kLow = Math.round((tall / 4.0 - laid.center[2] - lattice.originZ) / lattice.spacing);
    var kHigh = Math.round((3.0 * tall / 4.0 - laid.center[2] - lattice.originZ) / lattice.spacing);
    // Only rays that pass through the hole (a ray beyond it could find a gap of the sparse infill instead).
    var reach = Math.floor(holeRadius / lattice.spacing) - 1;
    var jCentre = Math.round((holeY - laid.center[1] - lattice.originY) / lattice.spacing);
    var iCentre = Math.round((holeX - laid.center[0] - lattice.originX) / lattice.spacing);
    for (k in kLow...kHigh + 1) for (d in -reach...reach + 1) {
      var y = lattice.originY + (jCentre + d) * lattice.spacing + laid.center[1];
      var x = lattice.originX + (iCentre + d) * lattice.spacing + laid.center[0];
      wallOn(laid.stock.ray(jCentre + d, k, stockkit.StockAxis.X), laid.center[0], holeX, y, true);
      wallOn(laid.stock.ray(iCentre + d, k, stockkit.StockAxis.Y), laid.center[1], holeY, x, false);
    }
    // x^2 + y^2 + D x + E y + F = 0 by least squares (Kasa), solved by its normal equations.
    var m = [for (_ in 0...3) [0.0, 0.0, 0.0, 0.0]];
    for (point in holeWall) {
      var u = point[0] - holeX, v = point[1] - holeY, rhs = -(u * u + v * v);
      var row = [u, v, 1.0];
      for (a in 0...3) { for (b in 0...3) m[a][b] += row[a] * row[b]; m[a][3] += row[a] * rhs; }
    }
    for (pivot in 0...3) for (r in 0...3) if (r != pivot) {
      var f = m[r][pivot] / m[pivot][pivot];
      for (c in 0...4) m[r][c] -= f * m[pivot][c];
    }
    var dCoef = m[0][3] / m[0][0], eCoef = m[1][3] / m[1][1], fCoef = m[2][3] / m[2][2];
    var holeDiameter = 2.0 * Math.sqrt(dCoef * dCoef / 4.0 + eCoef * eCoef / 4.0 - fCoef);
    var holeOffCentre = Math.sqrt(dCoef * dCoef + eCoef * eCoef) / 2.0;
    check(holeWall.length > 100, 'the hole has a printed holeWall to measure: ${holeWall.length} points');
    check(Math.abs(holeDiameter - 0.006) <= 0.0003, 'the hole measures ${holeDiameter * 1000} mm across, CAD 6');
    check(holeOffCentre <= 0.0002, 'the hole is ${holeOffCentre * 1e6} um from where CAD has it');
    Sys.println('printer $project: printed in ${Math.round(ticks * dt * 10) / 10} s (${Math.round((Sys.time() - wall) * 10) / 10} s wall), ' +
      '${Math.round(cube.volume * 1e10) / 10} mm^3 in the cube of ${Math.round(laid.deposited * 1e10) / 10} laid for ${Math.round(fed * 1e10) / 10} fed (CAD ${Math.round(cad * 1e10) / 10}, ' +
      '${Math.round((cube.volume / cad - 1.0) * 1000) / 10} %), spans ' +
      '${Math.round(cube.low[0] * 1e4) / 10}..${Math.round(cube.high[0] * 1e4) / 10} x ${Math.round(cube.low[1] * 1e4) / 10}..${Math.round(cube.high[1] * 1e4) / 10} x ' +
      '${Math.round(cube.low[2] * 1e4) / 10}..${Math.round(cube.high[2] * 1e4) / 10} mm (to the um: ${[for (a in 0...3) Math.round(cube.low[a] * 1e6)]}..${[for (a in 0...3) Math.round(cube.high[a] * 1e6)]}), ' +
      'hole ${Math.round(holeDiameter * 1e6)} um across, ${Math.round(holeOffCentre * 1e6)} um off its CAD centre (${holeWall.length} holeWall points), ' +
      '${laid.beads} beads laid in ${Math.round(player.depositSeconds * 10) / 10} s');
    simulation.dispose();
    session.dispose();
  }

  /**
   * Input shaping on the printer in the shared simulation (MuJoCo, through the virtual device): a 50 mm move along x with the
   * job's shapers and without them. The shaped move is the same move delayed by the shapers' mean delay and ending their tail
   * later, and the carriage ends where it was sent, to within one microstep of the belt (12.5 um). The belts are springs of
   * damping ratio 0.1 in MuJoCo (they state it), so the carriage swings there after the plain move, at the belts' frequency
   * (about 53 Hz on x). The shaped move leaves it at least five times less shaking, at the shapers' design frequency and ten
   * percent either side of it, as the printing gate's analytic acceptance says (PLAN, AM12 and AM5b belt damping).
   */
  static function checkPrinterShaping(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/printer/materia.project.json");
    var ends:Array<Float> = [], finishes:Array<Int> = [], rings:Array<Float> = [], swings:Array<Int> = [];
    var step = 0.0;
    // The move plain, shaped for the belts' frequency, and shaped for a frequency ten percent off either way.
    var variants = [{shaped: false, detune: 1.0}, {shaped: true, detune: 1.0}, {shaped: true, detune: 0.9}, {shaped: true, detune: 1.1}];
    for (variant in variants) {
      var generated = MateriaProjectRunner.loadProject(manifest);
      var job = generated.printJob;
      if (job == null || job.shaping == null) throw "the printer example states no input shapers";
      if (!variant.shaped) job.shaping = null;
      else for (shaper in job.shaping) shaper.frequency *= variant.detune;
      // Only motion: a move to the start, a pause, then the 50 mm move that is measured; no heating, no extrusion.
      job.source = "G21\nG90\nM83\nG0 X30 Y30 F9000\nG4 P1000\nG1 X80 Y30 F9000\n";
      var session = new ProjectDocumentSession(null, false);
      session.openGeneratedProject(generated, manifest);
      var definition = session.projectAssemblyDefinition, physical = session.projectPhysical;
      if (definition == null || physical == null) throw "the printer has no assembly";
      var model = AssemblySimulationBridge.toRobotModel(definition, physical, session.projectAssemblyState).model;
      var world = new RobotWorld();
      var simulation = new ApplicationSimulation(world);
      simulation.setBackend(ApplicationSimulation.MUJOCO);
      // A tick of a millisecond: the carriage rings at 53 Hz, which a tick of 10 ms cannot see.
      simulation.setTimestep(0.001);
      check(simulation.rebuild(session.sensors, session.scene, session), "the printer builds: " + simulation.error);
      var player = simulation.printPlayer();
      if (player == null) throw "the printer has no print player";
      var x = -1;
      for (index in 0...model.joints.length) if (model.joints[index].id == "x") x = index;
      var dt = simulation.timestep, ticks = 0, limit = Math.round(30.0 / dt), finished = -1;
      // The carriage, once the plan's setpoint has arrived at X 80 mm (the machine's x is the nozzle's distance from the
      // bed's middle in the assembly, 110 mm less than the machine's), and for 0.3 s after.
      var carriageLink = model.links.indexOf(model.joints[x].child);
      var carriage:Array<Float> = [], pathPoints:Array<Array<Float>> = [], arrived = false, origin:Null<Array<Float>> = null;
      while (ticks++ < limit && player.failure == null && finished < 0) {
        simulation.step();
        var frame = world.snapshot().robots()[0];
        if (origin == null && Math.abs(frame.setpointPositions.get(x) + 0.110 - 0.030) < 1e-9) origin = simulation.physicalLinkPosition(0, carriageLink);
        if (!arrived && Math.abs(frame.setpointPositions.get(x) + 0.110 - 0.080) < 1e-9) arrived = true;
        if (arrived) {
          carriage.push(frame.positions.get(x) + 0.110);
          pathPoints.push(simulation.physicalLinkPosition(0, carriageLink));
        }
        if (player.passes == 1) finished = ticks;
      }
      check(player.failure == null, "the move runs: " + player.failure);
      check(finished > 0, "the move completes");
      for (_ in 0...Math.round(0.3 / dt)) {
        simulation.step();
        carriage.push(world.snapshot().robots()[0].positions.get(x) + 0.110);
        pathPoints.push(simulation.physicalLinkPosition(0, carriageLink));
      }
      ends.push(carriage[carriage.length - 1]);
      finishes.push(finished);
      step = dt;
      // What is left shaking: how far the carriage strays from where it settles in the 150 ms after the setpoint arrives.
      // The carriage as physics has it, along the way it settles from where it was when the setpoint arrived.
      var last = pathPoints[pathPoints.length - 1];
      check(origin != null, "the carriage is seen at X 30 mm");
      var direction = [for (axis in 0...3) last[axis] - origin[axis]];
      var length = Math.sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
      var along = [for (point in pathPoints) (direction[0] * (point[0] - last[0]) + direction[1] * (point[1] - last[1]) + direction[2] * (point[2] - last[2])) / Math.max(length, 1e-9)];
      var tail = Math.round(0.05 / dt), settled = 0.0, ringing = 0.0;
      for (index in along.length - tail...along.length) settled += along[index] / tail;
      for (index in 0...Math.round(0.15 / dt)) ringing = Math.max(ringing, Math.abs(along[index] - settled));
      // How often the carriage passes through where it settles: a spring that rings does, one that creeps does not.
      var crossings = 0;
      for (index in 1...Math.round(0.3 / dt)) if ((along[index] - settled) * (along[index - 1] - settled) < 0.0) crossings++;
      swings.push(crossings);
      rings.push(ringing);
      simulation.dispose();
      session.dispose();
    }
    for (end in ends) check(Math.abs(end - 0.080) <= 20e-6, 'every move ends at X 80 mm: ${[for (value in ends) value * 1000].join(", ")} mm');
    // The belts' damping ratio rides into the simulation (the printer's states 0.1): the carriage, which the device leaves to
    // the belts, swings through its rest position instead of creeping to it, at the belts' frequency (the gate's analytic
    // check and the simulation agree on the plant). A shaper at that frequency, and one ten percent either side of it,
    // leave at least five times less of that swing.
    check(rings[0] > 100e-6 && swings[0] >= 2, 'the plain move leaves the carriage swinging in MuJoCo: ${rings[0] * 1e6} um, ${swings[0]} crossings');
    for (index in 1...variants.length)
      check(rings[index] * 5.0 <= rings[0], 'shaped at ${Math.round(variants[index].detune * 100)} % of the belts\' frequency the carriage swings ' +
        '${rings[index] * 1e6} um, the plain move ${rings[0] * 1e6} um: under 5x less');
    var later = (finishes[1] - finishes[0]) * step;
    check(later >= 0.01 && later <= 0.06, 'the shaped move completes ${later * 1000} ms after the plain one (the shapers\' delay and tail)');
    Sys.println('printer shaping: the moves end within ${Math.round(Math.max(Math.max(Math.abs(ends[0] - 0.080), Math.abs(ends[1] - 0.080)), Math.max(Math.abs(ends[2] - 0.080), Math.abs(ends[3] - 0.080))) * 1e7) / 10} um of X 80 mm in MuJoCo; ' +
      'the shaped one completes ${Math.round(later * 1000)} ms later; the carriage swings ${Math.round(rings[0] * 1e6)} um plain and ${Math.round(rings[1] * 1e6)} um shaped ' +
      '(${Math.round(rings[0] / Math.max(rings[1], 1e-9))}x less; ${Math.round(rings[2] * 1e6)} um at -10 %, ${Math.round(rings[3] * 1e6)} um at +10 %), ${swings[0]} crossings plain');
  }

  /**
   * The editor's link to a printer: the print is sent to a Moonraker (a fake on a real socket) and started, and the
   * printer model follows the machine over the WebSocket subscription, each update putting the model's axes and
   * extruder where the nozzle is (`motion_report.live_position`, not the end of the queued moves), so the twin is
   * exactly the machine's state after the one update. Then a Klipper config is installed: uploaded, the firmware
   * restarted, read back.
   */
  static function checkPrinterMirror(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/printer/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var job = session.printJob, definition = session.projectAssemblyDefinition;
    if (job == null || definition == null) throw "the printer project has no print to send";
    var runtime = NativeKitRuntime.start();
    var fake = FakeMoonrakerServer.start(runtime, 36292);
    var link = new PrinterLink();
    var seen:Array<MoonrakerStatus> = [];
    link.onStatus = function(status) {
      seen.push(status);
      PrinterMirror.follow(session, status);
    };
    check(PrinterLink.parse("printer.local").port == 7125 && PrinterLink.parse("10.0.0.5:80").port == 80, "an address is host or host:port");
    var refused = false;
    try PrinterLink.parse("host:port") catch (_:Dynamic) refused = true;
    check(refused, "an address with a bad port is refused");
    fake.route = [[10.0, 10.0, 0.2, 0.0], [60.0, 10.0, 0.2, 5.0], [60.0, 60.0, 0.2, 10.0]];
    fake.speed = 30.0;
    link.connect("127.0.0.1:36292");
    function pump(until:Void->Bool, what:String):Void {
      var limit = Sys.time() + 5.0;
      while (!until() && Sys.time() < limit && link.error == null) {
        runtime.events.poll(); fake.poll(); link.poll(Sys.time()); runtime.events.wait(0.001);
      }
      check(link.error == null, '$what: ${link.error}');
      check(until(), '$what timed out');
    }
    pump(function() return link.connected() && link.status != null, "subscribing");
    link.send("printer.gcode", job.source, true);
    pump(function() return fake.state == "printing", "sending the print");
    check(fake.files.get("printer.gcode") == job.source, "the machine received the program whole");
    pump(function() return seen[seen.length - 1].state == "printing", "the start of the print is pushed");
    // The twin follows: one update of the machine and the model stands exactly where the nozzle is, not where the queue ends.
    var unit = 0.001 / session.assemblyUnitMetres();
    function modelAt(id:String):Float {
      var record = session.projectAssemblyState;
      if (record == null) throw "no assembly state";
      return new AssemblyState(definition, record).joint(id);
    }
    var worstOff = 0.0, worstExtruder = 0.0, ahead = 0.0;
    for (step in 0...40) {
      var before = seen.length;
      fake.advance(0.05);
      pump(function() return seen.length > before, "update " + step);
      check(seen.length == before + 1, "one advance of the machine is one update");
      var status = seen[seen.length - 1];
      for (axis in 0...3) worstOff = Math.max(worstOff, Math.abs(modelAt(job.axes[axis]) - fake.position[axis] * unit));
      worstExtruder = Math.max(worstExtruder, Math.abs(modelAt(job.extruder) * job.gearRadius * 1000.0 - fake.position[3]));
      ahead = Math.max(ahead, Math.abs(modelAt(job.axes[0]) - status.queuedPosition[0] * unit));
    }
    check(worstOff < 1e-9, 'the model is where the nozzle is after each update (off by ${worstOff})');
    check(ahead > 1.0, 'and not where the queued moves end, which is ahead of it (${ahead} units)');
    check(worstExtruder < 1e-6, 'the extruder follows the filament position to ${worstExtruder} mm');
    check(fake.position[0] > 40.0 && fake.state == "printing" && modelAt(job.axes[0]) / unit > 10.0, "the toolhead went somewhere and the model with it");
    link.pause();
    pump(function() return fake.state == "paused", "pause");
    link.resume();
    pump(function() return fake.state == "printing", "resume");
    link.cancel();
    pump(function() return fake.state == "cancelled", "cancel");
    pump(function() return seen[seen.length - 1].state == "cancelled", "the cancel is mirrored");
    // A Klipper config goes to the machine and is read back.
    var text = "[stepper_x]\nrotation_distance: 40\nmicrosteps: 16\n\n[extruder]\nrotation_distance: 22.93\ngear_ratio: 54:18\n";
    link.installConfig("printer.cfg", text, KlipperConfig.parse(text));
    pump(function() return link.configMismatches != null, "installing the config");
    var first = link.configMismatches;
    if (first == null) throw "the install did not finish";
    check(first.length == 0 && fake.configFiles.get("printer.cfg") == text && fake.restarts == 1, 'Klipper runs the config that was sent ($first)');
    fake.deviations.set("extruder/rotation_distance", "23.5");
    link.installConfig("printer.cfg", text, KlipperConfig.parse(text));
    pump(function() return link.configMismatches != null, "installing it again");
    var second = link.configMismatches;
    if (second == null) throw "the second install did not finish";
    check(second.length == 1 && second[0].indexOf("rotation_distance") >= 0, 'a value Klipper changed is reported ($second)');
    check(link.summary().indexOf("differs") >= 0, "and the link says so");
    link.disconnect();
    fake.stop();
    runtime.dispose();
    Sys.println('printer link: ${seen.length} updates mirrored, the model exactly at the nozzle, ${Math.round(ahead * 10) / 10} units behind the queue end; config install checked');
    session.dispose();
  }

  /** The filament a printer program feeds, millimetres: the sum of its extrusion words (relative extrusion). */
  static function programFilament(source:String):Float {
    var total = 0.0;
    for (line in source.split("\n")) {
      var words = line.split(";")[0].split(" ");
      if (words.length == 0 || ["G0", "G1", "G2", "G3"].indexOf(words[0]) < 0) continue;
      for (word in words) if (word.length > 1 && word.charAt(0) == "E") total += Std.parseFloat(word.substr(1));
    }
    return total;
  }

  static function median(values:Array<Float>):Float {
    var sorted = values.copy();
    sorted.sort(Reflect.compare);
    return sorted.length == 0 ? Math.NaN : sorted[sorted.length >> 1];
  }

  /**
   * The printer prints the small tab the project ships, on the shared MuJoCo simulation through the virtual device that
   * runs its heaters: the hotend and bed heat up and the program waits for them, the print finishes with no stall, fault
   * or failure, the extruder feeds the filament the program asks for, and slowing the print down keeps the filament
   * per millimetre of path.
   */
  static function printSmallTab(session:ProjectDocumentSession, simulation:ApplicationSimulation, world:RobotWorld, model:RobotModel):Void {
    var player = simulation.printPlayer(), job = session.printJob;
    if (player == null || job == null) throw "the printer has no print player";
    function joint(id:String):Int {
      for (index in 0...model.joints.length) if (model.joints[index].id == id) return index;
      throw 'the printer has no joint $id';
    }
    var e = joint("e");
    // The material is measured by rays: half a tenth of a millimetre between them keeps a bead of 0.4 mm to eight.
    player.stockSpacing = 0.00005;
    var dt = simulation.timestep, every = 5;
    function robot():robotkit.core.RobotSnapshot return world.snapshot().robots()[0];
    var first = robot();
    var e0 = first.positions.get(e);
    var hotendAt = -1.0, bedAt = -1.0, ticks = 0, limit = Math.round(1500.0 / dt);
    // The filament each whole layer takes: pressure advance runs the extruder ahead of the path while it speeds up and
    // behind while it slows down, so filament per path holds over a layer, not sample by sample (and a path measured
    // from samples cuts more of each corner the faster it is run). Layer 2 prints at full speed and layer 4 at half; both
    // are solid with the same infill angle: the same path, so the same filament.
    var fedByLayer:Array<Float> = [for (_ in 0...8) 0.0];
    var lastE = e0, slowed = false, restored = false, detail = "";
    var wall = Sys.time();
    var slowestStep = 0.0, slowestAt = 0.0;
    while (player.passes == 0 && player.failure == null && ticks++ < limit) {
      var stepBegan = Sys.time();
      simulation.step();
      var stepTook = Sys.time() - stepBegan;
      if (stepTook > slowestStep) { slowestStep = stepTook; slowestAt = ticks * dt; }
      if (ticks % every != 0) continue;
      var seen = robot();
      var now = ticks * dt;
      var hotend = player.heaterReading("hotend"), bed = player.heaterReading("bed");
      if (hotendAt < 0 && hotend != null && hotend.atTarget) hotendAt = now;
      if (bedAt < 0 && bed != null && bed.atTarget) bedAt = now;
      // From the third layer to the fifth the print runs at half speed: the filament per millimetre of path must not change.
      if (detail == "" && player.layer == 2) detail = player.statusDetail();
      if (!slowed && player.layer >= 3) { player.setSpeedOverride(0.5); slowed = true; }
      if (slowed && !restored && player.layer >= 5) { player.setSpeedOverride(1.0); restored = true; }
      var fed = (seen.positions.get(e) - lastE) * job.gearRadius;
      if (player.layer >= 0 && player.layer < 8) fedByLayer[player.layer] += fed;
      lastE = seen.positions.get(e);
    }
    var wallSeconds = Sys.time() - wall;
    check(player.failure == null, "the printer prints its tab: " + player.failure);
    check(player.passes == 1, 'the print finishes in ${ticks * dt} s (limit ${limit * dt} s)');
    check(hotendAt > 0 && bedAt > 0, "both heaters reached their targets before the print");
    var end = robot();
    var fedMillimetres = (end.positions.get(e) - e0) * job.gearRadius * 1000.0;
    var asked = programFilament(job.source);
    check(Math.abs(fedMillimetres - asked) <= 0.01 * asked, 'the extruder feeds ${fedMillimetres} mm of filament, the program asks for ${asked} mm');
    var slow = fedByLayer[4], fast = fedByLayer[2];
    check(fast > 0.001, 'layer 2 takes filament: ${fast * 1000.0} mm');
    check(Math.abs(slow - fast) <= 0.02 * fast, 'the layer printed at half speed takes ${slow * 1000.0} mm of filament, the same layer at full speed ${fast * 1000.0} mm');
    // The panel shows the print: its layer and temperatures in the status, and the controls a CNC program has.
    check(detail.indexOf('layer 2/${player.layerCount}') >= 0 && detail.indexOf("hotend 2") >= 0 && detail.indexOf("bed 6") >= 0,
      'the print status names the layer and the temperatures: $detail');
    var fonts = FontCollection.create();
    fonts.add("haxeon/packages/ui/vendor/harfbuzz/perf/fonts/Roboto-Regular.ttf");
    var theme = Theme.dark();
    var ui = new UiContext(null, fonts, theme);
    function find(node:RenderNode, key:String):Null<RenderNode> {
      if (node.styleKey == key) return node;
      for (child in node.children) {
        var found = find(child, key);
        if (found != null) return found;
      }
      return null;
    }
    var shown = ui.submit(new app.editor.CncPanel().build(simulation, theme.tokens), new LayoutFrame(480.0, 720.0));
    check(find(shown, "cnc-hold") != null && find(shown, "cnc-speed") != null, "the panel offers hold and a speed override for a print");
    // What the nozzle laid, measured on the printed material: the tab's volume, its box, and nothing beyond its footprint
    // and a bead (the skirt and the purge line are away from it).
    var laid = player.stock;
    if (laid == null) throw "the print laid no material";
    var tab = measurePrinted(laid.stock, 0.105, 0.115, 0.105, 0.115, 0.0005, laid.center);
    var cad = 10.0 * 10.0 * 1.2 * 1e-9;
    check(Math.abs(tab.volume - cad) <= 0.03 * cad, 'the tab holds ${tab.volume * 1e9} mm^3 of material, its CAD volume is ${cad * 1e9}');
    for (axis in 0...3) {
      var wanted = [[0.105, 0.115], [0.105, 0.115], [0.0, 0.0012]][axis];
      check(Math.abs(tab.low[axis] - wanted[0]) <= 0.0002 && Math.abs(tab.high[axis] - wanted[1]) <= 0.0002,
        'the tab spans ${tab.low[axis] * 1000}..${tab.high[axis] * 1000} mm on axis $axis, CAD ${wanted[0] * 1000}..${wanted[1] * 1000}');
    }
    check(tab.outside <= 1e-12, 'nothing is laid next to the tab but the skirt: ${tab.outside * 1e9} mm^3');
    var skirt = laid.deposited - tab.volume;
    check(skirt > 0 && laid.beads > 100, 'the skirt and the purge line were laid too ($skirt m^3 in ${laid.beads} beads)');
    var checks = player.planChecks();
    check(checks.count(PlanDiagnosticKind.StepperStall) == 0, 'no plan of the print stalls a stepper (${checks.count(PlanDiagnosticKind.StepperStall)} findings)');
    check(end.safety == 0 && end.faultCode == 0, "the print ends with the runtime ready and no fault");
    Sys.println('printer: SmallTab printed in ${Math.round(ticks * dt * 10) / 10} s (heat-up: hotend ${Math.round(hotendAt * 10) / 10} s, bed ${Math.round(bedAt * 10) / 10} s), ' +
      '${player.layerCount} layers, filament ${Math.round(fedMillimetres * 100) / 100} of ${Math.round(asked * 100) / 100} mm, ' +
      'a layer takes ${Math.round(fast * 1e5) / 100} mm of filament at full speed and ${Math.round(slow * 1e5) / 100} mm at half; ' +
      'plan checks: ${checks.plans} plans, ${checks.flagged} flagged, worst torque ${Math.round(checks.worstTorqueRatio * 100)} % (${checks.worstMotor}), ' +
      'worst deviation ${Math.round(checks.worstDeviation * 1e6)} um (${checks.worstAxis}); ' +
      'motion cost ${Math.round(player.motionSeconds * 10) / 10} s of ${Math.round(wallSeconds * 10) / 10} s wall, slowest tick ${Math.round(player.slowestUpdate * 1000)} ms, ' +
      'the program started in ${Math.round(player.runSeconds * 1000)} ms; the slowest simulated tick took ${Math.round(slowestStep * 1000)} ms, at ${Math.round(slowestAt * 10) / 10} s');
    Sys.println('printer: the print laid ${Math.round(laid.deposited * 1e10) / 10} mm^3 in ${laid.beads} beads for ${Math.round(asked * 2.405 * 10) / 10} mm^3 of filament; the tab holds ' +
      '${Math.round(tab.volume * 1e10) / 10} mm^3 of its CAD ${Math.round(cad * 1e10) / 10} (${Math.round((tab.volume / cad - 1.0) * 1000) / 10} %), spans ' +
      '${Math.round(tab.low[0] * 1e4) / 10}..${Math.round(tab.high[0] * 1e4) / 10} x ${Math.round(tab.low[1] * 1e4) / 10}..${Math.round(tab.high[1] * 1e4) / 10} x ' +
      '${Math.round(tab.low[2] * 1e4) / 10}..${Math.round(tab.high[2] * 1e4) / 10} mm, ${Math.round(tab.outside * 1e12) / 1000} mm^3 beside it; ' +
      'laying ${Math.round(player.depositSeconds * 10) / 10} s, meshing ${Math.round(player.meshingSeconds * 10) / 10} s');
  }

  static function checkCoreXyPlotter(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/corexy/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var definition = session.projectAssemblyDefinition, physical = session.projectPhysical;
    if (definition == null || physical == null) throw "the plotter has no assembly to build a robot from";
    var model = AssemblySimulationBridge.toRobotModel(definition, physical, session.projectAssemblyState).model;
    check(model.couplings.length == 16 && model.actuators.length == 2, 'the plotter has 16 belt couplings and two motors, got ${model.couplings.length} and ${model.actuators.length}');
    var leaders = new Map<String, Int>();
    for (coupling in model.couplings) leaders.set(coupling.follower, (leaders.exists(coupling.follower) ? leaders.get(coupling.follower) : 0) + 1);
    check(leaders.get("pulleyA-turn") == 2 && leaders.get("pulleyB-turn") == 2 && leaders.get("idlerAStart-turn") == 1,
      "each motor's pulley follows two axes, a gantry idler one");
    var compiled = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    check(compiled.couplings.length == 16, "the runtime blueprint carries a coupling for each term");
    var xLimits = model.coupledLimits("x"), yLimits = model.coupledLimits("y");
    check(Math.abs(xLimits.requireVelocity() - 0.6496) < 1e-3 && Math.abs(xLimits.requireVelocity() - yLimits.requireVelocity()) < 1e-12,
      'both axes get 650 mm/s from the two motors, got ${xLimits.requireVelocity()} and ${yLimits.requireVelocity()}');
    check(xLimits.requireAcceleration() > yLimits.requireAcceleration() && yLimits.requireAcceleration() > 20,
      'the carriage accelerates harder than the gantry it rides: ${xLimits.requireAcceleration()} against ${yLimits.requireAcceleration()}');
    Sys.println('corexy plotter: axes to ${Math.round(xLimits.requireVelocity() * 1e4) / 10} mm/s and ${Math.round(xLimits.requireAcceleration() * 1e3) / 1e3} m/s²');
    var world = new RobotWorld();
    var simulation = new ApplicationSimulation(world);
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    check(simulation.rebuild(session.sensors, session.scene, session), "the plotter builds in the shared simulation: " + simulation.error);
    var player = simulation.programPlayer();
    if (player == null) throw "the generated plotter has no app program player";
    check(generated.machineMotion != null && generated.machineMotion.virtualDevice == true && generated.machineMotion.belts.length == 2,
      "the plotter artifact carries its two physical belt routes");
    simulation.capturePresentationSnapshot();
    var initialDisplays = simulation.beltDisplayUpdates();
    var moving = false;
    var tick = 0;
    while (player.passes == 0 && player.failure == null && tick++ < 1000) {
      simulation.step();
      var seen = world.snapshot().robots()[0];
      for (joint in 0...model.joints.length)
        if (model.joints[joint].id == "x" && Math.abs(seen.positions.get(joint)) > 0.01) moving = true;
      if (tick % 40 == 0) simulation.capturePresentationSnapshot();
    }
    check(player.failure == null && player.passes > 0 && moving,
      'the plotter plays its square and diagonals in the app (${player.failure}, $tick ticks)');
    check(simulation.beltDisplayUpdates() > initialDisplays, "belt geometry follows the presented moving pose");
    check(simulation.reset(), "the plotter simulation resets");
    check(player.passes == 0, "reset clears the program's pass count");
    var observed = world.snapshot().robots();
    check(observed.length == 1, "the plotter is the one robot in the world");
    var worst = 0.0;
    for (joint in 0...model.joints.length) worst = Math.max(worst, Math.abs(observed[0].positions.get(joint)));
    check(worst < 1e-3, 'reset restores the plotter to home, its joints at most $worst from home');
    for (_ in 0...25) simulation.step();
    check(player.motion.running && player.failure == null, "the plotter's program restarts after reset");
    simulation.dispose();
    session.dispose();
  }

  static function checkMobileBase(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/mobile-base/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var limitSession = new ProjectDocumentSession(null, false);
    limitSession.openGeneratedProject(generated, manifest);
    var wheelSection = limitSession.mobileBase;
    if (wheelSection == null) throw "the mobile base project should declare its drive";
    var wheels = checkLimitsFromDrives(limitSession, [wheelSection.leftWheel, wheelSection.rightWheel], "the mobile base");
    var wheelLimits = wheels.get(wheelSection.leftWheel);
    // A NEMA 23 on 24 V turns 137 rad/s with half its holding torque (0.63 N m); the 10:1 gearhead at 90% makes that 13.7 rad/s and 5.7 N m.
    check(wheelLimits != null && Math.abs(wheelLimits[0] - 13.71) < 0.02 && Math.abs(wheelLimits[1] - 5.67) < 0.01,
      'the wheels run to ${wheelLimits == null ? 0 : wheelLimits[0]} rad/s and ${wheelLimits == null ? 0 : wheelLimits[1]} N m');
    limitSession.dispose();
    var drive = generated.mobileBase;
    if (drive == null) throw "the mobile base project should declare its drive";
    var section:materia.project.SceneArtifact.SceneArtifactMobileBase = cast drive;
    var wheelRadius:Float = section.wheelRadius;
    // Driven by hand here: without its mission the robot waits for commands.
    check(generated.missions != null, "the mobile base cell ships a mission");
    generated.missions = null;
    function pose(simulation:ApplicationSimulation, id:String):{position:Array<Float>, rotation:Array<Float>} {
      for (entry in simulation.capturePresentationSnapshot().environment) if (entry.id == id)
        return {position: entry.position, rotation: entry.rotation};
      throw 'mobile base has no simulated pose for $id';
    }
    function yaw(rotation:Array<Float>):Float
      return Math.atan2(2 * (rotation[3] * rotation[2] + rotation[0] * rotation[1]),
        1 - 2 * (rotation[1] * rotation[1] + rotation[2] * rotation[2]));
    for (backend in [ApplicationSimulation.DETERMINISTIC, ApplicationSimulation.MUJOCO]) {
      var label = backend == ApplicationSimulation.MUJOCO ? "MuJoCo" : "deterministic";
      var session = new ProjectDocumentSession(null, false);
      session.openGeneratedProject(generated, manifest);
      check(session.mobileBase != null, "opening the mobile base project installs its drive");
      var world = new RobotWorld();
      var simulation = new ApplicationSimulation(world);
      simulation.setBackend(backend);
      check(simulation.rebuild(session.sensors, session.scene, session),
        'mobile base builds on the $label backend: ${simulation.error}');
      var base = simulation.mobileBase();
      if (base == null) throw 'mobile base has no drive on the $label backend';
      var id = "assembly:" + (cast generated.assemblyDefinition:AssemblyDefinition).id;
      var odometry:robotkit.mobile.DifferentialOdometry = base.driveModel.createOdometry();
      function step(seconds:Float):Void {
        var until = simulation.activeSession().simulationTime() + seconds - 1e-9;
        while (simulation.activeSession().simulationTime() < until) {
          simulation.step();
          var observed = world.snapshot().robot(id);
          if (observed == null) throw "mobile base is missing from the world snapshot";
          odometry.update(observed);
        }
      }
      step(0.1);
      var startPlate = pose(simulation, "project:robot/basePlate"), startLidar = pose(simulation, "project:robot/lidar");
      // One second straight ahead at 0.4 m/s.
      base.command(new robotkit.mobile.Twist2(0.4, 0.0));
      var t0 = simulation.activeSession().simulationTime();
      step(1.0);
      var travelled = simulation.activeSession().simulationTime() - t0;
      var plate = pose(simulation, "project:robot/basePlate"), lidar = pose(simulation, "project:robot/lidar");
      var dx = plate.position[0] - startPlate.position[0];
      check(Math.abs(dx - 0.4 * travelled) < 0.02 && Math.abs(plate.position[1] - startPlate.position[1]) < 1e-3 &&
        Math.abs(plate.position[2] - startPlate.position[2]) < 1e-3,
        '$label: the chassis rolls 0.4 m/s straight ahead, moved ${dx} m in ${travelled} s');
      check(Math.abs(lidar.position[0] - startLidar.position[0] - dx) < 1e-3 &&
        Math.abs(lidar.position[2] - startLidar.position[2]) < 1e-3, '$label: the lidar rides the chassis');
      var observed = world.snapshot().robot(id);
      if (observed == null) throw "mobile base is missing from the world snapshot";
      var wheelRate = 0.4 / wheelRadius;
      var leftRate = observed.velocities.get(odometry.leftWheelJoint);
      var rightRate = observed.velocities.get(odometry.rightWheelJoint);
      check(odometry.leftDirection == 1 && odometry.rightDirection == -1,
        '$label: the authored wheel shafts point outward');
      check(Math.abs(leftRate - wheelRate) < 0.05 * wheelRate &&
        Math.abs(rightRate + wheelRate) < 0.05 * wheelRate,
        '$label: the wheels spin at v / r, the right one negative (${leftRate}, ${rightRate}; ' +
          'CAD joint indices ${odometry.leftWheelJoint}, ${odometry.rightWheelJoint})');
      // Half a second turning in place at 1 rad/s.
      var startYaw = yaw(plate.rotation);
      base.command(new robotkit.mobile.Twist2(0.0, 1.0));
      t0 = simulation.activeSession().simulationTime();
      step(0.5);
      var turned = simulation.activeSession().simulationTime() - t0;
      var spun = pose(simulation, "project:robot/basePlate");
      check(Math.abs(yaw(spun.rotation) - startYaw - turned) < 0.05 &&
        Math.abs(spun.position[0] - plate.position[0]) < 1e-3,
        '$label: the chassis turns in place counter-clockwise, ${yaw(spun.rotation) - startYaw} rad in ${turned} s');
      base.stop();
      step(0.1);
      var estimate = odometry.current();
      check(Math.abs(estimate.x - dx) < 0.02 && Math.abs(estimate.yaw - (yaw(spun.rotation) - startYaw)) < 0.05,
        '$label: wheel odometry follows the chassis (${estimate.x} m, ${estimate.yaw} rad)');
      Sys.println('mobile base ($label): ${Math.round(dx * 1000)} mm in ${Math.round(travelled * 100) / 100} s, ' +
        'turned ${Math.round((yaw(spun.rotation) - startYaw) * 1000) / 1000} rad, odometry ${Math.round(estimate.x * 1000)} mm');
      simulation.clear();
    }
  }

  /**
   * The mobile manipulator runs its round on its own: it stands where each goTo says, picks the
   * workpiece up, sets it down centred on the other table's seat and carries it back, and its chassis
   * never touches the room's walls, tables, pillar or dock. MuJoCo only, like the arm example's own
   * check: the test backend's contacts between robot links and free objects reach too far for parts
   * handled this closely, and push the workpiece off its table as the arm passes.
   */
  static function checkMobileMission(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/mobile-base/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var section:materia.project.SceneArtifact.SceneArtifactMobileBase = cast generated.mobileBase;
    var work:materia.project.SceneArtifact.SceneArtifactMission = cast generated.missions[0];
    var steps = work.steps;
    var definition = requiredAssemblyDefinition(generated.assemblyDefinition);
    var placement = new AssemblyState(definition, generated.assemblyState);
    var halfLength:Float = cast section.footprintLength, halfWidth:Float = cast section.footprintWidth;
    halfLength /= 2; halfWidth /= 2;
    function yaw(rotation:Array<Float>):Float
      return Math.atan2(2 * (rotation[3] * rotation[2] + rotation[0] * rotation[1]),
        1 - 2 * (rotation[1] * rotation[1] + rotation[2] * rotation[2]));
    /** Separating-axis test of two rectangles on the floor: centre, half extents and heading. */
    function overlap(ax:Float, ay:Float, ahx:Float, ahy:Float, ayaw:Float,
        bx:Float, by:Float, bhx:Float, bhy:Float, byaw:Float):Bool {
      for (angle in [ayaw, ayaw + Math.PI / 2, byaw, byaw + Math.PI / 2]) {
        var ux = Math.cos(angle), uy = Math.sin(angle);
        function reach(hx:Float, hy:Float, heading:Float):Float
          return hx * Math.abs(Math.cos(heading) * ux + Math.sin(heading) * uy) +
            hy * Math.abs(-Math.sin(heading) * ux + Math.cos(heading) * uy);
        var gap = Math.abs((bx - ax) * ux + (by - ay) * uy);
        if (gap > reach(ahx, ahy, ayaw) + reach(bhx, bhy, byaw)) return false;
      }
      return true;
    }
    for (backend in [ApplicationSimulation.MUJOCO]) {
      var label = backend == ApplicationSimulation.MUJOCO ? "MuJoCo" : "deterministic";
      var session = new ProjectDocumentSession(null, false);
      session.openGeneratedProject(generated, manifest);
      var simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      check(simulation.rebuild(session.sensors, session.scene, session),
        'mobile base cell builds on the $label backend: ${simulation.error}');
      var mission = simulation.missionPlayer();
      if (mission == null) throw 'mobile base cell has no mission on the $label backend';
      check(mission.obstacles.length == 8, '$label: the map has the four walls, two tables, pillar and dock');
      var log:Array<String> = [];
      var lastDone = 0;
      var trail:Array<String> = [];
      while (mission.completed < steps.length && simulation.activeSession().simulationTime() < 300) {
        simulation.step();
        var failure = mission.failure;
        if (failure != null) throw '$label: the mission failed after ${mission.completed} steps: $failure\n${trail.join("\n")}';
        var poses = simulation.capturePresentationSnapshot().environment;
        var plate = [for (entry in poses) if (entry.id == "project:robot/basePlate") entry][0];
        var heading = yaw(plate.rotation);
        trail.push('${Math.round(simulation.activeSession().simulationTime() * 100) / 100} s: ' +
          '${Math.round(plate.position[0] * 1000)}, ${Math.round(plate.position[1] * 1000)} mm, ' +
          '${Math.round(heading * 1000) / 1000} rad, step ${mission.stepIndex}');
        if (trail.length > 150) trail.shift();
        for (box in mission.obstacles) if (overlap(plate.position[0], plate.position[1], halfLength, halfWidth, heading,
            box.x, box.y, box.halfX, box.halfY, box.yaw))
          throw '$label: the chassis hits ${box.id} at ${plate.position[0]}, ${plate.position[1]}\n' +
            [for (index in 0...trail.length) if (index % 5 == 0) trail[index]].join("\n");
        if (mission.completed == lastDone) continue;
        var step = steps[lastDone];
        var now = Math.round(simulation.activeSession().simulationTime() * 10) / 10;
        switch step.kind {
          case "goTo":
            var goal:materia.project.SceneArtifact.SceneArtifactFloorPose = cast step.pose;
            var error = Math.sqrt(Math.pow(plate.position[0] - goal.x, 2) + Math.pow(plate.position[1] - goal.y, 2));
            var turn = Math.abs(Math.atan2(Math.sin(heading - goal.yaw), Math.cos(heading - goal.yaw)));
            check(error < 0.08 && turn < 0.08, '$label: step $lastDone (goTo) ends ${error} m and ${turn} rad off');
          case "pick":
            check(simulation.heldObjectIds().join(",") == "project:workpiece", '$label: step $lastDone picks the workpiece up');
          case "place":
            check(simulation.heldObjectIds().length == 0, '$label: step $lastDone lets the workpiece go');
            // Let it settle, then it should rest centred on the seat.
            for (_ in 0...50) simulation.step();
            var at:materia.project.SceneArtifact.SceneArtifactPlace = cast step.at;
            var seat = placement.worldConnector(at.occurrence, at.connector);
            var metres = generated.metresPerUnit;
            var piece = [for (entry in simulation.capturePresentationSnapshot().environment)
              if (entry.id == "project:workpiece") entry][0];
            var off = Math.sqrt(Math.pow(piece.position[0] - seat.x * metres, 2) + Math.pow(piece.position[1] - seat.y * metres, 2));
            check(off < 0.02, '$label: step $lastDone sets the workpiece ${off} m from the centre of ${at.occurrence}\'s seat');
            check(piece.position[2] > seat.z * metres && piece.position[2] < seat.z * metres + 0.05,
              '$label: step $lastDone leaves the workpiece standing on ${at.occurrence} (centre at ${piece.position[2]} m)');
          default:
        }
        log.push('${step.kind} ${now}');
        lastDone = mission.completed;
      }
      check(mission.completed >= steps.length,
        '$label: the robot runs its whole round in five minutes, finished ${mission.completed} of ${steps.length} steps');
      Sys.println('mobile mission ($label): ${log.join(", ")} s');
      simulation.clear();
    }
  }

  /**
   * The base sees what the room's map lacks. A box dropped on the route ahead of the moving base, as if it
   * had fallen off a shelf, is read off the lidar: the base slows, stops short of it without touching it,
   * replans round it, and still completes its round (so the lidar never mistakes the room for obstacles
   * either). MuJoCo, like the mission check.
   */
  static function checkMobileObstacle(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/mobile-base/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var section:materia.project.SceneArtifact.SceneArtifactMobileBase = cast generated.mobileBase;
    var work:materia.project.SceneArtifact.SceneArtifactMission = cast generated.missions[0];
    var halfLength:Float = cast section.footprintLength, halfWidth:Float = cast section.footprintWidth;
    halfLength /= 2; halfWidth /= 2;
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    check(session.robotSensors.length == 1 && session.robotSensors[0].kind == "lidar",
      "the mobile base cell mounts one lidar on the robot");
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    check(simulation.rebuild(session.sensors, session.scene, session), 'mobile base cell builds: ${simulation.error}');
    var mission = simulation.missionPlayer();
    if (mission == null) throw "mobile base cell has no mission";
    check(mission.guard != null, "a mission with a lidar guards the base's motion");
    var active = simulation.activeSession();
    if (active == null) throw "mobile base cell has no session";
    var dt = simulation.timestep;
    function yaw(rotation:Array<Float>):Float
      return Math.atan2(2 * (rotation[3] * rotation[2] + rotation[0] * rotation[1]),
        1 - 2 * (rotation[1] * rotation[1] + rotation[2] * rotation[2]));
    /** The chassis' pose on the floor: x, y, heading. */
    function chassis():Array<Float> {
      var plate = [for (entry in simulation.capturePresentationSnapshot().environment) if (entry.id == "project:robot/basePlate") entry][0];
      return [plate.position[0], plate.position[1], yaw(plate.rotation)];
    }
    /** Gap between two rectangles on the floor (centre, half extents, heading), negative when they overlap. */
    function gap(a:Array<Float>, ahx:Float, ahy:Float, b:Array<Float>, bhx:Float, bhy:Float):Float {
      var worst = Math.NEGATIVE_INFINITY;
      for (angle in [a[2], a[2] + Math.PI / 2, b[2], b[2] + Math.PI / 2]) {
        var ux = Math.cos(angle), uy = Math.sin(angle);
        function reach(hx:Float, hy:Float, heading:Float):Float
          return hx * Math.abs(Math.cos(heading) * ux + Math.sin(heading) * uy) +
            hy * Math.abs(-Math.sin(heading) * ux + Math.cos(heading) * uy);
        var separation = Math.abs((b[0] - a[0]) * ux + (b[1] - a[1]) * uy) - reach(ahx, ahy, a[2]) - reach(bhx, bhy, b[2]);
        if (separation > worst) worst = separation;
      }
      return worst;
    }
    var trail:Array<String> = [];
    function step():Void {
      simulation.step();
      var failure = mission.failure;
      if (failure != null) throw 'the mission failed after ${mission.completed} steps: $failure\n${trail.join("\n")}';
    }
    // Cruise down the first leg.
    step();
    var before = chassis();
    var speed = 0.0;
    while (speed < 0.4 && active.simulationTime() < 20) {
      step();
      var now = chassis();
      speed = Math.sqrt(Math.pow(now[0] - before[0], 2) + Math.pow(now[1] - before[1], 2)) / dt;
      before = now;
    }
    check(speed >= 0.4 && mission.stepIndex == 0, 'the base cruises down its first leg (${speed} m/s)');
    var cruise = speed;
    // The route ahead of the base: drop the box on it, close enough that braking is called for.
    var overlay = mission.overlay();
    if (overlay == null) throw "a driving mission draws its overlay";
    var route = overlay.route;
    var nearest = 0;
    for (index in 0...route.length) {
      var near = Math.pow(route[nearest].x - before[0], 2) + Math.pow(route[nearest].y - before[1], 2);
      if (Math.pow(route[index].x - before[0], 2) + Math.pow(route[index].y - before[1], 2) < near) nearest = index;
    }
    var travelled = 0.0, spot = route[nearest];
    for (index in nearest + 1...route.length) {
      travelled += Math.sqrt(Math.pow(route[index].x - route[index - 1].x, 2) + Math.pow(route[index].y - route[index - 1].y, 2));
      spot = route[index];
      if (travelled >= DROP_AHEAD) break;
    }
    // The box falls onto the cell's floor slab.
    var half = [0.2, 0.2, 0.25];
    active.stop();
    var box = active.createObject(nativekit.sim.MotionType.Dynamic, nativekit.sim.SimShape.box(half[0], half[1], half[2]),
      new nativekit.sim.SimPose(spot.x, spot.y, half[2] + 0.05, 0, 0, 0, 1), 2.0);
    var dropped = active.simulationTime();
    function boxFloorPose():Array<Float> {
      var frame = active.capture();
      var pose = frame.objectPose(box);
      frame.dispose();
      return [pose.x, pose.y, yaw([pose.qx, pose.qy, pose.qz, pose.qw])];
    }
    var landed:Null<Array<Float>> = null;
    var slowest = cruise, stopped = false, approached = false, replans = 0, closest = Math.POSITIVE_INFINITY;
    var seen = 0;
    var position = chassis();
    var steps = work.steps.length;
    while (mission.completed < steps && active.simulationTime() < 300) {
      step();
      var now = chassis();
      speed = Math.sqrt(Math.pow(now[0] - position[0], 2) + Math.pow(now[1] - position[1], 2)) / dt;
      position = now;
      var guard = mission.guard;
      if (guard != null) switch guard.state {
        case Approaching(_, _, _): approached = true;
        case _:
      }
      if (mission.stepIndex == 0) replans = Std.int(Math.max(replans, mission.replans()));
      var pose = boxFloorPose();
      if (active.simulationTime() > dropped + 0.5) {
        if (landed == null) landed = pose;
        var clear = gap(now, halfLength, halfWidth, pose, half[0], half[1]);
        closest = Math.min(closest, clear);
        if (clear <= 0.0) throw 'the chassis touches the dropped box at ${now[0]}, ${now[1]}\n${trail.join("\n")}';
        if (mission.stepIndex == 0 && clear < 1.5) {
          slowest = Math.min(slowest, speed);
          if (speed < 0.02) {
            stopped = true;
            var view = mission.overlay();
            if (view != null) seen = Std.int(Math.max(seen, view.obstacles.length));
          }
        }
      }
      trail.push('${Math.round(active.simulationTime() * 100) / 100} s: ${Math.round(now[0] * 1000)}, ${Math.round(now[1] * 1000)} mm, ' +
        '${Math.round(speed * 100) / 100} m/s, step ${mission.stepIndex}, replans ${mission.replans()}, ${Std.string(mission.guard == null ? null : mission.guard.state)}, ${mission.sensed.obstacles().length} sensed');
      if (trail.length > 400) trail.shift();
    }
    check(mission.completed >= steps, 'the robot still runs its whole round in five minutes, finished ${mission.completed} of ${steps} steps (${mission.navigating()})\n' +
      [for (index in 0...trail.length) if (index % 8 == 0) trail[index]].join("\n"));
    check(approached && slowest < cruise * 0.5, 'the base slows for the box (${cruise} m/s down to ${slowest} m/s)');
    check(stopped, 'the base stops short of the box (slowest ${slowest} m/s)');
    check(replans > 0, 'the base replans round the box (${replans} replans)');
    check(seen > 0, "the overlay shows the obstacle the lidar added to the costmap");
    var ghost = mission.overlay();
    if (ghost == null || ghost.odometry == null || ghost.outline.length < 3) throw "the overlay carries the odometry ghost";
    // Odometry from the wheel joints follows the base's drive to within a few centimetres over the round.
    var believed:robotkit.mobile.Pose2 = cast ghost.odometry, truth = chassis();
    check(Math.sqrt(Math.pow(believed.x - truth[0], 2) + Math.pow(believed.y - truth[1], 2)) < 0.05,
      'the odometry ghost follows the base (${Math.round(Math.sqrt(Math.pow(believed.x - truth[0], 2) + Math.pow(believed.y - truth[1], 2)) * 1000)} mm off)');
    var rest = boxFloorPose(), settled:Array<Float> = cast landed;
    check(Math.abs(rest[0] - settled[0]) < 0.03 && Math.abs(rest[1] - settled[1]) < 0.03,
      'the base never pushes the box (${Math.round(Math.sqrt(Math.pow(rest[0] - settled[0], 2) + Math.pow(rest[1] - settled[1], 2)) * 1000)} mm)');
    Sys.println('mobile obstacle: cruise ${Math.round(cruise * 100) / 100} m/s, slowest ${Math.round(slowest * 1000) / 1000} m/s, ' +
      'closest ${Math.round(closest * 1000)} mm, ${replans} replans, round done at ${Math.round(active.simulationTime())} s');
    simulation.clear();
  }

  /** The overlay outlines the costmap's blocked area as runs: a 2 x 2 block, a cell of margin all round, is four sides of 2 m. */
  static function checkMissionOverlayEdge():Void {
    var grid = new robotkit.navigation.OccupancyGrid2(0.5, new robotkit.mobile.Pose2(1.0, 2.0), 6, 5, "map",
      robotkit.navigation.OccupancyCell.Free);
    for (x in 2...4) for (y in 1...3) grid.setCell(x, y, robotkit.navigation.OccupancyCell.Occupied);
    var edge = MissionOverlayView.blockedEdge(new robotkit.navigation.Costmap2(grid, 0.0, false, 0.0, 0.0));
    check(edge.length == 16, 'the blocked area has four edge runs (${edge.length / 4})');
    var total = 0.0, low = Math.POSITIVE_INFINITY, high = Math.NEGATIVE_INFINITY;
    for (run in 0...4) {
      var dx = edge[run * 4 + 2] - edge[run * 4], dy = edge[run * 4 + 3] - edge[run * 4 + 1];
      total += Math.sqrt(dx * dx + dy * dy);
      low = Math.min(low, Math.min(edge[run * 4], edge[run * 4 + 2]));
      high = Math.max(high, Math.max(edge[run * 4], edge[run * 4 + 2]));
    }
    check(Math.abs(total - 8.0) < 1e-9 && Math.abs(low - 1.5) < 1e-9 && Math.abs(high - 3.5) < 1e-9,
      'the edge runs round the blocked area in map coordinates (length ${total}, x ${low} to ${high})');
  }

  /** How far along its route (m) the base has the box dropped ahead of it. */
  static inline var DROP_AHEAD:Float = 1.2;

  /**
   * The arm example opens with its suction tool and its mission, and on MuJoCo the arm works it: picks
   * the workpiece off its pad, sets it on the other, and brings it back, each pick confirmed by the
   * tool's vacuum sensor, and a reset puts the workpiece back for the next run.
   */
  static function checkCobotArms(root:String, ?sizes:Array<Int>):Void {
    for (size in sizes == null ? [500, 850, 900, 1300] : sizes) {
      var filename = size == 850 ? "materia.project.json" : 'materia.reach$size.project.json';
      var manifest = FileSystem.fullPath(root + "/machinekit/examples/cobot-arm/" + filename);
      var generated = MateriaProjectRunner.loadProject(manifest);
      check(generated.robotMotions == null || generated.robotMotions.length == 0,
        "the cobot runs its declared mission through the planner");
      var session = new ProjectDocumentSession(null, false);
      session.openGeneratedProject(generated, manifest);
      var joints = ["j1", "j2", "j3", "j4", "j5", "j6"];
      var limits = checkLimitsFromDrives(session, joints, "the cobot");
      var world = new RobotWorld();
      var simulation = new ApplicationSimulation(world);
      simulation.setBackend(ApplicationSimulation.MUJOCO);
      check(simulation.rebuild(session.sensors, session.scene, session), "cobot builds: " + simulation.error);
      var mission = simulation.missionPlayer();
      if (mission == null) throw "The cobot preview has no looping mission";
      var indexes:Array<Int> = [];
      for (name in joints) {
        var found = -1;
        for (i in 0...mission.robot.model.joints.length)
          if (mission.robot.model.joints[i].name == name) found = i;
        check(found >= 0, 'the cobot has joint $name');
        indexes.push(found);
      }
      var worst = 0.0, ticks = 0;
      var peak = [0.0, 0, 0, 0, 0, 0];
      while (mission.completed < 3 && simulation.activeSession().simulationTime() < 90) {
        simulation.step(); ticks++;
        if (mission.failure != null) throw 'Cobot $size mission: ${mission.failure}';
        var seen = mission.robot.robot.snapshot();
        check(seen.faultCode == 0, "the cobot runtime stays healthy");
        for (i in 0...indexes.length) {
          var index = indexes[i];
          var error = Math.abs(seen.positions.get(index) - seen.setpointPositions.get(index));
          peak[i] = Math.max(peak[i], error);
          worst = Math.max(worst, error);
        }
        for (contact in mission.simulation.robotContacts(mission.robot.runtime))
          if (contact.active && contact.distance < -1e-5 &&
              contact.otherKind == robotkit.runtime.RobotContactOtherKind.RobotLink)
            throw 'Cobot $size self collision: links ${contact.linkIndex}, ${contact.otherLink}, ${contact.distance} m';
      }
      check(mission.completed >= 3, 'cobot $size completes its motion: ${mission.completed} steps');
      Sys.println('cobot $size peaks rad: $peak');
      check(worst < 0.001, 'cobot $size joint tracking below 1 mrad, got $worst rad');
      Sys.println('cobot $size: ${simulation.activeSession().simulationTime()} s, peak joint tracking ${worst * 1000} mrad, $ticks ticks');
      simulation.dispose(); session.dispose();
    }
  }

  /** Runs the actual Cartesian picker mission and measures its placement and drive budget. */
  static function checkGantryPicker(root:String, yaw:Bool = false, paced:Bool = false, startup:Null<String> = null):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/gantry-picker/" + (yaw ? "materia.yaw.project.json" : "materia.project.json"));
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition:AssemblyDefinition = cast generated.assemblyDefinition;
    if (startup == "far") {
      // A displaced authored pose retains known counter coordinates. Unknown-origin
      // cold starts outside the runtime search window are checked separately below.
      var displaced = new AssemblyState(definition, generated.assemblyState);
      displaced.setJoint("x", 300); displaced.setJoint("y", 200);
      generated.assemblyState = displaced.record();
    } else if (startup == "unreferenced") {
      if (generated.missions == null) throw "Picker startup test needs a mission";
      generated.missions[0].powerUpOffsets = [{joint: "x", offset: 0.3}, {joint: "y", offset: 0.2}];
    }
    var placement = new AssemblyState(definition, generated.assemblyState);
    var session = new ProjectDocumentSession(null, false);
    var simulation = new ApplicationSimulation(new RobotWorld());
    session.openGeneratedProject(generated, manifest);
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      check(simulation.rebuild(session.sensors, session.scene, session), "gantry picker builds: " + simulation.error);
      var mission = simulation.missionPlayer();
      if (mission == null) throw "gantry picker has no mission";
      var handlingChannel = singleHandlingChannel(mission);
      var initialHandling = handlingRunner(mission, handlingChannel);
      var toolKinematics = mission.toolKinematics.get(handlingChannel);
      if (toolKinematics == null) throw "gantry picker has no tool kinematics";
      var toolLink = toolKinematics.link;
      var motion = initialHandling.motion;
      var model = mission.robot.model;
      check(motion.compiler.solver.jointCount() == (yaw ? 4 : 3), "picker plans XYZ and its selected C head");
      var restoredModel = robotkit.model.RobotModelCodec.decode(robotkit.model.RobotModelCodec.encode(model));
      for (contact in model.switches) if (contact.role == "home") {
        var restored = [for (candidate in restoredModel.switches) if (candidate.id == contact.id) candidate][0];
        check(restored.homeAfter.join("/") == contact.homeAfter.join("/"), "RobotModel codec preserves home dependencies");
        var name = contact.joint.split("/").pop();
        if (name == "x" || name == "y")
          check(contact.homeAfter.length == 1 && contact.homeAfter[0].split("/").pop() == "z",
            "picker retains machine-authored Z clearance dependencies through serialization and runtime compilation");
      }
      for (index in 0...motion.jointIndices.length) {
        var joint = model.joints[motion.jointIndices[index]];
        var coupled = model.coupledLimits(joint.id, new SteadyLoads());
        check(motion.compiler.maxVelocity[index] <= coupled.requireVelocity() + 1e-9,
          "picker planner respects coupled speed on " + joint.id);
        if (coupled.maxAcceleration != null)
          check(motion.compiler.maxAcceleration[index] <= coupled.requireAcceleration() + 1e-9,
            "picker planner respects coupled acceleration on " + joint.id);
        else check([for (assumption in motion.compiler.planningAssumptions)
          if (assumption.indexOf('joint "${joint.id}" acceleration') >= 0) assumption].length == 1,
          "picker states the assumed acceleration for a servo without a derived cap");
      }
      function pose(id:String):app.ApplicationSimulation.SimulationPoseVisual {
        var found = [for (entry in simulation.capturePresentationSnapshot().environment) if (entry.id == id) entry];
        if (found.length != 1) throw "picker has no unique pose for " + id;
        return found[0];
      }
      check(mission.statusLabel() == "Waiting for home switches", "picker explains its startup wait");
      if (paced) simulation.start();
      var lastReport = Sys.time();
      var homingPhases = new Map<String, Float>();
      var lastDone = 0, placed = 0, ticks = 0;
      var bytes = 0.0, measuredTicks = 0;
      var previousPlan:Null<motionkit.trajectory.ExecutionPlan> = null;
      var planned = 0;
      var supports:Array<{object:Int, x:Float, y:Float, z:Float}> = [];
      for (index in 0...6) {
        var carton = simulation.environmentObject("project:box" + index);
        if (carton == null) throw "picker has no carton body";
        for (seatId in ["infeedSeat" + index, "slot" + index]) {
          var seat = placement.worldConnector(seatId, "top");
          supports.push({object: cast carton.handle, x: seat.x * generated.metresPerUnit,
            y: seat.y * generated.metresPerUnit, z: seat.z * generated.metresPerUnit});
        }
      }
      while (!mission.finished && simulation.activeSession().simulationTime() < 600) {
        var homeView = @:privateAccess mission.homing;
        if (homeView != null && homeView.homingStatus() != "Complete") {
          var key = homeView.homingAxis() + ":" + homeView.homingStatus();
          homingPhases.set(key, (homingPhases.exists(key) ? homingPhases.get(key) : 0.0) + simulation.timestep);
        }
        var before = hl.Gc.totalAllocated();
        var priorPlan = motion.executor.plan;
        if (paced) {
          simulation.runTicks(1);
          Sys.sleep(simulation.timestep);
          if (Sys.time() - lastReport >= 5) {
            lastReport = Sys.time();
            Sys.println('picker realtime: t=${simulation.activeSession().simulationTime()} completed=${mission.completed} homing=${mission.homingSeconds} status=${mission.statusLabel(true)} failure=${mission.failure}');
          }
        } else simulation.step();
        var currentHandling = handlingRunner(mission, handlingChannel);
        motion = currentHandling.motion;
        var allocated = hl.Gc.totalAllocated() - before;
        ticks++;
        if (mission.failure != null) {
          var failed = mission.robot.runtime.snapshot();
          if (startup == "unreferenced") {
            check(mission.failure.indexOf("search reached its travel boundary without a switch") >= 0,
              "Unknown-origin startup outside the coordinate search window faults safely");
            simulation.clear(); session.dispose();
            Sys.println("picker unreferenced startup: bounded search refusal passed");
            return;
          }
          throw 'gantry picker mission: ${mission.failure}; time=${simulation.activeSession().simulationTime()} names=${mission.robot.robot.description().joints} offsets=${[for (joint in 0...failed.q.length) mission.robot.runtime.referenceOffset(joint)]} q=${failed.q.toArray()} setpoint=${mission.robot.robot.snapshot().setpointPositions.toArray()}';
        }
        var plan = motion.executor.plan;
        // Planning happens on a worker. Measure ordinary execution ticks separately from plan admission.
        if (priorPlan != null && plan == priorPlan) { bytes += allocated; measuredTicks++; }
        if (plan != null && plan != previousPlan) {
          previousPlan = plan; planned++;
          check(plan.checked != null, "every picker plan has a drive check");
          for (sample in 0...101) {
            var state = plan.evaluate(plan.durationSeconds * sample / 100);
            for (axis in 0...state.velocities.length)
              check(Math.abs(state.velocities[axis]) <= motion.compiler.maxVelocity[axis] * 1.000001 + 1e-9,
                "picker planned speed remains within its coupled limit");
          }
        }
        for (contact in mission.simulation.robotContacts(mission.robot.runtime)) {
          if (!contact.active || contact.distance >= -0.0005) continue;
          // Tables and seats belong to the fixed root. Cartons settle on their
          // own authored pads; allow only shallow, vertical support there. The
          // fixed root reports it against the carton, and a held carton reports
          // the same support as a payload load on its carrying link.
          var supported = false;
          var supportDepth = currentHandling.pressDepth + 0.002;
          var carton = contact.carriedObject != 0 ? contact.carriedObject
            : contact.linkIndex == 0 && contact.otherKind == robotkit.runtime.RobotContactOtherKind.Object ? contact.otherObject : 0;
          var onFixedRoot = contact.carriedObject == 0 ||
            (contact.otherKind == robotkit.runtime.RobotContactOtherKind.RobotLink && contact.otherLink == 0);
          if (carton != 0 && onFixedRoot && contact.distance >= -supportDepth && Math.abs(contact.normal.z) >= 0.99) {
            for (seat in supports) if (carton == seat.object &&
                Math.abs(contact.position.x - seat.x) <= 0.0451 &&
                Math.abs(contact.position.y - seat.y) <= 0.0451 &&
                Math.abs(contact.position.z - seat.z) <= supportDepth) supported = true;
          }
          if (supported) continue;
          if (contact.linkIndex != toolLink) {
            var body = mission.simulation.linkPose(0, contact.linkIndex);
            for (hull in mission.robot.shapes) if (hull.link == contact.linkIndex) {
              var lo = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
              var hi = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
              var points = hull.shape.convexVertices();
              for (vertex in 0...Std.int(points.length / 3)) {
                var point = rotateVector(points[vertex * 3], points[vertex * 3 + 1],
                  points[vertex * 3 + 2], body.rotation);
                for (axis in 0...3) {
                  lo[axis] = Math.min(lo[axis], point[axis] + body.position[axis]);
                  hi[axis] = Math.max(hi[axis], point[axis] + body.position[axis]);
                }
              }
              var point = [contact.position.x, contact.position.y, contact.position.z];
              var near = true;
              for (axis in 0...3) if (point[axis] < lo[axis] - 0.002 || point[axis] > hi[axis] + 0.002) near = false;
              if (near) Sys.println("picker contact candidate: " + hull.shape.name + "; bounds=" + lo.join(",") + " / " + hi.join(","));
            }
          }
          // The compliant cup deliberately presses at most 3 mm into its intended grasp surface.
          check(contact.linkIndex == toolLink && contact.otherKind == robotkit.runtime.RobotContactOtherKind.Object &&
            contact.distance >= -0.004, "picker has no unintended robot contact: link=" + contact.linkIndex +
              " (" + model.links[contact.linkIndex].id + "), otherKind=" + contact.otherKind +
              ", otherLink=" + contact.otherLink + ", object=" + contact.otherObject +
              ", distance=" + contact.distance + ", point=" + contact.position.x + "," + contact.position.y + "," + contact.position.z +
              "; time=" + simulation.activeSession().simulationTime() + "; completed=" + mission.completed);
        }
        var held = simulation.heldObjectIds();
        check(held.length <= 1, "picker holds at most one carton");
        if (mission.completed == lastDone) continue;
        check(mission.completed == lastDone + 1, "picker completes each step individually");
        var step = mission.mission.steps[lastDone];
        if (step.kind == "pick") {
          var at = step.at;
          if (at == null) throw "picker pick has no carton";
          check(held.join(",") == "project:" + at.occurrence, "picker picks the intended carton");
        } else {
          check(held.length == 0, "picker releases each carton");
          for (_ in 0...50) {
            if (paced) { simulation.runTicks(1); Sys.sleep(simulation.timestep); }
            else simulation.step();
          }
          var at = step.at;
          if (at == null) throw "picker place has no slot";
          var seat = placement.worldConnector(at.occurrence, at.connector);
          var box = pose("project:box" + placed);
          var scale = generated.metresPerUnit;
          var error = Math.sqrt(Math.pow(box.position[0] - seat.x * scale, 2) +
            Math.pow(box.position[1] - seat.y * scale, 2) +
            Math.pow(box.position[2] - (seat.z * scale + 0.025), 2));
          check(error <= 0.002, "picker carton is within 2 mm of its slot: " + error);
          var targetYaw:Float = step.yaw == null ? 0.0 : step.yaw;
          var angle = new robotkit.spatial.Quat(box.rotation[0], box.rotation[1], box.rotation[2], box.rotation[3])
            .angularDistance(robotkit.spatial.Quat.fromAxisAngle(new robotkit.spatial.Vec3(0, 0, 1), targetYaw));
          check(angle <= 2 * Math.PI / 180, "picker carton is within 2 degrees of its slot");
          placed++;
        }
        lastDone = mission.completed;
      }
      check(mission.finished && mission.completed == 12 && placed == 6, "picker places all six cartons: completed=" +
        mission.completed + ", placed=" + placed + ", plans=" + planned + ", homing=" + mission.homingSeconds +
        ", time=" + simulation.activeSession().simulationTime() + ", motion=" + motion.sessionState() +
        ", q=" + mission.robot.runtime.snapshot().q.toArray().join(","));
      check(mission.statusLabel() == "Mission complete", "picker reports mission completion");
      check(mission.homingSeconds > 0 && mission.homingSeconds < (startup == "far" ? 100 : 15),
        "picker homes within its startup simulation budget without long backoff overruns: " + mission.homingSeconds);
      check(planned > 0 && motion.checks.plans > 0, "picker executes physically checked plans");
      check(motion.checks.count(PlanDiagnosticKind.StepperStall) == 0, "picker drive checks report no stall");
      check(measuredTicks > 0 && bytes / measuredTicks < 200000, "picker allocates below its assumed 200 KB execution-tick budget");
      var phaseNames = [for (key in homingPhases.keys()) key];
      phaseNames.sort(Reflect.compare);
      for (key in phaseNames) Sys.println("picker homing " + key + ": " + homingPhases.get(key) + " s");
      Sys.println((yaw ? "gantry C-head yaw picker mission: " : "gantry picker mission: ") + simulation.activeSession().simulationTime() + " s, six cartons; homing " + mission.homingSeconds + " s; " +
        Math.round(bytes / measuredTicks) + " bytes per execution tick; " + planned + " plans, no stalls");
      simulation.clear(); session.dispose();
    } catch (error:Dynamic) { simulation.clear(); session.dispose(); throw error; }
  }

  /** PP1: model extraction and all legal branches on the authored gantry examples. */
  static function checkNativeCandidateSet(numeric:motionkit.robot.ManipulatorKinematics,target:motionkit.kinematics.Pose3,
      q:Array<Float>,policy:motionkit.path.OrientationPolicy,candidates:Array<motionkit.robot.CartesianCandidateSampler.LatticeCandidate>):Void {
    check(candidates.length>0,'authored native sampler returns candidates for $policy at q=$q');
    var found=false;
    for(candidate in candidates) {
      var same=true;
      for(joint in 0...q.length)if(Math.abs(candidate.q[joint]-q[joint])>1e-5)same=false;
      found=found || same;
      var actual=numeric.forward(candidate.q);
      check(motionkit.path.PoseMath.distance(actual,target)<1e-6,"authored native candidate preserves mounted TCP position");
      check(motionkit.robot.ToolFreedom.orientationError(actual,target,policy)<1e-6,"authored native candidate satisfies orientation freedom");
      for(joint in 0...q.length) {
        var bound=numeric.manipulator.group.limitsOf(joint);
        check(candidate.q[joint]>=bound.lower-1e-9 && candidate.q[joint]<=bound.upper+1e-9,"authored native candidate respects compiled bounds");
      }
    }
    switch policy {case Cone(_, _):default:check(found,"authored native candidate set retains original legal configuration");}
  }

  static function checkCartesianAnalyticExamples(root:String):Void {
    for (fixture in [{example: "gantry-picker", file: "materia.project.json"},
        {example: "gantry-picker", file: "materia.yaw.project.json"},
        {example: "gantry-welder", file: "materia.project.json"}]) {
      var manifest = FileSystem.fullPath(root + "/machinekit/examples/" + fixture.example + "/" + fixture.file);
      var generated = MateriaProjectRunner.loadProject(manifest);
      var session = new ProjectDocumentSession(null, false);
      session.openGeneratedProject(generated, manifest);
      var simulation = new ApplicationSimulation(new RobotWorld());
      try {
        check(simulation.rebuild(session.sensors, session.scene, session), "analytic gantry model builds: " + simulation.error);
        var mission = simulation.missionPlayer();
        if (mission == null) throw "analytic gantry has no mission group";
        var motion:motionkit.robot.ManipulatorMotion;
        if (fixture.example == "gantry-welder") {
          var welding = mission.welding;
          if (welding == null) throw "analytic welder has no welding group";
          motion = welding.motion;
        } else {
          var handling = handlingRunner(mission, singleHandlingChannel(mission));
          motion = handling.motion;
        }
        var numeric:motionkit.robot.ManipulatorKinematics = cast motion.compiler.solver;
        var group = numeric.manipulator;
        var analytic:motionkit.robot.AnalyticIk = new motionkit.robot.CartesianAnalyticIk(group);
        var sampler = new motionkit.robot.CartesianCandidateSampler(group);
        var before = group.numericSolveCount();
        for (sample in 0...100) {
          var q = [for (joint in 0...analytic.jointCount()) {
            var bounds = group.group.limitsOf(joint);
            var fraction = 0.5 + 0.45 * Math.sin((sample + 1) * (joint + 1) * 1.61803398875);
            bounds.lower + fraction * (bounds.upper - bounds.lower);
          }];
          var target = numeric.forward(q);
          if(sample%10==0) {
            var axis=group.tcpPose(q).transformVector(new robotkit.spatial.Vec3(0,0,1)).toArray();
            for(policy in [motionkit.path.OrientationPolicy.Fixed,motionkit.path.OrientationPolicy.FreeAboutTool,
                motionkit.path.OrientationPolicy.Cone(axis,0.1)])
              checkNativeCandidateSet(numeric,target,q,policy,sampler.sample(target,q,policy,4,1,4));
          }

          for (freedom in [motionkit.path.OrientationPolicy.Fixed, motionkit.path.OrientationPolicy.FreeAboutTool]) {
            var original = false;
            for (answer in analytic.branches(target, q, freedom)) {
              var same = true;
              for (joint in 0...q.length) same = same && Math.abs(answer.q[joint] - q[joint]) < 1e-6;
              original = original || same;
              var actual = numeric.forward(answer.q);
              check(motionkit.path.PoseMath.distance(actual, target) < 1e-6,
                "authored gantry branch preserves the mounted TCP position");
              check(motionkit.robot.ToolFreedom.orientationError(actual, target, freedom) < 1e-6,
                "authored gantry branch preserves the permitted tool orientation");
            }
            check(original, "authored gantry analytic branches contain the original configuration");
          }
        }
        check(group.numericSolveCount() == before, "Cartesian analytic branches use no numeric pose solves");
        Sys.println('Authored ${fixture.example}/${fixture.file}: ${analytic.family()} 200 round trips and 30 combined native sets passed');
      } catch (error:Dynamic) {
        simulation.dispose(); session.dispose(); throw error;
      }
      simulation.dispose(); session.dispose();
    }
  }

  /** PP1: analytic round trips on the authored MachineKit arm, including its mounted tool. */
  static function checkCobotAnalytic(root:String):Void {
    for (size in [500,850,900,1300]) {
      var filename = size == 850 ? "materia.project.json" : 'materia.reach$size.project.json';
      var manifest = FileSystem.fullPath(root + "/machinekit/examples/cobot-arm/" + filename);
      var generated = MateriaProjectRunner.loadProject(manifest);
      var session = new ProjectDocumentSession(null,false);
      session.openGeneratedProject(generated,manifest);
      var simulation = new ApplicationSimulation(new RobotWorld());
      try {
        check(simulation.rebuild(session.sensors,session.scene,session),"analytic cobot builds: " + simulation.error);
        var mission = simulation.missionPlayer();
        if (mission == null) throw "analytic cobot has no mission";
        var arm:Null<robotkit.manipulation.Manipulator> = null;
        for (frame in mission.robot.missionFrames) {
          var candidate = new robotkit.manipulation.Manipulator(mission.robot.model,mission.robot.model.links[0].id,frame.id);
          if (candidate.group.count() == 6) { arm = candidate; break; }
        }
        if (arm == null) throw "analytic cobot has no six-joint tool chain";
        var group = arm;
        var numeric = new motionkit.robot.ManipulatorKinematics(arm);
        var analytic = new motionkit.robot.EaikAnalyticIk(arm);
        var sampler = new motionkit.robot.SerialCandidateSampler(group);
        var before = group.numericSolveCount();
        for (sample in 0...200) {
          var q = [for (joint in 0...6) {
            var bounds = arm.group.limitsOf(joint);
            var fraction = 0.5 + 0.45*Math.sin((sample+1)*(joint+1)*1.61803398875);
            bounds.lower + fraction*(bounds.upper-bounds.lower);
          }];
          var target = numeric.forward(q), predicted = analytic.forward(q);
          if(sample%10==0) {
            var axis=group.tcpPose(q).transformVector(new robotkit.spatial.Vec3(0,0,1)).toArray();
            for(policy in [motionkit.path.OrientationPolicy.Fixed,motionkit.path.OrientationPolicy.FreeAboutTool,
                motionkit.path.OrientationPolicy.Cone(axis,0.1)])
              checkNativeCandidateSet(numeric,target,q,policy,sampler.sample(target,q,policy,[],4,1,4));
          }

          check(Math.sqrt(Math.pow(target.x-predicted.x,2)+Math.pow(target.y-predicted.y,2)+Math.pow(target.z-predicted.z,2)) < 1e-6,
            'authored Cobot $size analytic FK');
          var found = false;
          for (branch in analytic.branches(target,q)) {
            var same = true;
            for (joint in 0...6) if (Math.abs(branch.q[joint]-q[joint]) > 1e-5) same = false;
            found = found || same;
          }
          check(found,'authored Cobot $size original branch at sample $sample');
        }
        check(group.numericSolveCount() == before,"authored Cobot analytic checks use no numeric IK");
        Sys.println('Authored Cobot $size: 200 analytic round trips and 60 combined native sets passed');
      } catch (error:Dynamic) {
        simulation.dispose();session.dispose();throw error;
      }
      simulation.dispose();session.dispose();
    }
  }

  static function checkRobotArmAnalytic(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-arm/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    try {
      check(simulation.rebuild(session.sensors, session.scene, session), "analytic arm model builds: " + simulation.error);
      var mission = simulation.missionPlayer();
      if (mission == null) throw "analytic arm fixture has no mission group";
      var handling = handlingRunner(mission, singleHandlingChannel(mission));
      var numeric:motionkit.robot.ManipulatorKinematics = cast handling.motion.compiler.solver;
      var group = numeric.manipulator;
      var arm = new robotkit.manipulation.Manipulator(group.robot, group.rootLink, group.flangeFrame,
        group.flangeTTcp, group.model, null, group.profile);
      var at = robotkit.spatial.Transform3.identity();
      var geometry:Array<Dynamic> = [];
      for (joint in arm.pathJoints()) {
        var frame = at.compose(robotkit.spatial.Transform3.fromArrays(joint.parentFramePosition, joint.parentFrameRotation));
        if (joint.type != robotkit.model.JointType.Fixed)
          geometry.push({id: joint.id, origin: frame.translation.toArray(),
            axis: frame.transformVector(robotkit.spatial.Vec3.fromArray(joint.axis)).toArray()});
        at = frame.compose(robotkit.spatial.Transform3.fromArrays(joint.childFramePosition, joint.childFrameRotation).inverse());
      }
      Sys.println("Authored RobotArm joint geometry " + haxe.Json.stringify(geometry));
      var analytic = new motionkit.robot.EaikKinematics(arm);
      var tolerance = new motionkit.kinematics.IkTolerance(1e-6, 1e-6);
      for (sample in 0...100) {
        var q = [for (joint in 0...6) {
          var bounds = arm.group.limitsOf(joint);
          var fraction = 0.5 + 0.45 * Math.sin((sample + 1) * (joint + 1) * 1.61803398875);
          bounds.lower + fraction * (bounds.upper - bounds.lower);
        }];
        var target = numeric.forward(q);
        var solved = analytic.solvePose(target, q, tolerance);
        if (solved == null) throw 'authored RobotArm has no analytic branch at sample $sample';
        var answer:Array<Float> = cast solved;
        for (joint in 0...6) check(Math.abs(answer[joint] - q[joint]) < 1e-5,
          'authored RobotArm analytic branch contains original joint $joint at sample $sample');
        var predicted = analytic.forward(q);
        check(Math.sqrt(Math.pow(predicted.x - target.x, 2) + Math.pow(predicted.y - target.y, 2) +
          Math.pow(predicted.z - target.z, 2)) < 1e-6, "authored RobotArm analytic FK matches model FK");
      }
      Sys.println("Authored MachineKit RobotArm: 100 analytic round trips passed");
    } catch (error:Dynamic) {
      simulation.dispose(); session.dispose(); throw error;
    }
    simulation.dispose(); session.dispose();
  }

  /** A scene prop outside controller ownership must block a joint plan before submission. */
  static function checkJointSceneClearance(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/cobot-arm/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition = requiredAssemblyDefinition(generated.assemblyDefinition);
    var placement = new AssemblyState(definition, generated.assemblyState);
    var flange = placement.worldPose("toolFlange");
    generated.objects.push({id: "joint-plan-obstacle", label: "Independent scene obstacle", type: "rectangle",
      x: flange.x * 0.001, y: flange.y * 0.001, z: flange.z * 0.001,
      width: 0.08, height: 0.08, depth: 0.08, collisionEnabled: true, dynamicBody: false,
      mass: 0.0, red: 1.0, green: 0.0, blue: 0.0, visible: true});
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    check(simulation.rebuild(session.sensors, session.scene, session), "scene-clearance fixture builds: " + simulation.error);
    var mission = simulation.missionPlayer();
    if (mission == null) throw "Scene-clearance fixture needs its joint mission";
    var ticks = 0;
    while (mission.failure == null && ticks < 300) { simulation.step(); ticks++; }
    var failure = mission.failure;
    check(failure != null && failure.indexOf("joint-plan-obstacle") >= 0,
      "an independently owned scene obstacle rejects the joint plan: " + failure);
    check(mission.completed == 0, "the blocked joint plan is never completed");
    simulation.dispose(); session.dispose();
  }

  /** A held blank's swept path follows its carrier, rather than remaining a fixed scene obstacle. */
  static function checkHeldJointSceneClearance(root:String, nested:Bool = false):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/cobot-arm/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition = requiredAssemblyDefinition(generated.assemblyDefinition);
    var initial = new AssemblyState(definition, generated.assemblyState);
    var ending = new AssemblyState(definition, generated.assemblyState);
    var task = generated.missions[0];
    if (task == null) throw "Held-clearance fixture needs its joint mission";
    var targets = task.steps[0].joints;
    if (targets == null) throw "Held-clearance fixture needs joint targets";
    for (target in targets) ending.setJoint(target.joint, target.position);
    task.loop = false;
    task.steps = [task.steps[0]];
    var start = materia.assembly.AssemblyFrames.compose(initial.worldPose("toolFlange"),
      materia.assembly.AssemblyFrames.translation(0, 0, 150));
    var finish = materia.assembly.AssemblyFrames.compose(ending.worldPose("toolFlange"),
      materia.assembly.AssemblyFrames.translation(0, 0, 150));
    for (held in [true, false]) {
      var point = held ? start : finish;
      generated.objects.push({id: held ? "held-clearance-probe" : "held-plan-obstacle", label: "Held clearance fixture",
        type: "rectangle", x: point.x * 0.001, y: point.y * 0.001, z: point.z * 0.001,
        width: 0.01, height: 0.01, depth: 0.01, collisionEnabled: true, dynamicBody: held,
        mass: held ? 0.001 : 0.0, red: 1.0, green: 0.0, blue: 0.0, visible: true});
    }
    if (nested) generated.objects.push({id:"held-clearance-carrier", label:"Retained assembly carrier",
      type:"rectangle", x:start.x*0.001-0.08, y:start.y*0.001, z:start.z*0.001,
      width:0.01, height:0.01, depth:0.01, collisionEnabled:true, dynamicBody:true,
      mass:0.001, red:0.0, green:1.0, blue:0.0, visible:true});
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    check(simulation.rebuild(session.sensors, session.scene, session), "held-clearance fixture builds: " + simulation.error);
    var mission = simulation.missionPlayer(), object = simulation.environmentObject("held-clearance-probe");
    if (mission == null || object == null) throw "Held-clearance fixture needs its robot and loose object";
    var flange = [for (hull in mission.robot.shapes) if (hull.shape.name == "toolFlange") hull];
    if (flange.length != 1) throw "Held-clearance fixture needs one flange hull";
    var link = flange[0].link, pose = mission.simulation.linkPose(mission.robotIndex, link);
    var carrier = new robotkit.spatial.Transform3(new robotkit.spatial.Vec3(pose.position[0], pose.position[1], pose.position[2]),
      new robotkit.spatial.Quat(pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]));
    var relative = carrier.inverse().compose(new robotkit.spatial.Transform3(
      new robotkit.spatial.Vec3(start.x * 0.001, start.y * 0.001, start.z * 0.001), robotkit.spatial.Quat.identity()));
    if (nested) {
      var parent = simulation.environmentObject("held-clearance-carrier");
      if (parent == null) throw "Retained clearance fixture lost its independent carrier";
      mission.simulation.holdObjectOnObject(object,parent,new nativekit.sim.SimPose(0.08,0,0));
      var forced = new Map<String,String>();
      forced.set("held-clearance-carrier",mission.robot.model.links[link].id);
      function checkLeaf(snapshot:cadbridge.CellCollisionSnapshot):Void {
        var leaf = [for (hull in snapshot.hulls) if (hull.member == "held-clearance-probe") hull];
        check(leaf.length > 0,"The retained child keeps its own clearance geometry");
        for (hull in leaf) check(switch hull.owner {
          case Held(owner,body):owner == mission.robotId() && body == mission.robot.model.links[link].id;
          case _:false;
        },"Retained child collision ownership reaches the plate's actual or planned robot link");
      }
      checkLeaf(mission.cellCollisionSnapshot([],forced));
      var parentRelative = carrier.inverse().compose(new robotkit.spatial.Transform3(
        new robotkit.spatial.Vec3(start.x*0.001-0.08,start.y*0.001,start.z*0.001),robotkit.spatial.Quat.identity()));
      mission.simulation.holdObjectOnLink(parent,mission.robotIndex,link,
        new nativekit.sim.SimPose(parentRelative.translation.x,parentRelative.translation.y,parentRelative.translation.z,
          parentRelative.rotation.x,parentRelative.rotation.y,parentRelative.rotation.z,parentRelative.rotation.w));
      check(mission.simulation.objectCarrier(object) == mission.simulation.objectBody(parent),
        "The child retains its physical parent instead of being rebound directly to the robot");
      checkLeaf(mission.cellCollisionSnapshot([]));
    } else {
      mission.simulation.holdObjectOnLink(object, mission.robotIndex, link,
        new nativekit.sim.SimPose(relative.translation.x, relative.translation.y, relative.translation.z,
          relative.rotation.x, relative.rotation.y, relative.rotation.z, relative.rotation.w));
      check(mission.simulation.objectCarrier(object) == mission.simulation.linkBody(mission.robotIndex, link),
        "the probe has its physical robot carrier");
    }
    var ticks = 0;
    while (mission.failure == null && ticks < 300) { simulation.step(); ticks++; }
    var failure = mission.failure;
    check(failure != null && failure.indexOf("held-clearance-probe") >= 0 && failure.indexOf("held-plan-obstacle") >= 0,
      "the carried object's future collision rejects the joint plan: " + failure);
    check(mission.completed == 0, "the blocked carried-object plan is never completed");
      simulation.dispose(); session.dispose();
  }

  static function checkScrewdrivingEquipment(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-screwdriver/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var tools = generated.robotTools;
    if (tools == null || tools.length != 1 || tools[0].kind != "screwdriver") throw "Equipment project must carry one screwdriver";
    var tool = tools[0], driver = tool.screwdriver;
    if (driver == null) throw "Equipment project lost its screwdriver description";
    check(generated.missions == null, "The equipment preview has no tightening mission yet");
    var session = new ProjectDocumentSession(null, false);
    try {
      session.openGeneratedProject(generated, manifest);
      for (backend in [ApplicationSimulation.MUJOCO, ApplicationSimulation.DETERMINISTIC]) {
        var simulation = new ApplicationSimulation(new RobotWorld());
        simulation.setBackend(backend);
        try {
          check(simulation.rebuild(session.sensors, session.scene, session),
            'Screwdriver equipment builds on backend $backend: ${simulation.error}');
          check(simulation.missionPlayer() == null, "The equipment has no driving mission before SD6");
          simulation.step();
          var driverFrame = false;
          for (robot in simulation.snapshot().robots()) for (frame in robot.sensors.toArray())
            if (frame.kind == robotkit.tool.ScrewdriverContract.SENSOR_KIND)
              driverFrame = robotkit.tool.ScrewdriverContract.valid(frame.values.toArray());
          check(driverFrame,"The equipment publishes an idle screwdriver frame on both backends");
        } catch (error:Dynamic) { simulation.clear(); throw error; }
        simulation.clear();
      }
    } catch (error:Dynamic) { session.dispose(); throw error; }
    session.dispose();
    Sys.println("Screwdriver equipment project passed on MuJoCo and the test backend");
  }

  /** Actual vacuum loss while the arm is moving must stop before a replacement can start. */
  static function checkScrewdrivingLoss(root:String):Void {
    var manifest = FileSystem.fullPath(root+"/machinekit/examples/robot-screwdriver/materia.assembly.project.json");
    for (backend in [ApplicationSimulation.MUJOCO,ApplicationSimulation.DETERMINISTIC]) for (pickup in [true,false]) {
      Sys.println('Screwdriving CAD: begin transport loss backend $backend, pickup $pickup');
      var generated = MateriaProjectRunner.loadProject(manifest), definition = generated.missions[0];
      if (definition == null || definition.steps.length != 4) throw "Transport-loss cell lost its CAD mission";
      var job = materia.project.SceneScrewInvocation.job(definition.steps[0].screw,definition.steps);
      if (job == null) throw "Transport-loss cell lost its first fastening";
      definition.steps = [definition.steps[0]];
      var session = new ProjectDocumentSession(null,false), simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        session.openGeneratedProject(generated,manifest);
        check(simulation.rebuild(session.sensors,session.scene,session),'Transport-loss cell builds: ${simulation.error}');
        var mission = simulation.missionPlayer(), clock = simulation.activeSession();
        var original = simulation.environmentObject("project:"+job.screw);
        if (mission == null || clock == null || original == null) throw "Transport-loss cell lost its original body";
        var injected = false, observedStop = false;
        var previousArm:Array<Float> = [], previousPhase = "";
        while (!mission.finished && mission.failure == null && clock.simulationTime() < 300) {
          simulation.step();
          var active = @:privateAccess mission.screwing;
          if (active != null) {
            var phase = Std.string(@:privateAccess active.phase);
            if (injected && phase == "ProcessStop") observedStop = true;
            var arm = @:privateAccess mission.screwArm;
            if (arm == null) throw "Transport-loss cell lost its arm";
            var robot = @:privateAccess mission.robot;
            var snapshot = robot.runtime.snapshot();
            var positions = [for (index in arm.jointIndices()) snapshot.q.get(index)], moving = false;
            if (phase == previousPhase && previousArm.length == positions.length)
              for (index in 0...positions.length)
                if (Math.abs(positions[index]-previousArm[index]) > 0.000001) moving = true;
            if (!injected && phase == (pickup ? "PickupRetract" : "WorkApproach")) {
              if (moving) {
                var tools = @:privateAccess simulation.tools;
                if (tools == null || tools.screwdrivers.length != 1) throw "Transport-loss cell lost its observer";
                var driver = tools.screwdrivers[0];
                if (driver.carried() != original) throw "Vacuum-loss injection has no original body held";
                driver.runtime.writeOutput(driver.channels.vacuum,ProcessEventValue.Digital(false));
                injected = true;
              }
            }
            previousArm = positions; previousPhase = phase;
          }
        }
        check(injected && observedStop,'Actual arm motion loses vacuum and enters the controlled-stop phase (injected $injected, stopped $observedStop, failure ${mission.failure})');
        check(mission.finished && mission.failure == null && mission.completed == 1,
          'Transport loss recovers and installs its reserve: ${mission.failure}, time ${clock.simulationTime()}');
        check(mission.screwRuns.length == 1,"Transport loss retains one named fastening report");
        var report = mission.screwRuns[0];
        check(report.finished && !report.failed && report.fault == null && report.attempts.length == 2 &&
          report.attempts[0].screw == job.screw && report.attempts[0].result.outcome == robotkit.tool.ScrewdriverContract.SCREW_LOST &&
          report.attempts[0].disposition == processkit.screwdriving.FasteningRunReport.ScrewDisposition.Lost,
          "The original body is reported lost, with no fabricated rejection or arm fault");
        var installed = report.installed();
        if (installed == null) throw "Transport-loss replacement has no installed identity";
        var body = simulation.environmentObject("project:"+installed);
        check(installed != job.screw && body != null && mission.assemblyParts.get("project:"+job.frame).carries(body) &&
          simulation.heldObjectIds().length == 0,"The exact replacement is installed after the controlled stop");
        var robot = @:privateAccess mission.robot;
        check(robot.runtime.snapshot().faultCode == 0,"Transport loss does not dismiss or fabricate a runtime safety fault");
        simulation.clear(); session.dispose();
      } catch (error:Dynamic) {simulation.clear(); session.dispose(); throw error;}
      Sys.println('Screwdriving CAD: end transport loss backend $backend, pickup $pickup');
    }
    Sys.println("Screwdriving transport loss passed during pickup and transfer on both backends");
  }

  /** Four actual CAD screws, separate replacements and physical reject-bin recovery. */
  static function checkScrewdrivingAssembly(root:String):Void {
    var manifest = FileSystem.fullPath(root+"/machinekit/examples/robot-screwdriver/materia.assembly.project.json");
    for (backend in [ApplicationSimulation.MUJOCO,ApplicationSimulation.DETERMINISTIC]) for (damaged in [false,true]) {
      Sys.println('Screwdriving CAD: begin whole backend $backend, damaged $damaged');
      var generated = MateriaProjectRunner.loadProject(manifest), definition = generated.missions[0];
      if (definition == null || definition.steps.length != 4) throw "Whole screw cell lost its mission";
      var jobs:Array<materia.project.SceneScrewDefinition.SceneScrew> = [];
      for (step in definition.steps) {var job = materia.project.SceneScrewInvocation.job(step.screw,definition.steps); if (job == null) throw "Whole cell lost a fastening"; jobs.push(job);}
      check([for (job in jobs) job.screw].join(",") == "work/flange/screw1,work/flange/screw3,work/flange/screw2,work/flange/screw4","Whole flange uses CAD cross order");
      var session = new ProjectDocumentSession(null,false), simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        session.openGeneratedProject(generated,manifest);
        check(simulation.rebuild(session.sensors,session.scene,session),'Whole screw cell builds: ${simulation.error}');
        var mission = simulation.missionPlayer(), clock = simulation.activeSession();
        var original = simulation.environmentObject("project:"+jobs[0].screw);
        if (mission == null || clock == null || original == null) throw "Whole cell lost its identified first screw";
        var injected = false, observedStep = -1;
        while (!mission.finished && mission.failure == null && clock.simulationTime() < 600) {
          if (mission.stepIndex != observedStep) {
            observedStep = mission.stepIndex;
            Sys.println('Whole screw transition $observedStep at ${clock.simulationTime()}');
          }
          simulation.step();
          if (damaged && !injected) {
            var tools = @:privateAccess simulation.tools;
            if (tools == null || tools.screwdrivers.length != 1) throw "Whole cell lost its mounted observer";
            var driver = tools.screwdrivers[0];
            var setup = @:privateAccess driver.setups.get(jobs[0].slot);
            if (setup != null && setup.screw == original) {
              var priorWork = setup.work;
              setup.work = () -> {var state = priorWork(); state.damagedLeadIn = true; return state;};
              injected = true;
            }
          }
        }
        var currentScrew = @:privateAccess mission.screwing;
        var currentPhase = currentScrew == null ? "none" : Std.string(@:privateAccess currentScrew.phase);
        check(mission.finished && mission.failure == null && mission.completed == 4,
          'Whole screw sequence completes: ${mission.failure}, time ${clock.simulationTime()}, step ${mission.stepIndex}, phase ${currentPhase}, q ${(@:privateAccess mission.robot).runtime.snapshot().q.toArray()}');
        check(!damaged || injected,"Cross-thread injection targets the original body rather than its replacement");
        check(mission.screwRuns.length == 4,"Every fastening has a run report");
        for (index in 0...jobs.length) {
          var job = jobs[index], report = mission.screwRuns[index];
          check(report.fastening == job.fastening && report.finished && !report.failed && report.fault == null,"Each named fastening finishes with a measured OK");
          check(report.attempts.length == (damaged && index == 0 ? 2 : 1),"Only the damaged screw consumes its one replacement");
          var last = report.attempts[report.attempts.length-1];
          check(last.result.outcome == robotkit.tool.ScrewdriverContract.OK && Math.abs(last.result.torqueNm-job.program.targetTorque) < 1e-8,"Final torque comes from the actual qualified cycle");
          var installed = report.installed();
          if (installed == null) throw "Completed fastening has no installed body";
          var body = simulation.environmentObject("project:"+installed);
          if (body == null) throw "Completed fastening lost its actual installed occurrence";
          var fixture = mission.assemblyParts.get("project:"+job.frame);
          check(fixture.carries(body),"The exact threaded fixture carries the actual installed body");
          var actual = mission.assemblyParts.get("project:"+installed).pose(), live = fixture.pose();
          var socket = AssemblyFrames.compose({x:actual.position[0],y:actual.position[1],z:actual.position[2],
            qx:actual.rotation[0],qy:actual.rotation[1],qz:actual.rotation[2],qw:actual.rotation[3]},job.socket);
          var seat = AssemblyFrames.compose({x:live.position[0],y:live.position[1],z:live.position[2],
            qx:live.rotation[0],qy:live.rotation[1],qz:live.rotation[2],qw:live.rotation[3]},job.seat);
          var offset = AssemblyFrames.compose(AssemblyFrames.inverse(seat),socket);
          check(Math.sqrt(offset.x*offset.x+offset.y*offset.y) <= job.simulation.leadInMm/1000 && Math.abs(offset.z) < 0.00015,
            'Installed screw "$installed" remains on its actual CAD seat: ${haxe.Json.stringify(offset)}');
        }
        if (damaged) {
          var first = mission.screwRuns[0];
          check(first.attempts[0].result.outcome == robotkit.tool.ScrewdriverContract.CROSS_THREAD && first.attempts[0].result.angle > 0,
            "Recovery retains the measured cross-thread result and driven angle");
          check(first.installed() != jobs[0].screw,"Recovery installs a distinct identified replacement");
          for (_ in 0...100) simulation.step();
          var bin = mission.assemblyParts.get("project:reject/floor").pose(), dropped = mission.simulation.objectPose(original);
          check(Math.abs(dropped.x-bin.position[0]) < 0.036 && Math.abs(dropped.y-bin.position[1]) < 0.036 && dropped.z > bin.position[2] && dropped.z < bin.position[2]+0.06,
            'The actual failed screw rests inside the physical reject bin: ${haxe.Json.stringify(dropped)}');
        }
        check(simulation.heldObjectIds().length == 0,"Whole assembly leaves no body on the bit");
        simulation.clear(); session.dispose();
      } catch (error:Dynamic) {simulation.clear(); session.dispose(); throw error;}
      Sys.println('Screwdriving CAD: end whole backend $backend, damaged $damaged');
    }
    Sys.println("Screwdriving CAD: begin whole portable export");
    var portable = materia.project.SceneArtifact.decode(MateriaProjectRunner.exportArtifact(manifest));
    var flags = portable.project, mission = portable.missions[0];
    check(flags != null && flags.dynamicParts.length == 8 && mission != null && mission.steps.length == 4,
      "Portable assembly retains all eight identified screw supplies and four CAD fastenings");
    if (mission == null) throw "Portable assembly lost its mission";
    for (step in mission.steps) {
      var job = materia.project.SceneScrewInvocation.job(step.screw,mission.steps);
      check(job != null && job.recovery != null,"Portable assembly preserves each exact replacement and reject frame");
    }
    Sys.println("Screwdriving CAD: end whole portable export");
    Sys.println("Screwdriving whole assembly passed on both backends with cross-thread replacement recovery");
  }

  /**
   * The whole flange with physical thread engagement: each screw is driven through a stiff thread coupling on MuJoCo
   * instead of the process model's kinematic pose, and must reach the same qualified outcome on its CAD seat.
   */
  static function checkScrewPhysicalThread(root:String):Void {
    var manifest = FileSystem.fullPath(root+"/machinekit/examples/robot-screwdriver/materia.assembly.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest), definition = generated.missions[0];
    if (definition == null || definition.steps.length != 4) throw "Whole screw cell lost its mission";
    definition.screwEngagement = {mode: materia.project.SceneArtifact.SceneArtifactScrewEngagementMode.PhysicalThread,
      maximumAxialForceN: 200, threadStiffnessNPerM: 1.0e7};
    var jobs = [for (step in definition.steps) materia.project.SceneScrewInvocation.job(step.screw,definition.steps)];
    var session = new ProjectDocumentSession(null,false), simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      session.openGeneratedProject(generated,manifest);
      check(simulation.rebuild(session.sensors,session.scene,session),'Physical-thread screw cell builds: ${simulation.error}');
      var mission = simulation.missionPlayer(), clock = simulation.activeSession();
      if (mission == null || clock == null) throw "Physical-thread cell lost its mission or session";
      var engaged = 0, wasAttached = false, traceAt = 0.0;
      while (!mission.finished && mission.failure == null && clock.simulationTime() < 600) {
        simulation.step();
        var tools = @:privateAccess simulation.tools;
        var attached = tools != null && tools.screwdrivers.length == 1 && tools.screwdrivers[0].physicalThreadAttached();
        if (attached && !wasAttached) engaged++;
        if (Sys.getEnv("PHYSICAL_THREAD_TRACE") == "1" && tools != null && tools.screwdrivers.length == 1 &&
            clock.simulationTime() >= traceAt) {
          traceAt = clock.simulationTime() + 0.25;
          var driver = tools.screwdrivers[0], body = driver.carried();
          var pose = body == null ? null : @:privateAccess driver.simulation.objectPose(body);
          var entry = screwEntryError(driver, pose);
          Sys.println('Physical thread trace ${Math.round(clock.simulationTime()*100)/100} s step ${mission.stepIndex}: ' +
            '${haxe.Json.stringify(driver.reading())}, carriage ${Std.string(@:privateAccess driver.carriage)}, ' +
            'spindle ${driver.spindlePosition()}, screw ${pose == null ? "none" : '${pose.x},${pose.y},${pose.z}'}, ' +
            'entry $entry');
        }
        wasAttached = attached;
      }
      check(mission.finished && mission.failure == null && mission.completed == 4,
        'Physical-thread sequence completes: ${mission.failure}, time ${clock.simulationTime()}, step ${mission.stepIndex}');
      check(engaged == 4, 'Every fastening is driven through the physical thread: $engaged engagements');
      for (index in 0...jobs.length) {
        var job = jobs[index], report = mission.screwRuns[index];
        if (job == null) throw "Physical-thread cell lost a fastening";
        var last = report.attempts[report.attempts.length-1];
        check(report.finished && !report.failed && last.result.outcome == robotkit.tool.ScrewdriverContract.OK &&
          Math.abs(last.result.torqueNm-job.program.targetTorque) < 1e-8,
          'Physical thread keeps the qualified outcome of "${job.fastening}": ${last.result.outcome}, ${last.result.torqueNm} N m');
        var installed = report.installed();
        if (installed == null) throw "Completed fastening has no installed body";
        var actual = mission.assemblyParts.get("project:"+installed).pose(), live = mission.assemblyParts.get("project:"+job.frame).pose();
        var socket = AssemblyFrames.compose({x:actual.position[0],y:actual.position[1],z:actual.position[2],
          qx:actual.rotation[0],qy:actual.rotation[1],qz:actual.rotation[2],qw:actual.rotation[3]},job.socket);
        var seat = AssemblyFrames.compose({x:live.position[0],y:live.position[1],z:live.position[2],
          qx:live.rotation[0],qy:live.rotation[1],qz:live.rotation[2],qw:live.rotation[3]},job.seat);
        var offset = AssemblyFrames.compose(AssemblyFrames.inverse(seat),socket);
        check(Math.sqrt(offset.x*offset.x+offset.y*offset.y) <= job.simulation.leadInMm/1000 && Math.abs(offset.z) < 0.00015,
          'Physically threaded screw "$installed" seats on its CAD seat: ${haxe.Json.stringify(offset)}');
      }
      simulation.clear(); session.dispose();
    } catch (error:Dynamic) {simulation.clear(); session.dispose(); throw error;}
    Sys.println("Screwdriving physical thread passed: four MuJoCo fastenings through a stiff thread coupling");
  }

  /** Register a known displaced CAD fixture by screwdriver touch before its named screws run. */
  static function checkScrewdriverTouchRegistration(root:String):Void {
    var manifest = FileSystem.fullPath(root+"/machinekit/examples/robot-screwdriver/materia.registered.project.json");
    var backends = Sys.getEnv("PROJECT_SOURCE_DETERMINISTIC_ONLY") == "1"
      ? [ApplicationSimulation.DETERMINISTIC] : [ApplicationSimulation.MUJOCO,ApplicationSimulation.DETERMINISTIC];
    for (backend in backends) {
      var generated = MateriaProjectRunner.loadProject(manifest), missionDefinition = generated.missions[0];
      var definition = generated.assemblyDefinition, stateRecord = generated.assemblyState;
      if (missionDefinition == null || missionDefinition.steps.length != 5 || definition == null || stateRecord == null)
        throw "Touch-registration cell lost its probe step, four fastenings or assembly state";
      var find = missionDefinition.steps[0], job = find.contactWork;
      if (find.kind != "findWork" || job == null || job.frame != "work/plate")
        throw "Touch-registration cell lost its named CAD work frame";
      var tools = generated.robotTools;
      if (tools == null || tools.length != 1 || tools[0].kind != "screwdriver") throw "Touch-registration cell lost its screwdriver";
      var sensors = generated.robotSensors == null ? [] : [for (sensor in generated.robotSensors)
        if (sensor.kind == "tool_contact" && sensor.mount.occurrence == tools[0].contact.occurrence &&
          sensor.mount.connector == tools[0].contact.connector) sensor];
      if (sensors.length != 1) throw "Touch-registration cell needs one digital sensor at its screwdriver TCP";

      var flat = materia.assembly.AssemblyDefinitionFlattener.flatten(definition);
      var original = new AssemblyState(flat,stateRecord), nominal = original.worldPose(job.frame);
      var joints = flat.joints;
      var released = ["work-mount", "fixtureLeft-mount", "fixtureRight-mount"];
      if (joints == null || [for (id in released) if ([for (joint in joints) if (joint.id == id) joint].length == 1) id].length != released.length)
        throw "Touch-registration test cannot release the fixture and side stops from their authored table mates";
      flat.joints = [for (joint in joints) if (released.indexOf(joint.id) < 0) joint];
      generated.assemblyDefinition = flat;
      var displaced = new AssemblyState(flat);
      for (coordinate in stateRecord.jointCoordinates) displaced.setJoint(coordinate.joint,coordinate.value);
      for (pose in stateRecord.rootPoses) displaced.setRootPose(pose.occurrence,pose.pose);
      var turn = robotkit.spatial.Quat.fromAxisAngle(new robotkit.spatial.Vec3(0,0,1),Math.PI/360);
      var rotation = turn.multiply(new robotkit.spatial.Quat(nominal.qx,nominal.qy,nominal.qz,nominal.qw));
      var roots = materia.assembly.AssemblyDefinitionCodec.rootOccurrences(flat);
      var fixtureRoots = ["work/plate", "fixtureLeft", "fixtureRight"];
      for (rootId in fixtureRoots) if (!roots.exists(rootId))
        throw 'Touch-registration test cannot identify movable fixture root "$rootId"';
      var movedFrame = {x:nominal.x-10,y:nominal.y+7,z:nominal.z,
        qx:rotation.x,qy:rotation.y,qz:rotation.z,qw:rotation.w};
      var delta = AssemblyFrames.compose(movedFrame,AssemblyFrames.inverse(nominal));
      for (rootId in fixtureRoots) displaced.setRootPose(rootId,AssemblyFrames.compose(delta,original.worldPose(rootId)));
      generated.assemblyState = displaced.record();
      var knownPose = displaced.worldPose(job.frame), scale = generated.metresPerUnit;
      var knownWork = new robotkit.spatial.Transform3(new robotkit.spatial.Vec3(
        knownPose.x*scale,knownPose.y*scale,knownPose.z*scale),
        new robotkit.spatial.Quat(knownPose.qx,knownPose.qy,knownPose.qz,knownPose.qw));

      var session = new ProjectDocumentSession(null,false), simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        session.openGeneratedProject(generated,manifest);
        check(simulation.rebuild(session.sensors,session.scene,session),
          'Displaced touch-registration cell builds on backend $backend: ${simulation.error}');
        var mission = simulation.missionPlayer(), clock = simulation.activeSession();
        if (mission == null || clock == null) throw "Touch-registration cell has no live mission";
        var touched = false;
        while (!mission.finished && mission.failure == null && clock.simulationTime() < 900) {
          simulation.step();
          var snapshot = mission.robot.runtime.snapshot();
          for (index in 0...snapshot.sensors.length) {
            var frame = snapshot.sensors.get(index);
            if (frame.sensorId == sensors[0].id && frame.kind == "tool_contact" && frame.values.length == 1 && frame.values.get(0) == 1.0)
              touched = true;
          }
        }
        var evidence = mission.registeredWork.get(job.frame);
        check(mission.finished && mission.failure == null && mission.completed == 5 && touched && evidence != null,
          'Touch registration and named fastening complete on backend $backend: ${mission.failure}, time ${clock.simulationTime()}');
        if (evidence == null) throw "Touch-registration mission lost its accepted evidence";
        check(evidence.measurements.length == 6 && evidence.envelope.containsWorkFrame(evidence.rootWork),
          "Six typed observations and their remaining pose bounds authorize the measured work frame");
        for (measurement in evidence.measurements) check(measurement.sensorId == sensors[0].id &&
          measurement.sensorKind == "tool_contact" && measurement.coordinateFrame == mission.robot.rootLink &&
          measurement.unit == "m" && measurement.sourceClockId != "unspecified" &&
          measurement.uncertaintyMeters <= job.measurementError,
          "Accepted screwdriver touch retains its identified, fresh metric source record");
        check(evidence.world.translation.sub(knownWork.translation).norm() < 0.0005 &&
          evidence.world.rotation.angularDistance(knownWork.rotation) < 0.002,
          "Registration follows the known translated and rotated fixture without editing nominal CAD");
        var firstJob = materia.project.SceneScrewInvocation.job(missionDefinition.steps[1].screw,missionDefinition.steps);
        if (firstJob == null) throw "Touch-registration mission lost its first canonical fastening";
        check(mission.screwRuns.length == 4 && mission.screwRuns[0].fastening == firstJob.fastening,
          "The first physical screw report preserves its named CAD fastening identity");
      } catch (error:Dynamic) { simulation.clear(); session.dispose(); throw error; }
      simulation.clear(); session.dispose();
      Sys.println('Screwdriver touch registration passed on backend $backend');
    }
  }

  /** Buffered supply deposits each identified screw; pickup and tightening remain physical. */
  static function checkScrewFeeder(root:String):Void {
    var manifest = FileSystem.fullPath(root+"/machinekit/examples/robot-screwdriver/materia.feeder.project.json");
    var backendFilter = Sys.getEnv("PROJECT_SOURCE_FEEDER_BACKEND");
    var backends = backendFilter == "mujoco" ? [ApplicationSimulation.MUJOCO]
      : backendFilter == "deterministic" ? [ApplicationSimulation.DETERMINISTIC]
      : [ApplicationSimulation.MUJOCO,ApplicationSimulation.DETERMINISTIC];
    var failureFilter = Sys.getEnv("PROJECT_SOURCE_FEEDER_FAILURE");
    var failures = failureFilter == "normal" ? [false] : failureFilter == "jam" ? [true] : [false,true];
    for (backend in backends) for (jam in failures) {
      Sys.println('Screw feeder CAD: begin backend $backend, jam $jam');
      var generated = MateriaProjectRunner.loadProject(manifest), definition = generated.missions[0];
      if (definition == null || definition.screwFeeders == null || definition.screwFeeders.length != 1)
        throw "Fed flange lost its one replenishing material path";
      var jobs:Array<materia.project.SceneScrewDefinition.SceneScrew> = [];
      for (step in definition.steps) {
        var job = materia.project.SceneScrewInvocation.job(step.screw,definition.steps);
        if (job == null) throw "Fed flange lost its CAD fastening"; jobs.push(job);
      }
      check(jobs.length == 4 && [for (job in jobs) job.presenter].join(",") == "presenter/nest,presenter/nest,presenter/nest,presenter/nest",
        "All four CAD screws use the same replenished pickup nest");
      var session = new ProjectDocumentSession(null,false), simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        session.openGeneratedProject(generated,manifest);
        check(simulation.rebuild(session.sensors,session.scene,session),'Fed flange builds: ${simulation.error}');
        var mission = simulation.missionPlayer(), clock = simulation.activeSession(), tools = @:privateAccess simulation.tools;
        if (mission == null || clock == null || tools == null || tools.screwFeeders.length != 1) throw "Fed flange lost its live device";
        var feeder = tools.screwFeeders[0];
        var injected = false, witnessed:Map<String,Bool> = [];
        while (!mission.finished && mission.failure == null && clock.simulationTime() < 600) {
          try simulation.step() catch (error:Dynamic) {
            Sys.println('Screw feeder step failed: backend=$backend, jam=$jam, time=${clock.simulationTime()}, ' +
              'step=${mission.stepIndex}, completed=${mission.completed}, deliveries=${feeder.model.deliveries}, ' +
              'delivered=${feeder.model.delivered}: $error');
            throw error;
          }
          var delivered = feeder.model.delivered;
          var feedback = feeder.model.reading();
          if (delivered != null && feedback != null && feedback.present) witnessed.set(delivered,true);
          if (jam && !injected && feeder.model.deliveries == 1 && feeder.model.delivered == null) {
            feeder.model.setJam(true); injected = true;
          }
        }
        if (jam) check(injected && mission.failure != null && mission.failure.indexOf("jam") >= 0 && feeder.model.deliveries == 1,
          "A physical pickup followed by a jam stops the next delivery with its diagnostic");
        else {
          check(mission.finished && mission.failure == null && feeder.model.deliveries == 4,"One buffered delivery supplies each fastening");
          var results = mission.screwResults;
          check(results.length == 4,"Fed flange records all four results");
          for (job in jobs) check(witnessed.exists(job.screw),"Each identified screw is sensed at the nest before pickup");
          for (result in results) check(result.outcome == robotkit.tool.ScrewdriverContract.OK && Math.abs(result.torqueNm-16.5) < 1e-8,
            "Feeding preserves the published M6 tightening target");
          check(simulation.heldObjectIds().length == 0,"Fed flange releases the final screw");
          check(simulation.reset() && feeder.model.deliveries == 0 && feeder.model.bulk() == 4,
            "Reset restores physical stock and its authored feeder inventory");
          for (_ in 0...10) simulation.step();
          var resetReading = feeder.model.reading();
          check(feeder.model.deliveries == 0 && resetReading != null && !resetReading.present,
            "Reset has no latent release and the sensed pickup nest is physically empty");
        }
        simulation.clear(); session.dispose();
      } catch (error:Dynamic) {simulation.clear(); session.dispose(); throw error;}
      Sys.println('Screw feeder CAD: end backend $backend, jam $jam');
    }
    var portable = materia.project.SceneArtifact.decode(MateriaProjectRunner.exportArtifact(manifest));
    var work = portable.missions[0];
    check(work != null && work.screwFeeders != null && work.screwFeeders.length == 1 && work.screwFeeders[0].inventory.length == 4,
      "Portable fed flange retains one material path and all four inventory identities");
    if (backendFilter == null && failureFilter == null)
      Sys.println("Screw feeder CAD passed on both backends with physical replenishment and jam diagnostics");
    else
      Sys.println("Selected screw feeder CAD cases passed");
  }

  /** Air delivery reaches the actual tool socket; the arm never visits a remote pickup nest. */
  static function checkBlowFeed(root:String):Void {
    var manifest = FileSystem.fullPath(root+"/machinekit/examples/robot-screwdriver/materia.blow-feed.project.json");
    for (backend in [ApplicationSimulation.MUJOCO,ApplicationSimulation.DETERMINISTIC]) for (failure in ["none","damaged","missing","blocked","nose-jam","stop"]) {
      Sys.println('Blow-feed CAD: begin backend $backend, $failure');
      var generated = MateriaProjectRunner.loadProject(manifest), work = generated.missions[0];
      if (work == null || work.blowFeeds == null || work.blowFeeds.length != 1) throw "Blown flange lost its receiving path";
      for (step in work.steps) {
        var job = materia.project.SceneScrewInvocation.job(step.screw,work.steps);
        check(job != null && job.presenter == null && job.pickup == null,"Blown CAD jobs contain no remote presenter pose");
      }
      var session = new ProjectDocumentSession(null,false), simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        session.openGeneratedProject(generated,manifest);
        check(simulation.rebuild(session.sensors,session.scene,session),'Blown flange builds: ${simulation.error}');
        var mission = simulation.missionPlayer(), clock = simulation.activeSession(), tools = @:privateAccess simulation.tools;
        if (mission == null || clock == null || tools == null || tools.blowFeeds.length != 1) throw "Blown flange lost its live device";
        var feeder = tools.blowFeeds[0];
        var firstJob = materia.project.SceneScrewInvocation.job(work.steps[0].screw,work.steps);
        if (firstJob == null) throw "Blown flange lost its first CAD fastening";
        var original = simulation.environmentObject("project:"+firstJob.screw);
        if (original == null) throw "Blown flange lost its first physical screw";
        if (failure == "missing") feeder.model.setTransitLoss(true);
        if (failure == "blocked") feeder.model.setBlocked(true);
        if (failure == "nose-jam") feeder.model.setNoseJam(true);
        var stopped = false, damaged = false, witnessed:Map<String,Bool> = [], observedStep = -1;
        while (!mission.finished && mission.failure == null && clock.simulationTime() < 600) {
          if (mission.stepIndex != observedStep) {
            observedStep = mission.stepIndex;
            Sys.println('Blow-feed transition $observedStep at ${clock.simulationTime()}');
          }
          simulation.step();
          var reading = feeder.model.reading(), delivered = feeder.model.transport.delivered;
          if (reading != null && reading.present && delivered != null) witnessed.set(delivered,true);
          if (failure == "damaged" && !damaged) {
            if (tools.screwdrivers.length != 1) throw "Blown flange lost its screwdriver observer";
            var driver = tools.screwdrivers[0];
            var setup = @:privateAccess driver.setups.get(firstJob.slot);
            if (setup != null && setup.screw == original) {
              var priorWork = setup.work;
              setup.work = () -> {var state = priorWork(); state.damagedLeadIn = true; return state;};
              damaged = true;
            }
          }
          if (failure == "stop" && !stopped && feeder.model.airOn) {
            simulation.stop(); feeder.safe(); stopped = true;
            for (_ in 0...100) feeder.poll(clock.simulationTime());
            check(!feeder.model.airOn && feeder.model.transport.deliveries == 0,"Stop during air delivery cancels release and retains stock");
            check(simulation.reset(),"Explicit cell reset recovers interrupted delivery");
            break;
          }
        }
        var currentScrew = @:privateAccess mission.screwing;
        var currentPhase = currentScrew == null ? "none" : Std.string(@:privateAccess currentScrew.phase);
        var currentResult = currentScrew == null ? null : currentScrew.tighteningResult;
        var screwDetail = currentResult == null ? "none" :
          'outcome ${currentResult.outcome}, torque ${currentResult.torqueNm}, angle ${currentResult.angle}, cause ${currentScrew.failure()}';
        if (failure == "none") {
          check(mission.finished && mission.failure == null && feeder.model.transport.deliveries == 4,
            'Blown flange tightens four delivered screws: ${mission.failure}, time ${clock.simulationTime()}, step ${mission.stepIndex}, phase ${currentPhase}, screw ${screwDetail}, deliveries ${feeder.model.transport.deliveries}, joints ${(@:privateAccess mission.robot).robot.description().joints.join(",")}, q ${(@:privateAccess mission.robot).runtime.snapshot().q.toArray()}');
          check(mission.screwResults.length == 4,"Blown flange records the unchanged qualified controller outcomes");
          for (result in mission.screwResults) check(result.outcome == robotkit.tool.ScrewdriverContract.OK && Math.abs(result.torqueNm-16.5) < 1e-8,"Blow feed preserves the published M6 tightening target");
          check([for (id in witnessed.keys()) id].length == 4,"Every actual received body is witnessed at the tool");
          check(simulation.heldObjectIds().length == 0 && simulation.reset() && feeder.model.transport.bulk() == 4,"Blown inventory and physical poses reset together");
        } else if (failure == "damaged") {
          check(damaged && mission.finished && mission.failure == null && mission.completed == 4 && feeder.model.transport.deliveries == 4,
            'Rejected blow-fed screw is physically accounted before the next delivery: ${mission.failure}');
          check(mission.screwRuns.length == 4 && mission.screwRuns[0].attempts.length == 2 &&
            mission.screwRuns[0].attempts[0].result.outcome == robotkit.tool.ScrewdriverContract.CROSS_THREAD &&
            mission.screwRuns[0].attempts[0].disposition == processkit.screwdriving.FasteningRunReport.ScrewDisposition.Rejected &&
            mission.screwRuns[0].installed() != firstJob.screw,
            "CAD reject-bin confirmation releases the exact delivered identity and installs its declared reserve");
          check([for (id in witnessed.keys()) id].length == 4 && simulation.heldObjectIds().length == 0,
            "All four primary screws have measured receipt and the recovered cell releases the tool");
        } else if (failure == "stop") check(stopped,"Stop case reaches actual metering before explicit recovery");
        else check(mission.failure != null && (failure == "missing" ? mission.failure.indexOf("receipt") >= 0 : failure == "blocked" ? mission.failure.indexOf("blocked") >= 0 : mission.failure.indexOf("nosepiece") >= 0)
          && !feeder.model.airOn,"Missing receipt, tube block and receiver jam stop with their own diagnosis");
        simulation.clear(); session.dispose();
      } catch (error:Dynamic) {simulation.clear(); session.dispose(); throw error;}
      Sys.println('Blow-feed CAD: end backend $backend, $failure');
    }
    var portable = materia.project.SceneArtifact.decode(MateriaProjectRunner.exportArtifact(manifest));
    check(portable.missions != null && portable.missions[0].blowFeeds != null && portable.missions[0].blowFeeds.length == 1,"Portable CAD retains the receiving path");
    Sys.println("Blow-feed CAD passed on both backends with measured receipt, faults and explicit stop recovery");
  }

  /** The held screw's lead-in point against the live hole entry, as the screwdriver's engagement test sees it. */
  static function screwEntryError(driver:processkit.simulation.SimulatedScrewdriver, pose:Null<nativekit.sim.SimPose>):String {
    var setup:Null<processkit.simulation.SimulatedScrewdriver.ScrewdriverSetup> = @:privateAccess driver.selected;
    if (setup == null || pose == null) return "none";
    var state = setup.work();
    var reference = new robotkit.spatial.Transform3(new robotkit.spatial.Vec3(state.head.x, state.head.y, state.head.z),
      new robotkit.spatial.Quat(state.head.qx, state.head.qy, state.head.qz, state.head.qw));
    var screwPose = new robotkit.spatial.Transform3(new robotkit.spatial.Vec3(state.screwPose.x, state.screwPose.y,
      state.screwPose.z), new robotkit.spatial.Quat(state.screwPose.qx, state.screwPose.qy, state.screwPose.qz,
      state.screwPose.qw));
    var actual = new robotkit.spatial.Transform3(new robotkit.spatial.Vec3(pose.x, pose.y, pose.z),
      new robotkit.spatial.Quat(pose.qx, pose.qy, pose.qz, pose.qw));
    var lead = actual.compose(screwPose.inverse().compose(reference))
      .transformPoint(reference.inverse().transformPoint(robotkit.spatial.Vec3.fromArray(state.engagementPoint)));
    var error = lead.sub(robotkit.spatial.Vec3.fromArray(state.hole));
    var axis = robotkit.spatial.Vec3.fromArray(state.direction).normalized();
    var along = error.dot(axis);
    return 'along ${along * 1000} mm radial ${Math.sqrt(Math.max(0, error.dot(error) - along * along)) * 1000} mm';
  }

  /** MuJoCo contact wrenches must drive the first named CAD fastening into an offset hole. */
  static function checkScrewEntrySearch(root:String):Void {
    var manifest = FileSystem.fullPath(root+"/machinekit/examples/robot-screwdriver/materia.force-search.project.json");
    var registrationManifest = FileSystem.fullPath(root+"/machinekit/examples/robot-screwdriver/materia.registered-force-search.project.json");
    var forceCase = Sys.getEnv("PROJECT_SOURCE_FORCE_CASE");
    var offsets = forceCase == "nominal" ? [0.0] : forceCase == "offset" ? [0.001] : [0.0,0.001];
    for (offsetMeters in offsets) {
      Sys.println('Force-search CAD: begin lateral offset ${offsetMeters*1000} mm');
      var forceProject = MateriaProjectRunner.loadProject(manifest), forceMission = forceProject.missions[0];
      var generated = MateriaProjectRunner.loadProject(registrationManifest), missionDefinition = generated.missions[0];
      var definition = generated.assemblyDefinition, stateRecord = generated.assemblyState;
      if (missionDefinition == null || missionDefinition.steps.length != 5 || missionDefinition.steps[0].kind != "findWork" ||
          missionDefinition.steps[0].contactWork == null || forceMission == null || forceMission.screwEntrySearch == null ||
          definition == null || stateRecord == null)
        throw "Force-search cell lost its profile, registration step, four named fastenings or CAD assembly";
      var carrierOccurrences = [for (occurrence in definition.occurrences)
        if (occurrence.id == "arm/tool/driver/carrier") occurrence];
      if (carrierOccurrences.length != 1) throw "Registered force-search cell lost its screwdriver carrier";
      var carrierDefinition = [for (component in definition.definitions)
        if (component.id == carrierOccurrences[0].definition) component];
      var carrierTcp = carrierDefinition.length != 1 ? [] : [for (connector in carrierDefinition[0].connectors)
        if (connector.name == "tcp") connector];
      if (carrierTcp.length != 1 || Math.abs(carrierTcp[0].frame.z - 100) > 1e-8)
        throw "Registered force-search screwdriver nose reach is " +
          (carrierTcp.length == 1 ? Std.string(carrierTcp[0].frame.z) : "missing") +
          " mm; expected 100 mm to clear the robot wrist";
      missionDefinition.steps = [missionDefinition.steps[0], missionDefinition.steps[1]];
      var candidateProfile = forceMission.screwEntrySearch;
      if (candidateProfile == null) throw "Force-search project lost its commissioned search profile";
      var profile:materia.project.SceneArtifact.SceneArtifactScrewEntrySearch = cast candidateProfile;
      Sys.println('Force-search commissioned run bound: ${profile.maximumRunSeconds} s');
      missionDefinition.screwEntrySearch = profile;
      var forceSensors = forceProject.robotSensors == null ? [] : [for (sensor in forceProject.robotSensors)
        if (sensor.id == profile.sensor && sensor.kind == "force_torque") sensor];
      if (forceSensors.length != 1) throw "Force-search cell lost its unique mounted force/torque sensor";
      if (generated.robotSensors == null) throw "Registered force-search cell lost its robot sensor declarations";
      var registeredForceSensors = [for (sensor in generated.robotSensors)
        if (sensor.id == profile.sensor && sensor.kind == "force_torque") sensor];
      if (registeredForceSensors.length == 0) generated.robotSensors.push(forceSensors[0]);
      else if (registeredForceSensors.length != 1)
        throw 'Registered force-search cell has ${registeredForceSensors.length} copies of sensor "${profile.sensor}"';

      var session = new ProjectDocumentSession(null,false), simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(ApplicationSimulation.MUJOCO);
      try {
        session.openGeneratedProject(generated,registrationManifest);
        check(simulation.rebuild(session.sensors,session.scene,session),
          'Registered force-search CAD builds with touch and wrench observers: ${simulation.error}');
        var mission = simulation.missionPlayer(), clock = simulation.activeSession();
        if (mission == null || clock == null) throw "Force-search cell lost its live mission or session";
        var shifted = offsetMeters == 0.0, searchPrepared = false;
        var observed:Null<processkit.screwdriving.ForceGuidedScrewEntry> = null;
        var startedAt = 0.0, settledAt:Null<Float> = null, phases:Array<String> = [], traceAt = 0.0;
        // Registration and the approach come first; once the search starts it has its own bound, then the tightening.
        while (!mission.finished && mission.failure == null &&
          clock.simulationTime() < (observed == null ? 900.0 : startedAt + profile.maximumRunSeconds + 600.0)) {
          var registration = @:privateAccess mission.registeredWork.get("work/plate");
          if (!searchPrepared && registration != null) {
            var searchWorld = registration.world;
            if (offsetMeters > 0) {
              var workU = registration.world.transformVector(new robotkit.spatial.Vec3(1,0,0)).normalized();
              searchWorld = new robotkit.spatial.Transform3(
                registration.world.translation.add(workU.scale(offsetMeters)), registration.world.rotation);
              shifted = true;
            }
            // Preserve the saved uncertainty budget for spiral planning after
            // registration; the offset case injects a residual inside that bound.
            var searchEnvelope = new processkit.perception.ContactPoseEnvelope(
              registration.envelope.nominalWork, registration.envelope.translationBound,
              registration.envelope.rotationBound, registration.envelope.measurementError);
            @:privateAccess mission.registeredWork.set("work/plate", {world:searchWorld,
              rootWork:registration.rootWork, envelope:searchEnvelope, measurements:registration.measurements});
            searchPrepared = true;
          }
          try simulation.step() catch (error:Dynamic) {
            var tools = @:privateAccess simulation.tools;
            if (tools != null && tools.screwdrivers.length == 1) {
              var driver = tools.screwdrivers[0], supply = driver.supply;
              var selected = @:privateAccess driver.selected;
              var attached = driver.carried();
              var controls = supply == null ? null : supply.controls();
              var names = ["work/flange/screw1","work/flange/screw2","work/flange/screw3","work/flange/screw4"];
              var attachedName = "none", selectedName = "none";
              for (name in names) {
                var object = simulation.environmentObject("project:"+name);
                if (object != null && object == attached) attachedName = name;
                if (object != null && selected != null && object == selected.screw) selectedName = name;
              }
              Sys.println('Force-search observer failure step=${mission.stepIndex} time=${clock.simulationTime()} ' +
                'active=${mission.screwing == null ? "none" : Std.string(@:privateAccess mission.screwing.phase)} ' +
                'slot=${controls == null ? -1 : controls.program} vacuum=${controls == null ? false : controls.vacuum} ' +
                'selected=$selectedName attached=$attachedName attachedFlag=${driver.physicalThreadAttached()} error=$error');
              var live:robotkit.runtime.Simulation = @:privateAccess simulation.simulation;
              var runtimes:Array<robotkit.runtime.RobotRuntime> = @:privateAccess live.robots;
              if (runtimes.length > 0) {
                var missionRobot:AssemblyRobot = @:privateAccess mission.robot;
                for (contact in live.robotContacts(runtimes[0])) if (contact.active) {
                  var otherParts = [for (part in missionRobot.parts) if (part.linkIndex == contact.otherLink) part.id];
                  Sys.println('Force-search contact link=${contact.linkIndex} toolPiece=${contact.toolPieceIndex} ' +
                    'otherKind=${contact.otherKind} otherRobot=${contact.otherRobot} otherLink=${contact.otherLink} ' +
                    'object=${contact.otherObject} otherPiece=${contact.otherPieceIndex} otherParts=${otherParts.join(",")} ' +
                    'distance=${contact.distance} position=${contact.position.x},${contact.position.y},${contact.position.z} ' +
                    'force=${contact.force.x},${contact.force.y},${contact.force.z}');
                }
              }
            }
            throw error;
          }
          var active = @:privateAccess mission.screwing;
          if (active != null && mission.stepIndex == 1) {
            var candidate:Null<processkit.screwdriving.ScrewEntrySearch> = @:privateAccess active.entrySearch;
            if (candidate != null && Std.isOfType(candidate, processkit.screwdriving.ForceGuidedScrewEntry)) {
              var search:processkit.screwdriving.ForceGuidedScrewEntry = cast candidate;
              if (observed == null) { observed = search; startedAt = clock.simulationTime(); }
              var control = search.controller, origin = search.origin;
              if (control != null && origin != null && (phases.length == 0 || phases[phases.length - 1] != Std.string(control.phase))) {
                phases.push(Std.string(control.phase));
                var snapshot = search.servo.robot.snapshot();
                var tcp = search.servo.manipulator.tcpPose([for (index in search.servo.manipulator.jointIndices())
                  snapshot.positions.get(index)]);
                var moved = tcp.translation.sub(origin.translation);
                Sys.println('Force-search ${control.phase} at ${Math.round((clock.simulationTime() - startedAt) * 100) / 100} s: ' +
                  'depth ${moved.dot(control.axisN) * 1000} mm of ${control.entryDepth * 1000}, lateral ' +
                  '${moved.dot(control.axisU) * 1000}, ${moved.dot(control.axisV) * 1000} mm, ' +
                  '${control.compressionN} N, peak ${search.peakForceN} N');
              }
              if (control != null && origin != null && Sys.getEnv("FORCE_SEARCH_TRACE") == "1" &&
                  clock.simulationTime() >= traceAt) {
                traceAt = clock.simulationTime() + 1.0;
                var snapshot = search.servo.robot.snapshot();
                var tcp = search.servo.manipulator.tcpPose([for (index in search.servo.manipulator.jointIndices())
                  snapshot.positions.get(index)]);
                var moved = tcp.translation.sub(origin.translation);
                var tools = @:privateAccess simulation.tools;
                var driver:Null<processkit.simulation.SimulatedScrewdriver> = null;
                if (tools != null && tools.screwdrivers.length == 1) driver = tools.screwdrivers[0];
                var screwText = "none", welded = false, feed = 0.0;
                var slide = search.feed;
                var feedJoint = slide == null ? 0.0 : snapshot.positions.get(slide.joint) * 1000;
                var armSpeed = 0.0;
                for (index in search.servo.manipulator.jointIndices())
                  armSpeed = Math.max(armSpeed, Math.abs(snapshot.velocities.get(index)));
                if (driver != null) {
                  welded = driver.welded();
                  screwText = 'reading ${haxe.Json.stringify(driver.reading())} ';
                  feed = driver.reading().strokeMm;
                  var screwBody = driver.carried();
                  if (screwBody != null) {
                    var screwPose = @:privateAccess driver.simulation.objectPose(screwBody);
                    screwText += '${screwPose.x},${screwPose.y},${screwPose.z}, entry ${screwEntryError(driver, screwPose)}';
                  }
                }
                Sys.println('Force-search trace ${Math.round((clock.simulationTime() - startedAt) * 100) / 100} s ' +
                  '${control.phase}: tcp depth ${moved.dot(control.axisN) * 1000} mm, lateral ' +
                  '${moved.dot(control.axisU) * 1000}, ${moved.dot(control.axisV) * 1000} mm, ' +
                  '${control.compressionN} N, welded $welded, feed joint $feedJoint mm, arm speed $armSpeed, ' +
                  'trajectory ${snapshot.trajectoryActive}, screw $screwText');
                if (control.compressionN > 0.1) {
                  var live:robotkit.runtime.Simulation = @:privateAccess simulation.simulation;
                  var missionRobot:AssemblyRobot = @:privateAccess mission.robot;
                  for (contact in live.robotContacts(missionRobot.runtime)) if (contact.active)
                    Sys.println('  contact link=${contact.linkIndex} toolPiece=${contact.toolPieceIndex} ' +
                      'otherKind=${contact.otherKind} otherLink=${contact.otherLink} object=${contact.otherObject} ' +
                      'distance=${contact.distance} force=${contact.force.x},${contact.force.y},${contact.force.z}');
                }
              }
              if (control != null && !search.running() && settledAt == null) settledAt = clock.simulationTime();
            }
          }
        }
        check(shifted, "The bounded registration estimate is biased before the first screwdriver approach");
        if (observed == null) throw 'Force-search mission never started its entry search: ${mission.failure}';
        var search:processkit.screwdriving.ForceGuidedScrewEntry = observed;
        var control = search.controller, origin = search.origin, entry = search.measuredEntry;
        check(search.complete && search.failure == null && control != null && origin != null && entry != null,
          'Force search seats the screw within its commissioned bounds: ${search.failure}; phases ${phases.join(" > ")}, ' +
          'peak ${search.peakForceN} N, mission ${mission.failure}');
        var seconds = (settledAt == null ? clock.simulationTime() : settledAt) - startedAt;
        var touched:processkit.screwdriving.ScrewEntryController = cast control;
        // On the nominal axis the screw passes the clearance bore untouched; offset, it lands on the flange face and
        // must find the bore by searching it, under its commissioned force bound.
        check(search.peakForceN <= profile.maximumForceN &&
          (offsetMeters == 0 || (touched.searches >= 1 && search.peakForceN >= profile.contactThresholdN)),
          'MuJoCo contact stays inside its commissioned force range: peak ${search.peakForceN} N, ' +
          '${touched.searches} searched surfaces');
        check(mission.finished && mission.failure == null && mission.screwRuns.length == 1 && mission.screwResults.length >= 1,
          'Force search hands the first CAD fastening to the qualified driver: ${mission.failure}, finished ' +
          '${mission.finished}, step ${mission.stepIndex}, runs ${mission.screwRuns.length}, results ' +
          '${mission.screwResults.length}, time ${clock.simulationTime()}');
        for (result in mission.screwResults)
          check(result.outcome == robotkit.tool.ScrewdriverContract.OK && Math.abs(result.torqueNm - 16.5) < 1e-8,
            'Force-search entry preserves the unchanged qualified tightening program: ${result.fastening}, ' +
            'outcome ${result.outcome}, torque ${result.torqueNm}');
        var seated:robotkit.spatial.Transform3 = entry, start:robotkit.spatial.Transform3 = origin;
        var axes:processkit.screwdriving.ScrewEntryController = control;
        var delta = seated.translation.sub(start.translation);
        var lateral = Math.sqrt(Math.pow(delta.dot(axes.axisU), 2) + Math.pow(delta.dot(axes.axisV), 2));
        // The screw seats on the true hole axis, `offsetMeters` from where registration aimed, within the bore's play.
        check(Math.abs(lateral - offsetMeters) <= 0.0005,
          'The seated screw sits on the physical hole axis: ${lateral * 1000} mm from the aim, hole offset ' +
          '${offsetMeters * 1000} mm');
        Sys.println('Force-search CAD: offset ${offsetMeters * 1000} mm seated in ${Math.round(seconds * 100) / 100} s, ' +
          'peak ${search.peakForceN} N, ${axes.searches} searched surfaces');
        simulation.clear(); session.dispose();
      } catch (error:Dynamic) {simulation.clear(); session.dispose(); throw error;}
      Sys.println('Force-search CAD: end lateral offset ${offsetMeters*1000} mm');
    }
    if (forceCase == null)
      Sys.println("Force-search CAD passed with measured MuJoCo contact and offset-hole tightening handover");
    else
      Sys.println("Selected force-search CAD case passed");
  }

  /** Staged execution shares physical supplies and preserves the installed body's measured engagement. */
  static function checkScrewdrivingStaged(root:String):Void {
    var manifest = FileSystem.fullPath(root+"/machinekit/examples/robot-screwdriver/materia.staged.project.json");
    for (backend in [ApplicationSimulation.MUJOCO,ApplicationSimulation.DETERMINISTIC]) for (scenario in 0...5) {
      Sys.println('Screwdriving CAD: begin staged backend $backend, scenario $scenario');
      var generated = MateriaProjectRunner.loadProject(manifest), definition = generated.missions[0];
      if (definition == null || definition.steps.length != 8) throw "Staged CAD cell needs four snug and four final passes";
      var jobs:Array<materia.project.SceneScrewDefinition.SceneScrew> = [];
      for (index in 0...4) {
        var job = materia.project.SceneScrewInvocation.job(definition.steps[index].screw,definition.steps);
        if (job == null) throw "Staged CAD cell lost a canonical job";
        jobs.push(job);
      }
      check([for (job in jobs) job.screw].join(",") == "work/flange/screw1,work/flange/screw3,work/flange/screw2,work/flange/screw4",
        "Snug follows the same CAD cross order as final");
      var first = jobs[0], recovery = first.recovery;
      if (recovery == null) throw "Staged CAD cell lost its reserve supply";
      // The same explicitly weak test joint is qualified before both passes.
      // It seats at prevailing torque, then strips before the published final target.
      if (scenario == 2 || scenario == 3) first.simulation.stripTorque = 8;
      if (scenario >= 2) definition.steps = [definition.steps[0],definition.steps[4]];
      var count = scenario >= 2 ? 1 : 4;
      var session = new ProjectDocumentSession(null,false), simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        session.openGeneratedProject(generated,manifest);
        check(simulation.rebuild(session.sensors,session.scene,session),'Staged CAD cell builds: ${simulation.error}');
        var mission = simulation.missionPlayer(), clock = simulation.activeSession();
        if (mission == null || clock == null) throw "Staged CAD cell lost its live mission";
        var injected:Array<String> = [], observedStep = -1;
        while (!mission.finished && mission.failure == null && clock.simulationTime() < 800) {
          if (mission.stepIndex != observedStep) {
            observedStep = mission.stepIndex;
            Sys.println('Staged screw transition $observedStep at ${clock.simulationTime()}');
            if (observedStep == count) check(mission.screwRuns.length == count,
              "Every snug report is complete before the first final begins");
          }
          simulation.step();
          if (scenario == 1 || scenario == 3 || scenario == 4) {
            var tools = @:privateAccess simulation.tools;
            if (tools == null || tools.screwdrivers.length != 1) throw "Staged CAD cell lost its observer";
            var setup = @:privateAccess tools.screwdrivers[0].setups.get(2);
            var identities = scenario == 4 ? [first.screw,recovery.replacement.screw] : [first.screw];
            for (identity in identities) if (setup != null && setup.screw == simulation.environmentObject("project:"+identity) && injected.indexOf(identity) < 0) {
              var originalWork = setup.work;
              setup.work = () -> {var state = originalWork(); state.damagedLeadIn = true; return state;};
              injected.push(identity);
            }
          }
        }
        check(mission.finished && mission.failure == null && mission.completed == 2*count,
          'Staged mission completes backend $backend scenario $scenario: ${mission.failure}, step ${mission.stepIndex}, time ${clock.simulationTime()}');
        check(mission.screwRuns.length == 2*count,"Both stages retain their own reports");
        check(injected.length == (scenario == 4 ? 2 : scenario == 1 || scenario == 3 ? 1 : 0),
          "Snug damage targets only the exact intended bodies");
        for (index in 0...count) {
          var job = jobs[index], snug = mission.screwRuns[index], finalPass = mission.screwRuns[index+count];
          check(snug.fastening == job.fastening && finalPass.fastening == job.fastening &&
            snug.pass == processkit.screwdriving.FasteningRunReport.FasteningPass.SnugPass &&
            finalPass.pass == processkit.screwdriving.FasteningRunReport.FasteningPass.FinalPass &&
            snug.finished && finalPass.finished && snug.fault == null && finalPass.fault == null,
            "Reports identify each CAD connection and distinguish snug from final");
          if (scenario == 4) {
            check(snug.failed && snug.attempts.length == 2 && snug.installed() == null &&
              finalPass.failed && finalPass.skipped != null && finalPass.attempts.length == 0 && finalPass.installed() == null,
              "Exhausted snug explicitly skips final without invented measurements or another body");
          } else {
            var seated = snug.attempts[snug.attempts.length-1];
            check(!snug.failed && seated.result.outcome == robotkit.tool.ScrewdriverContract.OK &&
              seated.result.torqueNm == job.simulation.prevailingTorque,
              "Snug stops at the actual head seat with prevailing torque");
            var continued = finalPass.attempts[0];
            check(continued.screw == snug.installed() && continued.result.angle > seated.result.angle,
              "Final recontacts the actual installed body and retains cumulative angle from snug");
            check(snug.attempts.length == (index == 0 && (scenario == 1 || scenario == 3) ? 2 : 1),
              "Only the intended snug failure consumes its reserve");
            if (scenario == 2 || scenario == 3) {
              check(finalPass.failed && finalPass.attempts.length == (scenario == 2 ? 2 : 1) && finalPass.installed() == null,
                "Final shares the one reserve across passes and reports exhaustion without a third supply");
              for (attempt in finalPass.attempts) check(attempt.result.outcome == robotkit.tool.ScrewdriverContract.STRIPPED &&
                attempt.disposition == processkit.screwdriving.FasteningRunReport.ScrewDisposition.Rejected,
                "Every final NOK is measured and its actual body is rejected");
            } else {
              check(!finalPass.failed && finalPass.attempts.length == 1 && continued.result.outcome == robotkit.tool.ScrewdriverContract.OK &&
                Math.abs(continued.result.torqueNm-job.program.targetTorque) < 1e-8,
                "Final reaches the unchanged published torque using its retained joint");
              var installed = finalPass.installed();
              if (installed == null) throw "Final report lost its installed body";
              var body = simulation.environmentObject("project:"+installed);
              check(body != null && mission.assemblyParts.get("project:"+job.frame).carries(body),
                "The exact threaded fixture owns the final installed body");
              var actual = mission.assemblyParts.get("project:"+installed).pose();
              var live = mission.assemblyParts.get("project:"+job.frame).pose();
              var socket = AssemblyFrames.compose({x:actual.position[0],y:actual.position[1],z:actual.position[2],
                qx:actual.rotation[0],qy:actual.rotation[1],qz:actual.rotation[2],qw:actual.rotation[3]},job.socket);
              var seat = AssemblyFrames.compose({x:live.position[0],y:live.position[1],z:live.position[2],
                qx:live.rotation[0],qy:live.rotation[1],qz:live.rotation[2],qw:live.rotation[3]},job.seat);
              var offset = AssemblyFrames.compose(AssemblyFrames.inverse(seat),socket);
              check(Math.sqrt(offset.x*offset.x+offset.y*offset.y) <= job.simulation.leadInMm/1000 && Math.abs(offset.z) < 0.00015,
                'Final installed screw "$installed" remains on its actual CAD seat: ${haxe.Json.stringify(offset)}');
            }
          }
        }
        check(simulation.heldObjectIds().length == 0,"Staged completion leaves no screw on the bit");
        if (scenario > 0) {
          for (_ in 0...100) simulation.step();
          var bin = mission.assemblyParts.get("project:reject/floor").pose();
          for (report in mission.screwRuns) for (attempt in report.attempts)
            if (attempt.disposition == processkit.screwdriving.FasteningRunReport.ScrewDisposition.Rejected) {
              var rejectedBody = simulation.environmentObject("project:"+attempt.screw);
              if (rejectedBody == null) throw "Rejected staged screw lost its actual body";
              var dropped = mission.simulation.objectPose(rejectedBody);
              check(Math.abs(dropped.x-bin.position[0]) < 0.036 && Math.abs(dropped.y-bin.position[1]) < 0.036 &&
                dropped.z > bin.position[2] && dropped.z < bin.position[2]+0.06,
                'Actual rejected staged screw "${attempt.screw}" rests inside the physical bin: ${haxe.Json.stringify(dropped)}');
            }
        }
        simulation.clear(); session.dispose();
      } catch (error:Dynamic) {simulation.clear(); session.dispose(); throw error;}
      Sys.println('Screwdriving CAD: end staged backend $backend, scenario $scenario');
    }
    var portable = materia.project.SceneArtifact.decode(MateriaProjectRunner.exportArtifact(manifest));
    var mission = portable.missions[0], flags = portable.project;
    check(mission != null && mission.steps.length == 8 && flags != null && flags.dynamicParts.length == 8,
      "Portable staged CAD keeps eight passes and exactly eight distinct supplies");
    Sys.println("Screwdriving staged assembly passed on both backends with retained engagement and shared replacement recovery");
  }

  static function checkScrewdrivingOne(root:String, virtualNutrunner:Bool = false):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-screwdriver/materia.drive.project.json");
    for (backend in [ApplicationSimulation.MUJOCO, ApplicationSimulation.DETERMINISTIC]) for (absent in [false,true]) {
      if (virtualNutrunner) Sys.println('Virtual nutrunner CAD: begin backend $backend, absent $absent');
      else Sys.println('Screwdriving CAD: begin backend $backend, absent $absent');
      var generated = MateriaProjectRunner.loadProject(manifest);
      var work = generated.missions[0];
      if (work == null || work.steps.length != 1) throw "Drive-one CAD must publish one mission step";
      var authoredMission = virtualNutrunner ? haxe.Json.stringify(work) : null;
      var step = work.steps[0], job = materia.project.SceneScrewInvocation.job(step.screw,work.steps);
      if (job == null) throw "Drive-one CAD lost its named screw packet";
      var assembly = requiredAssemblyDefinition(generated.assemblyDefinition);
      var screws = [for (item in assembly.occurrences) if (item.id == job.screw) item];
      if (screws.length != 1) throw "Drive-one CAD lost its identified screw occurrence";
      var screwComponents = [for (item in assembly.definitions) if (item.id == screws[0].definition) item];
      if (screwComponents.length != 1) throw "Drive-one CAD lost its screw component";
      var screwPieces = screwComponents[0].collisionHulls;
      check(screwPieces != null && screwPieces.length > 1,"The CAD screw retains separate head, socket and shank collision pieces");
      if (Sys.getEnv("SCREWDRIVING_TRACE") == "1") Sys.println("Screwdriving CAD job: "+haxe.Json.stringify(job));
      check(step.kind == "screw" && job.screw == "work/flange/screw1" && job.frame == "work/plate" &&
        job.leadMm == 1 && job.program.targetTorque == 16.5,"The actual M6 12.9 fastening supplies its lead and published target");
      var session = new ProjectDocumentSession(null,false);
      var simulation = new ApplicationSimulation(new RobotWorld());
      simulation.virtualNutrunner = virtualNutrunner;
      simulation.setBackend(backend);
      try {
        session.openGeneratedProject(generated,manifest);
        check(simulation.rebuild(session.sensors,session.scene,session),'Drive-one CAD builds: ${simulation.error}');
        var mission = simulation.missionPlayer(), clock = simulation.activeSession();
        var body = simulation.environmentObject("project:"+job.screw);
        if (mission == null || clock == null || body == null) throw "Drive-one CAD lost its mission or identified free screw";
        if (absent) {
          var before = mission.simulation.objectPose(body);
          body.setPoseForNextTick(new nativekit.sim.SimPose(before.x+0.1,before.y,before.z,before.qx,before.qy,before.qz,before.qw));
        }
        var traceAt = 0.0;
        while (!mission.finished && mission.failure == null && clock.simulationTime() < 180) {
          simulation.step();
          if (Sys.getEnv("SCREWDRIVING_TRACE") == "1" && clock.simulationTime() >= traceAt) {
            var readings = [for (robot in simulation.snapshot().robots()) for (frame in robot.sensors.toArray())
              if (frame.kind == robotkit.tool.ScrewdriverContract.SENSOR_KIND) frame.values.toArray()];
            var arm = @:privateAccess mission.robot;
            var contacts = [for (contact in mission.simulation.robotContacts(arm.runtime)) if (contact.active) contact];
            Sys.println("Screwdriving CAD trace: "+haxe.Json.stringify({time:clock.simulationTime(),
              body:mission.simulation.objectPose(body),readings:readings,contacts:contacts,
              joints:[for (i in 0...arm.model.joints.length) arm.robot.snapshot().positions.get(i)],physical:[for (i in 0...arm.model.joints.length) {id:arm.model.joints[i].id,position:mission.simulation.jointSample(arm.parts[0].robotIndex,i).get_position()}]}));
            traceAt += 0.5;
          }
        }
        if (mission.failure != null && !absent) {
          var frames = [for (robot in simulation.snapshot().robots()) for (frame in robot.sensors.toArray())
            if (frame.sensorId == job.presentSensor || frame.kind == robotkit.tool.ScrewdriverContract.SENSOR_KIND)
              {id:frame.sensorId,values:frame.values.toArray()}];
          Sys.println("Screwdriving CAD diagnostic: "+haxe.Json.stringify({body:mission.simulation.objectPose(body),
            presenter:mission.assemblyParts.get("project:"+job.presenter).pose(),pickup:job.pickup,sensors:frames}));
        }
        var results = mission.screwResults;
        if (virtualNutrunner) {
          check(haxe.Json.stringify(work) == authoredMission,"The RKD6 nutrunner executes the unchanged authored SD6 mission");
          var tools = @:privateAccess simulation.tools;
          if (tools == null || tools.screwdrivers.length != 1) throw "Virtual nutrunner lost its mounted tool";
          var driver = tools.screwdrivers[0];
          check(Std.isOfType(driver.supply,processkit.simulation.VirtualNutrunnerSupply),"The CAD spindle uses the actual RKD6 supply");
          var supply = cast(driver.supply,processkit.simulation.VirtualNutrunnerSupply);
          check(haxe.Int64.compare(@:privateAccess supply.sequence,haxe.Int64.ofInt(0)) > 0,
            "The unchanged mission receives real seven-value SENSOR6 acquisitions");
          var toolIndex = @:privateAccess supply.robotIndex;
          var acquired = mission.simulation.virtualDeviceSensor(toolIndex,0).get_sample();
          var physical = mission.simulation.jointSample(toolIndex,@:privateAccess driver.spindleJoint);
          var defaults = new robotkit.runtime.VirtualDeviceOptions();
          var phaseNs = haxe.Int64.toFloat(defaults.offsetTicks) * 1e9 / haxe.Int64.toFloat(defaults.tickHz);
          var acquisitionNs = haxe.Int64.toFloat(acquired.get_sequence()) * mission.simulation.fixedTimestepSeconds * 1e9;
          check(acquired.get_value_count() == 7 &&
            Math.abs(haxe.Int64.toFloat(acquired.get_source_timestamp_ns())-phaseNs-acquisitionNs) < 1.0,
            "SENSOR6 retains the physical acquisition tick and the actual board oscillator phase");
          check(haxe.Int64.compare(acquired.get_sequence(),physical.get_step_index()) <= 0 &&
            physical.get_simulation_time()-acquisitionNs*1e-9 < processkit.screwdriving.ScrewdriverFrames.MAXIMUM_SILENCE,
            "UART feedback is a completed fresh physical acquisition, not a restamped host value");
          var arm = @:privateAccess mission.robot;
          var rejected = false;
          try mission.simulation.jointSample(toolIndex,arm.blueprint.jointCount) catch (_:Dynamic) rejected = true;
          check(rejected,"Physical joint queries reject an undeclared joint");
        }
        check(results.length == 1 && results[0].fastening == job.fastening,
          'The measured result names the actual fastening: ${mission.failure}');
        if (absent) {
          check(mission.failure != null && results[0].outcome == robotkit.tool.ScrewdriverContract.NO_SCREW,
            'The emptied physical nest reports no screw: ${mission.failure}');
        } else {
          check(mission.finished && mission.failure == null && results[0].outcome == robotkit.tool.ScrewdriverContract.OK &&
            Math.abs(results[0].torqueNm-job.program.targetTorque) < 1e-8 && results[0].angle > 2*Math.PI,
            'The real M6 reaches its target and completes retract on backend $backend: ${mission.failure}');
          for (_ in 0...50) simulation.step();
          var screw = mission.assemblyParts.get("project:"+job.screw), fixture = mission.assemblyParts.get("project:"+job.frame);
          var actual = screw.pose(), live = fixture.pose();
          var socket = AssemblyFrames.compose({x:actual.position[0],y:actual.position[1],z:actual.position[2],
            qx:actual.rotation[0],qy:actual.rotation[1],qz:actual.rotation[2],qw:actual.rotation[3]},job.socket);
          var seat = AssemblyFrames.compose({x:live.position[0],y:live.position[1],z:live.position[2],
            qx:live.rotation[0],qy:live.rotation[1],qz:live.rotation[2],qw:live.rotation[3]},job.seat);
          var offsetX = socket.x-seat.x, offsetY = socket.y-seat.y;
          check(Math.sqrt(offsetX*offsetX+offsetY*offsetY) <= job.simulation.leadInMm/1000 && Math.abs(socket.z-seat.z) < 0.00015 &&
            fixture.carries(body) && simulation.heldObjectIds().length == 0,
            'The M6 remains on its CAD seat after vacuum release, retraction and gravity: socket ${haxe.Json.stringify(socket)}, seat ${haxe.Json.stringify(seat)}');
          var visible = [for (pose in simulation.capturePresentationSnapshot().environment) if (pose.id == "project:"+job.screw) pose];
          var physical = mission.simulation.objectPose(body);
          check(visible.length == 1 && Math.abs(visible[0].position[0]-physical.x) < 1e-9 &&
            Math.abs(visible[0].position[1]-physical.y) < 1e-9 && Math.abs(visible[0].position[2]-physical.z) < 1e-9 &&
            Math.abs(visible[0].rotation[0]-physical.qx) < 1e-9 && Math.abs(visible[0].rotation[1]-physical.qy) < 1e-9 &&
            Math.abs(visible[0].rotation[2]-physical.qz) < 1e-9 && Math.abs(visible[0].rotation[3]-physical.qw) < 1e-9,
            "The screw's runtime presentation carries the actual rotated body pose for its cosmetic-thread children");
        }
        if (virtualNutrunner) {
          var tools = @:privateAccess simulation.tools;
          if (tools == null || tools.screwdrivers.length != 1) throw "Shared process proof lost its actual CAD tool";
          var driver = tools.screwdrivers[0];
          var supply = cast(driver.supply,processkit.simulation.VirtualNutrunnerSupply);
          var vacuum = supply.controls().vacuum;
          var common = new processkit.screwdriving.ScrewdriverProcessDevice(
            new processkit.screwdriving.ChannelScrewdriverOutputs(driver.runtime,driver.channels),driver,driver.channels,job.slot);
          var actualFault = processkit.screwdriving.ScrewdriverSensor.faultMessage(driver.reading().result);
          common.prepare();
          check(common.ready() == (actualFault == null) && common.fault() == actualFault,
            "The shared process boundary retains measured controller feedback without fabricating a presenter-failure NOK");
          common.apply([new robotkit.execution.FiredProcessEvent(Int64.ofInt(0),driver.channels.start,
            ProcessEventValue.Digital(false),Int64.ofInt(0),Int64.ofInt(0),0)]);
          common.safe(); for (_ in 0...50) simulation.step();
          var controls = supply.controls();
          check(!common.ready() && !controls.start && !controls.reverse && controls.program == job.slot && controls.vacuum == vacuum,
            "Shared named process channels execute on RKD6 and controlled safe retains vacuum");
        }
      } catch (error:Dynamic) { simulation.clear(); session.dispose(); throw error; }
      simulation.clear(); session.dispose();
      if (virtualNutrunner) Sys.println('Virtual nutrunner CAD: end backend $backend, absent $absent');
      else Sys.println('Screwdriving CAD: end backend $backend, absent $absent');
    }
    if (virtualNutrunner) Sys.println("Virtual nutrunner CAD: begin portable export");
    else Sys.println("Screwdriving CAD: begin portable export");
    var portable = materia.project.SceneArtifact.decode(MateriaProjectRunner.exportArtifact(manifest));
    var flags = portable.project, task = portable.missions[0];
    check(flags != null && flags.dynamicParts.length == 1 && flags.dynamicParts[0] == "work/flange/screw1" &&
      task != null && task.steps.length == 1 && task.steps[0].screw != null,
      "The portable artifact retains the identified free screw and its CAD mission");
    if (virtualNutrunner) Sys.println("Virtual nutrunner CAD: end portable export");
    else Sys.println("Screwdriving CAD: end portable export");
    Sys.println("Screwdriving CAD mission passed on MuJoCo and the test backend, including the empty presenter");
  }

  static function checkRobotArm(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-arm/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition:AssemblyDefinition = cast(generated.assemblyDefinition, AssemblyDefinition);
    var placement = new AssemblyState(definition, generated.assemblyState);
    check(generated.robotMotions == null || generated.robotMotions.length == 0, "the arm project ships no joint tracks");
    var tools = generated.robotTools;
    check(tools != null && tools.length == 1 && tools[0].sensor != null, "the arm's suction tool reports on a vacuum sensor");
    var work:materia.project.SceneArtifact.SceneArtifactMission = cast generated.missions[0];
    check(work != null && work.loop == true && [for (step in work.steps) step.kind].join(",") == "pick,place,pick,place",
      "the arm project carries the workpiece to the other pad and back, on repeat");
    var steps = work.steps;
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    checkArmHierarchy(session);
    var armLimits = checkLimitsFromDrives(session, ["j1", "j2", "j3", "j4", "j5", "j6"], "the arm");
    var j1 = armLimits.get("j1"), j6 = armLimits.get("j6");
    // The 200 W servo (1.91 N m peak, 5000 rpm) through a 250:1 gearbox at 0.85; the 50 W one through 130:1.
    check(j1 != null && j6 != null && Math.abs(j1[0] - 2.094) < 0.001 && Math.abs(j1[1] - 405.9) < 0.5 &&
      Math.abs(j6[0] - 4.028) < 0.001 && Math.abs(j6[1] - 52.7) < 0.2, 'the arm joints run to ${j1} and ${j6}');
    checkArmDrag(session, generated.metresPerUnit);
    checkFreeParts(definition, generated.physical);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    check(simulation.rebuild(session.sensors, session.scene, session),
      "robot arm builds in the shared simulation: " + simulation.error);
    var mission = simulation.missionPlayer();
    if (mission == null) throw "the robot arm has no mission";
    simulation.step();
    function pose(id:String):app.ApplicationSimulation.SimulationPoseVisual {
      var found = [for (entry in simulation.capturePresentationSnapshot().environment) if (entry.id == id) entry];
      check(found.length == 1, 'robot arm publishes a pose for $id');
      return found[0];
    }
    var startPoses = simulation.capturePresentationSnapshot().environment;
    check([for (entry in startPoses) if (StringTools.startsWith(entry.id, "project:")) entry].length ==
      definition.occurrences.length, "robot arm publishes a pose for every part, tool and free workpiece included");
    var startWork = pose("project:workpiece").position.copy();
    var lift = 0.0, lastDone = 0, log:Array<String> = [];
    var previousPlanningSeconds = 0.0, previousIkSolves = 0;
    while (mission.completed < steps.length && simulation.activeSession().simulationTime() < 120) {
      simulation.step();
      var failure = mission.failure;
      if (failure != null) throw 'the arm mission failed after ${mission.completed} steps: $failure';
      var held = simulation.heldObjectIds();
      check(held.length == 0 || held.join(",") == "project:workpiece", "only the workpiece is ever held (" + held.join(",") + ")");
      lift = Math.max(lift, pose("project:workpiece").position[2] - startWork[2]);
      if (mission.completed == lastDone) continue;
      var step = steps[lastDone];
      if (Sys.getEnv("PROCESS_PATH_BENCHMARK") == "1") {
        var handling = handlingRunner(mission, singleHandlingChannel(mission));
        var metrics = handling.motion.planningMetrics();
        Sys.println("PROCESS_PATH_RUN " + haxe.Json.stringify({example: "handling", backend: ApplicationSimulation.MUJOCO,
          run: mission.completed, planningSeconds: metrics.seconds - previousPlanningSeconds,
          numericIkSolves: metrics.numericIkSolves - previousIkSolves}));
        previousPlanningSeconds = metrics.seconds;
        previousIkSolves = metrics.numericIkSolves;
      }
      if (step.kind == "pick") {
        check(held.join(",") == "project:workpiece", 'step $lastDone picks the workpiece up');
      } else {
        check(held.length == 0, 'step $lastDone lets the workpiece go');
        for (_ in 0...50) simulation.step();
        var at:materia.project.SceneArtifact.SceneArtifactPlace = cast step.at;
        var seat = placement.worldConnector(at.occurrence, at.connector);
        var metres = generated.metresPerUnit;
        var piece = pose("project:workpiece").position;
        var off = Math.sqrt(Math.pow(piece[0] - seat.x * metres, 2) + Math.pow(piece[1] - seat.y * metres, 2));
        check(off < 0.01, 'step $lastDone sets the workpiece ${off} m from the centre of ${at.occurrence}');
        check(piece[2] > seat.z * metres && piece[2] < seat.z * metres + 0.05,
          'step $lastDone leaves the workpiece standing on ${at.occurrence} (centre at ${piece[2]} m)');
      }
      log.push('${step.kind} ${Math.round(simulation.activeSession().simulationTime() * 10) / 10}');
      lastDone = mission.completed;
    }
    check(mission.completed >= steps.length, 'the arm runs its whole round in two minutes, finished ${mission.completed} steps');
    check(lift > 0.05, 'the workpiece is lifted off its pad (rose at most $lift m)');
    if (Sys.getEnv("PROCESS_PATH_BENCHMARK") == "1")
      Sys.println("PROCESS_PATH_QUALITY " + haxe.Json.stringify({example: "handling", backend: ApplicationSimulation.MUJOCO,
        completed: mission.completed, cycleSeconds: simulation.activeSession().simulationTime(), liftMetres: lift}));
    // A reset puts the workpiece back and starts the mission over.
    check(simulation.reset(), "the arm simulation resets");
    check(simulation.heldObjectIds().length == 0, "a reset holds nothing");
    simulation.step();
    var restored = pose("project:workpiece").position;
    check(Math.abs(restored[0] - startWork[0]) < 0.002 && Math.abs(restored[2] - startWork[2]) < 0.002,
      "a reset puts the workpiece back on its pad");
    var again = false;
    var until = simulation.activeSession().simulationTime() + 30;
    while (!again && simulation.activeSession().simulationTime() < until) {
      simulation.step();
      again = simulation.heldObjectIds().length > 0;
    }
    check(again, 'the mission picks again after a reset (${mission.failure})');
    // A reset in the middle of a step cancels that step through the robot's runtime, still answering, and the
    // mission starts over without a fault.
    check(simulation.heldObjectIds().length > 0, "the workpiece is held when the reset comes");
    check(simulation.reset(), "the arm simulation resets with a step running");
    check(mission.failure == null && mission.stepIndex == 0 && mission.completed == 0,
      'a reset mid-step starts the mission over without a failure (${mission.failure})');
    var thrice = false;
    until = simulation.activeSession().simulationTime() + 30;
    while (!thrice && simulation.activeSession().simulationTime() < until) {
      simulation.step();
      thrice = simulation.heldObjectIds().length > 0;
    }
    check(thrice && mission.failure == null, 'the mission picks again after a reset mid-step (${mission.failure})');
    session.dispose();
    Sys.println('robot arm mission: ${log.join(", ")} s');
  }

  /**
   * The welding cell opens with its torch and a mission that welds one T-joint fillet. On MuJoCo the arm welds it: the
   * simulated welder lights the arc at the seam's start, the torch follows the seam, and the bead the welder deposited
   * reaches the leg the weldment asked for along the seam's length. A second run loses the arc part-way: the weld is
   * interrupted, the torch re-approaches and restarts with an overlap, and the bead is continuous.
   */
  static function checkTrackArm(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/track-arm/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      check(simulation.rebuild(session.sensors, session.scene, session), "track arm builds: " + simulation.error);
      var mission = simulation.missionPlayer();
      if (mission == null) throw "Track arm has no mission";
      var handlingChannel = singleHandlingChannel(mission);
      var indices = handlingRunner(mission, handlingChannel).motion.jointIndices;
      check(indices.length == 7 && mission.robot.model.joints[indices[0]].name == "track",
        "track leads six logical arm joints");
      var travel = 0.0, lastDone = 0;
      while (!mission.finished && simulation.activeSession().simulationTime() < 180) {
        simulation.step();
        check(mission.failure == null, "track positioning mission: " + mission.failure);
        var positions = mission.robot.robot.snapshot().positions;
        travel = Math.max(travel, positions.get(indices[0]));
        if (mission.completed > lastDone) {
          Sys.println('track arm positioning step ${mission.completed}: ${simulation.activeSession().simulationTime()} s, track ${positions.get(indices[0])} m');
          lastDone = mission.completed;
        }
      }
      check(mission.finished && mission.completed == 6, "both track stations complete");
      check(travel >= 2.9, "track carries the arm between table stations");
      var work = [for (part in simulation.capturePresentationSnapshot().environment) if (part.id == "project:workpiece") part];
      check(work.length == 1 && Math.abs(work[0].position[0] - 2.85) < 0.005, "part reaches the far landing pad");
      var make = mission.newHandlingByChannel.get(handlingChannel);
      if (make == null) throw "Track arm cannot create coordinated motion";
      var runner = make();
      check(Math.abs(runner.home[0] - 3.0) < 0.001, "handling retains its current external station");
      checkTrackArmPath(simulation, mission, runner.motion);
    } catch (error:Dynamic) { simulation.clear(); session.dispose(); throw error; }
    simulation.clear(); session.dispose();
    checkWholeWeldment(root, ApplicationSimulation.MUJOCO, "track MuJoCo", "track-arm");
  }

  static function checkTrackArmPath(simulation:ApplicationSimulation, mission:MissionPlayer,
      motion:motionkit.robot.ManipulatorMotion):Void {
    var handlingChannel = singleHandlingChannel(mission);
    var solver = motion.compiler.solver;
    var entry = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    motion.run(new motionkit.program.MotionProgram([
      motionkit.program.MotionOp.MoveJ(motionkit.program.MoveTarget.JointTarget(entry),
        new motionkit.MotionOptions(), motionkit.program.Blend.ExactStop)]));
    var limit = simulation.activeSession().simulationTime() + 60;
    while (!motion.completed && motion.failure == null && simulation.activeSession().simulationTime() < limit) {
      motion.update(simulation.timestep); simulation.step();
    }
    check(motion.completed && motion.failure == null, "track coordinated entry: " + motion.failure);
    var start = solver.forward(entry);
    var stop = new motionkit.kinematics.Pose3(start.x + 2.6, start.y, start.z, start.qx, start.qy, start.qz, start.qw);
    var path = new motionkit.path.PosePath(motionkit.robot.HandlingPlanRunner.FRAME, [new motionkit.path.PoseLine(
      new motionkit.path.PoseWaypoint(start, 0.002, 0.02), new motionkit.path.PoseWaypoint(stop, 0.002, 0.02),
      motionkit.path.OrientationPolicy.FreeAboutTool, 0.1, 0.08)]);
    // 1.2 m conservatively bounds this arm's links, joint housings and tool reach.
    check(path.length() > 2 * 1.2, "coordinated path exceeds twice the arm's reach");
    var kinematics:motionkit.robot.ManipulatorKinematics = cast solver;
    kinematics.preferredPosture = entry;
    check(kinematics.manipulator.external[0] && kinematics.manipulator.swivel == null,
      "coordinated planning derives the track without inventing a swivel");
    // Coordinated travel explicitly searches the rail. The handling runner's
    // default holds station axes while it picks and places at each station.
    var rail=kinematics.manipulator.group.limitsOf(0);
    var cells=Std.int(Math.ceil((rail.upper-rail.lower)/(motion.compiler.perJointMaxJump[0]*0.5)))+1;
    var sampling=new motionkit.robot.CandidateProblem.CandidateSamplingOptions(8,3,8,true,
      [new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(0,rail.lower,rail.upper,cells)]);
    var coordinated=motion.compiler.withJointPathPlanner(new motionkit.robot.StructuredJointPathPlanner(
      kinematics.manipulator,sampling,null,null,8,false,null,null,null,0,kinematics));
    motion=new motionkit.robot.ManipulatorMotion(motion.robot,coordinated,_ -> null,
      () -> ({events:[],overflow:false}),motion.jointIndices);
    motion.run(new motionkit.program.MotionProgram([
      motionkit.program.MotionOp.FollowPath(path, motionkit.robot.HandlingPlanRunner.FRAME, 0.08, [])]));
    var worst = 0.0, margin = Math.POSITIVE_INFINITY, jump = 0.0, previous = entry.copy(), last = entry.copy();
    limit = simulation.activeSession().simulationTime() + 120;
    while (!motion.completed && motion.failure == null && simulation.activeSession().simulationTime() < limit) {
      motion.update(simulation.timestep); simulation.step();
      var feedback = mission.robot.robot.snapshot().positions;
      last = [for (index in motion.jointIndices) feedback.get(index)];
      var tip = mission.toolContact(handlingChannel);
      var along = Math.max(0.0, Math.min(2.6, tip.x - start.x));
      worst = Math.max(worst, Math.sqrt(Math.pow(tip.x - start.x - along, 2) + Math.pow(tip.y - start.y, 2) + Math.pow(tip.z - start.z, 2)));
      for (index in 1...7) {
        var bound = kinematics.manipulator.group.limitsOf(index);
        margin = Math.min(margin, Math.min(last[index] - bound.lower, bound.upper - last[index]));
        jump = Math.max(jump, Math.abs(last[index] - previous[index]));
      }
      check(Math.PI/2 - 1.4 + last[3] < Math.PI/2 - 0.2, "coordinated path retains its elbow branch");
      check(Math.abs(last[6] - entry[6]) < 1.0, "coordinated path retains its wrist winding");
      previous = last.copy();
    }
    check(motion.completed && motion.failure == null, "track coordinated path: " + motion.failure);
    var end = mission.toolContact(handlingChannel);
    check(Math.sqrt(Math.pow(end.x - stop.x, 2) + Math.pow(end.y - stop.y, 2) + Math.pow(end.z - stop.z, 2)) < 0.002, "coordinated TCP reaches its endpoint within 2 mm");
    check(worst < 0.002, 'track path error stays within 2 mm ($worst m)');
    check(last[0] - entry[0] > 0.8 * 2.6, "track supplies most of the coordinated travel");
    check(margin > 0.15 && jump < 0.05, 'arm retains 0.15 rad posture margin without branch jumps ($margin margin, $jump jump)');
    Sys.println('track arm coordinated: 2.6 m path, ${last[0] - entry[0]} m track travel, ${worst * 1000} mm error, $margin rad posture margin, $jump rad maximum joint step');
  }

  static function checkRobotWelder(root:String):Void {
    checkWholeWeldment(root, ApplicationSimulation.MUJOCO, "MuJoCo");
    checkWholeWeldment(root, ApplicationSimulation.DETERMINISTIC, "test backend");
    checkRobotWelderOn(root, ApplicationSimulation.MUJOCO, "MuJoCo");
    checkRobotWelderOn(root, ApplicationSimulation.DETERMINISTIC, "test backend");
    checkWeldInTheAir(root);
    checkWeldCraterFault(root);
    checkWeldFollowsWorkpiece(root);
    checkWeldPost(root);
    checkWeldQualities(root);
  }

  static function checkWeldQuality(root:String, legMm:Float, backend:Int, label:String):Void {
    var cell = openWelder(root, legMm == 7 ? "materia.weave.project.json" : "materia.multipass.project.json", null, backend);
    var mission = cell.mission, welder = cell.welder, simulation = cell.simulation;
    var bead = cell.beads.beadOf(0).beads[0];
    var strikes = 0, previousArc = false, tick = 0;
    var limit = simulation.activeSession().simulationTime() + 300;
    while (!mission.finished && simulation.activeSession().simulationTime() < limit) {
      simulation.step();
      if (mission.failure != null) throw '$legMm mm weld: ${mission.failure}';
      var arc = welder.reading().arc;
      if (arc && !previousArc) {
        strikes++;
        if (strikes > 1) {
          var previousLeg = bead.meanLeg(0.3, 0.7);
          check(previousLeg > (strikes == 2 ? 0.0045 : 0.0074), "later pass starts after preceding bead is deposited");
          var deposited = new processkit.tool.WeldBeadWork(bead, () -> cell.beads.referenceOf(0));
          var tip = welder.tip();
          check(deposited.distance(tip[0], tip[1], tip[2]) < 0.0006, "later pass strikes on earlier bead metal");
          check(mission.depositedWeldHulls().length > 0, "later pass has deposited clearance geometry");
        }
      }
      previousArc = arc;
      tick++;
      if (tick % 5 == 0) {
        var local = cell.beads.toFrame(0, welder.tip());
        var along = 0.0;
        for (axis in 0...3) along += (local[axis] - bead.start[axis]) * bead.tangent[axis];
        var nearest = bead.pointAt(Math.max(0, Math.min(bead.length, along))), distance = 0.0;
        for (axis in 0...3) distance += Math.pow(local[axis] - nearest[axis], 2);
        var violation = mission.clearanceViolation(Math.sqrt(distance) <= processkit.WeldPathPlanner.CONTACT_ZONE);
        check(violation == null, violation == null ? "weld remains clear" : '$legMm mm clearance: ${violation.a}/${violation.b}');
      }
    }
    check(mission.finished && mission.completed == 1, "quality weld finishes");
    check(strikes == (legMm == 7 ? 1 : 3), "recipe executes the requested pass count without restarts");
    var achieved = bead.meanLeg(0.15, 0.85);
    check(Math.abs(achieved - legMm * 0.001) <= 0.0005, '$legMm mm target: measured ${achieved * 1000} mm');
    check(bead.gaps() == 0 && Math.abs(bead.extent() - bead.length) <= 0.002, "all passes cover the CAD seam");
    check(!welder.reading().arc, "quality weld finishes with arc off");
    Sys.println('robot welder ($label): $legMm mm target, ${strikes} strikes, leg ${achieved * 1000} mm, ' +
      '${simulation.activeSession().simulationTime()} s, ' + mission.planReport());
    simulation.clear();
    cell.session.dispose();
  }

  static function checkWeldQualities(root:String):Void {
    for (leg in [7.0, 10.0]) {
      checkWeldQuality(root, leg, ApplicationSimulation.MUJOCO, "MuJoCo");
      checkWeldQuality(root, leg, ApplicationSimulation.DETERMINISTIC, "test backend");
    }
  }

  /** The plate fillet on the side away from the arm: its nearest endpoint can have no reachable approach. */
  static function checkFarPlateWeld(root:String):Void {
    var cell = openWelder(root, "materia.project.json", function(generated) {
      var mission = cast(generated.missions[0], materia.project.SceneArtifact.SceneArtifactMission);
      var singles:Array<{step:materia.project.SceneArtifact.SceneArtifactMissionStep, y:Float}> = [];
      for (step in mission.steps) {
        var weld = step.weld;
        if (step.kind == "weld" && weld != null && weld.path.length == 1)
          singles.push({step: step, y: weld.path[0].start.position[1]});
      }
      check(singles.length == 2, "the CAD weldment has two plate fillets");
      singles.sort(function(a, b) return Reflect.compare(a.y, b.y));
      mission.steps = [singles[0].step];
    }, ApplicationSimulation.MUJOCO);
    var simulation = cell.simulation, mission = cell.mission;
    while (!mission.finished && simulation.activeSession().simulationTime() < 120) {
      simulation.step();
      if (mission.failure != null) throw 'far plate seam: ${mission.failure}';
    }
    check(mission.finished && mission.completed == 1, "the far plate seam finishes");
    var bead = cell.beads.beadOf(0);
    check(bead.gaps() == 0, "the far plate seam has no bead gap");
    var leg = bead.beads[0].meanLeg(0.3, 0.7);
    check(Math.abs(leg - 0.005) <= 0.0005, 'the far plate seam leg is within 0.5 mm of 5 mm: ${leg * 1000} mm');
    check(Math.abs(bead.beads[0].extent() - bead.beads[0].length) <= 0.002, "the far plate bead covers its CAD seam within 2 mm");
    Sys.println('robot welder: far plate seam, ${simulation.activeSession().simulationTime()} s, leg ${leg * 1000} mm, ' + mission.planReport());
    simulation.clear();
    cell.session.dispose();
  }

  /**
   * The cell's default mission welds the whole weldment: one step per run of seams (the plate's two sides, each tube post's
   * four as a closed run), planned reachable and clear. Every seam gets a bead as long as the seam with the leg the weldment
   * asked for, the wire tip stays on the seams, the arm and torch never come closer to the cell than the planner's margins
   * (measured on the simulated arm's real joints), and no arc is lost. The cycle time is reported.
   */
  static function checkWholeWeldment(root:String, backend:Int, label:String, example:String = "robot-welder"):Void {
    var onlyStep = example == "gantry-welder" ? Sys.getEnv("GANTRY_WELD_ONLY_STEP") : null;
    var cell = openWelder(root, example == "track-arm" ? "materia.welder.project.json" : "materia.project.json", onlyStep == null ? null : function(generated) {
      var mission:materia.project.SceneArtifact.SceneArtifactMission = cast generated.missions[0];
      var index = Std.parseInt(onlyStep);
      if (index == null || index < 0 || index >= mission.steps.length) throw "Invalid gantry weld step filter";
      mission.steps = [mission.steps[index]];
    }, backend, example);
    var simulation = cell.simulation, mission = cell.mission, welder = cell.welder, beads = cell.beads;
    var steps = mission.mission.steps;
    var expectedRuns = onlyStep == null && example != "track-arm" ? 4 : 1;
    check(steps.length == expectedRuns, 'the weld mission has $expectedRuns runs, got ${steps.length}');
    var seams = 0;
    for (step in steps) seams += cast(step.weld, materia.project.SceneArtifact.SceneArtifactWeld).path.length;
    check(seams == (onlyStep == null && example != "track-arm" ? 10 : cast(steps[0].weld, materia.project.SceneArtifact.SceneArtifactWeld).path.length), 'the runs hold the requested CAD seams, got $seams');
    var strayed = 0.0, nearest = 1e9, wireError = 0.0;
    var violation:Null<String> = null;
    var trackStart:Null<Float> = null, trackLast = 0.0;
    var armMargin = Math.POSITIVE_INFINITY, armJump = 0.0;
    var previousArm:Null<Array<Float>> = null;
    var restarts = 0, done = 0, tick = 0;
    var finished:Array<Float> = [];
    var limit = simulation.activeSession().simulationTime() + 600;
    while (!mission.finished && simulation.activeSession().simulationTime() < limit) {
      simulation.step();
      tick++;
      if (mission.failure != null) throw '$label: the weldment weld failed after ${simulation.activeSession().simulationTime()} s: ${mission.failure}; reading=${haxe.Json.stringify(welder.reading())}, tip=${welder.tip()}, max seam error=$strayed, wire angle=$wireError';
      if (mission.completed > done) {
        // The runner's restarts still belong to the weld that just finished.
        restarts += mission.weldRestarts();
        done = mission.completed;
        Sys.println('welder ($label): run $done planned in ' + mission.planReport());
        if (Sys.getEnv("PROCESS_PATH_BENCHMARK") == "1") {
          var planning:processkit.WeldingPlanRunner = cast mission.welding;
          var chosen:processkit.WeldPathPlanner.PlannedWeld = cast planning.lastPlan();
          Sys.println("PROCESS_PATH_RUN " + haxe.Json.stringify({example: example, backend: backend,
            run: done, planningSeconds: planning.planningSeconds, checkedPoses: chosen.checked,
            numericIkSolves: planning.planningIkSolves}));
        }
        finished.push(Math.round(simulation.activeSession().simulationTime() * 10) / 10);
      }
      var step = mission.weldingStep();
      if (step >= 0) {
        var tip = beads.toFrame(step, welder.tip());
        // How far the tip is from the seams of the step being welded: the torch is working there if close.
        var best = 1e9;
        for (bead in beads.beadOf(step).beads) {
          var relative = [for (axis in 0...3) tip[axis] - bead.start[axis]];
          var along = relative[0] * bead.tangent[0] + relative[1] * bead.tangent[1] + relative[2] * bead.tangent[2];
          var clamped = Math.min(bead.length, Math.max(0.0, along));
          best = Math.min(best, Math.sqrt(Math.pow(relative[0] - bead.tangent[0] * clamped, 2) + Math.pow(relative[1] - bead.tangent[1] * clamped, 2) +
            Math.pow(relative[2] - bead.tangent[2] * clamped, 2)));
        }
        nearest = Math.min(nearest, best);
        if (welder.reading().arc) {
          strayed = Math.max(strayed, best);
          if (example == "gantry-welder" || example == "track-arm") {
            var welding:processkit.WeldingPlanRunner = cast mission.welding;
            var motion = welding.motion;
            var observed = mission.robot.robot.snapshot();
            if (example == "track-arm") {
              var solver:motionkit.robot.ManipulatorKinematics = cast motion.compiler.solver;
              check(solver.manipulator.external[0] && solver.manipulator.swivel == null,
                "track welding derives one external axis without a swivel");
              var q = [for (index in motion.jointIndices) observed.positions.get(index)];
              if (trackStart == null) trackStart = q[0];
              trackLast = q[0];
              for (index in 1...7) {
                var bound = solver.manipulator.group.limitsOf(index);
                armMargin = Math.min(armMargin, Math.min(q[index] - bound.lower, bound.upper - q[index]));
                if (previousArm != null) armJump = Math.max(armJump, Math.abs(q[index] - previousArm[index]));
              }
              previousArm = q;
            }
            var seam:motionkit.path.PosePath = cast welding.seam;
            var desired = seam.poseAt(welding.travelled());
            var actual = motion.compiler.solver.forward([for (index in motion.jointIndices) observed.positions.get(index)]);
            var axis = new robotkit.spatial.Vec3(0, 0, 1);
            var wanted = new robotkit.spatial.Quat(desired.qx, desired.qy, desired.qz, desired.qw).rotate(axis);
            var seen = new robotkit.spatial.Quat(actual.qx, actual.qy, actual.qz, actual.qw).rotate(axis);
            wireError = Math.max(wireError, Math.acos(Math.max(-1.0, Math.min(1.0, wanted.dot(seen)))));
          }
        }
        if (tick % 5 == 0 && violation == null) {
          var found = mission.clearanceViolation(best <= processkit.WeldPathPlanner.CONTACT_ZONE);
          if (found != null) violation = '${found.a} is ${Math.round(found.distance * 10000) / 10} mm from ${found.b} (needs ${Math.round(found.required * 10000) / 10}) at ${Math.round(simulation.activeSession().simulationTime() * 10) / 10} s';
        }
      }
    }
    var time = Math.round(simulation.activeSession().simulationTime() * 10) / 10;
    check(mission.finished && mission.failure == null, '$label: the whole weldment is welded within $time s: ${mission.failure}');
    check(done == expectedRuns, '$label: $expectedRuns welds finished, got $done');
    check(violation == null, '$label: no clearance violation along the way: $violation');
    if (example == "gantry-welder" || example == "track-arm") check(wireError <= 2 * Math.PI / 180,
      '$label: work/travel directions stay within 2 degrees of the authored process wire axis ($wireError rad)');
    check(restarts == 0, '$label: the weldment is welded without losing the arc ($restarts restarts)');
    check(strayed < 0.0015, '$label: the wire tip stayed within ${strayed * 1000} mm of the seams while the arc burned');
    if (example == "track-arm") {
      Sys.println('track weld: ${Math.abs(trackLast - (trackStart == null ? 0.0 : trackStart))} m track travel, $armMargin rad posture margin, $armJump rad maximum joint step');
      check(trackStart != null && Math.abs(trackLast - (trackStart == null ? 0.0 : trackStart)) > 0.7 * 2.6,
        'track carries at least 70% of the 2.6 m weld ($trackStart to $trackLast m)');
      check(armMargin > 0.15 && armJump < 0.05,
        'track welding keeps 0.15 rad arm margin without IK branch jumps ($armMargin margin, $armJump jump)');
      check(beads.beads.length == 1 && beads.beads[0].bead.length > 2 * 1.2,
        "CAD weld seam exceeds twice the conservative 1.2 m arm reach");
    }
    var legs:Array<String> = [];
    var lengths:Array<String> = [];
    for (entry in beads.beads) {
      var weld = entry.weld;
      var bead = entry.bead;
      var leg = bead.meanLeg(0.3, 0.7);
      legs.push(Std.string(Math.round(leg * 10000) / 10));
      lengths.push(Std.string(Math.round(bead.extent() * 10000) / 10));
      check(Math.abs(leg - weld.legSize) <= 0.0005, '$label: step ${entry.step} segment ${entry.segment}: the bead leg ${leg * 1000} mm is within 0.5 mm of the ${weld.legSize * 1000} mm asked for');
      check(Math.abs(bead.extent() - bead.length) <= 0.002, '$label: step ${entry.step} segment ${entry.segment}: the bead is as long as the seam (${bead.extent() * 1000} mm of ${bead.length * 1000})');
    }
    for (step in 0...steps.length) {
      var path = beads.beadOf(step);
      check(path.gaps() == 0 && path.stray < 0.1 * path.deposited, '$label: step $step: the bead has no gap and little missed the seams');
    }
    Sys.println('welder ($label): whole weldment, ${steps.length} runs and $seams seams, cycle ${time} s (runs done at ${finished.join(", ")} s), ' +
      'tip within ${Math.round(strayed * 10000) / 10} mm of the seams, legs ${legs.join("/")} mm, bead lengths ${lengths.join("/")} mm, no clearance violation');
    if (Sys.getEnv("PROCESS_PATH_BENCHMARK") == "1")
      Sys.println("PROCESS_PATH_QUALITY " + haxe.Json.stringify({example: example, backend: backend,
        runs: done, seams: seams, cycleSeconds: time, clearanceViolation: violation,
        trackTravelMetres: example == "track-arm" ? Math.abs(trackLast - (trackStart == null ? 0.0 : trackStart)) : null,
        armMarginRadians: example == "track-arm" ? armMargin : null,
        maxArmStepRadians: example == "track-arm" ? armJump : null,
        seamErrorMetres: strayed, wireErrorRadians: wireError,
        legsMillimetres: [for (entry in beads.beads) entry.bead.meanLeg(0.3, 0.7) * 1000],
        lengthsMillimetres: [for (entry in beads.beads) entry.bead.extent() * 1000]}));
    simulation.clear();
    cell.session.dispose();
  }

  static function checkGantryWelder(root:String):Void
    checkWholeWeldment(root, ApplicationSimulation.MUJOCO, "gantry MuJoCo", "gantry-welder");

  /** A welder cell open in the simulation on MuJoCo, from one of the example's manifests. */
  static function openWelder(root:String, manifestName:String, ?adjust:app.GeneratedAssemblyScene -> Void,
      ?backend:Int, example:String = "robot-welder"):{session:ProjectDocumentSession, simulation:ApplicationSimulation, mission:MissionPlayer, welder:processkit.simulation.SimulatedWelder,
      beads:WeldBeads} {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/" + example + "/" + manifestName);
    var generated = MateriaProjectRunner.loadProject(manifest);
    if (adjust != null) adjust(generated);
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(backend == null ? ApplicationSimulation.MUJOCO : backend);
    check(simulation.rebuild(session.sensors, session.scene, session), "the welding cell builds: " + simulation.error);
    var mission = simulation.missionPlayer(), welder = simulation.welder(), beads = simulation.weldBeads();
    if (mission == null || welder == null || beads == null) throw "the welding cell has no mission, welder or weld metal";
    if (example == "gantry-welder") {
      var welding:processkit.WeldingPlanRunner = cast mission.welding;
      var motion = welding.motion;
      check(motion.compiler.solver.jointCount() == 5, "the gantry welds with XYZ and CA");
      var solver:motionkit.robot.ManipulatorKinematics = cast motion.compiler.solver;
      check(processkit.WeldingPlanRunner.wristJointIndices(solver.manipulator).length == 2,
        "the gantry welding wrist derives its two rotary limits");
    }
    return {session: session, simulation: simulation, mission: mission, welder: welder, beads: beads};
  }

  /**
   * A dropout of the supply while the crater is being filled, after the torch has travelled the whole seam, does not make
   * the torch travel the seam again: the arc ends as the program ends it, the weld is finished, with no restart and no
   * stretch welded twice.
   */
  static function checkWeldCraterFault(root:String):Void {
    var cell = openWelder(root, "materia.seam.project.json");
    var simulation = cell.simulation, mission = cell.mission, welder = cell.welder;
    var path = cell.beads.beadOf(0);
    var bead = path.beads[0];
    var target = 0.005;
    var dropped = false;
    var limit = simulation.activeSession().simulationTime() + 90;
    while (mission.completed < 1 && simulation.activeSession().simulationTime() < limit) {
      simulation.step();
      if (mission.failure != null) throw 'the weld failed after ${simulation.activeSession().simulationTime()} s: ${mission.failure}';
      // The crater fill piles metal into the last station: when it has, the supply drops out for a moment.
      if (!dropped && bead.leg(bead.count - 1) > 1.25 * target) {
        welder.supplyReady = false;
        simulation.step();
        welder.supplyReady = true;
        dropped = true;
      }
    }
    check(dropped, "the crater fill was reached");
    check(mission.completed >= 1 && mission.failure == null, "the weld finishes in spite of the dropout in the crater: " + mission.failure);
    var overlap = 0;
    for (station in 0...bead.count) if (bead.runs(station) >= 2) overlap++;
    check(mission.weldRestarts() == 0 && overlap == 0, 'a fault after the seam is travelled does not weld it again (${mission.weldRestarts()} restarts, $overlap mm overlap)');
    check(Math.abs(bead.extent() - bead.length) <= 0.004 && bead.gaps() == 0, "and the bead is the seam's length, whole");
    for (_ in 0...15) simulation.step();
    check(!welder.reading().arc && welder.reading().fault == 0, "the arc is out and the welder clear afterwards");
    Sys.println('robot welder: a dropout in the crater: finished in ${Math.round(simulation.activeSession().simulationTime() * 10) / 10} s, no restart');
    simulation.clear();
    cell.session.dispose();
  }

  /**
   * The workpiece is not where it was designed: its table stands a few millimetres and degrees off. The weld step is
   * given relative to the workpiece's reference member, so the weld finds the seam where the workpiece is: the wire tip
   * follows the seam in its real place, and the bead is as good as for the workpiece as designed.
   */
  static function checkWeldFollowsWorkpiece(root:String):Void {
    var shiftX = 6.0, shiftY = -4.0, turn = 3.0 * Math.PI / 180.0;
    var cell = openWelder(root, "materia.seam.project.json", function(generated) {
      var definition = generated.assemblyDefinition;
      if (definition == null) throw "the welding cell has an assembly";
      var table = [for (occurrence in definition.occurrences) if (occurrence.id == "table") occurrence];
      if (table.length != 1) throw "the welding cell has a table";
      var pose = table[0].initialPose;
      // The table's own origin is its centre; its pose is turned about it and moved (millimetres).
      var q = [pose.qx, pose.qy, pose.qz, pose.qw], r = [0.0, 0.0, Math.sin(turn / 2), Math.cos(turn / 2)];
      pose.qx = r[3] * q[0] + r[0] * q[3] + r[1] * q[2] - r[2] * q[1];
      pose.qy = r[3] * q[1] - r[0] * q[2] + r[1] * q[3] + r[2] * q[0];
      pose.qz = r[3] * q[2] + r[0] * q[1] - r[1] * q[0] + r[2] * q[3];
      pose.qw = r[3] * q[3] - r[0] * q[0] - r[1] * q[1] - r[2] * q[2];
      pose.x += shiftX;
      pose.y += shiftY;
    });
    var simulation = cell.simulation, mission = cell.mission, welder = cell.welder, beads = cell.beads;
    var generated = MateriaProjectRunner.loadProject(FileSystem.fullPath(root + "/machinekit/examples/robot-welder/materia.seam.project.json"));
    var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast cast(generated.missions[0], materia.project.SceneArtifact.SceneArtifactMission).steps[0].weld;
    var live = beads.referenceOf(0);
    // Where the plate stood as designed: the seam's start in the world is the start in the plate's frame placed by the plate's pose.
    var designed = new cadkit.modeling.AssemblyState(cast generated.assemblyDefinition, generated.assemblyState).worldPose("work/basePlate");
    var moved = Math.sqrt(Math.pow(live.position[0] - designed.x * 0.001, 2) + Math.pow(live.position[1] - designed.y * 0.001, 2));
    check(moved > 0.005, 'the workpiece stands ${moved * 1000} mm from where it was designed');
    // The seam in the world as the plate really stands: the path's ends turned and placed by the plate's live pose.
    function world(point:Array<Float>):Array<Float> {
      var turned = AssemblyRobot.rotate(live.rotation, point);
      return [for (axis in 0...3) turned[axis] + live.position[axis]];
    }
    var segment = weld.path[0];
    var from = world(segment.start.position), to = world(segment.stop.position);
    var along = [for (axis in 0...3) to[axis] - from[axis]];
    var length = Math.sqrt(along[0] * along[0] + along[1] * along[1] + along[2] * along[2]);
    var unit = [for (axis in 0...3) along[axis] / length];
    var strayed = 0.0;
    var limit = simulation.activeSession().simulationTime() + 90;
    while (mission.completed < 1 && simulation.activeSession().simulationTime() < limit) {
      simulation.step();
      if (mission.failure != null) throw 'the weld failed after ${simulation.activeSession().simulationTime()} s: ${mission.failure}';
      if (welder.reading().arc) {
        var tip = welder.tip();
        var relative = [for (axis in 0...3) tip[axis] - from[axis]];
        var s = relative[0] * unit[0] + relative[1] * unit[1] + relative[2] * unit[2];
        var off = Math.sqrt(Math.max(0.0, relative[0] * relative[0] + relative[1] * relative[1] + relative[2] * relative[2] - s * s));
        strayed = Math.max(strayed, off);
      }
    }
    check(mission.completed >= 1, "the weld of the displaced workpiece finishes");
    var bead = beads.beadOf(0).beads[0];
    check(strayed < 0.0008, 'the wire tip stayed within ${strayed * 1000} mm of the seam where the workpiece really is');
    var achieved = bead.meanLeg(0.15, 0.85);
    check(Math.abs(achieved - weld.legSize) <= 0.0005, 'the bead leg ${achieved * 1000} mm is within 0.5 mm of the ${weld.legSize * 1000} mm asked for');
    check(Math.abs(bead.extent() - bead.length) <= 0.004 && bead.gaps() == 0 && beads.beadOf(0).stray < 0.1 * beads.beadOf(0).deposited,
      "the bead is the seam's length, whole, and on the seam");
    Sys.println('robot welder: workpiece ${Math.round(moved * 10000) / 10} mm off its design: tip within ${Math.round(strayed * 10000) / 10} mm of the seam, leg ${Math.round(achieved * 10000) / 10} mm');
    simulation.clear();
    cell.session.dispose();
  }

  /**
   * One tube post's perimeter welds as a single step: four sides, the torch turning at each corner. The wire tip follows
   * the four sides, and each side gets a bead of the leg asked for between its corners.
   */
  static function checkWeldPost(root:String):Void {
    var cell = openWelder(root, "materia.post.project.json");
    var simulation = cell.simulation, mission = cell.mission, welder = cell.welder, beads = cell.beads;
    var generated = MateriaProjectRunner.loadProject(FileSystem.fullPath(root + "/machinekit/examples/robot-welder/materia.post.project.json"));
    var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast cast(generated.missions[0], materia.project.SceneArtifact.SceneArtifactMission).steps[0].weld;
    check(weld.path.length == 4, "the post's step is a path of four sides");
    var path = beads.beadOf(0);
    var strayed = 0.0;
    var restarts = 0;
    var limit = simulation.activeSession().simulationTime() + 120;
    while (mission.completed < 1 && simulation.activeSession().simulationTime() < limit) {
      simulation.step();
      if (mission.failure != null) throw 'the post weld failed after ${simulation.activeSession().simulationTime()} s: ${mission.failure}';
      if (welder.reading().arc) {
        // The tip is over some side (the bead decides which); how far from its line is the smallest distance to any side.
        var tip = beads.toFrame(0, welder.tip());
        var best = 1e9;
        for (bead in path.beads) {
          var relative = [for (axis in 0...3) tip[axis] - bead.start[axis]];
          var along = relative[0] * bead.tangent[0] + relative[1] * bead.tangent[1] + relative[2] * bead.tangent[2];
          var clamped = Math.min(bead.length, Math.max(0.0, along));
          var off = Math.sqrt(Math.pow(relative[0] - bead.tangent[0] * clamped, 2) + Math.pow(relative[1] - bead.tangent[1] * clamped, 2) +
            Math.pow(relative[2] - bead.tangent[2] * clamped, 2));
          best = Math.min(best, off);
        }
        strayed = Math.max(strayed, best);
      }
    }
    check(mission.completed >= 1, 'the post weld finishes within ${simulation.activeSession().simulationTime()} s');
    restarts = mission.weldRestarts();
    check(restarts == 0, 'the post is welded without losing the arc ($restarts restarts)');
    check(strayed < 0.0015, 'the wire tip stayed within ${strayed * 1000} mm of the post\'s seams');
    var legs:Array<Float> = [];
    for (bead in path.beads) {
      var leg = bead.meanLeg(0.3, 0.7);
      legs.push(Math.round(leg * 10000) / 10);
      check(Math.abs(leg - weld.legSize) <= 0.0006, 'a side\'s bead leg ${leg * 1000} mm is within 0.6 mm of the ${weld.legSize * 1000} mm asked for');
      check(bead.extent() > bead.length - 0.004, 'a side is welded from corner to corner (${bead.extent() * 1000} mm of ${bead.length * 1000})');
    }
    check(path.gaps() == 0 && path.stray < 0.1 * path.deposited, "the post's bead has no gap and little missed the seams");
    Sys.println('robot welder: post of four sides in one step: ${Math.round(simulation.activeSession().simulationTime() * 10) / 10} s, tip within ${Math.round(strayed * 10000) / 10} mm, legs ${legs.join("/")} mm');
    simulation.clear();
    cell.session.dispose();
  }

  /**
   * A seam the torch holds the wire clear of the work never strikes: the supply faults for want of an arc, the weld retries
   * its limited restarts, and then the mission stops with the reason, the arc out and no metal laid.
   */
  static function checkWeldInTheAir(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-welder/materia.seam.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var work:materia.project.SceneArtifact.SceneArtifactMission = cast generated.missions[0];
    var seam:materia.project.SceneArtifact.SceneArtifactWeld = cast work.steps[0].weld;
    // Move the seam off the work, 30 mm out of the corner on its open side: the torch follows it 30 mm above the plate and
    // 30 mm from the upright.
    for (end in [seam.path[0].start, seam.path[0].stop]) {
      end.position[1] += 0.03;
      end.position[2] += 0.03;
    }
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    check(simulation.rebuild(session.sensors, session.scene, session), "the welding cell builds: " + simulation.error);
    var mission = simulation.missionPlayer(), welder = simulation.welder(), beads = simulation.weldBeads();
    if (mission == null || welder == null || beads == null) throw "the welding cell has no mission, welder or weld metal";
    var faults = 0, previous = 0;
    var limit = simulation.activeSession().simulationTime() + 120;
    while (mission.failure == null && simulation.activeSession().simulationTime() < limit) {
      simulation.step();
      var fault = welder.reading().fault;
      if (fault != 0 && previous == 0) faults++;
      previous = fault;    }
    var failure = mission.failure;
    check(failure != null && failure.indexOf("restarts") >= 0, "a weld in the air gives up after its restarts: " + failure);
    check(faults >= 3, "the supply faulted for want of an arc each time ($faults)");
    check(!welder.reading().arc && beads.beadOf(0).deposited == 0.0, "no arc burned and no metal was laid");
    Sys.println('robot welder: a seam in the air: ${failure} after ${Math.round(simulation.activeSession().simulationTime() * 10) / 10} s, $faults no-arc faults');
    simulation.clear();
    session.dispose();
  }

  static function checkRobotWelderOn(root:String, backend:Int, label:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-welder/materia.seam.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var tools = generated.robotTools;
    check(tools != null && tools.length == 1 && tools[0].kind == "torch" && tools[0].torch != null, "the cell carries a torch tool");
    var work:materia.project.SceneArtifact.SceneArtifactMission = cast generated.missions[0];
    check(work != null && work.steps.length == 1 && work.steps[0].kind == "weld", "the cell's mission welds one seam");
    var seam:materia.project.SceneArtifact.SceneArtifactWeld = cast work.steps[0].weld;
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(backend);
    check(simulation.rebuild(session.sensors, session.scene, session), "the welding cell builds in the simulation: " + simulation.error);
    var mission = simulation.missionPlayer(), welder = simulation.welder(), beads = simulation.weldBeads();
    if (mission == null || welder == null || beads == null) throw "the welding cell has no mission, welder or weld metal";
    simulation.step();
    var restPose = welder.reading();
    check(!restPose.arc && restPose.fault == 0 && !restPose.touch, "the torch rests with its arc out, no fault and no touch");
    var path = beads.beadOf(0);
    var bead = path.beads[0];
    var target = seam.path.length == 1 ? seam.legSize : seam.legSize;
    var log:Array<String> = [];
    for (run in 0...2) {
      if (run == 1) {
        check(simulation.reset(), "the welding cell resets");
        check(path.covered() == 0 && path.deposited == 0, "a reset leaves the workpiece bare");
        check(session.scene.runtimePartNodes("project:" + seam.metal).length == 0, "a reset takes the bead off the weld metal part");
        simulation.step();
      }
      var lost = false, established = 0, current = 0.0, peak = 0.0, lowest = 1e9, previousArc = false, ignited = 0.0, strayed = 0.0;
      var startDwellTicks = 0, craterDwellTicks = 0;
      var limit = simulation.activeSession().simulationTime() + 90;
      while (mission.completed < 1 && simulation.activeSession().simulationTime() < limit) {
        simulation.step();
        var failure = mission.failure;
        if (failure != null) throw 'run $run: the weld failed after ${simulation.activeSession().simulationTime()} s: $failure';
        var reading = welder.reading();
        // Observe the actual host barriers, after their native plans finish, rather than inferring dwell from cycle time.
        var active = @:privateAccess mission.welding;
        if (active != null) {
          var planning = @:privateAccess active.motion.planner;
          var progress = active.motion.progress();
          if (planning != null && progress.block >= 0 && progress.block < planning.blocks.blocks.length) {
            var block = planning.blocks.blocks[progress.block];
            var index = @:privateAccess active.motion.planIndex;
            if (index >= block.plans.length && block.barrier != null) switch block.barrier {
              case Dwell(seconds):
                check(Math.abs(seconds - 0.15) < 1e-8, "executed weld dwell preserves recipe duration");
                if (bead.extent() < bead.length / 2) startDwellTicks++; else craterDwellTicks++;
              case _: {}
            }
          }
        }
        if (reading.arc) {
          // How far the wire tip strays from the seam line while the arc burns.
          var tip = beads.toFrame(0, welder.tip());
          var relative = [for (axis in 0...3) tip[axis] - bead.start[axis]];
          var along = relative[0] * bead.tangent[0] + relative[1] * bead.tangent[1] + relative[2] * bead.tangent[2];
          var off = Math.sqrt(Math.max(0.0, relative[0] * relative[0] + relative[1] * relative[1] + relative[2] * relative[2] - along * along));
          strayed = Math.max(strayed, off);
          established++;
          current = reading.currentA;
          peak = Math.max(peak, current);
          lowest = Math.min(lowest, current);
          if (!previousArc && ignited == 0.0) ignited = simulation.activeSession().simulationTime();
        }
        previousArc = reading.arc;
        // The second run loses the arc once, half way along the seam: the supply drops out for a moment.
        if (run == 1 && !lost && bead.extent() > bead.length / 2) {
          welder.supplyReady = false;
          simulation.step();
          welder.supplyReady = true;
          lost = true;
        }
      }
      check(mission.completed >= 1, 'run $run: the weld finishes within 90 s (${mission.stepIndex}, ${simulation.activeSession().simulationTime()} s)');
      // The bead is shown on the weld metal part, one mesh chunk per 20 mm of seam, once the last growth is meshed.
      for (_ in 0...15) simulation.step();
      var shown = session.scene.runtimePartNodes("project:" + seam.metal).length;
      check(shown == Math.ceil(bead.count / WeldBeads.CHUNK), 'run $run: the weld metal shows as $shown mesh chunks');
      var reading = welder.reading();
      check(!reading.arc && reading.fault == 0, 'run $run: the arc is out and the welder is clear afterwards');
      check(established > 100, 'run $run: the arc was established for ${established} ticks');
      check(peak > 200 && peak < 300, 'run $run: the arc ran near 250 A, peaking at $peak A');
      check(strayed < 0.002, 'run $run: the wire tip stayed within ${strayed * 1000} mm of the seam while the arc burned');
      // The bead: the leg reached is a result, compared with the leg asked for; the bead runs the seam's length.
      var achieved = bead.meanLeg(0.15, 0.85);
      var range = bead.legRange(0.15, 0.85);
      check(Math.abs(achieved - target) <= 0.0005, 'run $run: the bead leg ${achieved * 1000} mm is within 0.5 mm of the ${target * 1000} mm asked for');
      check(bead.gaps() == 0, 'run $run: the bead has no gap (${bead.gaps()} stations bare)');
      check(Math.abs(bead.extent() - bead.length) <= 0.004, 'run $run: the bead is ${bead.extent() * 1000} mm long on a ${bead.length * 1000} mm seam');
      check(path.stray < 0.1 * path.deposited, 'run $run: little metal missed the seam (${path.stray / path.deposited})');
      var overlap = 0;
      for (station in 0...bead.count) if (bead.runs(station) >= 2) overlap++;
      if (run == 0) check(overlap == 0 && mission.weldRestarts() == 0, 'run 0: an undisturbed weld has no restart and no overlap');
      else {
        check(mission.weldRestarts() == 1, 'run 1: the weld restarted once (${mission.weldRestarts()})');
        check(overlap >= 1 && overlap <= 5, 'run 1: the restart overlaps the bead by $overlap mm');
        check(range.max <= target + 0.001, 'run 1: restart peak ${range.max * 1000} mm stays within 1 mm of the target');
      }
      if (run == 0) check(startDwellTicks >= 14 && craterDwellTicks >= 14,
        'both 0.15 s dwells execute: start $startDwellTicks ticks, crater $craterDwellTicks ticks');
      log.push('run $run: ${Math.round(simulation.activeSession().simulationTime() * 10) / 10} s, leg ${Math.round(achieved * 10000) / 10} mm ' +
        '(${Math.round(range.min * 10000) / 10}..${Math.round(range.max * 10000) / 10}), bead ${Math.round(bead.extent() * 1000)} mm, ' +
        '${established} ticks of arc up to ${Math.round(peak)} A, tip within ${Math.round(strayed * 10000) / 10} mm of the seam, ${overlap} mm overlap, ${mission.weldRestarts()} restarts, stray ' +
        '${Math.round(100 * path.stray / path.deposited)}%, dwells $startDwellTicks/$craterDwellTicks ticks');
    }
    simulation.clear();
    session.dispose();
    for (line in log) Sys.println('robot welder ($label): ' + line);
  }

  /**
   * The router example opens, its axes simulate as prismatic joints in metres, and its CNC job, CAM
   * made from a NEMA 23 motor plate, mills and drills the plate out of the stock in real time, changing
   * tools on the way: the stock loses exactly the plate's recesses and holes, nothing is cut from the
   * part, and no rapid or holder touches stock.
   */
  /** The screw router's cycle time and worst drive deviation, for the belt router to be compared with. */
  static var screwRouterSeconds = 0.0;
  static var screwRouterDeviation = 0.0;

  /** Exercise the generated mill's complete masses, screw relation and installed brake. */
  static function checkMillBrake(model:RobotModel):Void {
    check(model.brakes.length == 1, "mill has one installed Z brake");
    var brake = model.brakes[0];
    var motorIndex = -1, zIndex = -1;
    for (i in 0...model.joints.length) {
      if (model.joints[i].id == brake.joint) motorIndex = i;
      if (model.joints[i].id == "z") zIndex = i;
    }
    check(motorIndex >= 0 && zIndex >= 0, "installed mill brake binds the motor of the mechanical Z axis");
    var requestedZ = -0.03, motorTarget = 0.0, screwTerms = 0;
    for (coupling in model.couplings) if (coupling.leader == model.joints[zIndex].id && coupling.follower == brake.joint) {
      motorTarget += coupling.ratio * requestedZ + coupling.offset;
      screwTerms++;
    }
    check(screwTerms == 1, "mill brake check uses the installed Z screw relation");
    for (jammed in [false, true]) {
      var blueprint = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
      if (jammed) blueprint.brakePhysics[0] = new robotkit.runtime.RobotRuntimeBrakePhysicsBlueprint(
        motorIndex, brake.holdingEffort, 1.0, brake.applicationDelay);
      var physical = new SimulationHarness(0.01, 1, SimulationSpace.MUJOCO);
      try {
        var runtime = physical.simulation.addRobot(blueprint);
        runtime.submitTargets([robotkit.core.JointTarget.position(zIndex, requestedZ),
          robotkit.core.JointTarget.position(motorIndex, motorTarget)], 1);
        for (tick in 0...4) physical.step();
        var waiting = runtime.snapshot();
        var contacts = [for (sensor in waiting.sensors.toArray()) if (sensor.sensorId == brake.releasedSensor) sensor];
        check(contacts.length == 1 && contacts[0].values.get(0) == 0 && Math.abs(waiting.q.get(zIndex)) < 0.0001,
          "actual mill waits for physical released contact before moving its Z load");
        check(runtime.channelValue(brake.releaseChannel) == ProcessEventValue.Digital(true),
          "actual mill energizes release while waiting for physical contact");
        for (tick in 0...100) physical.step();
        var moving = runtime.snapshot();
        if (jammed) {
          check(moving.faultCode == RobotKitRuntime.RobotKitRuntimeConstants.RK_FAULT_BRAKE_RELEASE_TIMEOUT,
            "actual mill faults when the brake cannot release within the controller timeout");
          check(Math.abs(moving.q.get(zIndex)) < 0.0001 && Math.abs(moving.effort.get(motorIndex)) < 1e-6,
            'release fault removes motor effort while the applied brake holds the actual Z load: q=${moving.q.get(zIndex)}, effort=${moving.effort.get(motorIndex)}, motor=${moving.q.get(motorIndex)}');
          check(runtime.channelValue(brake.releaseChannel) == ProcessEventValue.Digital(false),
            "release fault de-energizes the device coil");
        } else {
          check(moving.q.get(zIndex) < -0.02,
            'actual Z load moves after release: q=${moving.q.get(zIndex)}, motor=${moving.q.get(motorIndex)}, ' +
            'setpoint=${moving.setpoint.get(zIndex)}, safety=${moving.safety}, fault=${moving.faultCode}, ' +
            'coil=${runtime.channelValue(brake.releaseChannel)}, jammed=$jammed, ' +
            'delay=${blueprint.brakePhysics[0].releaseDelay}, sourceNs=${moving.sourceTimestampNs}');
          runtime.submitStop(2, false);
          for (tick in 0...30) physical.step();
          var stopped = runtime.snapshot();
          check(runtime.channelValue(brake.releaseChannel) == ProcessEventValue.Digital(true) &&
            Math.abs(stopped.q.get(zIndex) - moving.q.get(zIndex)) < 0.0001,
            "ordinary stop retains released brake and powered Z hold");
          runtime.submitStop(3, true);
          for (tick in 0...50) physical.step();
          var applied = runtime.snapshot();
          check(runtime.channelValue(brake.releaseChannel) == ProcessEventValue.Digital(false) &&
            Math.abs(applied.effort.get(motorIndex)) < 1e-6 && Math.abs(applied.effort.get(zIndex)) < 1e-6,
            "emergency stop de-energizes the brake and disables motor and follower effort");
          var appliedContacts = [for (sensor in applied.sensors.toArray()) if (sensor.sensorId == brake.releasedSensor) sensor];
          check(appliedContacts.length == 1 && appliedContacts[0].values.get(0) == 0,
            "actual mill reports applied physical brake contact after emergency stop");
          for (tick in 0...50) physical.step();
          var held = runtime.snapshot();
          var drift = held.q.get(zIndex) - applied.q.get(zIndex);
          var sag = applied.q.get(zIndex) - stopped.q.get(zIndex);
          check(Math.abs(drift) < 0.00001 && Math.abs(sag) < 0.0002,
            'installed brake holds the actual mill Z load: sag=$sag m, settled drift=$drift m');
          runtime.resetSafety(4);
          for (tick in 0...50) physical.step();
          var reset = runtime.snapshot();
          check(reset.faultCode == 0 && Math.abs(reset.q.get(zIndex) - held.q.get(zIndex)) < 0.0001 &&
            runtime.channelValue(brake.releaseChannel) == ProcessEventValue.Digital(true),
            "mill reset releases afresh while holding the actual stopped Z pose");
          Sys.println('actual mill Z brake: ${brake.holdingEffort} N m; e-stop sag ${sag * 1000} mm, ' +
            'settled drift ${drift * 1000} mm over 0.5 s');
        }
        physical.dispose();
      } catch (error:Dynamic) {
        physical.dispose();
        throw error;
      }
    }
  }

  /** The moving-table mill machines its single-tool bearing block in the shared backend. */
  static function checkBenchMill(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/bench-mill/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition = generated.assemblyDefinition;
    if (definition == null) throw "Mill project has no assembly definition";
    var model = AssemblySimulationBridge.toRobotModel(definition, generated.physical).model;
    checkMillBrake(model);
    var job = generated.cncJob;
    if (job == null) throw "Mill project has no CNC job";
    var axisNames = job.axes;
    var axes = [for (joint in model.joints) if (axisNames.indexOf(Std.string(joint.id)) >= 0) joint];
    check([for (joint in axes) Std.string(joint.id)].join(",") == "y,x,z", "mill has three mechanical axes");
    var blueprint = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    check(blueprint.fastestPositionLoopRate() == 4000, "mill driver loop rate survives project generation");
    var processOwned = blueprint.processJoints();
    var processNames = [for (index in 0...model.joints.length) if (processOwned[index]) Std.string(model.joints[index].id)];
    processNames.sort(Reflect.compare);
    check(processNames.join(",") == "spindle-motor-turn,spindle-turn",
      "the spindle motor and belt-coupled cartridge both reject controller position targets");
    check(blueprint.servoStabilityInterval() > 0 && Math.isFinite(blueprint.servoStabilityInterval()),
      "mill servo stability bound comes from its reflected inertia");
    var limits:Array<String> = [];
    for (joint in axes) {
      var rate = model.coupledLimits(joint.id, new SteadyLoads());
      var rapid = rate.requireVelocity();
      var acceleration = rate.requireAcceleration();
      check(rapid >= 8.0 / 60, 'mill ${joint.id} rapid reaches 8 m/min, got $rapid');
      limits.push('${joint.id}: ${Math.round(rapid * 6000) / 100} m/min, ${Math.round(acceleration * 100) / 100} m/s²');
    }
    Sys.println('bench mill derived limits: ${limits.join("; ")}');
    var session = new ProjectDocumentSession(null, false);
    var simulation = new ApplicationSimulation(new RobotWorld());
    session.openGeneratedScene(generated.objects, manifest,
      generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
      generated.localCentersByDefinition, generated.metresPerUnit, generated.physical,
      generated.recipeDocument, generated.robotMotions, null, generated.cncJob,
      generated.mobileBase, generated.missions, generated.robotTools, generated.robotSensors,
      generated.machineMotion, generated.assemblyModel, generated.controllers, generated.signals);
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      check(simulation.rebuild(session.sensors, session.scene, session), "mill builds: " + simulation.error);
      var player = simulation.cncPlayer();
      if (player == null) throw "Mill has no CNC player";
      simulation.step(); // Compile and initialise before counting steady runtime allocations, as on the router.
      var allocated = hl.Gc.totalAllocated(), ticks = 0;
      while (player.passes == 0 && simulation.activeSession().simulationTime() < 600 && ticks++ < 1000000) {
        simulation.step();
        var failure = simulation.cncFailure();
        if (failure != null) {
          var observed = simulation.snapshot().robots()[0];
          throw 'Mill CNC fault at ${simulation.activeSession().simulationTime()} s: $failure; joints ' +
            [for (i in 0...model.joints.length) '${model.joints[i].id}=${observed.positions.get(i)}'].join(", ");
        }
      }
      var bytes = (hl.Gc.totalAllocated() - allocated) / ticks;
      var finishedState = simulation.snapshot().robots()[0];
      check(player.passes == 1, 'mill completes one pass at ${simulation.activeSession().simulationTime()} s; velocities ' +
        [for (i in 0...model.joints.length) '${model.joints[i].id}=${finishedState.velocities.get(i)}'].join(", ") +
        "; sensors " + [for (sensor in finishedState.sensors.toArray()) '${sensor.sensorId}=${sensor.values.toArray()}'].join(", "));
      var stock = simulation.machiningStock();
      if (stock == null) throw "Mill has no stock";
      var removal = (Math.PI * (121 * 7 + 2 * 25 * 5) + (144 - (4 - Math.PI) * 12.25) * 3) / 1e9;
      var deviation = stock.deviation();
      check(player.atSpeedSensorId != null, "mill waits for physical spindle readiness feedback");
      check(player.axisTrackingPeak.length == 3, "mill reports all three tracking errors");
      for (error in player.axisTrackingPeak)
        check(Math.isFinite(error) && error < 0.00015, "mill servo peak tracking stays below 0.15 mm through the complete job");
      Sys.println('bench mill tracking at 10 ms (axis order x,y,z), peak mm: ' +
        [for (error in player.axisTrackingPeak) error * 1000].join(", "));
      Sys.println('bench mill measurement: ${simulation.activeSession().simulationTime()} s, ${stock.removed * 1e9}/${removal * 1e9} mm³, ' +
        'leftover ${deviation.leftover * 1e9}, gouge ${deviation.gouge * 1e9}, rapid ${stock.rapidContacts}, collisions ${stock.collisions}, ' +
        '${player.planChecks().plans} plans, $ticks ticks, $bytes bytes/tick');
      check(Math.abs(stock.removed - removal) < removal * 0.02, "mill removal matches the bearing block closed form");
      check(deviation.leftover < removal * 0.02, 'mill leaves only slivers: ${deviation.leftover}');
      check(deviation.gouge < 1e-9, 'mill does not gouge: ${deviation.gouge}');
      check(stock.rapidContacts == 0 && stock.collisions == 0,
        'mill rapids and holder clear stock: ${stock.rapidContacts}, ${stock.collisions}');
      var checks = player.planChecks();
      Sys.println('mill drive findings: ${checks.count(PlanDiagnosticKind.ServoPeakTorque)} peak torque, ' +
        '${checks.count(PlanDiagnosticKind.ServoRatedTorque)} rated RMS torque; physics interval required by the drive: ' +
        '${Math.min(1.0 / blueprint.fastestPositionLoopRate(), blueprint.servoStabilityInterval())} s');
      check(checks.count(PlanDiagnosticKind.StepperStall) == 0, "mill drives never stall");
      check(checks.count(PlanDiagnosticKind.Accuracy) == 0, "mill drive compliance stays within tolerance");
      Sys.println('bench mill bearing block: ${Math.round(simulation.activeSession().simulationTime() * 10) / 10} s, ' +
        '${Math.round(stock.removed * 1e10) / 10}/${Math.round(removal * 1e10) / 10} mm³ removed, ' +
        '${Math.round(deviation.leftover * 1e10) / 10} mm³ leftover, ${Math.round(deviation.gouge * 1e10) / 10} mm³ gouge; ' +
        '${checks.plans} plans, ${checks.flagged} flagged; ${Math.round(bytes / 100) / 10} KB/tick');
      simulation.dispose();
      session.dispose();
      check(bytes < 80000, 'mill allocates below 80 KB/tick, got $bytes');
    } catch (error:Dynamic) {
      simulation.dispose();
      session.dispose();
      throw error;
    }
  }

  /** The enclosure's pressure-clamped vise holds the bearing-block job. */
  static function checkEnclosedMillProject(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/bench-mill/materia.enclosed.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    check(generated.controllers != null && generated.controllers.length == 1 &&
      generated.controllers[0].id == "cncController", "enclosed mill exports its wired CNC controller");
    var definition = requiredAssemblyDefinition(generated.assemblyDefinition);
    var model = AssemblySimulationBridge.toRobotModel(definition, generated.physical).model;
    var doorActuator:Null<Actuator> = null;
    for (actuator in model.actuators) if (actuator.id == "door-cylinder") doorActuator = actuator;
    var doorEffort:Float = doorActuator == null || doorActuator.maxEffort == null ? 0.0 : doorActuator.maxEffort;
    check(Math.abs(doorEffort - 294.524311274) < 0.01,
      'assembly bridge keeps the 25 mm door cylinder force in newtons, got $doorEffort');
    var viseActuator:Null<Actuator> = null;
    for (actuator in model.actuators) if (actuator.id == "vise-cylinder") viseActuator = actuator;
    var viseEffort:Float = viseActuator == null || viseActuator.maxEffort == null ? 0.0 : viseActuator.maxEffort;
    check(Math.abs(viseEffort - 482.54863159) < 0.01,
      'assembly bridge keeps the 32 mm vise cylinder force in newtons, got $viseEffort');
    check([for (joint in model.joints) if (joint.type == JointType.Prismatic) joint].length == 5,
      "enclosed project retains three mill axes, door and jaw");
    check(generated.cncJob != null, "enclosed mill exports its clamped bearing-block job");
    var state = new AssemblyState(definition);
    var opening = state.worldConnector("chipTray", "doorOpening");
    var datum = state.worldConnector("mill/vise/body", "datum");
    check(opening.y < datum.y && opening.z < datum.z, "door opening and vise datum survive project generation");
    if (doorActuator == null || viseActuator == null) throw "Enclosed mill lost its pneumatic drives";
    var enclosedRate = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()).fastestPositionLoopRate();
    checkPneumaticDynamics(model, doorActuator, viseActuator, Math.ceil(enclosedRate * 0.01));
    var session = new ProjectDocumentSession(null, false);
    var simulation = new ApplicationSimulation(new RobotWorld());
    session.openGeneratedScene(generated.objects, manifest,
      generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
      generated.localCentersByDefinition, generated.metresPerUnit, generated.physical,
      generated.recipeDocument, generated.robotMotions, null, generated.cncJob,
      generated.mobileBase, generated.missions, generated.robotTools, generated.robotSensors,
      generated.machineMotion, generated.assemblyModel, generated.controllers, generated.signals);
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      check(simulation.rebuild(session.sensors, session.scene, session),
        "enclosed mill builds: " + simulation.error);
      var player = simulation.cncPlayer();
      if (player == null) throw "Enclosed mill lost its CNC player";
      var ticks = 0;
      while (player.passes == 0 && simulation.activeSession().simulationTime() < 600 && ticks++ < 1000000) {
        simulation.step();
        var failure = simulation.cncFailure();
        if (failure != null) throw 'Enclosed mill CNC fault at ${simulation.activeSession().simulationTime()} s: $failure';
      }
      check(player.passes == 1, "enclosed mill completes one bearing-block pass");
      var stock = simulation.machiningStock();
      if (stock == null) throw "Enclosed mill lost its machining stock";
      var removal = (Math.PI * (121 * 7 + 2 * 25 * 5) + (144 - (4 - Math.PI) * 12.25) * 3) / 1e9;
      check(Math.abs(stock.removed - removal) < removal * 0.02 && stock.rapidContacts == 0 && stock.collisions == 0,
        "enclosed mill cuts the bearing block without rapid or holder contact");
      Sys.println('enclosed mill bearing block: ${simulation.activeSession().simulationTime()} s, ${stock.removed * 1e9} mm³ removed');
    } catch (error:Dynamic) {
      simulation.dispose();
      session.dispose();
      throw error;
    }
    simulation.dispose();
    session.dispose();
    Sys.println('enclosed mill project: ${generated.objects.length} objects, five mechanical slides');
    checkSingleCabinetMotion(generated, manifest);
  }

  static function checkSingleCabinetMotion(generated:GeneratedAssemblyScene, manifest:String):Void {
    var definition = requiredAssemblyDefinition(generated.assemblyDefinition);
    var model = AssemblySimulationBridge.toRobotModel(definition, generated.physical).model;
    check(generated.controllers != null && generated.controllers.length == 1,
      "the enclosed mill is a single-cabinet assembly");
    var identity = "assembly:" + definition.id;
    var relations = [for (term in model.couplings) if (term.leader == "mill/x" && term.follower == "mill/screwX-turn") term];
    check(relations.length == 1, "saved mill tracks use the exported screw relation");
    var relation = relations[0];
    generated.cncJob = null;
    generated.robotMotions = [
      new RobotMotionTrack(identity, 0, false,
        [{time: 0.0, position: 0.0}, {time: 0.5, position: 10.0}], "mill/screwX-turn"),
      new RobotMotionTrack(identity, 0, false,
        [{time: 0.0, position: -relation.offset / relation.ratio},
         {time: 0.5, position: (10.0 - relation.offset) / relation.ratio}], "mill/x")];
    var trackedSession = new ProjectDocumentSession(null, false);
    trackedSession.openGeneratedProject(generated, manifest);
    var tracked = new ApplicationSimulation(new RobotWorld());
    tracked.setBackend(ApplicationSimulation.MUJOCO);
    try {
      check(tracked.rebuild(trackedSession.sensors, trackedSession.scene, trackedSession),
        "single-cabinet saved motion retains its robot id: " + tracked.error);
      check(tracked.simulatedRobotIds().join(",") == identity,
        "adding one cabinet preserves the assembly robot identity");
      var robot = tracked.world.robot(identity);
      if (robot == null) throw "Tracked single-cabinet robot is missing";
      var motor = robot.description().joints.indexOf("mill/screwX-turn");
      var axis = robot.description().joints.indexOf("mill/x");
      if (motor < 0 || axis < 0) throw "Tracked mill has no X motor and slide";
      var start = robot.snapshot().positions.get(axis);
      for (_ in 0...100) tracked.step();
      check(Math.abs(robot.snapshot().positions.get(motor) - 10.0) < 0.01 &&
        Math.abs(robot.snapshot().positions.get(axis) - start) > 0.001,
        "a motion track with the original assembly robot id drives its motor and coupled slide");
    } catch (error:Dynamic) {tracked.dispose(); trackedSession.dispose(); throw error;}
    tracked.dispose(); trackedSession.dispose();
  }

  /** One cell assembly produces independent CNC and cobot runtimes. */
  static function checkTendingControllers(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/machine-tending/materia.controllers.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    check(generated.controllers != null && generated.controllers.length == 2 &&
      generated.signals != null && generated.signals.length == RobotInterfaceChannels.cncOutputs().length +
        RobotInterfaceChannels.robotRequests().length,
      "tending cell exports two wired controllers and all robot-interface signals");
    var wires = generated.signals;
    if (wires == null) throw "Tending cell lost its signal wires";
    var robotSignals = [for (wire in wires) if (wire.from.controller == "robotController") wire];
    check(robotSignals.length == RobotInterfaceChannels.robotRequests().length,
      "the robot cabinet owns every robot request output");
    var first = robotSignals[0], output = false, published = false, now = 0.0;
    var publishedSequence = Int64.ofInt(0);
    var bus = new CellSignals(wires, () -> now,
      (controller, channel) -> channel == first.name && output,
      (controller, channel, on, sequence, stamp) -> {
        if (channel == first.name) {
          check(Int64.compare(sequence, publishedSequence) > 0, "Cell signal samples remain monotonic across Reset");
          publishedSequence = sequence;
          published = on;
        }
      });
    bus.feed();
    var initial = bus.input(first.to.controller, first.name);
    check(initial != null && switch initial {case Digital(on): !on; case _: false;},
      "the receiving controller starts with the output low");
    output = true;
    var sameTick = bus.input(first.to.controller, first.name);
    check(sameTick != null && switch sameTick {case Digital(on): !on; case _: false;},
      "an output change is not visible in its own controller tick");
    now = 0.01;
    bus.feed();
    var nextTick = bus.input(first.to.controller, first.name);
    check(nextTick != null && switch nextTick {case Digital(on): on; case _: false;} && published,
      "the wired digital output reaches the next controller tick and its sensor");
    bus.reset();
    var resetInput = bus.input(first.to.controller, first.name);
    check(resetInput != null && switch resetInput {case Digital(on): !on; case _: false;},
      "Reset clears the wired input before the next sample");
    bus.feed();
    var alarmBit = RobotInterfaceChannels.alarmCodeBits()[0];
    var alarmWire = [for (wire in wires) if (wire.name == alarmBit) wire];
    check(alarmWire.length == 1, "the CNC alarm code has a declared cell output");
    var alarmOutput = false;
    var alarmBus = new CellSignals(wires, () -> now,
      (controller, channel) -> channel == alarmBit && alarmOutput,
      (_controller, _channel, _on, _sequence, _stamp) -> {});
    alarmBus.feed();
    alarmOutput = true;
    var alarmSameTick = alarmBus.input(alarmWire[0].to.controller, alarmBit);
    check(alarmSameTick != null && switch alarmSameTick {case Digital(on): !on; case _: false;},
      "the PLC alarm code remains behind the same controller tick boundary");
    now += 0.01;
    alarmBus.feed();
    var alarmNextTick = alarmBus.input(alarmWire[0].to.controller, alarmBit);
    check(alarmNextTick != null && switch alarmNextTick {case Digital(on): on; case _: false;},
      "the declared alarm-code wire publishes its digital input on the next signal image");
    var writtenController:Null<String> = null, writtenName:Null<String> = null, writtenValue = false;
    var writerBus = new CellSignals(wires, () -> now, (_controller, _channel) -> false,
      (_controller, _channel, _on, _sequence, _stamp) -> {},
      (controller, channel, on) -> {writtenController = controller; writtenName = channel; writtenValue = on;});
    var writeRobotOutput = writerBus.outputFor(first.from.controller);
    writeRobotOutput(first.name, true);
    check(writtenController == first.from.controller && writtenName == first.name && writtenValue,
      "a mission's typed output callback writes through its owning controller");
    var crossControllerWriteRejected = false;
    try writerBus.writeControllerSignal(first.to.controller, first.name, true) catch (_:Dynamic) crossControllerWriteRejected = true;
    check(crossControllerWriteRejected, "a robot mission cannot write an output owned by the CNC cabinet");
    // This scenario isolates cabinet outputs, physical valves and belt refresh when no CNC player
    // owns the spindle; the full physical cut is covered by checkTendingRobotInterface.
    generated.cncJob = null;
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      check(simulation.rebuild(session.sensors, session.scene, session),
        "tending cell builds two runtimes: " + simulation.error);
      var ids = simulation.simulatedRobotIds();
      check(ids.length == 2, "tending cell has two simulated robots");
      var joints = new Map<String, String>();
      for (id in ids) {
        var robot = simulation.world.robot(id);
        if (robot == null) throw 'Missing simulated robot "$id"';
        for (jointId in robot.description().joints) {
          check(!joints.exists(jointId), 'Joint "$jointId" belongs to two runtimes');
          joints.set(jointId, id);
        }
      }
      check(joints.get("mill/mill/x") == "assembly:tending-controllers:mill/cncController" &&
        joints.get("arm/j1") == "assembly:tending-controllers:robotController",
        "CNC and arm joints follow their wired controllers");
      simulation.step();
      check(simulation.simulatedRobotIds().length == 2, "both robots advance one shared MuJoCo world");
      var arm = simulation.world.robot("assembly:tending-controllers:robotController");
      if (arm == null) throw "Tending cell has no robot controller runtime";
      var physical = @:privateAccess simulation.simulation;
      if (physical == null) throw "Tending cell has no physical simulation";
      var runtimes = @:privateAccess physical.robots;
      var armRuntime = runtimes[ids.indexOf(arm.id())];
      function sensed(snapshot:robotkit.runtime.RobotSnapshot, id:String):Float {
        for (frame in snapshot.sensors.toArray()) if (frame.sensorId == id && frame.values.length > 0)
          return frame.values.get(0);
        return -1.0;
      }
      var grippers:Array<{name:String, channel:String, jawIndex:Int, sensor:String, valve:String}> = [
        {name: "A", channel: DualArmGripperTool.TOOL_CHANNEL_A,
          jawIndex: arm.description().joints.indexOf("arm/tool/gripperA/jaw-left-slide"),
          sensor: "arm/tool/gripperA/", valve: "arm/tool/valveA/"},
        {name: "B", channel: DualArmGripperTool.TOOL_CHANNEL_B,
          jawIndex: arm.description().joints.indexOf("arm/tool/gripperB/jaw-left-slide"),
          sensor: "arm/tool/gripperB/", valve: "arm/tool/valveB/"}
      ];
      for (gripper in grippers) if (gripper.jawIndex < 0)
        throw 'Tending arm has no physical gripper ${gripper.name} jaw joint';
      var initialJawPositions = [for (gripper in grippers) armRuntime.snapshot().q.get(gripper.jawIndex)];
      for (index in 0...grippers.length) {
        var gripper = grippers[index], other = grippers[1 - index];
        var jawStart = initialJawPositions[index], otherStart = initialJawPositions[1 - index];
        armRuntime.writeDigitalOutput(gripper.channel, true);
        var closedEmpty = false, closeCoilOff = false, otherStayedOpen = false;
        for (_ in 0...100) {
          simulation.step();
          var snapshot = armRuntime.snapshot();
          closedEmpty = snapshot.q.get(gripper.jawIndex) > jawStart + 0.01 &&
            sensed(snapshot, gripper.sensor + "jaw-closed-empty") > 0.5;
          closeCoilOff = switch armRuntime.channelValue(gripper.valve + "coilA") {
            case Digital(on): !on;
            case _: false;
          };
          otherStayedOpen = Math.abs(snapshot.q.get(other.jawIndex) - otherStart) < 0.001;
          if (closedEmpty && closeCoilOff && otherStayedOpen) break;
        }
        check(closedEmpty, 'gripper ${gripper.name} closes its real jaws to the closed-empty switch');
        check(closeCoilOff, 'gripper ${gripper.name} close coil de-energizes after its pulse');
        check(otherStayedOpen, 'gripper ${gripper.name} command leaves the other physical gripper open');

        armRuntime.writeDigitalOutput(gripper.channel, false);
        var reopened = false, openCoilOff = false;
        for (_ in 0...100) {
          simulation.step();
          var snapshot = armRuntime.snapshot();
          reopened = snapshot.q.get(gripper.jawIndex) < jawStart + 0.001 &&
            sensed(snapshot, gripper.sensor + "jaw-open") > 0.5;
          openCoilOff = switch armRuntime.channelValue(gripper.valve + "coilB") {
            case Digital(on): !on;
            case _: false;
          };
          if (reopened && openCoilOff) break;
        }
        check(reopened, 'gripper ${gripper.name} open command returns its jaws to the reed switch');
        check(openCoilOff, 'gripper ${gripper.name} open coil de-energizes after its pulse');
      }
      check(simulation.environmentObject("project:floorMarker") != null &&
        simulation.environmentObject("project:mill/mill/base") == null &&
        simulation.environmentObject("project:arm/baseFlange") == null,
        "uncontrolled parts are environment while both machine roots remain with their robots");
      var target = simulation.world.robot("assembly:tending-controllers:" + first.to.controller);
      if (target == null) throw "Tending signal has no target runtime";
      var frames = target.snapshot().sensors;
      var received = false;
      for (index in 0...frames.length) {
        var frame = frames.get(index);
        if (frame.sensorId == first.name && frame.values.length == 1 && frame.values.get(0) == 0.0)
          received = true;
      }
      check(received, "the other controller receives a digital sensor frame on the shared tick");
      var source = simulation.world.robot("assembly:tending-controllers:" + first.from.controller);
      var signals = simulation.cellSignals;
      if (source == null || signals == null) throw "Tending signal has no source runtime or bus";
      // Plans anchor to commanded state; servo tracking samples are observations.
      var pose = source.snapshot().setpointPositions.toArray();
      var zero = [for (_ in pose) 0.0];
      var hold = new TrajectorySegment(Int64.ofInt(0), Int64.ofInt(200000000),
        [for (position in pose) [position, 0.0]]);
      var outputEvent = new ProcessTimedEvent(Int64.ofInt(10000000), first.name, ProcessEventValue.Digital(true));
      source.submit(robotkit.core.RobotCommand.ExecutionPlan(new ExecutionPlanSubmission(
        Int64.ofInt(1801), Int64.ofInt(simulation.appliedRevision), Int64.ofInt(0),
        RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE |
          RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_EVENTS, pose, zero, zero, [hold],
        null, null, null, null, null, true, [outputEvent])));
      var fired = false;
      for (_ in 0...100) {
        simulation.step();
        if (signals.readOutput(first.from.controller, first.name)) {
          fired = true;
          var input = signals.input(first.to.controller, first.name);
          check(input != null && switch input {case Digital(on): !on; case _: false;},
            "a real runtime output stays invisible to the receiver in its firing tick");
          break;
        }
      }
      check(fired, "the source runtime fires its authored digital output");
      simulation.step();
      var nextInput = signals.input(first.to.controller, first.name);
      check(nextInput != null && switch nextInput {case Digital(on): on; case _: false;},
        "the real runtime output crosses the cabinet bus on the following tick");
      var high = false;
      var nextFrames = target.snapshot().sensors;
      for (index in 0...nextFrames.length) {
        var frame = nextFrames.get(index);
        if (frame.sensorId == first.name && frame.values.length == 1 && frame.values.get(0) == 1.0)
          high = true;
      }
      check(high, "the receiving runtime publishes the real high digital input");
      check(simulation.cncPlayer() == null, "the controller-only cell has no CNC job player");
      simulation.capturePresentationSnapshot();
      var beltUpdates = simulation.beltDisplayUpdates();
      var spindleChannels = new toolpathkit.motion.ToolpathChannels("mill/mill");
      for (name in [spindleChannels.spindleDirection, spindleChannels.spindleSpeed])
        check([for (channel in target.description().channels) if (channel.id == name) channel].length == 1,
          "the mill controller owns spindle output " + name);
      // Spindle speed is a cabinet process output, independently owned from
      // joint motion. Exercise it without inventing a full-axis hold plan.
      var spindleRuntime = runtimes[ids.indexOf(target.id())];
      spindleRuntime.writeOutput(spindleChannels.spindleDirection, ProcessEventValue.Analog(1.0));
      spindleRuntime.writeOutput(spindleChannels.spindleSpeed, ProcessEventValue.Analog(1000.0));
      var spindleIndex = target.description().joints.indexOf("mill/mill/spindle-motor-turn");
      if (spindleIndex < 0) throw "Cell has no spindle motor joint";
      var spindleStart = target.snapshot().positions.get(spindleIndex);
      for (_ in 0...100) simulation.step();
      simulation.capturePresentationSnapshot();
      check(Math.abs(target.snapshot().positions.get(spindleIndex) - spindleStart) > 0.1,
        "cell spindle outputs rotate its physical motor without a CNC player");
      check(simulation.beltDisplayUpdates() > beltUpdates,
        "cell spindle belt animates on its owning controller without a CNC job");
      var spindleSpeed = Math.abs(target.snapshot().velocities.get(spindleIndex));
      check(spindleSpeed > 1.0, "the spindle is rotating before emergency stop");
      target.stop(robotkit.core.StopMode.Emergency);
      simulation.step();
      check(Math.abs(target.snapshot().velocities.get(spindleIndex)) > 0.9 * spindleSpeed,
        "an emergency-stopped spindle coasts instead of applying full braking torque");
      Sys.println('tending controllers: ${ids.length} robots, ${[for (_ in joints.keys()) _].length} disjoint joints');
    } catch (error:Dynamic) {
      simulation.dispose(); session.dispose(); throw error;
    }
    simulation.dispose(); session.dispose();
  }

  /** A physical pneumatic gripper carries a dynamic blank between the robot-arm cell's pads. */
  static function checkParallelGripperMission(root:String):Void {
    var arm = new RobotArm(true, new ArmGripperTool(32, 60, 60, 16));
    if (arm.tool == null) throw "Physical-gripper fixture has no end effector";
    var cell = new MachineAssembly();
    cell.include("arm", arm);
    cell.addComponent("airSupply", new AirSupply(), AssemblyFrames.translation(3000, 3000, 0));
    cell.connectPorts("gripper-air", "airSupply", "air", "arm", "compressedAir");
    var artifact = AssemblyPreview.scene(cell, "mt8-parallel-gripper");
    artifact.robotTools = RobotScene.robotTools(arm.tool, "arm/tool");
    artifact.missions = [{loop: true, steps: [
      {kind: "pick", at: {occurrence: "arm/workpiece", connector: "top"}},
      {kind: "place", at: {occurrence: "arm/padPlace", connector: "top"}},
      {kind: "pick", at: {occurrence: "arm/workpiece", connector: "top"}},
      {kind: "place", at: {occurrence: "arm/padPick", connector: "top"}}
    ]}];
    var generated = new ProjectArtifactLoader().load(SceneArtifact.encode(artifact));
    var blanks = [for (object in generated.objects) if (object.id == "project:arm/workpiece") object];
    if (blanks.length != 1) throw "Physical-gripper fixture needs one generated blank";
    blanks[0].dynamicBody = true;
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-arm/materia.project.json");
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      check(simulation.rebuild(session.sensors, session.scene, session),
        "physical-gripper fixture builds: " + simulation.error);
      var mission = simulation.missionPlayer();
      var blank = simulation.environmentObject("project:arm/workpiece");
      if (mission == null || blank == null) throw "Physical-gripper mission lost its player or dynamic blank";
      var handlingChannel = singleHandlingChannel(mission);
      var toolKinematics = mission.toolKinematics.get(handlingChannel);
      if (toolKinematics == null) throw "Physical-gripper mission has no tool kinematics";
      var jawIndex = -1;
      for (index in 0...mission.robot.model.joints.length)
        if (mission.robot.model.joints[index].id == "arm/tool/gripper/jaw-left-slide") jawIndex = index;
      if (jawIndex < 0) throw "Physical-gripper mission has no driven jaw joint";
      var armJointIndices = [for (index in 0...mission.robot.model.joints.length)
        if (StringTools.startsWith(mission.robot.model.joints[index].id, "arm/j")) index];
      if (armJointIndices.length != 6) throw "Physical-gripper fixture needs all six arm joints";
      check(blank.motion == nativekit.sim.MotionType.Dynamic,
        "the workpiece remains a dynamic body throughout physical handling");
      var workpiece = blank;
      var gripStart:Null<robotkit.spatial.Transform3> = null;
      var carryStart:Null<robotkit.spatial.Transform3> = null;
      var fingerLinks = [
        mission.robot.part("project:arm/tool/gripper/fingerLeft").linkIndex,
        mission.robot.part("project:arm/tool/gripper/fingerRight").linkIndex
      ];
      var blankCorners = [for (x in [-0.03, 0.03]) for (y in [-0.03, 0.03]) for (z in [-0.025, 0.025])
        new robotkit.spatial.Vec3(x, y, z)];
      // Bound seating while the support toes take the blank's load off the table. Start the
      // transport measurement at the stopped end of the planned pickup lift, before the arm
      // accelerates into its carry motion.
      var freeCarryCenterZ = (RobotArm.TABLE_TOP + RobotArm.PAD_HEIGHT +
        RobotArm.WORKPIECE_HEIGHT) / 1000.0;
      var pickupTopZ = freeCarryCenterZ - RobotArm.WORKPIECE_HEIGHT / 2000.0 +
        motionkit.robot.HandlingPlanRunner.DEFAULT_APPROACH_HEIGHT - 0.002;
      var worstSlip = 0.0, carrySlip = 0.0, carryTranslationSlip = 0.0, carryRotationSlip = 0.0;
      var worstPickupSeatSlip = 0.0, carryPickupSeatSlip = 0.0;
      var peakArmTorque = 0.0;
      var carryPeakTime = 0.0, carryPeakZ = 0.0;
      var carryLeftContacts = 0, carryRightContacts = 0;
      var carryContactGapSteps = 0;
      var carryOtherContacts:Array<String> = [];
      var carryStartTool:Null<robotkit.spatial.Transform3> = null;
      var carryActive = false;
      var stableGripSeconds = 0.0;
      var carryMotionObserved = false, measuredCarryMotionObserved = false;
      var lastSimulationTime = simulation.activeSession().simulationTime();
      var observedFingerContact = false, remainedFreeDynamicBody = true;
      var requiredSteps = 50 * 4;
      while (mission.completed < requiredSteps && simulation.activeSession().simulationTime() < 1500) {
        simulation.step();
        var simulationTime = simulation.activeSession().simulationTime();
        var elapsed = simulationTime - lastSimulationTime;
        lastSimulationTime = simulationTime;
        var runtimeSnapshot = mission.robot.runtime.snapshot();
        if (mission.failure != null)
        {
          var state = runtimeSnapshot;
          var gripSignal = -1.0, emptySignal = -1.0;
          for (frame in state.sensors.toArray()) {
            if (frame.values.length == 0) continue;
            if (frame.sensorId == "arm/tool/gripper/gripped") gripSignal = frame.values.get(0);
            if (frame.sensorId == "arm/tool/gripper/jaw-closed-empty") emptySignal = frame.values.get(0);
          }
          var failedPartPose = mission.simulation.objectPose(workpiece);
          throw 'Parallel-gripper mission failed after ${mission.completed} of $requiredSteps steps: ${mission.failure}; ' +
            't=${simulation.activeSession().simulationTime()} q=${state.q.toArray()} ' +
            'gripped=$gripSignal empty=$emptySignal slip=$worstSlip ' +
            'fingerContact=$observedFingerContact peakArmTorque=$peakArmTorque N m ' +
            'part=(${failedPartPose.x},${failedPartPose.y},${failedPartPose.z})';
        }
        var grippedSignal = -1.0;
        for (frame in runtimeSnapshot.sensors.toArray())
          if (frame.sensorId == "arm/tool/gripper/gripped" && frame.values.length > 0)
            grippedSignal = frame.values.get(0);
        var closeCommand = switch mission.robot.runtime.channelValue("arm/tool/gripper/body.close") {
          case Digital(value): value;
          case _: false;
        };
        var settledJaw = Math.abs(runtimeSnapshot.dq.get(jawIndex)) < 0.005;
        if (grippedSignal > 0.5 && closeCommand && settledJaw)
          stableGripSeconds += elapsed;
        else
          stableGripSeconds = 0.0;
        if (grippedSignal > 0.5 && closeCommand && stableGripSeconds >= 0.05) {
          carryActive = true;
          remainedFreeDynamicBody = remainedFreeDynamicBody &&
            mission.simulation.objectCarrier(workpiece).rawValue() == 0;
          var link = mission.simulation.linkPose(mission.robotIndex, toolKinematics.link);
          var toolTip = toolKinematics.tip;
          var world_T_link = new robotkit.spatial.Transform3(
            robotkit.spatial.Vec3.fromArray(link.position), robotkit.spatial.Quat.fromArray(link.rotation));
          var link_T_tcp = new robotkit.spatial.Transform3(
            new robotkit.spatial.Vec3(toolTip.x, toolTip.y, toolTip.z),
            new robotkit.spatial.Quat(toolTip.qx, toolTip.qy, toolTip.qz, toolTip.qw));
          var tcp = world_T_link.compose(link_T_tcp);
          var objectPose = mission.simulation.objectPose(workpiece);
          var world_T_object = new robotkit.spatial.Transform3(
            new robotkit.spatial.Vec3(objectPose.x, objectPose.y, objectPose.z),
            new robotkit.spatial.Quat(objectPose.qx, objectPose.qy, objectPose.qz, objectPose.qw));
          var relative = tcp.inverse().compose(world_T_object);
          if (gripStart == null) gripStart = relative;
          if (objectPose.z > freeCarryCenterZ && carryStart == null) {
            var initial = gripStart;
            if (initial != null) for (corner in blankCorners) {
              var seatSlip = relative.transformPoint(corner).sub(initial.transformPoint(corner)).norm();
              carryPickupSeatSlip = Math.max(carryPickupSeatSlip, seatSlip);
            }
            worstPickupSeatSlip = Math.max(worstPickupSeatSlip, carryPickupSeatSlip);
          }
          if (carryStart == null && objectPose.z >= pickupTopZ) {
            carryStart = relative;
            carryStartTool = tcp;
            carrySlip = 0.0;
            carryTranslationSlip = 0.0;
            carryRotationSlip = 0.0;
            carryPeakTime = 0.0;
            carryPeakZ = 0.0;
            carryLeftContacts = 0;
            carryRightContacts = 0;
            carryContactGapSteps = 0;
            carryOtherContacts = [];
            carryMotionObserved = false;
          } else if (carryStart != null && objectPose.z > freeCarryCenterZ) {
            var start = carryStart;
            var toolStart = carryStartTool;
            if (toolStart != null && (tcp.translation.sub(toolStart.translation).norm() > 0.001 ||
                tcp.rotation.angularDistance(toolStart.rotation) > 0.001)) carryMotionObserved = true;
            measuredCarryMotionObserved = measuredCarryMotionObserved || carryMotionObserved;
            var translationDelta = relative.translation.sub(start.translation);
            var translationSlip = translationDelta.norm();
            var rotationSlip = relative.rotation.angularDistance(start.rotation);
            carryTranslationSlip = Math.max(carryTranslationSlip, translationSlip);
            carryRotationSlip = Math.max(carryRotationSlip, rotationSlip);
            if (start != null) for (corner in blankCorners) {
              var slip = relative.transformPoint(corner).sub(start.transformPoint(corner)).norm();
              if (slip > carrySlip) {
                carryPeakTime = simulationTime;
                carryPeakZ = objectPose.z;
              }
              carrySlip = Math.max(carrySlip, slip);
              worstSlip = Math.max(worstSlip, slip);
            }
          }
          for (jointIndex in armJointIndices)
            peakArmTorque = Math.max(peakArmTorque, Math.abs(runtimeSnapshot.effort.get(jointIndex)));
          var leftContactThisStep = false, rightContactThisStep = false;
          for (contact in mission.simulation.robotContacts(mission.robot.runtime)) {
            var fingerIndex = fingerLinks.indexOf(contact.linkIndex);
            if (contact.active && contact.otherKind == robotkit.runtime.RobotContactOtherKind.Object &&
                contact.otherObject == workpiece.handle.rawValue() && fingerIndex >= 0) {
              observedFingerContact = true;
              if (fingerIndex == 0) { carryLeftContacts++; leftContactThisStep = true; }
              else { carryRightContacts++; rightContactThisStep = true; }
            } else if (contact.active && fingerIndex >= 0) {
              var otherLink = contact.otherLink >= 0 && contact.otherLink < mission.robot.model.links.length
                ? mission.robot.model.links[contact.otherLink].id : 'link-${contact.otherLink}';
              var extra = '${contact.otherKind}:${contact.otherObject}/$otherLink@${contact.distance}';
              if (carryOtherContacts.length < 8 && carryOtherContacts.indexOf(extra) < 0)
                carryOtherContacts.push(extra);
            }
          }
          if (objectPose.z > freeCarryCenterZ && (!leftContactThisStep || !rightContactThisStep))
            carryContactGapSteps++;
        }
        else if (carryActive) {
          check(carryLeftContacts > 0 && carryRightContacts > 0,
            'each physical carry contacts both pads (left=$carryLeftContacts right=$carryRightContacts)');
          check(carryContactGapSteps == 0,
            'both pads maintain contact throughout free carry (gap steps=$carryContactGapSteps)');
          check(carryMotionObserved,
            "the gripper slip measurement includes arm motion after the stopped pickup lift");
          check(carryPickupSeatSlip < 0.001,
            'the blank seats on the support toes within 1 mm during pickup (${carryPickupSeatSlip * 1000} mm)');
          check(carrySlip < 0.0005,
            'the dynamic blank slips ${carrySlip * 1000} mm relative to the TCP while both pads hold; ' +
            'translation=${carryTranslationSlip * 1000} mm rotation=${carryRotationSlip * 1000} mrad ' +
            'peakAt=${carryPeakTime}s z=${carryPeakZ}m ' +
            'contactGapSteps=$carryContactGapSteps otherContacts=${carryOtherContacts.join(",")} ' +
            'peakArmTorque=$peakArmTorque N m');
          carryActive = false;
          gripStart = null;
          carryStart = null;
          carryStartTool = null;
        }
      }
      check(mission.completed >= requiredSteps,
        'physical gripper completes 50 pick/carry/place round trips (${mission.completed} steps)');
      check(observedFingerContact, "the workpiece load reaches the arm through real jaw contact");
      check(remainedFreeDynamicBody, "the workpiece remains a free body instead of being kinematically attached");
      check(worstSlip < 0.0005,
        'the dynamic blank slips ${worstSlip * 1000} mm relative to the gripper TCP over 50 round trips');
      check(worstPickupSeatSlip < 0.001,
        'the support toes seat the blank within 1 mm during pickup (${worstPickupSeatSlip * 1000} mm worst)');
      check(measuredCarryMotionObserved, "the dynamic grip was measured during actual transport");
      check(peakArmTorque > 0.01 && Math.isFinite(peakArmTorque),
        'the arm servos produce a finite torque while carrying the dynamic blank (${peakArmTorque} N m peak)');

      // Let the next pick plan against the blank, then remove it from the target at the next
      // physics step. The real jaws should close empty and the handling skill should report failure.
      simulation.step();
      workpiece.setPoseForNextTick(new nativekit.sim.SimPose(3.0, 3.0, 1.0));
      var missDeadline = simulation.activeSession().simulationTime() + 120;
      while (mission.failure == null && simulation.activeSession().simulationTime() < missDeadline)
        simulation.step();
      check(mission.failure != null && mission.failure.indexOf("did not confirm a grasp") >= 0,
        'a blank removed during the pick is reported as a missed grasp: ${mission.failure}');
      var emptyClosed = false;
      for (frame in mission.robot.runtime.snapshot().sensors.toArray())
        if (frame.sensorId == "arm/tool/gripper/jaw-closed-empty" && frame.values.length > 0)
          emptyClosed = frame.values.get(0) > 0.5;
      check(emptyClosed, "a missing blank drives the physical jaws to the closed-empty switch");
      Sys.println('parallel gripper: 50 physical round trips, ${Math.round(worstSlip * 1000000) / 1000} mm peak slip, ' +
        '${Math.round(worstPickupSeatSlip * 1000000) / 1000} mm pickup seating, ' +
        '${Math.round(peakArmTorque * 100) / 100} N m peak arm torque, empty grasp reported');
    } catch (error:Dynamic) {
      simulation.dispose(); session.dispose(); throw error;
    }
    simulation.dispose(); session.dispose();
  }

  /** Scripted robot outputs exercise the cabinet bus, interlocks, real valves and one CNC pass. */
  static function checkTendingRobotInterface(root:String):Void {
    var tendingCell = new TendingControllers();
    var cell = SceneArtifact.decode(tendingCell.previewArtifact());
    var cellAssembly = cell.assemblyDefinition, cellAssemblyState = cell.assemblyState;
    if (cellAssembly == null || cellAssemblyState == null)
      throw "Tending CNC preview has no assembly state";
    var loadCell = new AssemblyState(cellAssembly, cellAssemblyState);
    var infeedBlanks = [for (item in cellAssembly.occurrences)
      if (StringTools.startsWith(item.id, "infeed/blank-")) item];
    if (infeedBlanks.length != 6) throw "Tending cell has no six free infeed blank occurrences";
    for (index in 0...3) {
      var axis = ["x", "y", "z"][index];
      loadCell.setJoint('mill/mill/$axis', tendingCell.mill.loadPosition[index]);
    }
    loadCell.setJoint("mill/door", 0.0);
    var trayBlankPose = loadCell.worldPose(infeedBlanks[0].id);
    // Keep the scripted probe outside the machine's collision envelope until the door opens.
    var blankStagePose = {x: trayBlankPose.x - 2000, y: trayBlankPose.y, z: trayBlankPose.z,
      qx: trayBlankPose.qx, qy: trayBlankPose.qy, qz: trayBlankPose.qz, qw: trayBlankPose.qw};
    // Reuse the real infeed blank definition as the free probe that the scripted robot loads.
    cellAssembly.occurrences.push({id: "interface-test-blank",
      definition: infeedBlanks[0].definition, initialPose: blankStagePose});
    var cellState = new AssemblyState(cellAssembly, cellAssemblyState);
    for (axis in ["x", "y", "z"]) {
      var jointId = 'mill/mill/$axis';
      var joint = [for (item in cellState.definition.joints) if (item.id == jointId) item];
      var lower = joint.length == 1 ? joint[0].limits.lower : null;
      var upper = joint.length == 1 ? joint[0].limits.upper : null;
      if (lower == null || upper == null)
        throw 'Tending cell has no bounded CNC axis "$jointId"';
      cellState.setJoint(jointId, (lower + upper) / 2.0);
    }
    cellState.setJoint("mill/door", 0.0);
    cell.assemblyState = cellState.record();
    var enclosed = SceneArtifact.decode(BenchMillPreview.enclosed());
    var job = enclosed.machining;
    if (job == null) throw "Enclosed mill has no bearing-block job";
	job.axes = [for (axis in job.axes) "mill/" + axis];
	job.spindle = "mill/" + job.spindle;
	// The static vise blank is the source geometry, but this run must machine the
	// free scripted blank so presence selection, contacts, and mass all use its body.
	job.stocks = ["interface-test-blank"];
    if (job.toolPart != null) job.toolPart = "mill/" + job.toolPart;
    job.channelScope = "mill/" + job.channelScope;
    job.controllerId = "mill/cncController";
    var targetId = job.target;
    if (targetId == null) throw "Enclosed job has no target part";
    var targetParts = [for (part in enclosed.parts) if (part.id == targetId) part];
    if (targetParts.length != 1) throw "Enclosed job target geometry is missing";
    var cellTargets = [for (part in cell.parts) if (part.id == targetId) part];
    if (cellTargets.length == 0) cell.parts.push(targetParts[0]);
    else if (cellTargets.length != 1) throw "Generated tending project has duplicate machining target geometry";
    cell.machining = job;
    var generated = new ProjectArtifactLoader().load(SceneArtifact.encode(cell));
    if (generated.assemblyDefinition == null || generated.assemblyState == null)
      throw "Generated tending CNC project has no assembly state";
    var testBlank:Null<SceneObjectData> = null;
    for (object in generated.objects) {
      if (object.id == "project:interface-test-blank") testBlank = object;
    }
    if (testBlank == null) throw "Generated tending CNC project lost its scripted blank";
    testBlank.dynamicBody = true;
    testBlank.collisionEnabled = true;
    var generatedState = new cadkit.modeling.AssemblyState(generated.assemblyDefinition, generated.assemblyState);
    var generatedSensors = generated.assemblyDefinition.sensors;
    if (generatedSensors == null) throw "Generated tending cell lost its field sensors";
    var stockSensor = [for (sensor in generatedSensors) if (sensor.id == "mill/stock-present") sensor];
    var sensorOccurrence = stockSensor.length == 1 ? stockSensor[0].occurrence : null;
    var sensorConnector = stockSensor.length == 1 ? stockSensor[0].connector : null;
    if (sensorOccurrence == null || sensorConnector == null)
      throw "Generated tending cell lost its vise presence-sensor connector";
    var sensorPose = generatedState.worldConnector(sensorOccurrence, sensorConnector);
    if (Math.abs(generatedState.joint("mill/mill/y") -
        (cellState.joint("mill/mill/y"))) > 1e-9)
      throw 'Generated tending CNC project changed Y from ${cellState.joint("mill/mill/y")} to ${generatedState.joint("mill/mill/y")}';
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/machine-tending/materia.controllers.project.json");
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      check(simulation.rebuild(session.sensors, session.scene, session),
        "tending CNC cell builds: " + simulation.error);
      var player = simulation.cncPlayer();
      var signals = simulation.cellSignals;
      if (player == null || signals == null) throw "Tending CNC cell lost its interface runtime";
      var physicalSimulation:Simulation = @:privateAccess simulation.simulation;
      var machineRuntime = (@:privateAccess player.robot).runtime;
      function signal(name:String):Bool {
        var value = signals.input("robotController", name);
        return value != null && switch value {case Digital(on): on; case _: false;};
      }
      function sensorImage():String {
        var owner = simulation.world.robot("assembly:tending-controllers:mill/cncController");
        if (owner == null) return "missing CNC runtime";
        var snapshot = owner.snapshot();
        var doorIndex = owner.description().joints.indexOf("mill/door");
        var door = doorIndex < 0 ? "missing door joint" : 'doorQ=${snapshot.positions.get(doorIndex)}';
        var jawIndex = owner.description().joints.indexOf("mill/mill/vise/jaw");
        var jaw = jawIndex < 0 ? "missing vise jaw" : 'jawQ=${snapshot.positions.get(jawIndex)}';
        var values:Array<String> = [];
        for (frame in snapshot.sensors.toArray())
          values.push(frame.sensorId + "=" + (frame.values.length == 0 ? "empty" : Std.string(frame.values.get(0))));
        return '$door, $jaw; ${values.join(",")}';
      }
      function livePresenceFrame():Null<materia.assembly.AssemblyRecord.AssemblyFrame> {
        var sensorRuntime = simulation.world.robot("assembly:tending-controllers:mill/cncController");
        if (sensorRuntime == null) return null;
        var presence:Null<robotkit.core.SensorFrame> = null;
        for (sample in sensorRuntime.snapshot().sensors.toArray())
          if (sample.sensorId == "mill/stock-present") presence = sample;
        if (presence == null) return null;
        var presentation = simulation.capturePresentationSnapshot();
        var robotVisual = [for (robot in presentation.robots)
          if (robot.id == "assembly:tending-controllers:mill/cncController") robot];
        if (robotVisual.length != 1) return null;
        var link = [for (item in robotVisual[0].links) if (item.id == presence.linkId) item];
        if (link.length != 1) return null;
        var mount:materia.assembly.AssemblyRecord.AssemblyFrame = {
          x: presence.mountPosition.get(0), y: presence.mountPosition.get(1), z: presence.mountPosition.get(2),
          qx: presence.mountRotation.get(0), qy: presence.mountRotation.get(1),
          qz: presence.mountRotation.get(2), qw: presence.mountRotation.get(3)};
        var actualSensor = AssemblyRobot.compose({position: link[0].position, rotation: link[0].rotation}, mount);
        return {x: actualSensor.position[0], y: actualSensor.position[1], z: actualSensor.position[2],
          qx: actualSensor.rotation[0], qy: actualSensor.rotation[1],
          qz: actualSensor.rotation[2], qw: actualSensor.rotation[3]};
      }
      function looseBlankImage():String {
        var object = simulation.environmentObject("project:interface-test-blank");
        var active = simulation.activeSession();
        if (object == null || active == null) return "missing loose blank";
        var frame = active.capture();
        var pose = frame.objectPose(object);
        frame.dispose();
        var liveSensor = "missing live sensor mount";
        var sensorFrame = livePresenceFrame();
        if (sensorFrame != null) {
          var blankLocal = materia.assembly.AssemblyFrames.transformPoint(
            materia.assembly.AssemblyFrames.inverse(sensorFrame), pose.x, pose.y, pose.z);
          var presentation = simulation.capturePresentationSnapshot();
          var stockVisual = [for (item in presentation.environment) if (item.id == "project:mill/mill/stock") item];
          var stockImage = "missing";
          if (stockVisual.length == 1) {
            var stockPosition = stockVisual[0].position;
            stockImage = '(${stockPosition[0]},${stockPosition[1]},${stockPosition[2]})';
          }
          liveSensor = 'live=(${sensorFrame.x},${sensorFrame.y},${sensorFrame.z}); ' +
            'live-local=(${blankLocal.x},${blankLocal.y},${blankLocal.z}); stock=$stockImage';
        }
        return 'blank=(${pose.x},${pose.y},${pose.z}); ' +
          'stock-sensor=(${sensorPose.x * generated.metresPerUnit},' +
          '${sensorPose.y * generated.metresPerUnit},${sensorPose.z * generated.metresPerUnit}); $liveSensor';
      }
      function awaitSignal(name:String, expected:Bool, maxTicks:Int):Void {
        for (_ in 0...maxTicks) {
          simulation.step();
          if (signal(name) == expected) return;
          if (player.interfaceAlarm != RobotInterfaceAlarm.None) {
            var spindleStopped = @:privateAccess player.spindleStopped();
            var axesAtLoad = @:privateAccess player.axesAtLoad();
            var runtime = (@:privateAccess player.robot).runtime;
            var channels = @:privateAccess player.channels;
            var commandedSpeed = switch runtime.channelValue(channels.spindleSpeed) {
              case Analog(value): Std.string(value);
              case _: "<none>";
            };
            var commandedDirection = switch runtime.channelValue(channels.spindleDirection) {
              case Analog(value): Std.string(value);
              case _: "<none>";
            };
            var diagnosticRobot = simulation.world.robot("assembly:tending-controllers:mill/cncController");
            var spindleIndex = diagnosticRobot == null ? -1 :
              diagnosticRobot.description().joints.indexOf("mill/mill/spindle-turn");
            var actualSpindleVelocity = spindleIndex < 0 ? "<missing>" :
              Std.string(diagnosticRobot.snapshot().velocities.get(spindleIndex));
            throw 'Robot interface alarm ${player.interfaceAlarm} while waiting for $name=$expected ' +
              '(ready=${signal(RobotInterfaceChannels.CNC_READY)}, doorClosed=${signal(RobotInterfaceChannels.DOOR_CLOSED)}, ' +
              'doorOpen=${signal(RobotInterfaceChannels.DOOR_OPEN)}, clamped=${signal(RobotInterfaceChannels.CLAMPED)}, ' +
              'unclamped=${signal(RobotInterfaceChannels.UNCLAMPED)}, partPresent=${signal(RobotInterfaceChannels.PART_PRESENT)}, ' +
              'spindleStopped=$spindleStopped (v=$actualSpindleVelocity, command=$commandedSpeed, direction=$commandedDirection), ' +
              'axesAtLoad=$axesAtLoad, ' +
              'sensors=${sensorImage()})';
          }
          var failure = simulation.cncFailure();
          if (failure != null) throw 'CNC failed while waiting for $name: $failure';
        }
        var detail = name == RobotInterfaceChannels.PART_PRESENT ? looseBlankImage() :
          name == RobotInterfaceChannels.CYCLE_COMPLETE ? {
            var lines = (@:privateAccess player.source).split("\n");
            var line = player.currentLine > 0 && player.currentLine <= lines.length
              ? StringTools.trim(lines[player.currentLine - 1]) : "<between lines>";
            var progress = (@:privateAccess player.motion).progress();
            var playerRobot = @:privateAccess player.robot;
            var playerChannels = @:privateAccess player.channels;
            var spindleCommand = switch playerRobot.runtime.channelValue(playerChannels.spindleSpeed) {
              case Analog(value): Std.string(value);
              case _: "<no analog command>";
            };
            var spindleDirection = switch playerRobot.runtime.channelValue(playerChannels.spindleDirection) {
              case Analog(value): Std.string(value);
              case _: "<no analog command>";
            };
            var spindleDrive = [for (drive in playerRobot.blueprint.velocityDrives)
              if (drive.speedChannel == playerChannels.spindleSpeed)
                'joint=${drive.joint}, scale=${drive.radiansPerSpeedUnit}, maxRate=${drive.maxRate}, maxEffort=${drive.maxEffort}'];
            var owner = simulation.world.robot("assembly:tending-controllers:mill/cncController");
            var state = owner == null ? null : owner.snapshot();
            var joints = owner == null ? [] : owner.description().joints;
            var links = owner == null ? [] : owner.description().links;
            var robotIds = simulation.simulatedRobotIds();
            var watchedLinks = [for (name in ["mill/mill/spindle",
              "mill/mill/spindleMotorPulley", "mill/mill/holder", "mill/mill/tool"])
              links.indexOf(name)];
            var watchedContacts = [for (contact in physicalSimulation.robotContacts(machineRuntime))
              if (contact.active && watchedLinks.indexOf(contact.linkIndex) >= 0) {
                var otherRobotId = contact.otherRobot >= 0 && contact.otherRobot < robotIds.length
                  ? robotIds[contact.otherRobot] : "<unmapped robot>";
                var otherRobot = simulation.world.robot(otherRobotId);
                var otherLinks = otherRobot == null ? [] : otherRobot.description().links;
                var otherLink = contact.otherLink >= 0 && contact.otherLink < otherLinks.length
                  ? otherLinks[contact.otherLink] : 'link-${contact.otherLink}';
                '${links[contact.linkIndex]} -> ${contact.otherKind}/$otherRobotId/$otherLink d=${contact.distance}';
              }];
            var motionState:Array<String> = [];
            if (state != null) for (name in ["mill/mill/x", "mill/mill/y", "mill/mill/z",
                "mill/mill/spindle-turn", "mill/mill/spindle-motor-turn"]) {
              var joint = joints.indexOf(name);
              if (joint >= 0) motionState.push('$name q=${state.positions.get(joint)}, ' +
                'target=${state.setpointPositions.get(joint)}, v=${state.velocities.get(joint)}, e=${state.efforts.get(joint)}');
            }
            'line=${player.currentLine} "$line", progress=${progress.op}/${progress.pathDistance}, ' +
              'passes=${player.passes}, CNC failure=${player.failure}, simulation failure=${simulation.cncFailure()}, ' +
              'spindle-command-rpm=$spindleCommand, direction=$spindleDirection, drive=${spindleDrive.join("; ")}, ' +
              'signals=${sensorImage()}, spindle-contacts=${watchedContacts.join("; ")}, ' +
              'joints=${motionState.join("; ")}';
          } : "";
        throw 'Timed out waiting for $name=$expected at ${simulation.activeSession().simulationTime()} s $detail';
      }
      function request(name:String, response:String, maxTicks:Int):Void {
        signals.writeSignal(name, true);
        awaitSignal(response, true, maxTicks);
        signals.writeSignal(name, false);
        simulation.step();
      }

      // Readiness is delayed until the derived G53 load position has been reached.
      awaitSignal(RobotInterfaceChannels.CNC_READY, true, 10000);
      var machineRobot = simulation.world.robot("assembly:tending-controllers:mill/cncController");
      if (machineRobot == null) throw "Tending CNC controller disappeared after reaching its load position";
      var armContacts = [for (contact in physicalSimulation.robotContacts(machineRuntime))
        if (contact.otherKind == robotkit.runtime.RobotContactOtherKind.RobotLink && contact.active)
          contact];
      check(armContacts.length == 0,
        'the checked ready pose leaves the CNC machine physically clear at its load position; ' +
        'contacts=${[for (contact in armContacts) "${contact.otherRobot}/${contact.otherLink} on ${contact.linkIndex}"].join("; ")}');
      simulation.step();
      request(RobotInterfaceChannels.REQUEST_DOOR_OPEN, RobotInterfaceChannels.DOOR_OPEN, 800);
      var blankObject = simulation.environmentObject("project:interface-test-blank");
      if (blankObject == null) throw "The scripted loose blank has no simulation body";
      var presenceFrame = livePresenceFrame();
      if (presenceFrame == null) throw "The vise presence sensor has no live simulation frame";
      // Place the free physics blank at the geometry-derived vise seat. Its environmental contact
      // with the moving jaw is what should stop the clamp at the preset blank width.
      var blankCentre = materia.assembly.AssemblyFrames.transformPoint(presenceFrame,
        BenchMill.STOCK_WIDTH * generated.metresPerUnit / 2,
        BenchMill.STOCK_DEPTH * generated.metresPerUnit / 2,
        BenchMill.STOCK_HEIGHT * generated.metresPerUnit / 2);
      blankObject.setPoseForNextTick(new nativekit.sim.SimPose(blankCentre.x, blankCentre.y, blankCentre.z,
        presenceFrame.qx, presenceFrame.qy, presenceFrame.qz, presenceFrame.qw));
      awaitSignal(RobotInterfaceChannels.PART_PRESENT, true, 20);
      request(RobotInterfaceChannels.REQUEST_UNCLAMP, RobotInterfaceChannels.UNCLAMPED, 800);
      request(RobotInterfaceChannels.REQUEST_CLAMP, RobotInterfaceChannels.CLAMPED, 800);
      if (!signal(RobotInterfaceChannels.CLAMPED)) throw 'Vise lost its clamped switch before door close (${sensorImage()}; ${looseBlankImage()})';
      request(RobotInterfaceChannels.REQUEST_DOOR_CLOSE, RobotInterfaceChannels.DOOR_CLOSED, 800);
      if (!signal(RobotInterfaceChannels.CLAMPED)) throw 'Vise lost its clamped switch during door close (${sensorImage()}; ${looseBlankImage()})';
      if (!signal(RobotInterfaceChannels.PART_PRESENT))
        throw 'The seated blank left the vise before machining (${sensorImage()}; ${looseBlankImage()})';
      function blankPosition():Array<Float> {
        var active = simulation.activeSession();
        var body = simulation.environmentObject("project:interface-test-blank");
        if (body == null) throw "The seated CNC blank lost its physics body";
        var frame = active.capture();
        var pose = frame.objectPose(body);
        frame.dispose();
        return [pose.x, pose.y, pose.z];
      }
      var seatedBeforeCut = blankPosition();
		request(RobotInterfaceChannels.CYCLE_START, RobotInterfaceChannels.CYCLE_COMPLETE, 11000);
      check(player.passes == 1, "the CNC controller runs one pass after cycle_start");
      check(player.interfaceAlarm == RobotInterfaceAlarm.None,
        "the scripted cycle completes without interlock alarms");
      var seatedAfterCut = blankPosition();
      var blankTravelSquared = 0.0;
      for (axis in 0...3) {
        var difference = seatedAfterCut[axis] - seatedBeforeCut[axis];
        blankTravelSquared += difference * difference;
      }
      var blankTravel = Math.sqrt(blankTravelSquared);
      check(blankTravel < 0.00005,
        'the vise holds the free blank within 0.05 mm through real CNC accelerations, moved ${blankTravel * 1000} mm');
      check(player.completedStocks.length == 1 && player.completedStocks[0].occurrence == "interface-test-blank",
        "the presence input selects and finishes the loose blank occurrence");
      var finished = player.completedStocks[0];
      var expectedRemoval = expectedBearingBlockRemoval();
      check(Math.abs(finished.removedVolume - expectedRemoval) < expectedRemoval * 0.02,
        'the free blank removes the bearing-block volume within 2%, got ${finished.removedVolume * 1e9} mm³');
      check(finished.targetMass > 0 && finished.massError < 0.02,
        'the machined blank mass matches the CAD target within 2%, got ${finished.finishedMass} kg vs ${finished.targetMass} kg');
      check(finished.rapidContacts == 0 && finished.collisions == 0,
        'the free-blank cutting path has no rapid contacts or holder collisions');
      check(signal(RobotInterfaceChannels.DOOR_CLOSED),
        "the enclosure remains closed immediately after the CNC cycle");
      check(!signal(RobotInterfaceChannels.CNC_READY),
        "the CNC interface stays not-ready while the spindle coasts down");
      var coastTicks = 0;
      while (!signal(RobotInterfaceChannels.CNC_READY) && coastTicks < 11000) {
        simulation.step();
        coastTicks++;
        if (!signal(RobotInterfaceChannels.DOOR_CLOSED))
          throw "The enclosure unlocked before the spindle reached standstill";
        if (player.interfaceAlarm != RobotInterfaceAlarm.None)
          throw 'Robot interface alarm ${player.interfaceAlarm} while waiting for spindle coast-down';
      }
      if (!signal(RobotInterfaceChannels.CNC_READY)) {
        var spindleIndex = machineRobot.description().joints.indexOf("mill/mill/spindle-turn");
        var spindleVelocity = spindleIndex < 0 ? Math.NaN :
          machineRobot.snapshot().velocities.get(spindleIndex);
        throw 'CNC did not become ready after spindle coast-down in ${coastTicks * 0.01} s; ' +
          'spindle velocity=$spindleVelocity rad/s, signals=${sensorImage()}';
      }
      request(RobotInterfaceChannels.REQUEST_DOOR_OPEN, RobotInterfaceChannels.DOOR_OPEN, 800);
      request(RobotInterfaceChannels.REQUEST_UNCLAMP, RobotInterfaceChannels.UNCLAMPED, 800);
      check(simulation.activeSession().simulationTime() < 120,
        "the scripted load/cut/unload sequence stays within two minutes");
      Sys.println('tending MT9 stock: ${finished.occurrence}, ${finished.removedVolume * 1e9} mm³ removed, ' +
        '${finished.finishedMass} kg / ${finished.targetMass} kg, held ${blankTravel * 1000} mm');
      Sys.println('tending robot interface: one physical CNC pass, ${coastTicks * 0.01} s spindle coast-down, ' +
        '${simulation.activeSession().simulationTime()} s total');
    } catch (error:Dynamic) {
      simulation.dispose(); session.dispose(); throw error;
    }
    simulation.dispose(); session.dispose();
  }

  /** A confirmed grasp carries every CAD piece even when stock has a separate cutter contact policy. */
  static function checkTendingPayloadCarriers(mission:MissionPlayer):Int {
    var checked = 0;
    for (channel in mission.carryingTools.keys()) if (mission.carryingTools.get(channel) == true) {
      var held = mission.heldParts.get(channel), kinematics = mission.toolKinematics.get(channel);
      if (held == null || kinematics == null) throw "The loaded route lost its grasp carrier";
      var tool = mission.handlingTools.get(channel);
      if (tool == null) throw "The loaded route lost its handling tool";
      var captured = mission.cellCollisionSnapshot([]);
      var payload = [for (body in captured.hulls) if (body.member == held.occurrence) body];
      check(payload.length > 0, "Confirmed machining stock has CAD clearance geometry");
      for (piece in payload) {
        var carrier = switch piece.owner {
          case Held(controller, link): controller == mission.robotId() && link == mission.robot.model.links[kinematics.link].id;
          case _: false;
        };
        check(carrier, "Machining stock keeps its typed controller and gripper carrier despite its editor collision flag");
      }
      checked++;
    }
    return checked;
  }

  /** Exported joint approaches must keep the same heading as the following insertion. */
  static function checkTendingHandlingOrientations(mission:MissionPlayer):Void {
    var placement = mission.placement;
    if (placement == null) throw "Tending reach needs its authored joint coordinates";
    var steps = mission.mission.steps;
    for (index in 0...steps.length) {
      var step = steps[index];
      if (step.kind != "place") continue;
      var channel = step.tool, at = step.at, yaw = step.toolYaw;
      if (channel == null || at == null || yaw == null || index == 0)
        throw "Tending placement needs a selected tool, seat and approach";
      var previous = steps[index - 1], poses = previous.poses;
      if (previous.kind != "moveVia" || poses == null || poses.length == 0)
        throw "Tending placement needs its reach-certified joint approach";
      var targets = poses[poses.length - 1].joints;
      if (targets == null) throw "Tending approach lost its mechanical joint targets";
      var tool = mission.handlingTools.get(channel);
      if (tool == null) throw "Tending placement selects an unknown tool";
      var group = mission.toolArm(tool);
      var q:Array<Float> = [for (_ in 0...group.group.count()) 0.0];
      var indices = [for (target in group.toJointTargets(q)) target.joint];
      for (joint in 0...indices.length) {
        var name = mission.robot.model.joints[indices[joint]].name;
        var matches = [for (target in targets) if (target.joint == name) target];
        if (matches.length != 1) throw 'Tending approach has no unique target for "$name"';
        q[joint] = matches[0].position - placement.joint(name);
      }
      var picks = [for (pick in steps) if (pick.kind == "pick" && pick.tool == channel) pick];
      if (picks.length == 0) throw "Tending placement has no preceding grasp";
      var pickAt = picks[0].at;
      if (pickAt == null) throw "Tending grasp lost its part frame";
      var grasp = mission.findGrasp(pickAt, channel);
      var rotation = MissionPlayer.yawRotation(yaw).multiply(
        mission.placementRotation(at.occurrence).multiply(grasp.toolRotation));
      var pose = new motionkit.robot.ManipulatorKinematics(group, 1e-8).forward(q);
      var desired = new motionkit.kinematics.Pose3(pose.x, pose.y, pose.z,
        rotation.x, rotation.y, rotation.z, rotation.w);
      var error = motionkit.path.PoseMath.angle(pose, desired);
      check(error <= 0.0002,
        'Tending placement $index approach disagrees with its insertion heading by $error rad');
    }
  }

  /** MT14 C6: measured first clamp to fourth outfeed of the four-part job, seconds (machine-tending PLAN). */
  static inline var TENDING_FOURTH_OUTFEED_SECONDS:Float = 284.06;

  /** Play prepares on a worker; UI frames, cancellation and transport remain usable. */
  static function checkTendingInteractive(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/machine-tending/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var work = "/tmp/materia-tending-interactive-" + Sys.getPid();
    FileSystem.createDirectory(work);
    var fonts = FontCollection.create();
    fonts.add(root + "/haxeon/packages/ui/vendor/harfbuzz/perf/fonts/Roboto-Regular.ttf");
    var editor = new ReferenceEditorApp(fonts, work + "/workspace.json");
    editor.simulation.backgroundPacing = true;
    var stopGate:Null<sys.thread.Lock> = null;
    try {
      editor.session.openGeneratedProject(generated, manifest);
      editor.documentChanged();
      var queued = Sys.time();
      editor.commands.execute("sim.play");
      var playMilliseconds = (Sys.time() - queued) * 1000;
      check(editor.simulationBuild != null, "Play queues preparation without building on the UI thread");
      editor.cancelSimulationBuild();
      var deadline = Sys.time() + 60.0;
      while (editor.simulationBuild != null && Sys.time() < deadline) { editor.tick(); Sys.sleep(0.001); }
      check(editor.simulationBuild == null && !editor.simulation.isActive(), "Cancelled preparation installs no partial simulation");
      editor.commands.execute("sim.play");
      var frames = 0, slowestFrame = 0.0;
      deadline = Sys.time() + 60.0;
      while (editor.simulationBuild != null && Sys.time() < deadline) {
        var started = Sys.time();
        editor.tick();
        editor.submit(new LayoutFrame(1320.0, 900.0));
        slowestFrame = Math.max(slowestFrame, Sys.time() - started);
        frames++;
        Sys.sleep(0.005);
      }
      check(editor.simulationBuild == null && editor.simulation.isRunning(),
        "Completed preparation starts background playback: " + editor.logLines.join(" | ") +
        "; simulation error=" + editor.simulation.error);
      check(frames > 0, "The editor renders while simulation preparation is pending");
      var simulation = editor.simulation;
      deadline = Sys.time() + 15.0;
      while (simulation.capturePresentationSnapshot().simulationTime < 0.2 && Sys.time() < deadline) {
        editor.tick(); editor.submit(new LayoutFrame(1320.0, 900.0)); Sys.sleep(0.005);
      }
      check(simulation.capturePresentationSnapshot().simulationTime >= 0.2 && simulation.error == null,
        "Background playback advances and publishes complete presentation snapshots");
      editor.commands.execute("sim.pause");
      var paused = simulation.activeSession().simulationTime();
      Sys.sleep(0.02);
      check(simulation.activeSession().simulationTime() == paused, "Pause joins playback before returning");
      editor.commands.execute("sim.step");
      check(simulation.activeSession().simulationTime() > paused, "Paused playback can step on the UI owner");
      editor.commands.execute("sim.reset");
      check(simulation.activeSession().simulationTime() == 0, "Reset restores the fixed simulation clock");
      editor.commands.execute("sim.play");
      deadline = Sys.time() + 60.0;
      while (editor.simulationBuild != null && Sys.time() < deadline) { editor.tick(); Sys.sleep(0.001); }
      check(simulation.isRunning(), "Playback starts before the delayed Stop check: " + editor.logLines.join(" | "));
      Sys.sleep(0.03);
      // Hold playback inside a command to prove Stop never joins it on the UI thread.
      var entered = new sys.thread.Lock();
      stopGate = new sys.thread.Lock();
      var gate = stopGate;
      simulation.control(function() { entered.release(); gate.wait(10.0); });
      check(entered.wait(5.0), "The controlled playback delay starts: " + editor.logLines.join(" | ") +
        "; error=" + simulation.error + "; playback=" + simulation.playbackFailure);
      var stopStarted = Sys.time();
      editor.commands.execute("sim.stop");
      var stopMilliseconds = (Sys.time() - stopStarted) * 1000;
      check(editor.simulationStop != null, "Stop queues shutdown without joining playback");
      check(stopMilliseconds < 100, "Stop returns promptly while playback is still busy");
      var shutdown = editor.simulationStop;
      editor.commands.execute("sim.play");
      editor.commands.execute("sim.reset");
      editor.commands.execute("sim.step");
      editor.commands.execute("sim.stop");
      check(editor.simulationStop == shutdown && editor.simulationBuild == null,
        "Transport commands cannot replace or restart an in-flight shutdown");
      var stopFrames = 0, slowestStopFrame = 0.0;
      for (_ in 0...5) {
        var frameStarted = Sys.time();
        editor.tick(); editor.submit(new LayoutFrame(1320.0, 900.0));
        slowestStopFrame = Math.max(slowestStopFrame, Sys.time() - frameStarted);
        stopFrames++;
      }
      check(editor.simulationStop != null, "UI frames continue while shutdown waits for playback");
      gate.release();
      stopGate = null;
      deadline = Sys.time() + 30.0;
      while (editor.simulationStop != null && Sys.time() < deadline) {
        var frameStarted = Sys.time();
        editor.tick(); editor.submit(new LayoutFrame(1320.0, 900.0));
        slowestStopFrame = Math.max(slowestStopFrame, Sys.time() - frameStarted);
        stopFrames++;
        Sys.sleep(0.001);
      }
      check(editor.simulationStop == null && !simulation.isActive() && !simulation.isRunning(),
        "Async Stop retires playback and native worlds: " + editor.logLines.join(" | "));
      check(editor.logLines.indexOf("Returned to design mode") >= 0,
        "Stop returns to design after all retirement workers finish");
      // A second build must not overlap the process-wide planner shutdown, and app close drains retirement.
      editor.commands.execute("sim.play");
      deadline = Sys.time() + 60.0;
      while (editor.simulationBuild != null && Sys.time() < deadline) { editor.tick(); Sys.sleep(0.001); }
      check(simulation.isRunning(), "Play can build and restart after asynchronous retirement");
      editor.commands.execute("sim.stop");
      Sys.println(haxe.Json.stringify({tendingInteractive: true, playMilliseconds: playMilliseconds,
        preparationFrames: frames, slowestPreparationFrameMilliseconds: slowestFrame * 1000,
        stopMilliseconds: stopMilliseconds, stopFrames: stopFrames,
        slowestStopFrameMilliseconds: slowestStopFrame * 1000}));
    } catch (error:Dynamic) { if (stopGate != null) stopGate.release(); editor.dispose(); fonts.dispose(); throw error; }
    editor.dispose(); fonts.dispose();
  }

  /** Four real blank-to-outfeed cycles through the saved robot mission and wired CNC interface. */
  static function checkMachineTending(root:String, scope:TendingCheck):Void {
    TendingCyclePlan.TendingCyclePlanChecks.run();
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/machine-tending/materia.project.json");
    var loadStarted = Sys.time();
    var loadControl = scope == TendingCheck.Performance ? new ProjectLoadControl() : null;
    var generated = MateriaProjectRunner.loadProject(manifest, null, loadControl);
    var loadSeconds = Sys.time() - loadStarted;
    var savedMission = generated.missions[0];
    if (savedMission == null || savedMission.steps.length == 0)
      throw "The playable tending project does not save its robot mission";
    TendingMissionChecks.check(savedMission);
    for (step in savedMission.steps) if (step.kind == "pick" || step.kind == "place")
      check(step.yaw == null && step.toolYaw != null && Math.isFinite(step.toolYaw),
        "each reach-certified tending grasp saves its tool roll separately from part heading");
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      var buildStarted = Sys.time();
      check(simulation.rebuild(session.sensors, session.scene, session),
        "the playable tending cell builds: " + simulation.error);
      var buildSeconds = Sys.time() - buildStarted;
      var mission = simulation.missionPlayer(), cnc = simulation.cncPlayer();
      if (mission == null || cnc == null) throw "Playable tending project lost its mission or CNC player";
      if (scope == TendingCheck.Performance) {
        var feedSeconds = 0.0, physicsSeconds = 0.0, presentationSeconds = 0.0;
        simulation.start();
        for (_ in 0...200) {
          var started = Sys.time();
          simulation.feedMembers();
          feedSeconds += Sys.time() - started;
          started = Sys.time();
          simulation.activeSession().stepPaced();
          physicsSeconds += Sys.time() - started;
        }
        for (_ in 0...20) {
          var started = Sys.time();
          simulation.capturePresentationSnapshot();
          presentationSeconds += Sys.time() - started;
        }
        Sys.println(haxe.Json.stringify({tendingPerformance: true, loadSeconds: loadSeconds,
          buildSeconds: buildSeconds, feedMilliseconds: feedSeconds * 5,
          physicsMilliseconds: physicsSeconds * 5, presentationMilliseconds: presentationSeconds * 50,
          loading: loadControl.profile(), records: session.scene.records().length}));
        simulation.dispose(); session.dispose(); return;
      }
      checkTendingHandlingOrientations(mission);
      if (scope == TendingCheck.Reach) {
        simulation.clear(); return;
      }
      if (scope == TendingCheck.Vise) {
        var nativeSession = simulation.activeSession();
        var freeBlank = simulation.environmentObject("project:infeed/blank-0-0");
        var station:Null<materia.project.SceneArtifact.SceneArtifactPlace> = null;
        for (step in mission.mission.steps) if (station == null && step.kind == "place") station = step.at;
        if (nativeSession == null || freeBlank == null || station == null) throw "Vise seating fixture is incomplete";
        var drives = [for (drive in cnc.robot.blueprint.cylinderDrives)
          if (cnc.robot.model.joints[drive.joint].id == "mill/mill/vise/jaw") drive];
        if (drives.length != 1) throw "Vise seating fixture needs one real cylinder";
        var drive = drives[0], retract = drive.channelB;
        if (retract == null) throw "Vise seating fixture needs two valve coils";
        cnc.robot.runtime.writeDigitalOutput(drive.channelA, false);
        cnc.robot.runtime.writeDigitalOutput(retract, true);
        for (_ in 0...100) nativeSession.step();
        var openJaw = cnc.robot.runtime.snapshot().q.get(drive.joint);
        var seat = mission.liveConnectorFrame(station);
        var blankPart = mission.assemblyParts.get("project:infeed/blank-0-0");
        var center = blankPart.center;
        var at = AssemblyFrames.transformPoint(seat, center[0], center[1], center[2]);
        freeBlank.setPoseForNextTick(new nativekit.sim.SimPose(at.x, at.y, at.z + 0.001,
          seat.qx, seat.qy, seat.qz, seat.qw));
        for (_ in 0...100) nativeSession.step();
        var before = mission.partFrame("infeed/blank-0-0");
        cnc.robot.runtime.writeDigitalOutput(retract, false);
        cnc.robot.runtime.writeDigitalOutput(drive.channelA, true);
        for (_ in 0...100) nativeSession.step();
        var after = mission.partFrame("infeed/blank-0-0");
        var jawTravel = cnc.robot.runtime.snapshot().q.get(drive.joint) - openJaw;
        var normal = new robotkit.spatial.Quat(after.qx, after.qy, after.qz, after.qw)
          .conjugate().rotate(new robotkit.spatial.Vec3(0, 0, 1));
        Sys.println('vise contact seating: before=${[before.x,before.y,before.z,before.qx,before.qy,before.qz,before.qw]} ' +
          'after=${[after.x,after.y,after.z,after.qx,after.qy,after.qz,after.qw]} normal=${normal.toArray()}');
        check(normal.z >= 1 - 1e-6, "The real pneumatic vise keeps a seated free blank aligned for machining");
        check(jawTravel > 0.0025 && jawTravel < 0.0035,
          'The real clamp closes onto the preset blank rather than an empty stroke ($jawTravel m)');
        simulation.dispose(); session.dispose(); return;
      }
      // Startup includes confirmed door and vise opening, insertion and clamping; use one bounded route
      // budget for nominal startup and for resuming after injected faults.
      var routeTickBudget = Std.int(Math.ceil(120.0 / simulation.timestep));
      checkTendingAlarmClassification();
      var blank = simulation.environmentObject("project:infeed/blank-0-0");
      if (blank == null) throw "Tending cell has no free first infeed blank";
      var blankVisuals = simulation.capturePresentationSnapshot().environment;
      var blankPose = [for (entry in blankVisuals) if (entry.id == "project:infeed/blank-0-0") entry];
      if (blankPose.length != 1) throw "Tending cell could not capture its initial blank pose";
      var originalBlank = new nativekit.sim.SimPose(blankPose[0].position[0], blankPose[0].position[1],
        blankPose[0].position[2], blankPose[0].rotation[0], blankPose[0].rotation[1],
        blankPose[0].rotation[2], blankPose[0].rotation[3]);
      if (scope == TendingCheck.Route || scope == TendingCheck.RealtimeRoute) {
        var realtime = scope == TendingCheck.RealtimeRoute;
        if (realtime) simulation.start();
        function advanceRoute():Void {
          if (realtime) {
            var current = mission.mission.steps[mission.stepIndex];
            // Slow frames during the door handshake must not age CNC brake
            // feedback faster than the simulated door and controller advance.
            if (current.kind == "waitFor" && current.waitFor != null &&
                current.waitFor.name == RobotInterfaceChannels.DOOR_OPEN)
              Sys.sleep(0.04);
            simulation.runTicks(1);
          } else simulation.step();
        }
        var clampCommand = -1;
        for (index in 0...mission.mission.steps.length) {
          var step = mission.mission.steps[index];
          if (step.kind == "signal" && step.signal != null &&
              step.signal.name == RobotInterfaceChannels.REQUEST_CLAMP && step.signal.value) {
            clampCommand = index;
            break;
          }
        }
        if (clampCommand < 0) throw "Tending route has no seated-clamp command";
        var clampConfirmed = -1;
        for (index in clampCommand...mission.mission.steps.length) {
          var step = mission.mission.steps[index];
          if (step.kind == "waitFor" && step.waitFor != null && step.waitFor.name == RobotInterfaceChannels.CLAMPED) {
            clampConfirmed = index; break;
          }
        }
        if (clampConfirmed < 0) throw "Tending route has no physical clamp confirmation";
        var routeTicks = 0, checkedPayloadCarrier = false;
        while (mission.stepIndex <= clampConfirmed && mission.failure == null && routeTicks < routeTickBudget) {
          advanceRoute(); routeTicks++;
          if (!checkedPayloadCarrier && checkTendingPayloadCarriers(mission) > 0) checkedPayloadCarrier = true;
        }
        var signals = simulation.cellSignals;
        var doorOpen = false;
        if (signals != null) doorOpen = switch signals.input("robotController", RobotInterfaceChannels.DOOR_OPEN) {
          case Digital(value): value;
          case _: false;
        };
        var clamped = signals != null && switch signals.input("robotController", RobotInterfaceChannels.CLAMPED) {
          case Digital(value): value;
          case _: false;
        };
        check(mission.stepIndex > clampConfirmed && mission.failure == null && doorOpen && clamped,
          'the robot stages outside, enters after confirmed opening, seats the blank and confirms physical clamping ' +
            '(failure=${mission.failure}, step=${mission.stepIndex}/$clampCommand, ticks=$routeTicks/$routeTickBudget, ' +
            'time=${simulation.activeSession().simulationTime()} s, ' +
            'doorOpen=$doorOpen, cncAlarm=${cnc.interfaceAlarm}, cncFailure=${cnc.failure}, ' +
            'runner=${mission.runner.status()})');
        check(checkedPayloadCarrier, "The startup route checks its confirmed payload carrier before placement");
        var cycleCommand = -1;
        for (index in clampConfirmed...mission.mission.steps.length) {
          var step = mission.mission.steps[index];
          if (step.kind == "signal" && step.signal != null &&
              step.signal.name == RobotInterfaceChannels.CYCLE_START && step.signal.value) {
            cycleCommand = index; break;
          }
        }
        if (cycleCommand < 0) throw "Tending route has no first machining request";
        // Clamp feedback reports jaw contact, not the end of its contact transient.
        // Check machining alignment at the authored start boundary, after the arm
        // retreats and the door closes; do not add a test-only settling delay.
        while (mission.stepIndex < cycleCommand && mission.failure == null && routeTicks < routeTickBudget) {
          advanceRoute(); routeTicks++;
        }
        var doorClosed = signals != null && switch signals.input("robotController", RobotInterfaceChannels.DOOR_CLOSED) {
          case Digital(value): value;
          case _: false;
        };
        check(mission.stepIndex == cycleCommand && mission.failure == null && doorClosed &&
            simulation.heldObjectIds().length == 0 && cnc.completedStocks.length == 0,
          "The startup route reaches machining readiness with the door closed and the gripper empty");
        var placed = mission.partFrame("infeed/blank-0-0");
        var stockAxis = new robotkit.spatial.Quat(placed.qx, placed.qy, placed.qz, placed.qw)
          .conjugate().rotate(new robotkit.spatial.Vec3(0, 0, 1));
        check(stockAxis.z >= 1 - 1e-6,
          'The robot seats and clamps its blank within the machining alignment bound (${stockAxis.toArray()})');
        Sys.println('tending startup route: blank seated and clamped at ${simulation.activeSession().simulationTime()} s');
        simulation.dispose(); session.dispose();
        return;
      }
      blank.setPoseForNextTick(new nativekit.sim.SimPose(3.0, 3.0, 3.0));
      check(simulation.emergencyStopRobot(mission.robotId()),
        "the tending cell accepts a targeted emergency stop for its robot");
      for (_ in 0...5) {
        if (mission.failure != null) break;
        simulation.step();
      }
      check(mission.failure != null && mission.alarmCode == MissionAlarmCode.EmergencyStop,
        "an emergency stop fails the active mission with the stable emergency alarm code");
      var completedBeforeRecovery = mission.completed;
      check(simulation.retryMission(),
        "after the stop is acknowledged, the mission can request safety reset and retry");
      for (_ in 0...5) {
        if (mission.failure == null) break;
        simulation.step();
      }
      check(mission.failure == null && mission.completed == completedBeforeRecovery,
        "emergency-stop recovery resumes the failed mission step and preserves its completed-step count");
      for (_ in 0...600) {
        if (mission.failure != null) break;
        simulation.step();
      }
      check(mission.failure != null && mission.alarmCode == MissionAlarmCode.EmptyInfeed &&
        mission.completed == completedBeforeRecovery,
        "an empty infeed pocket stops before grasp with the stable empty-infeed alarm");
      blank.setPoseForNextTick(originalBlank);
      check(simulation.retryMission(), "restocking the empty pocket makes its failed presence wait recoverable");
      // Restocking reaches physics at the next step; the presence adapter samples
      // after that step, and the controller consumes the fresh frame on the one after.
      for (_ in 0...12) simulation.step();
      var infeedPresence = "missing";
      for (frame in mission.robot.runtime.snapshot().sensors.toArray())
        if (frame.sensorId == "infeed/slot-0-0-present" && frame.values.length > 0)
          infeedPresence = Std.string(frame.values.get(0));
      check(mission.failure == null && mission.completed > completedBeforeRecovery,
        'restocking clears the fault and resumes at the infeed-presence check ' +
          '(failure=${mission.failure}, alarm=${mission.alarmCode}, completed=${mission.completed}, ' +
          'before=$completedBeforeRecovery, step=${mission.stepIndex}, presence=$infeedPresence, ' +
          'runner=${mission.runner.status()})');
      var firstPick = -1;
      for (index in 0...mission.mission.steps.length) {
        var step = mission.mission.steps[index];
        if (step.kind == "pick" && step.at != null && step.at.occurrence == "infeed/blank-0-0") {
          firstPick = index;
          break;
        }
      }
      if (firstPick < 0) throw "Tending mission has no first infeed pick to fault-inject";
      var approachTicks = 0;
      var approachTimeout = mission.stepIndex >= 0 && mission.stepIndex < mission.mission.steps.length
        ? mission.mission.steps[mission.stepIndex].timeoutSeconds : null;
      var approachLimit = approachTimeout == null ? 2000 : Std.int(approachTimeout / 0.01) + 1;
      while (mission.stepIndex < firstPick && mission.failure == null && approachTicks < approachLimit) {
        simulation.step(); approachTicks++;
      }
      check(mission.stepIndex == firstPick && mission.failure == null,
        'the mission confirms infeed presence and reaches its first pick after empty-slot recovery ' +
          '(failure=${mission.failure}, step=${mission.stepIndex}, completed=${mission.completed}, ' +
          'ticks=$approachTicks/$approachLimit, runner=${mission.runner.status()})');
      var firstPickStep = mission.mission.steps[firstPick];
      var originalPickTimeout = firstPickStep.timeoutSeconds;
      var retryLimit = originalPickTimeout == null ? 2000 : Std.int(originalPickTimeout / simulation.timestep) + 1;
      var closeTicks = 0, closeRequested = false;
      while (mission.failure == null && !closeRequested && closeTicks < retryLimit) {
        simulation.step(); closeTicks++;
        closeRequested = switch mission.robot.runtime.channelValue(DualArmGripperTool.TOOL_CHANNEL_A) {
          case Digital(value): value;
          case _: throw "The first gripper coil is not a digital output";
        };
      }
      check(closeRequested && mission.stepIndex == firstPick && mission.failure == null,
        'the missing-grasp injection waits for the real close command after planning ' +
          '(failure=${mission.failure}, ticks=$closeTicks/$retryLimit)');
      blank.setPoseForNextTick(new nativekit.sim.SimPose(3.0, 3.0, 3.0));
      for (_ in 0...retryLimit) {
        if (mission.failure != null) break;
        simulation.step();
      }
      check(mission.failure != null && mission.alarmCode == MissionAlarmCode.MissedGrasp,
        'a blank lost at physical closure faults the pick with the stable missed-grasp alarm ' +
          '(failure=${mission.failure}, alarm=${mission.alarmCode})');
      blank.setPoseForNextTick(originalBlank);
      check(simulation.retryMission(),
        "restoring the missing workpiece makes the failed pick recoverable without clearing other controllers");
      var retryTicks = 0, retryOpened = false;
      while ((mission.failure != null || mission.stepIndex <= firstPick) && retryTicks < retryLimit) {
        simulation.step(); retryTicks++;
        for (frame in mission.robot.runtime.snapshot().sensors.toArray())
          if (frame.sensorId == "arm/tool/gripperA/jaw-open" && frame.values.length > 0 && frame.values.get(0) >= 0.5)
            retryOpened = true;
        if (mission.failure != null && !mission.recoveryPending) break;
      }
      check(mission.failure == null && mission.stepIndex > firstPick,
        'the restored blank is physically grasped after retrying the failed pick ' +
          '(failure=${mission.failure}, step=${mission.stepIndex}, completed=${mission.completed}, ' +
          'ticks=$retryTicks/$retryLimit, runner=${mission.runner.status()})');
      check(retryOpened, "Retry opens the empty latched gripper before its new physical grasp");
      Sys.println('tending recovery: emergency stop, empty infeed and missed grasp passed at ${simulation.activeSession().simulationTime()} s');
      var carriedVisuals = simulation.capturePresentationSnapshot().environment;
      var carriedPose = [for (entry in carriedVisuals) if (entry.id == "project:infeed/blank-0-0") entry];
      if (carriedPose.length != 1) throw "The first blank disappeared before the part-drop fault check";
      var beforeDrop = new nativekit.sim.SimPose(carriedPose[0].position[0], carriedPose[0].position[1],
        carriedPose[0].position[2], carriedPose[0].rotation[0], carriedPose[0].rotation[1],
        carriedPose[0].rotation[2], carriedPose[0].rotation[3]);
      var gripperOccurrence = "arm/" + DualArmGripperTool.GRIPPER_A_BODY;
      var gripperPart = mission.robot.part("project:" + gripperOccurrence);
      var gripperFrame = mission.robot.toolFrames.get(gripperOccurrence);
      if (gripperFrame == null) throw "The first gripper has no TCP for part-drop recovery";
      var recoveredPhysics = simulation.simulation;
      if (recoveredPhysics == null) throw "Part-drop recovery has no physical simulation";
      function gripperTcpPose():robotkit.spatial.Transform3 {
        var link = recoveredPhysics.linkPose(gripperPart.robotIndex, gripperPart.linkIndex);
        return robotkit.spatial.Transform3.fromArrays(link.position, link.rotation).compose(
          robotkit.spatial.Transform3.fromArrays(gripperFrame.position, gripperFrame.rotation));
      }
      var blankBeforeDrop = robotkit.spatial.Transform3.fromArrays(
        [beforeDrop.x, beforeDrop.y, beforeDrop.z], [beforeDrop.qx, beforeDrop.qy, beforeDrop.qz, beforeDrop.qw]);
      var gripperToBlank = gripperTcpPose().inverse().compose(blankBeforeDrop);
      mission.robot.runtime.writeDigitalOutput(DualArmGripperTool.TOOL_CHANNEL_A, false);
      for (_ in 0...150) {
        if (mission.failure != null) break;
        simulation.step();
      }
      check(mission.failure != null && mission.alarmCode == MissionAlarmCode.PartDropped,
        "a physical jaw opening while carrying latches the stable part-dropped alarm");
      check(simulation.heldObjectIds().indexOf("project:infeed/blank-0-0") < 0,
        "open jaws do not report the dropped blank as physically held");
      check(Std.string(mission.runner.status()) != "Running",
        "a mission fault cancels the active arm plan instead of leaving motion running");
      var stopTicks = 0;
      while (mission.robot.runtime.snapshot().safety != RobotKitRuntimeConstants.RK_SAFETY_READY &&
          stopTicks < 1000) {
        simulation.step(); stopTicks++;
      }
      check(mission.robot.runtime.snapshot().safety == RobotKitRuntimeConstants.RK_SAFETY_READY,
        'the arm completes its controlled stop before the operator closes the gripper (ticks=$stopTicks)');
      // A stopped arm has moved during its controlled stop. Reseat the operator-
      // recovered blank at the current TCP using its measured grasp transform.
      var restoredBlank = gripperTcpPose().compose(gripperToBlank);
      blank.setPoseForNextTick(new nativekit.sim.SimPose(restoredBlank.translation.x,
        restoredBlank.translation.y, restoredBlank.translation.z, restoredBlank.rotation.x,
        restoredBlank.rotation.y, restoredBlank.rotation.z, restoredBlank.rotation.w));
      mission.robot.runtime.writeDigitalOutput(DualArmGripperTool.TOOL_CHANNEL_A, true);
      var gripRestored = false;
      var settledGripSeconds = 0.0;
      var recoveredJaw = -1;
      for (index in 0...mission.robot.model.joints.length)
        if (mission.robot.model.joints[index].id == "arm/tool/gripperA/jaw-left-slide") recoveredJaw = index;
      if (recoveredJaw < 0) throw "Part-drop recovery has no physical jaw joint";
      var recoveredFingers = [mission.robot.part("project:arm/tool/gripperA/fingerLeft").linkIndex,
        mission.robot.part("project:arm/tool/gripperA/fingerRight").linkIndex];
      for (_ in 0...150) {
        // The operator supports the recovered blank while closing the jaws.
        // Entering the width switch's band alone does not establish a grasp.
        if (settledGripSeconds == 0.0) blank.setPoseForNextTick(new nativekit.sim.SimPose(
          restoredBlank.translation.x, restoredBlank.translation.y, restoredBlank.translation.z,
          restoredBlank.rotation.x, restoredBlank.rotation.y, restoredBlank.rotation.z, restoredBlank.rotation.w));
        simulation.step();
        var recoverySnapshot = mission.robot.runtime.snapshot();
        var widthConfirmed = false;
        for (frame in recoverySnapshot.sensors.toArray())
          if (frame.sensorId == "arm/tool/gripperA/gripped" && frame.values.length > 0 && frame.values.get(0) >= 0.5)
            widthConfirmed = true;
        var fingerContact = [false, false];
        for (contact in recoveredPhysics.robotContacts(mission.robot.runtime)) {
          var finger = recoveredFingers.indexOf(contact.linkIndex);
          if (contact.active && contact.otherKind == robotkit.runtime.RobotContactOtherKind.Object &&
              contact.otherObject == blank.handle.rawValue() && finger >= 0) fingerContact[finger] = true;
        }
        if (widthConfirmed && fingerContact[0] && fingerContact[1] &&
            Math.abs(recoverySnapshot.dq.get(recoveredJaw)) < 0.005)
          settledGripSeconds += simulation.timestep;
        else settledGripSeconds = 0.0;
        gripRestored = settledGripSeconds >= 0.05;
        if (gripRestored) break;
      }
      check(gripRestored, "the recovered blank is supported by both settled physical jaws after operator support ends");
      check(simulation.heldObjectIds().indexOf("project:infeed/blank-0-0") >= 0,
        "both jaw contacts report the recovered dynamic blank as physically held");
      check(simulation.retryMission(), "the recovered jaw and blank allow the failed carry to resume");
      for (_ in 0...8) {
        simulation.step();
        if (mission.failure == null) break;
      }
      check(mission.failure == null && mission.stepIndex > firstPick,
        "part-drop recovery preserves the held part and resumes the interrupted carry step");
      Sys.println('tending recovery: dropped-part regrasp passed at ${simulation.activeSession().simulationTime()} s');
      function cellSignal(name:String):Bool {
        var bus = simulation.cellSignals;
        if (bus == null) return false;
        return switch bus.input("robotController", name) {
          case Digital(value): value;
          case _: false;
        };
      }
      function cncRequest(name:String):Bool {
        var bus = simulation.cellSignals;
        if (bus == null) return false;
        return switch bus.input("mill/cncController", name) {
          case Digital(value): value;
          case _: false;
        };
      }
      var doorCommand = -1, doorWait = -1;
      for (index in 0...mission.mission.steps.length) {
        var step = mission.mission.steps[index];
        if (step.kind == "signal" && step.signal != null &&
            step.signal.name == RobotInterfaceChannels.REQUEST_DOOR_OPEN && step.signal.value && doorCommand < 0)
          doorCommand = index;
        if (step.kind == "waitFor" && step.waitFor != null &&
            step.waitFor.name == RobotInterfaceChannels.DOOR_OPEN && doorWait < 0)
          doorWait = index;
      }
      if (doorCommand < 0 || doorWait < 0) throw "Tending mission has no door-open handshake to fault-inject";
      var doorApproachTicks = 0;
      while (mission.stepIndex < doorCommand && mission.failure == null && doorApproachTicks < routeTickBudget) {
        simulation.step(); doorApproachTicks++;
      }
      check(mission.stepIndex == doorCommand && mission.failure == null,
        'the first door-open command is ready for a blocked-switch fault ' +
          '(failure=${mission.failure}, step=${mission.stepIndex}/$doorCommand, completed=${mission.completed}, ' +
          'ticks=$doorApproachTicks/$routeTickBudget, runner=${mission.runner.status()})');
      var cncRobotIndex = -1;
      var robotIds = simulation.simulatedRobotIds();
      for (index in 0...robotIds.length)
        if (StringTools.endsWith(robotIds[index], ":mill/cncController")) cncRobotIndex = index;
      var doorJoint = cnc.robot.robot.description().joints.indexOf("mill/door");
      if (cncRobotIndex < 0 || doorJoint < 0)
        throw "Tending cell has no CNC door joint to hold against the open request";
      var physics = simulation.simulation;
      if (physics == null) throw "Tending door fault has no physical simulation to jam";
      var doorPart = cnc.robot.part("project:mill/doorRight");
      var doorFrame = AssemblyRobot.partPose(physics, doorPart);
      var doorCenterOffset = AssemblyRobot.rotate(doorFrame.rotation, doorPart.center);
      var stopHalf:Array<Float> = [0.003, 0.006, 0.025];
      var stopOffset = AssemblyRobot.rotate(doorFrame.rotation,
        [0.0125 + stopHalf[0] + 0.0002, 0.0, 0.0]);
      var stopCenter = [for (axis in 0...3)
        doorFrame.position[axis] + doorCenterOffset[axis] + stopOffset[axis]];
      var jammedSession = simulation.activeSession();
      if (jammedSession == null) throw "Tending door fault lost its simulation session";
      jammedSession.stop();
      // A fixed stop sits 0.2 mm beyond the closed door's right edge. The powered
      // cylinder must meet real contact and fail to reach the open limit switch.
      var doorStop = jammedSession.createObject(nativekit.sim.MotionType.Static,
        nativekit.sim.SimShape.box(stopHalf[0], stopHalf[1], stopHalf[2]),
        new nativekit.sim.SimPose(stopCenter[0], stopCenter[1], stopCenter[2], doorFrame.rotation[0],
          doorFrame.rotation[1], doorFrame.rotation[2], doorFrame.rotation[3]));
      for (_ in 0...800) {
        if (mission.failure != null) break;
        simulation.step();
      }
      check(mission.failure != null && mission.alarmCode == MissionAlarmCode.DoorJam,
        "a door that cannot reach its physical switch raises the stable door-jam alarm");
      jammedSession.stop();
      doorStop.dispose();
      var beforeDoorRetry = mission.completed;
      check(simulation.retryMission(), "removing the door jam makes its timed-out request recoverable");
      simulation.step();
      simulation.step();
      check(cncRequest(RobotInterfaceChannels.FAULT_RESET),
        "the CNC receives one complete sampled image with the mission fault-reset pulse high");
      simulation.step();
      check(!cncRequest(RobotInterfaceChannels.FAULT_RESET),
        "the mission lowers fault reset after that controller image, independent of render frames");
      var doorRetryTicks = 0;
      while ((mission.failure != null || mission.stepIndex <= doorWait) && doorRetryTicks < 1000) {
        simulation.step(); doorRetryTicks++;
      }
      var doorContacts:Array<String> = [];
      if (mission.failure != null || !cellSignal(RobotInterfaceChannels.DOOR_OPEN))
        for (contact in physics.robotContacts(cnc.robot.runtime)) {
          if (!contact.active || contact.linkIndex != doorPart.linkIndex) continue;
          var other = "world";
          switch contact.otherKind {
            case robotkit.runtime.RobotContactOtherKind.Object:
              other = 'object ${contact.otherObject}';
              for (entry in simulation.simulatedObjects)
                if (entry.object.handle.rawValue() == contact.otherObject) other = entry.id;
            case robotkit.runtime.RobotContactOtherKind.RobotLink:
              other = 'robot ${contact.otherRobot}/link ${contact.otherLink}';
              if (contact.otherRobot >= 0 && contact.otherRobot < simulation.simulatedLinks.length) {
                var links = simulation.simulatedLinks[contact.otherRobot];
                if (contact.otherLink >= 0 && contact.otherLink < links.length)
                  other = simulation.simulatedIds[contact.otherRobot] + "/" + links[contact.otherLink];
              }
            case _: {}
          }
          doorContacts.push('$other (${contact.distance * 1000} mm)');
        }
      check(mission.failure == null && mission.stepIndex > doorWait && mission.completed > beforeDoorRetry &&
        cellSignal(RobotInterfaceChannels.DOOR_OPEN),
        'door-jam recovery resets the PLC, opens the door and resumes its failed wait ' +
          '(failure=${mission.failure}, alarm=${mission.alarmCode}, step=${mission.stepIndex}/$doorWait, ' +
          'completed=${mission.completed}/$beforeDoorRetry, open=${cellSignal(RobotInterfaceChannels.DOOR_OPEN)}, ' +
          'request=${cncRequest(RobotInterfaceChannels.REQUEST_DOOR_OPEN)}, ' +
          'faultReset=${cncRequest(RobotInterfaceChannels.FAULT_RESET)}, cncAlarm=${cnc.interfaceAlarm}, ' +
          'door=${cnc.robot.robot.snapshot().positions.get(doorJoint)}, ' +
          'doorContacts=${doorContacts.join("; ")}, ' +
          'runner=${mission.runner.status()}, ticks=$doorRetryTicks/1000)');
      check(cellSignal(RobotInterfaceChannels.DOOR_OPEN) && mission.failure == null,
        "the first physical door request reaches its open switch before injected cycle interlock");
      // Request a cycle with the door open. This exercises the CNC alarm-code outputs and
      // robot-side classification through the same one-tick cell wiring as a physical PLC.
      mission.robot.runtime.writeDigitalOutput(RobotInterfaceChannels.CYCLE_START, true);
      simulation.step();
      mission.robot.runtime.writeDigitalOutput(RobotInterfaceChannels.CYCLE_START, false);
      for (_ in 0...5) {
        if (mission.failure != null) break;
        simulation.step();
      }
      check(mission.failure != null && mission.alarmCode == MissionAlarmCode.CncAlarm,
        "a physical cycle interlock arrives as a coded CNC alarm at the robot mission");
      var beforeCycleRetry = mission.completed;
      check(simulation.retryMission(), "clearing the invalid cycle request makes the failed step recoverable");
      for (_ in 0...20) {
        simulation.step();
        if (mission.failure == null) break;
      }
      check(mission.failure == null && mission.completed == beforeCycleRetry,
        "cycle-interlock recovery clears the PLC alarm and resumes the same mission step");
      Sys.println('tending recovery: physical door jam and CNC interlock passed at ${simulation.activeSession().simulationTime()} s');

      var clampCommand = -1, clampWait = -1;
      for (index in 0...mission.mission.steps.length) {
        var step = mission.mission.steps[index];
        if (step.kind == "signal" && step.signal != null &&
            step.signal.name == RobotInterfaceChannels.REQUEST_CLAMP && step.signal.value && clampCommand < 0)
          clampCommand = index;
        if (step.kind == "waitFor" && step.waitFor != null &&
            step.waitFor.name == RobotInterfaceChannels.CLAMPED) {clampWait = index; break;}
      }
      if (clampCommand < 0 || clampWait < 0) throw "Tending mission has no clamp handshake to fault-inject";
      var clampTicks = 0;
      while (mission.stepIndex < clampCommand && mission.failure == null && clampTicks < routeTickBudget) {
        simulation.step(); clampTicks++;
      }
      var placementDiagnostic = "";
      var failedStep = mission.mission.steps[mission.stepIndex];
      if (mission.failure != null && failedStep.kind == "place") {
        var destination = failedStep.at;
        if (destination == null) throw "Failed placement has no destination";
        var channel = mission.handlingToolChannel(failedStep);
        var liveSeat = mission.liveConnectorFrame(destination);
        var authored = mission.placement.worldPose(destination.occurrence);
        var goal = mission.placeContact(destination, failedStep.yaw, channel);
        var tcp = mission.toolContact(channel);
        var relativeRotation = mission.heldToolRotations.get(channel);
        var rotationImage:Array<Float> = relativeRotation == null ? [] : relativeRotation.toArray();
        placementDiagnostic = 'seat=${[liveSeat.x, liveSeat.y, liveSeat.z]}, ' +
          'authored=${[authored.x, authored.y, authored.z]}, goal=${goal.toArray()}, tcp=${tcp.toArray()}, ' +
          'graspRotation=$rotationImage, ' +
          'q=${mission.robot.runtime.snapshot().q.toArray()}';
      }
      check(mission.stepIndex == clampCommand && mission.failure == null,
        'the physical blank reaches the first seated-clamp command ' +
          '(failure=${mission.failure}, alarm=${mission.alarmCode}, step=${mission.stepIndex}/$clampCommand, ' +
          'completed=${mission.completed}, cncAlarm=${cnc.interfaceAlarm}, ' +
          'doorOpen=${cellSignal(RobotInterfaceChannels.DOOR_OPEN)}, ' +
          'doorClosed=${cellSignal(RobotInterfaceChannels.DOOR_CLOSED)}, ' +
          'requestCycle=${cncRequest(RobotInterfaceChannels.CYCLE_START)}, ' +
          'runner=${mission.runner.status()}, ticks=$clampTicks/$routeTickBudget, $placementDiagnostic)');
      check(cellSignal(RobotInterfaceChannels.PART_PRESENT),
        "the placed blank reaches the live vise's physical presence volume before fault removal");
      var seatedImages = [for (entry in simulation.capturePresentationSnapshot().environment)
        if (entry.id == "project:infeed/blank-0-0") entry];
      if (seatedImages.length != 1) throw "The first blank has no physical vise pose for clamp recovery";
      var seatedPose = new nativekit.sim.SimPose(seatedImages[0].position[0], seatedImages[0].position[1],
        seatedImages[0].position[2], seatedImages[0].rotation[0], seatedImages[0].rotation[1],
        seatedImages[0].rotation[2], seatedImages[0].rotation[3]);
      blank.setPoseForNextTick(new nativekit.sim.SimPose(3.0, 3.0, 3.0));
      for (_ in 0...30) {
        if (mission.failure != null) break;
        simulation.step();
      }
      check(mission.failure != null && mission.alarmCode == MissionAlarmCode.ClampWithoutPart,
        "a physical clamp request without a sensed blank raises the stable clamp alarm");
      blank.setPoseForNextTick(seatedPose);
      var beforeClampRetry = mission.completed;
      check(simulation.retryMission(), "reseating the missing blank makes the clamp alarm recoverable");
      var clampRetryTicks = 0;
      while ((mission.failure != null || mission.stepIndex <= clampWait) && clampRetryTicks < 500) {
        simulation.step(); clampRetryTicks++;
      }
      var retrySeat = mission.partFrame("infeed/blank-0-0");
      var seatPlace = mission.mission.steps[clampCommand - 1].at;
      if (seatPlace == null) throw "Clamp recovery has no preceding placement station";
      var retryVise = mission.liveConnectorFrame(seatPlace);
      check(mission.failure == null && mission.stepIndex > clampWait && mission.completed > beforeClampRetry,
        'clamp-alarm recovery reseats the same blank, clamps it and resumes the mission ' +
          '(failure=${mission.failure}, alarm=${mission.alarmCode}, step=${mission.stepIndex}/$clampWait, ' +
          'completed=${mission.completed}/$beforeClampRetry, present=${cellSignal(RobotInterfaceChannels.PART_PRESENT)}, ' +
          'clamped=${cellSignal(RobotInterfaceChannels.CLAMPED)}, open=${cellSignal(RobotInterfaceChannels.UNCLAMPED)}, ' +
          'request=${cncRequest(RobotInterfaceChannels.REQUEST_CLAMP)}, cncAlarm=${cnc.interfaceAlarm}, ' +
          'q=${cnc.robot.runtime.snapshot().q.toArray()}, ticks=$clampRetryTicks/500, ' +
          'blank=${[retrySeat.x,retrySeat.y,retrySeat.z,retrySeat.qx,retrySeat.qy,retrySeat.qz,retrySeat.qw]}, ' +
          'seat=${[retryVise.x,retryVise.y,retryVise.z]}, original=${[seatedPose.x,seatedPose.y,seatedPose.z]})');
      // Start at the first seated-and-clamped blank; exclude earlier recovery fault injections.
      Sys.println('tending recovery: missing-part clamp passed at ${simulation.activeSession().simulationTime()} s');
      var productionStart = simulation.activeSession().simulationTime();
      var ticks = 0, spindleTicks = 0, observedStocks = 0, maxSimultaneousHeld = 0;
      var cellTicks = 0, cellSpindleTicks = 0;
      var fourthOutfeedTime = 0.0;
      function sensedOutfeeds():Int {
        var count = 0;
        for (frame in mission.robot.runtime.snapshot().sensors.toArray())
          if (StringTools.startsWith(frame.sensorId, "outfeed/slot-") && frame.values.length > 0 && frame.values.get(0) >= 0.5)
            count++;
        return count;
      }
      // MT14 C1: at every cycle start the blank sits on the vise seat within 0.1 mm / 2 mrad, and both
      // jaws touch it: the moving jaw, and an opposing contact along the same normal on the fixed side.
      var seatChecks = 0;
      var jawJoints = [for (joint in cnc.robot.model.joints) if (joint.id == "mill/mill/vise/jaw") joint];
      if (jawJoints.length != 1) throw "Tending cell lost its vise jaw joint";
      var movingJaw = cnc.robot.model.links.indexOf(jawJoints[0].child);
      if (movingJaw < 0) throw "Tending cell lost its moving vise jaw link";
      var blankNames = [for (row in 0...2) for (column in 0...3) 'infeed/blank-$row-$column'];
      function checkSeatedAtCycleStart():Void {
        var seat = mission.liveConnectorFrame({occurrence: "mill/mill/vise/body", connector: "seat"});
        var nearest = "", offset = Math.POSITIVE_INFINITY;
        for (name in blankNames) {
          var frame = mission.partFrame(name);
          var distance = Math.sqrt(Math.pow(frame.x - seat.x, 2) + Math.pow(frame.y - seat.y, 2) + Math.pow(frame.z - seat.z, 2));
          if (distance < offset) { offset = distance; nearest = name; }
        }
        var placed = mission.partFrame(nearest);
        var dot = Math.abs(placed.qx * seat.qx + placed.qy * seat.qy + placed.qz * seat.qz + placed.qw * seat.qw);
        var tilt = 2 * Math.acos(Math.min(1.0, dot));
        var object = simulation.environmentObject("project:" + nearest);
        if (object == null) throw 'Seated blank $nearest has no physics object';
        var handle = object.handle.rawValue();
        var touching = [for (contact in cnc.robot.runtime.contacts())
          if (contact.active && contact.otherKind == robotkit.runtime.RobotContactOtherKind.Object && contact.otherObject == handle) contact];
        var moving = [for (contact in touching) if (contact.linkIndex == movingJaw) contact];
        var opposed = moving.length > 0 && [for (contact in touching) if (contact.linkIndex != movingJaw &&
          Math.abs(contact.normal.dot(moving[0].normal)) > 0.9) contact].length > 0;
        seatChecks++;
        check(offset <= 1e-4 && tilt <= 2e-3,
          'cycle start $seatChecks: $nearest sits on the vise seat within 0.1 mm / 2 mrad (${offset * 1000} mm, ${tilt * 1000} mrad)');
        check(moving.length > 0 && opposed,
          'cycle start $seatChecks: both vise jaws touch $nearest (${moving.length} moving-jaw contacts, opposed=$opposed)');
        Sys.println('tending seat at cycle start $seatChecks: $nearest ${offset * 1000} mm, ${tilt * 1000} mrad, ' +
          '${moving.length} moving-jaw / ${touching.length} vise contacts');
      }
      var phaseStep = mission.stepIndex, phaseStart = productionStart;
      var exchangeOnly = scope == TendingCheck.Exchange;
      var lastCycleStart = 0.0, lastCycleTime = 0.0, wasCycleActive = false;
      var cycleTimes:Array<Float> = [];
      var expectedRemoval = expectedBearingBlockRemoval();
      // Conservative collision paths can take longer than the former exchange
      // prototype. Bound this replay by its remaining authored step budgets.
      var productionDeadline = productionStart;
      for (index in mission.stepIndex...mission.mission.steps.length) {
        var step = mission.mission.steps[index];
        var timeout = step.timeoutSeconds;
        var condition = step.waitFor;
        productionDeadline += timeout != null ? timeout : condition != null ? condition.timeout : 1.0;
      }
      while (!mission.finished && simulation.activeSession().simulationTime() < productionDeadline) {
        try simulation.step() catch (error:Dynamic)
          throw 'Tending production step ${mission.stepIndex}/${mission.mission.steps.length}: $error';
        var heldNow = simulation.heldObjectIds().length;
        if (heldNow > maxSimultaneousHeld) maxSimultaneousHeld = heldNow;
        var signalBus = simulation.cellSignals;
        if (mission.failure != null)
          throw 'Tending mission failed at step ${mission.stepIndex}/${mission.mission.steps.length} after ' +
            '${simulation.activeSession().simulationTime()} s: ${mission.failure} ' +
            '(CNC interface alarm ${cnc.interfaceAlarm}, robot zone ${cnc.robotInZone ? "occupied" : "clear"}, ' +
            'CNC failure ${cnc.failure})';
        if (mission.stepIndex != phaseStep) {
          var phase = mission.mission.steps[phaseStep];
          var seconds = simulation.activeSession().simulationTime() - phaseStart;
          if (seconds >= 1.0)
            Sys.println('tending phase $phaseStep: ${phase.kind}, ' +
              '${phase.at == null ? "signals" : phase.at.occurrence}, $seconds s');
          phaseStep = mission.stepIndex; phaseStart = simulation.activeSession().simulationTime();
          var next = mission.mission.steps[mission.stepIndex];
          if (next.kind == "signal" && next.signal != null && next.signal.name == RobotInterfaceChannels.CYCLE_START &&
              next.signal.value) checkSeatedAtCycleStart();
        }
        var machine:Null<Robot> = null;
        for (id in simulation.simulatedRobotIds()) if (StringTools.endsWith(id, ":mill/cncController"))
          machine = simulation.world.robot(id);
        if (machine == null) throw "Playable tending cell lost its CNC controller runtime";
        var spindle = machine.description().joints.indexOf("mill/mill/spindle-turn");
        if (spindle < 0) throw "Playable tending cell has no spindle encoder";
        var spindleRunning = Math.abs(machine.snapshot().velocities.get(spindle)) > 5.0 * 2 * Math.PI / 60.0;
        if (fourthOutfeedTime == 0.0) {
          cellTicks++;
          if (spindleRunning) cellSpindleTicks++;
          if (cnc.completedStocks.length == 4 && heldNow == 0 && sensedOutfeeds() == 4)
            fourthOutfeedTime = simulation.activeSession().simulationTime();
        }
        var cycleState = signalBus == null ? null : signalBus.input("robotController", RobotInterfaceChannels.IN_CYCLE);
        var cycleActive = cycleState != null && switch cycleState {case Digital(value): value; case _: false;};
        if (cycleActive && !wasCycleActive) lastCycleStart = simulation.activeSession().simulationTime();
        wasCycleActive = cycleActive;
        if (cycleActive) ticks++;
        if (cycleActive && spindleRunning) spindleTicks++;
        if (cnc.interfaceAlarm != RobotInterfaceAlarm.None)
          throw 'Tending CNC interface alarm ${cnc.interfaceAlarm} after ${simulation.activeSession().simulationTime()} s';
        var finished = cnc.completedStocks;
        while (observedStocks < finished.length) {
          var stock = finished[observedStocks++];
          check(Math.abs(stock.removedVolume - expectedRemoval) < expectedRemoval * 0.02,
            'tending blank ${stock.occurrence} removes the bearing-block volume within 2%, got ${stock.removedVolume * 1e9} mm³');
          check(stock.targetMass > 0 && stock.massError <= 0.02,
            'finished blank mass is within 2% of its CAD target (${stock.massError * 100}%)');
          check(stock.leftover + stock.gouge < expectedRemoval * 0.02,
            'finished blank agrees with the CAD cut shape: leftover ${stock.leftover * 1e9} mm³, gouge ${stock.gouge * 1e9} mm³');
          check(stock.rapidContacts == 0 && stock.collisions == 0,
            'finished blank ${stock.occurrence} has no cutter rapid contacts or collisions');
          var now = simulation.activeSession().simulationTime();
          if (lastCycleStart <= 0) throw 'CNC cycle start was not observed before finishing ${stock.occurrence}';
          lastCycleTime = now - lastCycleStart;
          cycleTimes.push(lastCycleTime);
          Sys.println('tending cut: blank $observedStocks, $lastCycleTime s, ${stock.removedVolume * 1e9} mm³ removed, ' +
            '${stock.leftover * 1e9} mm³ leftover, ${stock.gouge * 1e9} mm³ gouge');
        }
        if (exchangeOnly && observedStocks == 2) {
          check(maxSimultaneousHeld == 2, "The focused exchange physically retains both workpieces");
          Sys.println('tending exchange: two cuts, ${simulation.activeSession().simulationTime() - productionStart} s, ' +
            'whole-cell spindle use ${100.0 * cellSpindleTicks / cellTicks}%');
          simulation.dispose(); session.dispose(); return;
        }
      }
      // Successful replayed requests also increment the cumulative counter.
      check(mission.finished && mission.stepIndex == mission.mission.steps.length - 1 &&
          mission.completed >= mission.mission.steps.length,
        'the unattended tending mission finishes every declared step (${mission.completed}/${mission.mission.steps.length}, ' +
          '${mission.statusLabel(true)}, time=${simulation.activeSession().simulationTime()}, deadline=$productionDeadline)');
      check(cnc.completedStocks.length == 4,
        'four loose infeed blanks are machined and reported, got ${cnc.completedStocks.length}');
      check(cnc.interfaceAlarm == RobotInterfaceAlarm.None && simulation.cncFailure() == null,
        'four tending cycles finish without an interlock or CNC alarm');
      var outfeedCount = sensedOutfeeds();
      check(outfeedCount == 4,
        'four finished blanks occupy their sensed outfeed seats, got $outfeedCount');
      check(maxSimultaneousHeld == 2,
        'the MT12 exchange retains both independently gripped workpieces, max $maxSimultaneousHeld');
      check(cycleTimes.length == 4 && lastCycleTime > 0 && ticks > 0 && spindleTicks > 0,
        'the tending run reports all four CNC cycle intervals and spindle time');
      check(fourthOutfeedTime > productionStart && fourthOutfeedTime <= simulation.activeSession().simulationTime(),
        "whole-cell measurement ends at the fourth sensed outfeed with every physical gripper empty");
      var productionSeconds = fourthOutfeedTime - productionStart;
      var spindleUse = spindleTicks / ticks;
      var cellSpindleUse = cellSpindleTicks / cellTicks;
      check(Math.abs(cnc.spindleUtilization - spindleUse) < 0.01,
        'CNC-panel spindle utilization agrees with encoder sampling, got ${cnc.spindleUtilization * 100}% vs ${spindleUse * 100}%');
      var measuredUse = cnc.spindleUtilization;
      for (_ in 0...20) simulation.step();
      check(Math.abs(cnc.spindleUtilization - measuredUse) < 1e-12,
        "spindle utilization holds its completed-cycle measurement while the cell is idle");
      // MT14 C6: the 75% target is cutting utilisation, the CNC's in-cycle time over the whole cell window.
      // Spindle-on use also counts run-up and coast-down, so it is reported but not the target.
      var cuttingSeconds = 0.0;
      for (seconds in cycleTimes) cuttingSeconds += seconds;
      var cuttingUse = cuttingSeconds / productionSeconds;
      Sys.println('tending production: ${cnc.completedStocks.length} blanks, first clamp to fourth outfeed ' +
        '${productionSeconds} s, cutting utilisation ${Math.round(cuttingUse * 1000) / 10}%, ' +
        'spindle-on use ${Math.round(cellSpindleUse * 1000) / 10}%, ' +
        'spindle active within cuts ${Math.round(cnc.spindleUtilization * 1000) / 10}%, cycle intervals ${cycleTimes.join(", ")} s');
      check(cuttingUse >= 0.75, 'MT12 needs at least 75% cutting utilisation, got ${cuttingUse * 100}%');
      check(productionSeconds <= TENDING_FOURTH_OUTFEED_SECONDS * 1.05,
        'four parts reach the outfeed within 5% of the measured ${TENDING_FOURTH_OUTFEED_SECONDS} s, took $productionSeconds s');
    } catch (error:Dynamic) {
      var cleanupError:Null<String> = null;
      try simulation.dispose() catch (disposeError:Dynamic) cleanupError = Std.string(disposeError);
      try session.dispose() catch (disposeError:Dynamic) {
        var sessionError = Std.string(disposeError);
        cleanupError = cleanupError == null ? sessionError : cleanupError + "; " + sessionError;
      }
      if (cleanupError != null) throw '$error (cleanup also failed: $cleanupError)';
      throw error;
    }
    simulation.dispose(); session.dispose();
  }

  /** Nominal material removed by the checked bearing-block toolpath, in cubic metres. */
  static function expectedBearingBlockRemoval():Float
    return (Math.PI * (121 * 7 + 2 * 25 * 5) + (144 - (4 - Math.PI) * 12.25) * 3) / 1e9;

  /** Run the assembled cylinders' real pressure and flow limits in MuJoCo. */
  static function checkPneumaticDynamics(model:RobotModel, doorActuator:Actuator, viseActuator:Actuator,
      physicsSubsteps:Int):Void {
    var doorMotor = doorActuator.drive, viseMotor = viseActuator.drive;
    var doorDrive = doorMotor == null ? null : doorMotor.pneumatic();
    var viseDrive = viseMotor == null ? null : viseMotor.pneumatic();
    if (doorDrive == null || viseDrive == null)
      throw "Enclosed mill needs typed double-solenoid door and vise cylinder drives";
    var doorChannelB = doorDrive.channelB;
    if (doorChannelB == null) throw "Enclosed mill door needs a second valve coil";
    var fixture = new RobotModel("enclosed mill pneumatic stroke");
    // This pressure/flow fixture has no collision geometry. Default fallback
    // boxes would put its independent door and vise sliders inside each other.
    fixture.collisionApproximation = robotkit.model.CollisionApproximation.None;
    var base = fixture.addLink(new Link("base"));
    var doorLink = fixture.addLink(new Link("door"));
    var viseLink = fixture.addLink(new Link("vise jaw"));
    var doorJointId = switch doorActuator.transmission {case SimpleTransmission(id, _, _): id;};
    var viseJointId = switch viseActuator.transmission {case SimpleTransmission(id, _, _): id;};
    var doorCompliance = 0.0, viseCompliance = 0.0;
    for (joint in model.joints) {
      if (joint.id == doorJointId) doorCompliance = joint.limits.overtravel;
      if (joint.id == viseJointId) viseCompliance = joint.limits.overtravel;
    }
    check(doorCompliance > 0.0 && viseCompliance > 0.0,
      "pneumatic end cushion compression comes from cylinder parts");
    var doorJoint = fixture.addJoint(new Joint("door", JointType.Prismatic, base, doorLink, doorJointId));
    doorJoint.axis = [0.0, 1.0, 0.0];
    doorJoint.limits = doorDrive.extendSign > 0.0 ?
      new JointLimits(0.0, doorDrive.stroke) : new JointLimits(-doorDrive.stroke, 0.0);
    doorJoint.limits.overtravel = doorCompliance;
    doorJoint.limitTimeConstant = doorDrive.limitTimeConstant(doorLink.mass, doorCompliance);
    doorJoint.limitDampingRatio = 1.0;
    var viseJoint = fixture.addJoint(new Joint("vise jaw", JointType.Prismatic, base, viseLink, viseJointId));
    viseJoint.axis = [1.0, 0.0, 0.0];
    // The preset blank stops the jaw before the cylinder's full empty stroke.
    var gripTravel = new machinekit.milling.PneumaticVise().grippedPosition * 0.001;
    viseJoint.limits = viseDrive.extendSign > 0.0 ?
      new JointLimits(0.0, gripTravel) : new JointLimits(-gripTravel, 0.0);
    viseJoint.limits.overtravel = viseCompliance;
    viseJoint.limitTimeConstant = viseDrive.limitTimeConstant(viseLink.mass, viseCompliance);
    viseJoint.limitDampingRatio = 1.0;
    fixture.addActuator(doorActuator);
    fixture.addActuator(viseActuator);
    var clampSwitch = [for (sensor in model.sensors) if (sensor.id == "vise-clamped") sensor];
    // The assembly bridge centres joint coordinates at the placed jaw (the
    // seated blank). Its switch must be true there and false at empty closure.
    var emptyTravel = viseDrive.stroke - gripTravel;
    check(clampSwitch.length == 1 && clampSwitch[0].windowLower < 0.0 &&
      0.0 < clampSwitch[0].windowUpper && clampSwitch[0].windowUpper < emptyTravel,
      "clamped switch sees the seated blank and excludes the empty full stroke");
    var fixtureSwitch = new robotkit.model.Sensor(clampSwitch[0].name,
      clampSwitch[0].kind, clampSwitch[0].updateRate, clampSwitch[0].id);
    fixtureSwitch.joint = clampSwitch[0].joint;
    fixtureSwitch.windowLower = clampSwitch[0].windowLower + gripTravel;
    fixtureSwitch.windowUpper = clampSwitch[0].windowUpper + gripTravel;
    fixtureSwitch.hysteresis = clampSwitch[0].hysteresis;
    fixture.addSensor(fixtureSwitch);
    var blueprint = RobotRuntimeCompiler.compile(fixture, new robotkit.profile.RobotProfile());
    var expectedDoorSeconds = doorDrive.stroke * 0.98 / doorDrive.ratedSpeed;
    var expectedDoorCloseSeconds = expectedDoorSeconds *
      doorDrive.extensionForce() / doorDrive.retractionForce();
    // MT10 widened the door. Derive both coil windows from this cylinder's
    // stroke and flow instead of reversing the older door at a fixed 1.6 s.
    var reverseTick = Std.int(Math.ceil(1.15 * doorDrive.stroke / doorDrive.ratedSpeed / 0.01)) + 2;
    var finalTick = reverseTick + Std.int(Math.ceil(1.15 * expectedDoorCloseSeconds / 0.01)) + 10;
    var physical = new SimulationHarness(0.01, physicsSubsteps, SimulationSpace.MUJOCO);
    try {
      var runtime = physical.simulation.addRobot(blueprint);
      var stroke = new TrajectorySegment(Int64.ofInt(0), Int64.fromFloat(finalTick * 10000000.0),
        [[0.0, 0.0], [0.0, 0.0]]);
      var events = [
        new ProcessTimedEvent(Int64.ofInt(10000000), doorDrive.channelA, ProcessEventValue.Digital(true)),
        new ProcessTimedEvent(Int64.ofInt(10000000), viseDrive.channelA, ProcessEventValue.Digital(true)),
        new ProcessTimedEvent(Int64.fromFloat(reverseTick * 10000000.0), doorDrive.channelA, ProcessEventValue.Digital(false)),
        new ProcessTimedEvent(Int64.fromFloat(reverseTick * 10000000.0), doorChannelB, ProcessEventValue.Digital(true))
      ];
      runtime.submitPlan(new ExecutionPlanSubmission(Int64.ofInt(1202), Int64.ofInt(blueprint.revision),
        Int64.ofInt(blueprint.calibrationRevision), RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE,
        [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [stroke], null, null, null, null, null, true, events), 1);
      var doorOpenStart = -1, doorOpenReached = -1;
      var doorCloseStart = -1, doorCloseReached = -1;
      var viseClampStart = -1, viseClampReached = -1;
      var viseClampEffort = 0.0;
      var lastDoorPosition = 0.0;
      var peakDoorPosition = 0.0, peakDoorEffort = 0.0;
      for (tick in 1...finalTick + 1) {
        physical.step(Int64.fromFloat(tick * 10000000.0));
        var snapshot = runtime.snapshot();
        if (snapshot.safety != RobotKitRuntimeConstants.RK_SAFETY_READY &&
            snapshot.safety != RobotKitRuntimeConstants.RK_SAFETY_STOPPING)
          throw 'Pneumatic runtime fault at tick $tick: safety=${snapshot.safety}, fault=${snapshot.faultCode}, q=${snapshot.q.get(0)},${snapshot.q.get(1)}, effort=${snapshot.effort.get(0)},${snapshot.effort.get(1)}';
        var doorPosition = snapshot.q.get(0) * doorDrive.extendSign;
        var visePosition = snapshot.q.get(1) * viseDrive.extendSign;
        lastDoorPosition = doorPosition;
        peakDoorPosition = Math.max(peakDoorPosition, doorPosition);
        peakDoorEffort = Math.max(peakDoorEffort, Math.abs(snapshot.effort.get(0)));
        var doorA = switch runtime.channelValue(doorDrive.channelA) {case Digital(value): value; case _: false;};
        var doorB = switch runtime.channelValue(doorChannelB) {case Digital(value): value; case _: false;};
        var viseA = switch runtime.channelValue(viseDrive.channelA) {case Digital(value): value; case _: false;};
        if (doorOpenStart < 0 && doorA) doorOpenStart = tick;
        if (doorOpenStart >= 0 && doorOpenReached < 0 && doorPosition >= doorDrive.stroke * 0.98)
          doorOpenReached = tick;
        if (doorCloseStart < 0 && doorB) doorCloseStart = tick;
        if (doorCloseStart >= 0 && doorCloseReached < 0 && doorPosition <= doorDrive.stroke * 0.02)
          doorCloseReached = tick;
        if (viseClampStart < 0 && viseA) viseClampStart = tick;
        if (viseClampStart >= 0 && viseClampReached < 0 && visePosition >= gripTravel * 0.98) {
          viseClampReached = tick;
          viseClampEffort = Math.abs(snapshot.effort.get(1));
        }
      }
      var doorOpenSeconds = (doorOpenReached - doorOpenStart) * 0.01;
      var doorCloseSeconds = (doorCloseReached - doorCloseStart) * 0.01;
      var viseClampSeconds = (viseClampReached - viseClampStart) * 0.01;
      var expectedViseSeconds = gripTravel * 0.98 / viseDrive.ratedSpeed;
      check(doorOpenStart > 0 && doorOpenReached > doorOpenStart &&
        Math.abs(doorOpenSeconds / expectedDoorSeconds - 1.0) <= 0.1,
        'MuJoCo door extension takes stroke / rated speed within 10%, measured $doorOpenSeconds s; start=$doorOpenStart reached=$doorOpenReached position=$lastDoorPosition peak=$peakDoorPosition effort=$peakDoorEffort stroke=${doorDrive.stroke} speed=${doorDrive.ratedSpeed} sign=${doorDrive.extendSign} drives=${blueprint.cylinderDrives.length}, trajectory=${runtime.snapshot().trajectoryTimeNs}, active=${runtime.snapshot().trajectoryActive}, safety=${runtime.snapshot().safety}');
      check(doorCloseStart > 0 && doorCloseReached > doorCloseStart &&
        Math.abs(doorCloseSeconds / expectedDoorCloseSeconds - 1.0) <= 0.1,
        'MuJoCo door retraction takes stroke / rated speed within 10%, measured $doorCloseSeconds s');
      check(viseClampStart > 0 && viseClampReached > viseClampStart &&
        Math.abs(viseClampSeconds - expectedViseSeconds) <= 0.03,
        'MuJoCo seated-blank clamp takes grip travel / rated speed within three controller ticks, measured $viseClampSeconds s');
      check(Math.abs(viseClampEffort - viseDrive.extensionForce()) <= viseDrive.extensionForce() * 0.05,
        'MuJoCo vise joint reports clamp force within 5% of p·A, measured $viseClampEffort N');
      var clampFrames = [for (frame in runtime.snapshot().sensors.toArray())
        if (frame.sensorId == clampSwitch[0].id) frame];
      check(clampFrames.length == 1 && clampFrames[0].values.get(0) == 1.0,
        "clamped switch closes against the seated-blank stop");
      Sys.println('pneumatic mill: door open $doorOpenSeconds s, close $doorCloseSeconds s, ' +
        'vise clamp against seated blank $viseClampSeconds s at $viseClampEffort N');
    } catch (error:Dynamic) {
      physical.dispose();
      throw error;
    }
    physical.dispose();
  }

  /** Test deployment wiring is explicit; no physical bench pins are inferred here. */
  static function routerDeviceBinding(model:RobotModel, stepTickHz:Int):DeviceBinding {
    var inputs:Array<robotkit.device.DeviceInput> = [];
    for (contact in model.switches) {
      var matches:Array<robotkit.model.Actuator> = [];
      for (actuator in model.actuators) switch actuator.transmission {
        case SimpleTransmission(shaft, _, _): if (shaft == contact.driveJoint) matches.push(actuator);
      }
      check(matches.length == 1, "Router switch has exactly one physical motor side: " + contact.id);
      inputs.push(new robotkit.device.DeviceInput(inputs.length, contact.id, matches[0].id, false));
    }
    var channels = DeviceLayout.forActuators(model).channels;
    return DeviceBinding.bind(model, new DeviceLayout(channels, inputs), stepTickHz);
  }

  /** Physical screw router homes using RKD6 switch captures, never host switch synthesis. */
  static function checkVirtualRouterHoming(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/cnc-router/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition:AssemblyDefinition = cast generated.assemblyDefinition;
    var model = AssemblySimulationBridge.toRobotModel(definition, generated.physical).model;
    var homes = [for (contact in model.switches) if (contact.role == "home") contact];
    check(homes.length == 4, "Physical router declares X, Z and two independent Y home switches");
    var binding = routerDeviceBinding(model, 40000);
    var axes = [for (joint in binding.model.joints) if (joint.type == JointType.Prismatic) {
      var limits = binding.model.coupledLimits(joint.id, new SteadyLoads());
      new motionkit.axis.MotionAxisBlueprint(joint.id, [joint.id], joint.limits.lower,
        joint.limits.upper, limits.requireVelocity(), limits.requireAcceleration(),
        joint.id == "z" ? joint.limits.upper : joint.limits.lower);
    }];
    var blueprint = motionkit.robot.MotionSystemBlueprint.fromRobotModel(binding.model, axes);
    var options = new robotkit.runtime.VirtualDeviceOptions();
    // Four motors' short home ramps require more than the default 921600-baud link.
    // This is the virtual deployment's rate, not a verified bench UART assignment.
    options.baud = 2000000;
    options.actuators = binding.virtualActuators(); options.inputs = binding.virtualInputs();
    var harness = new SimulationHarness(0.01);
    try {
      var runtime = harness.simulation.addRobot(blueprint.runtime, null, options);
      for (_ in 0...30) harness.step();
      var robot = new robotkit.simulation.SimulatedRobot("virtual-router", runtime, binding.model.name,
        [for (link in binding.model.links) link.name], [for (joint in binding.model.joints) joint.name]);
      var sides = robotkit.device.DeviceHomingSides.install(runtime, blueprint.runtime, binding, 0.01);
      var motion = new motionkit.robot.MotionSystem(robot, blueprint);
      motion.configureRuntimeHoming(runtime, () -> {}, sides);
      motion.home();
      var ticks = 0;
      while (motion.homingStatus() != "Complete" && ticks++ < 60000) {
        harness.step();
        motion.update(0.01);
      }
      check(motion.homingStatus() == "Complete", "Router homes through virtual RKD6: " + motion.homingStatus());
      var snapshot = runtime.snapshot();
      for (index in 0...binding.model.joints.length) if (binding.model.joints[index].type == JointType.Prismatic) {
        check(runtime.isReferenced(index), "Device-homed router axis is referenced");
        check(Math.abs(snapshot.q.get(index) - (binding.model.joints[index].id == "z"
          ? binding.model.joints[index].limits.upper : binding.model.joints[index].limits.lower)) < 0.00005,
          "Device-homed router returns to its reference within 50 micrometres");
      }
      harness.dispose();
    } catch (error:Dynamic) { harness.dispose(); throw error; }
  }

  static function checkCncRouter(root:String, belts:Bool = false):Void {
    var kind = belts ? "belt router" : "screw router";
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/cnc-router/" + (belts ? "belts/" : "") + "materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition:AssemblyDefinition = cast(generated.assemblyDefinition, AssemblyDefinition);
    var model = AssemblySimulationBridge.toRobotModel(definition, generated.physical).model;
    var axes = [for (joint in model.joints) if (joint.type == JointType.Prismatic) joint];
    check([for (joint in axes) Std.string(joint.id)].join(",") == "y,x,z", 'the $kind simulates axes y, x and z');
    var leads = [for (coupling in model.couplings) '${coupling.leader}:${Math.round(Math.abs(coupling.ratio))}'];
    leads.sort(Reflect.compare);
    if (belts) {
      // Pulley and idler on x, a pair on each Y belt, and the Z screw; 1 / 6.366 mm is 157 rad per metre.
      check(leads.join(",") == "x:157,x:157,y:157,y:157,y:157,y:157,z:3142", 'belt axes turn by their pulleys, got $leads');
    } else {
      // Four rigid bodies and four lead screws, each screw turning with its coupling on the motor shaft.
      check(model.joints.length == 7 && model.links.length == 8,
        'the router simulates as four rigid bodies, four screws, three axes and four screw joints, got ' +
        '${model.links.length} links and ${model.joints.length} joints');
      // A 2 mm lead turns its screw pi radians per millimetre: 3142 per metre.
      check(leads.join(",") == "x:3142,y:3142,y:3142,z:3142", 'each axis turns its screws by their lead, got $leads');
    }
    for (joint in axes) {
      var travel = joint.limits.upper - joint.limits.lower;
      check(Math.abs(travel - (Std.string(joint.id) == "z" ? 0.08 : 0.3)) < 1e-9,
        '$kind axis ${joint.id} travel is in metres, got $travel');
      check(joint.limits.overtravel > 0, '$kind axis ${joint.id} carries its overtravel');
    }
    var job = generated.cncJob;
    var controller = job == null ? null : job.controller;
    if (controller == null) throw "the router names the controller its steppers are wired to";
    // Each axis respects all its installed motor/driver channels and screw limits.
    // Steady loads may tighten those ceilings; deployed integer pulse periods
    // are authoritative, rather than re-quantizing rounded continuous rates.
    var binding = routerDeviceBinding(model, controller.stepTickHz);
    var wired = binding.model;
    var steady = new SteadyLoads();
    var derived:Array<String> = [], free:Array<String> = [];
    for (joint in axes) {
      var motorRatio = 0.0, critical = Math.POSITIVE_INFINITY, leadRatio = 0.0;
      for (coupling in model.couplings) if (coupling.leader == joint.id) {
        for (actuator in model.actuators) switch actuator.transmission {
          case SimpleTransmission(target, _, _): if (target == coupling.follower) motorRatio = Math.abs(coupling.ratio);
        }
        for (follower in model.joints) if (follower.id == coupling.follower) {
          var mechanical = follower.mechanicalLimits;
          if (mechanical != null && mechanical.velocity != null) {
            critical = Math.min(critical, mechanical.requireVelocity() / Math.abs(coupling.ratio));
            leadRatio = Math.abs(coupling.ratio);
          }
        }
      }
      check(motorRatio > 0, '$kind axis ${joint.id} has a motor');
      var ceiling = critical, channels = 0;
      for (channel in binding.channels) if (wired.joints[channel.jointIndex].id == joint.id) {
        var pulseRate = binding.stepTickHz / (channel.minStepTicks * channel.stepsPerUnit);
        ceiling = Math.min(ceiling, Math.min(channel.maxRate, pulseRate) / Math.abs(channel.ratio));
        channels++;
      }
      check(channels > 0, '$kind axis ${joint.id} has its own deployed pulse channels');
      var limits = wired.coupledLimits(joint.id, steady);
      check(limits.requireVelocity() > 0 && limits.requireVelocity() <= ceiling + 1e-9,
        '$kind axis ${joint.id} respects its motor, controller and screw ceilings: ${limits.requireVelocity()} m/s, ceiling $ceiling');
      check(limits.requireVelocity() <= wired.coupledLimits(joint.id).requireVelocity() + 1e-9,
        '$kind axis ${joint.id} steady loads cannot raise its unloaded speed limit');
      check(limits.requireAcceleration() > 0.5 && limits.requireAcceleration() < 50,
        '$kind axis ${joint.id} accelerates as its motors move it: ${limits.requireAcceleration()} m/s²');
      derived.push('${joint.id} ${Math.round(limits.requireVelocity() * 1e4) / 10} mm/s, ${Math.round(limits.requireAcceleration() * 100) / 100} m/s²' +
        (critical < Math.POSITIVE_INFINITY ? ' (screw held to ${Math.round(critical * (Math.abs(leadRatio) > 0 ? leadRatio : 1) * 60 / (2 * Math.PI))} rpm)' : ''));
      var bare = model.coupledLimits(joint.id);
      free.push('${joint.id} ${Math.round(bare.requireVelocity() * 1e4) / 10} mm/s, ${Math.round(bare.requireAcceleration() * 100) / 100} m/s²');
    }
    Sys.println('cnc $kind axes with the controller and steady loads: ${derived.join("; ")}');
    Sys.println('cnc $kind axes from their motors and screws alone: ${free.join("; ")}');
	check(job != null && job.loop && job.stocks.length == 1 && job.stocks[0] == "stock" && job.spindle == "spindle" && job.target != null &&
      job.loadedTool == 1 && [for (tool in job.tools) tool.number].join(",") == "1,2",
      "the router generates a looping job that machines its stock to a target part with an end mill and a drill");
    if (job != null) {
      var originalScope = job.channelScope;
      job.channelScope = "other-cabinet/process";
      check(app.CncProgramPlayer.channelsFor(job).spindleSpeed == "other-cabinet/process/spindle.speed",
        "CNC process scope is explicit and independent of the spindle part path");
      job.channelScope = originalScope;
    }
    var session = new ProjectDocumentSession(null, false);
    var simulation = new ApplicationSimulation(new RobotWorld());
    session.openGeneratedScene(generated.objects, manifest,
      generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
      generated.localCentersByDefinition, generated.metresPerUnit, generated.physical,
      generated.recipeDocument, generated.robotMotions, null, generated.cncJob);
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    check(simulation.rebuild(session.sensors, session.scene, session),
      "the router builds in the shared simulation: " + simulation.error);
    var player = simulation.cncPlayer();
    if (player == null) throw "the router has no CNC player";
    function toolPosition():Array<Float> {
      var tool = [for (pose in simulation.capturePresentationSnapshot().environment) if (pose.id == "project:tool") pose];
      check(tool.length == 1, "the router publishes its tool pose");
      return tool[0].position;
    }
    function partRotation(id:String):Array<Float> {
      var part = [for (pose in simulation.capturePresentationSnapshot().environment) if (pose.id == "project:" + id) pose];
      check(part.length == 1, 'the router publishes the pose of $id');
      return part[0].rotation;
    }
    simulation.step();
    var start = toolPosition();
    // The Z screw (and the X screw of the screw router) turns half a turn for every millimetre of its axis.
    var screwParts = belts ? ["", "screwZCoupling"] : ["screwXCoupling", "screwZCoupling"];
    var screwStart = [for (id in screwParts) id == "" ? [] : partRotation(id)];
    var lastHomingSeconds = player.homingSeconds, machiningReference = false;
    var lowest = 0.0, steps = 0, stepping = 0.0, tools:Array<Int> = [player.loadedTool];
    // Allocation is counted, not timed, so it holds whatever else the machine is doing.
    var allocatedBefore = hl.Gc.totalAllocated(), collectionsBefore = hl.Gc.collections();
    while (player.passes == 0 && simulation.activeSession().simulationTime() < 600.0 && steps++ < 1000000) {
      var before = Sys.time();
      simulation.step();
      stepping += Sys.time() - before;
      check(simulation.cncFailure() == null, 'the router program runs: ${simulation.cncFailure()}');
      if (machiningReference && steps % 10 == 0) lowest = Math.min(lowest, toolPosition()[2] - start[2]);
      if (player.loadedTool != tools[tools.length - 1]) tools.push(player.loadedTool);
      if (!machiningReference && lastHomingSeconds > 0 && player.homingSeconds == lastHomingSeconds) {
        // Homing establishes coordinate and drive zeros independently. Compare
        // physical screw travel from the completed homing reference.
        start = toolPosition();
        screwStart = [for (id in screwParts) id == "" ? [] : partRotation(id)];
        machiningReference = true;
        lowest = 0.0;
      } else if (machiningReference && steps % 1000 == 0) {
        // The X and Z screws turn half a turn for every millimetre their axes move.
        var now = toolPosition();
        for (axis in [0, 2]) {
          if (screwParts[axis == 0 ? 0 : 1] == "") continue;
          var before = screwStart[axis == 0 ? 0 : 1], after = partRotation(screwParts[axis == 0 ? 0 : 1]);
          var dot = Math.abs(before[0] * after[0] + before[1] * after[1] + before[2] * after[2] + before[3] * after[3]);
          var turned = Math.PI * 1000 * (now[axis] - start[axis]);
          check(Math.abs(dot - Math.abs(Math.cos(turned / 2))) < 0.02,
            'the ${axis == 0 ? "X" : "Z"} screw turns with its axis: ${2 * Math.acos(Math.min(1.0, dot))} rad for $turned');
        }
      }
      lastHomingSeconds = player.homingSeconds;
    }
    check(player.passes == 1,
      'the $kind finishes its program: total=${simulation.activeSession().simulationTime()} s, homing=${player.homingSeconds} s, line=${player.currentLine}, homingStatus=${@:privateAccess player.homing.homingStatus()}');
    var allocatedPerTick = (hl.Gc.totalAllocated() - allocatedBefore) / steps;
    var collections = hl.Gc.collections() - collectionsBefore;
    // About 46 KB a tick when measured (2026-10-02, sensor values pooled per snapshot): mostly robot snapshots, then the stock's cut moves.
    check(allocatedPerTick < 80000, 'the router allocates under 80 KB a simulated tick, got ${Math.round(allocatedPerTick)} bytes');
    // The pass ends with the drill; the next pass, started as this one is counted, loads the end mill again.
    var changes = tools.join(",");
    check(changes == "1,2" || changes == "1,2,1", 'the router starts with the end mill and changes to the drill, got $tools');
    var totalSeconds = simulation.activeSession().simulationTime();
    var homingSeconds = player.homingSeconds;
    var seconds = totalSeconds - homingSeconds;
    check(homingSeconds > 0 && seconds > 0, 'the $kind reports homing separately from machining');
    check(player.passes == 1, 'the router finishes one pass of its program, at $seconds s');
    // What the plan checks found over the pass: stepper stalls and the drives' stretch against the tolerance.
    var checks = player.planChecks();
    var stalls = checks.count(PlanDiagnosticKind.StepperStall), inaccurate = checks.count(PlanDiagnosticKind.Accuracy);
    var checkedPlans = checks.plans, flaggedPlans = checks.flagged, worstRatio = checks.worstTorqueRatio;
    var worstMotor = checks.worstMotor, worstDeviation = checks.worstDeviation, worstAxis = checks.worstAxis;
    var worstFinding = [for (diagnostic in checks.diagnostics) if (diagnostic.kind == PlanDiagnosticKind.Accuracy) diagnostic];
    worstFinding.sort((a, b) -> a.value < b.value ? 1 : a.value > b.value ? -1 : 0);
    var accuracyExample = worstFinding.length == 0 ? "" : player.describe(worstFinding[0]);
    check(checkedPlans > 0, "every plan the compiler makes is checked");
    var stallExamples = [for (diagnostic in checks.diagnostics) if (diagnostic.kind == PlanDiagnosticKind.StepperStall) player.describe(diagnostic)];
    // From 54 mm above the stock the 40 mm drill, 10 mm longer than the end mill, goes through the
    // 20 mm plate and its 1.65 mm point and 0.5 mm more into the spoilboard.
    check(Math.abs(lowest + 0.06615) < 0.0005, 'the drill goes through the plate, lowest $lowest m');
    var stock = simulation.machiningStock();
    if (stock == null) throw "the router cuts no stock";
    // The plate's recesses: a 38.3 mm pilot recess 6 mm deep, four 10 mm counterbores 5.4 mm deep, and
    // under them four 5.5 mm clearance holes through the rest of the 20 mm plate.
    var recesses = Math.PI * (0.01915 * 0.01915 * 0.006 + 4 * 0.005 * 0.005 * 0.0054 +
      4 * 0.00275 * 0.00275 * (0.020 - 0.0054));
    check(Math.abs(stock.removed - recesses) < recesses * 0.02,
      'the stock loses the plate\'s recesses, ${stock.removed} m³ removed against $recesses');
    // Both routers settle their measured axes before advancing an exact-stop move, so a rapid
    // may not remove stock even when a belt carriage lags during its preceding move.
    check(stock.rapidContacts == 0 && stock.collisions == 0,
      'no rapid runs through the stock and the holder never touches it (${stock.rapidContacts}, ${stock.collisions})');
    var deviation = stock.deviation();
    // Homing has finite repeatability. Bound missing target volume by the
    // machined wall/floor area swept through the stated positional tolerance.
    // A raw ray-stock volume comparison includes even sub-tolerance offsets.
    var wallArea = Math.PI * (0.0383 * 0.006 + 4 * 0.010 * 0.0054 + 4 * 0.0055 * (0.020 - 0.0054));
    var floorArea = Math.PI * (0.01915 * 0.01915 + 4 * (0.005 * 0.005 - 0.00275 * 0.00275));
    var gougeAllowance = (wallArea + floorArea) * MachiningStock.TOLERANCE;
    check(deviation.gouge < gougeAllowance + 1e-9,
      'the homed plate stays within its machining tolerance: gouge ${deviation.gouge} m³, surface-volume bound $gougeAllowance m³');
    check(deviation.leftover < recesses * 0.02,
      'only slivers of stock are left on the plate, leftover ${deviation.leftover} m³');
    check(stock.geometry().triangleCount() > 12, "the machined stock meshes");
    // The program ends away from where it started; the next pass runs from there.
    var secondPass = simulation.activeSession().simulationTime() + 5.0;
    while (simulation.activeSession().simulationTime() < secondPass && steps++ < 1000000) {
      simulation.step();
      check(simulation.cncFailure() == null, 'the looping program starts its next pass: ${simulation.cncFailure()}');
    }
    session.dispose();
    Sys.println('cnc $kind milled the motor plate in ${Math.round(seconds * 10) / 10} s of machining (homing ${Math.round(homingSeconds * 10) / 10} s, total ${Math.round(totalSeconds * 10) / 10} s): removed ' +
      '${Math.round(stock.removed * 1e10) / 10} mm³ of ${Math.round(recesses * 1e10) / 10}, leftover ' +
      '${Math.round(deviation.leftover * 1e10) / 10} mm³, gouge ${Math.round(deviation.gouge * 1e10) / 10} mm³; ' +
      '${Math.round(stepping / steps * 1e5) / 100} ms per simulated tick; ${stock.rapidContacts} ticks of rapid label cut');
    Sys.println('cnc $kind plan check: $checkedPlans plans, $flaggedPlans flagged, $stalls stepper stalls, $inaccurate over the ' +
      '${Math.round(player.checkOptions.tolerance * 1e6) / 1000} mm tolerance; worst torque ${Math.round(worstRatio * 1000) / 10}% of what ' +
      '$worstMotor can give; worst deviation ${Math.round(worstDeviation * 1e5) / 100} mm on $worstAxis' +
      (accuracyExample == "" ? "" : "; e.g. " + accuracyExample));
    Sys.println('cnc $kind per tick: motion ${Math.round(player.motionSeconds / steps * 1e5) / 100} ms, cutting ' +
      '${Math.round(player.cuttingSeconds / steps * 1e5) / 100} ms, meshing ${Math.round(player.meshingSeconds / steps * 1e5) / 100} ms; ' +
      'compile ${Math.round(player.runSeconds * 1000)} ms, slowest update ${Math.round(player.slowestUpdate * 1000)} ms; ' +
      '${Math.round(allocatedPerTick / 100) / 10} KB allocated a tick, $collections collections');
    check(stalls == 0, 'the planner\'s limits keep every stepper under its pull-out curve, got $stalls findings, e.g. ' +
      stallExamples.slice(0, 3).join("; "));
    if (belts) {
      Sys.println('cnc belt router against the screw router: ${Math.round(seconds * 10) / 10} s against ${Math.round(screwRouterSeconds * 10) / 10} s, ' +
        'worst drive deviation ${Math.round(worstDeviation * 1e5) / 100} mm against ${Math.round(screwRouterDeviation * 1e5) / 100} mm, ' +
        '$inaccurate plans over the tolerance against none');
      check(worstDeviation > 5 * screwRouterDeviation && inaccurate > 0,
        "belts stretch much further than screws turn loose: the belt router has plans over the tolerance, the screw router none");
    } else {
      screwRouterSeconds = seconds;
      screwRouterDeviation = worstDeviation;
      check(inaccurate == 0, 'the screw router stays within its tolerance: $inaccurate plans over, e.g. $accuracyExample');
    }
  }

  static function requiredAssemblyDefinition(value:Null<AssemblyDefinition>):AssemblyDefinition {
    if (value == null) throw "Generated machine project has no assembly definition";
    return value;
  }

  /**
   * The belt-driven router machines the same plate: its X and Y limits come from their pulleys (a
   * 20-tooth GT2 pulley has a 6.366 mm pitch radius, so a belt axis moves 20 times as fast as a screw
   * for the same motor speed) until the controller's step rate caps them, and its belts stretch.
   */
  static function checkBeltRouter(root:String):Void checkCncRouter(root, true);

  /**
   * The router's job answers the operator: it reports the line it runs, stops on a feed hold and
   * carries on, takes a speed override, and restarts at the drilling with the drill loaded.
   */
  static function checkCncControls(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/cnc-router/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var session = new ProjectDocumentSession(null, false);
    var simulation = new ApplicationSimulation(new RobotWorld());
    session.openGeneratedScene(generated.objects, manifest,
      generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
      generated.localCentersByDefinition, generated.metresPerUnit, generated.physical,
      generated.recipeDocument, generated.robotMotions, null, generated.cncJob);
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    check(simulation.rebuild(session.sensors, session.scene, session), "the router builds: " + simulation.error);
    var player = simulation.cncPlayer();
    if (player == null) throw "the router has no CNC player";
    function run(seconds:Float, ?until:Void->Bool):Bool {
      var end = simulation.activeSession().simulationTime() + seconds;
      while (simulation.activeSession().simulationTime() < end) {
        simulation.step();
        check(simulation.cncFailure() == null, 'the router keeps running: ${simulation.cncFailure()}');
        if (until != null && until()) return true;
      }
      return until == null;
    }
    var lines = player.sourceLines();
    check(run(600.0, () -> @:privateAccess player.homingComplete), "the router completes physical homing before machining controls");
    check(run(20.0, () -> player.currentLine > 0), "the player reports the line it runs");
    check(StringTools.trim(lines[player.currentLine - 1]).length > 0, "the running line is a line of the program");
    // The CNC panel, laid out on its own and operated by pointer.
    var fonts = FontCollection.create();
    fonts.add("haxeon/packages/ui/vendor/harfbuzz/perf/fonts/Roboto-Regular.ttf");
    var theme = Theme.dark();
    var ui = new UiContext(null, fonts, theme);
    var panel = new app.editor.CncPanel();
    var frame = new LayoutFrame(480.0, 720.0);
    function submit():RenderNode return ui.submit(panel.build(simulation, theme.tokens), frame);
    function find(node:RenderNode, key:String):Null<RenderNode> {
      if (node.styleKey == key) return node;
      for (child in node.children) {
        var found = find(child, key);
        if (found != null) return found;
      }
      return null;
    }
    function press(target:Null<RenderNode>, label:String):Void {
      if (target == null) throw 'the CNC panel shows $label';
      var resolved = target.resolved;
      if (resolved == null) throw 'the CNC panel lays out $label';
      var bounds = resolved.clippedViewportBounds();
      check(bounds.width > 0 && bounds.height > 0, 'the CNC panel shows $label on screen');
      var x = bounds.x + bounds.width / 2, y = bounds.y + bounds.height / 2;
      ui.pointerDown(x, y, 0);
      ui.pointerUp(x, y, 0);
      submit();
    }
    press(find(submit(), "cnc-hold"), "its hold button");
    check(run(5.0, () -> player.held()), "the panel's hold stops the machine");
    var heldLine = player.currentLine;
    run(1.0);
    check(player.held() && player.currentLine == heldLine, "a held machine stays on its line");
    press(find(submit(), "cnc-resume"), "its resume button");
    player.setSpeedOverride(0.5);
    check(run(5.0, () -> !player.held()), "the panel's resume carries on");
    // Pick a later line of the listing and restart there from the panel.
    var rows:Array<RenderNode> = [];
    function collect(node:RenderNode):Void {
      var resolved = node.resolved;
      if (node.styleType == "text" && resolved != null && resolved.clippedViewportBounds().height > 0)
        rows.push(node);
      for (child in node.children) collect(child);
    }
    var list = find(submit(), "list-content");
    check(list != null, "the panel lists the program");
    collect(cast list);
    check(rows.length > 4, 'the listing shows its lines, ${rows.length}');
    press(rows[rows.length - 2], "a line of the program");
    var picked = panel.selectedLine;
    check(picked > heldLine, 'clicking a line picks it to restart at, line $picked');
    press(find(submit(), "cnc-restart"), "its restart button");
    check(run(30.0, () -> player.currentLine >= picked),
      'the panel restarts the program at the picked line, now line ${player.currentLine}, held=${player.held()}, failure=${player.failure}, sim=${simulation.cncFailure()}');
    // Restart where the drill starts work: the first motion after the drill is loaded.
    var drillChange = -1;
    for (index in 0...lines.length) if (lines[index].indexOf("T2 M6") >= 0) drillChange = index + 1;
    check(drillChange > 0, "the program loads the drill");
    check(player.restartFromLine(drillChange), "the drilling has a line to restart at");
    check(run(60.0, () -> player.loadedTool == 2 && player.currentLine > drillChange),
      'a restart at the drilling loads the drill and runs from there, line ${player.currentLine}, tool ${player.loadedTool}');
    fonts.dispose();
    session.dispose();
    Sys.println('cnc controls: held at line $heldLine, restarted at line $picked from the panel, ' +
      'and at line ${player.currentLine} with tool ${player.loadedTool}');
  }

  /** A project named at launch builds in the background: queued at once, opened by tick(). */
  /**
   * Mates authored in the editor over a generated assembly (plan C4.5b): a pin seated on a plate's top face
   * and in its bore through the project's face descriptors, undone and redone, saved and reopened, a
   * contradicting mate reported as a conflict, and a mate removed.
   */
  static function checkMates(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/app/tests/fixtures/pin-plate/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var descriptors = generated.faceDescriptorsByDefinition;
    check(descriptors != null && descriptors.exists("plate") && descriptors.exists("pin"), "the project describes its faces");
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedScene(generated.objects, manifest, generated.geometryBySnapshot,
      generated.assemblyDefinition, generated.assemblyState, generated.localCentersByDefinition, generated.metresPerUnit,
      generated.physical, generated.recipeDocument, generated.robotMotions, descriptors);
    check(session.assemblyMateStatus() == null, "a project without mates shows no mate status");
    var top = describedFace(descriptors, "plate", "plane", 1), bore = describedFace(descriptors, "plate", "axis", 0);
    var base = describedFace(descriptors, "pin", "plane", -1), side = describedFace(descriptors, "pin", "axis", 0);

    var seat = session.addAssemblyFaceMate(AssemblyMateKind.Planar, "project:plate", top, "project:pin", base);
    var seated = session.assemblyMateResult;
    check(seated != null && seated.converged && seated.report.degreesOfFreedom == 3,
      'a planar mate leaves the pin three freedoms: ${session.assemblyMateStatus()}');
    var shaft = session.addAssemblyFaceMate(AssemblyMateKind.Coaxial, "project:plate", bore, "project:pin", side);
    check(session.assemblyMateStatus() == "Mates: 1 degrees of freedom free (pin)", 'the pin may only turn: ${session.assemblyMateStatus()}');
    check(session.assemblyPartStillFree("project:pin") && !session.assemblyPartStillFree("project:plate"),
      "the pin is still free to move, the grounded plate is not");
    expectPin(session, 30, 20, 10, "the pin stands in the bore");

    check(session.document.undo(), "the coaxial mate undoes");
    check(session.assemblyMates.mates.length == 1 && session.assemblyMates.connectors.length == 2,
      "undo removes the mate and its face connectors");
    check(session.document.redo(), "and redoes");
    expectPin(session, 30, 20, 10, "redo seats the pin again");
    checkMateDrag(session, generated.metresPerUnit);

    var output = "/tmp/materia-pin-plate-mates-" + Sys.getPid() + ".materia.json";
    session.save(output);
    var reopened = new ProjectDocumentSession(null, false);
    reopened.open(output);
    check(reopened.assemblyMates.mates.length == 2, "the mates survive a save and reopen");
    check(reopened.assemblyMateStatus() == "Mates: 1 degrees of freedom free (pin)", 'and place the pin again: ${reopened.assemblyMateStatus()}');
    expectPin(reopened, 30, 20, 10, "the reopened pin stands in the bore");
    FileSystem.deleteFile(output);

    var lift = session.addAssemblyFaceMate(AssemblyMateKind.Planar, "project:plate", top, "project:pin", base, 5);
    var status = session.assemblyMateStatus();
    check(status != null && StringTools.startsWith(status, "Mates conflict"), 'a contradicting mate is a conflict: $status');
    expectPin(session, 30, 20, 10, "a conflicting mate leaves the placement");
    check(session.removeAssemblyMate(lift), "the conflicting mate can be removed");
    check(session.assemblyMateStatus() == "Mates: 1 degrees of freedom free (pin)", 'removing it settles the mates: ${session.assemblyMateStatus()}');
    check(seat != shaft, "mates get distinct ids");
    checkMatePick(manifest, generated);
    checkMateJoint(manifest, generated);
  }

  /**
   * Mates to a joint (plan C4.5e): a seated pin's planar and coaxial mates leave it one turn, which becomes a
   * revolute joint from the plate; the pin keeps its place and turns on the joint; the joint and its coordinate
   * survive a save and reopen; undo brings the mates back.
   */
  static function checkMateJoint(manifest:String, generated:app.GeneratedAssemblyScene):Void {
    var descriptors = generated.faceDescriptorsByDefinition;
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedScene(generated.objects, manifest, generated.geometryBySnapshot,
      generated.assemblyDefinition, generated.assemblyState, generated.localCentersByDefinition, generated.metresPerUnit,
      generated.physical, generated.recipeDocument, generated.robotMotions, descriptors);
    var top = describedFace(descriptors, "plate", "plane", 1), bore = describedFace(descriptors, "plate", "axis", 0);
    var base = describedFace(descriptors, "pin", "plane", -1), side = describedFace(descriptors, "pin", "axis", 0);
    session.addAssemblyFaceMate(AssemblyMateKind.Planar, "project:plate", top, "project:pin", base);
    var single = session.assemblyMateJoint("project:pin");
    check(single != null && single.type == null, "one planar mate is not a joint");
    session.addAssemblyFaceMate(AssemblyMateKind.Coaxial, "project:plate", bore, "project:pin", side);
    var inferred = session.assemblyMateJoint("project:pin");
    check(inferred != null && inferred.type == materia.assembly.AssemblyDefinition.AssemblyJointType.Revolute,
      'planar + coaxial make a revolute joint: ${inferred == null ? "none" : inferred.reason}');
    var generationBefore = session.generation;
    var jointId = session.convertMatesToJoint("project:pin");
    var definition = session.projectAssemblyDefinition;
    check(definition != null && [for (joint in definition.joints) if (joint.id == jointId && joint.parent == "plate" && joint.child == "pin") joint].length == 1,
      "the joint joins the plate and the pin");
    check(session.assemblyMates.mates.length == 0 && session.assemblyMates.joints.length == 1, "the joint replaces the mates");
    check(session.generation != generationBefore, "the assembly's structure changed");
    expectPin(session, 30, 20, 10, "the jointed pin keeps its place");
    check(session.setAssemblyJointCoordinate(jointId, 0.5), "the joint turns");
    expectPin(session, 30, 20, 10, "turning it keeps the pin in the bore");

    var output = "/tmp/materia-pin-plate-joint-" + Sys.getPid() + ".materia.json";
    session.save(output);
    var reopened = new ProjectDocumentSession(null, false);
    reopened.open(output);
    FileSystem.deleteFile(output);
    var reopenedDefinition = reopened.projectAssemblyDefinition, reopenedState = reopened.projectAssemblyState;
    if (reopenedDefinition == null || reopenedState == null) throw "the reopened project has no assembly";
    check([for (joint in reopenedDefinition.joints) if (joint.id == jointId) joint].length == 1, "the joint survives a save and reopen");
    var coordinate = [for (value in reopenedState.jointCoordinates) if (value.joint == jointId) value.value];
    check(coordinate.length == 1 && Math.abs(coordinate[0] - 0.5) < 1e-9, 'and so does its coordinate: $coordinate');

    check(session.document.undo() && session.document.undo(), "the turn and the conversion undo");
    check(session.assemblyMates.mates.length == 2 && session.assemblyMates.joints.length == 0, "undo brings the mates back");
    var restored = session.projectAssemblyDefinition;
    check(restored != null && [for (joint in restored.joints) if (joint.id == jointId) joint].length == 0, "and removes the joint");
    expectPin(session, 30, 20, 10, "the pin is where the mates put it");
  }

  /**
   * The two-pick mate tool (plan C4.5c): a face that cannot take the mate and a second pick on the same part
   * are refused without losing the first pick; the second face on another part adds the mate. The mate then
   * shows on the part's inspector, and clearing it removes the mate (undoably).
   */
  static function checkMatePick(manifest:String, generated:app.GeneratedAssemblyScene):Void {
    var descriptors = generated.faceDescriptorsByDefinition;
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedScene(generated.objects, manifest, generated.geometryBySnapshot,
      generated.assemblyDefinition, generated.assemblyState, generated.localCentersByDefinition, generated.metresPerUnit,
      generated.physical, generated.recipeDocument, generated.robotMotions, descriptors);
    check(session.canMateFaces(), "a project with described faces can be mated");
    var top = describedFace(descriptors, "plate", "plane", 1), side = describedFace(descriptors, "pin", "axis", 0);
    var base = describedFace(descriptors, "pin", "plane", -1), bottom = describedFace(descriptors, "plate", "plane", -1);
    var tool = new MatePickTool(session, AssemblyMateKind.Planar);
    tool.pick("project:pin", side);
    check(tool.message.indexOf("cannot use") >= 0, 'a cylinder is refused for a planar mate: ${tool.message}');
    tool.pick(null, -1);
    check(tool.message == "Pick a face of an assembly part", 'a click on nothing asks again: ${tool.message}');
    tool.pick("project:plate", top);
    tool.pick("project:plate", bottom);
    check(tool.message == "Pick a face of another part" && !tool.finished, 'a second face on the same part is refused: ${tool.message}');
    tool.pick("project:pin", base);
    var mateId = tool.mateId;
    check(tool.finished && mateId != null && session.assemblyMates.mates.length == 1,
      'the second face adds the mate: ${tool.message}');
    check(tool.message == "Mates: 3 degrees of freedom free (pin)", 'the tool reports the result: ${tool.message}');

    session.scene.select("project:pin");
    var row = [for (property in session.scene.properties()) if (property.id == "assembly-mate:" + mateId) property];
    check(row.length == 1, "the pin's inspector lists its mate");
    var result = new PropertyBinding(row[0], session.scene.context()).apply(PropertyValue.Bool(false));
    check(session.assemblyMates.mates.length == 0, 'clearing the mate removes it ($result)');
    check(session.document.undo() && session.assemblyMates.mates.length == 1, "and undo brings it back");
  }

  /**
   * Dragging a mated part (plan C4.5d): a point on the seated pin's side pulled around the bore turns the pin,
   * which stays in the bore; the drag commits as one undoable edit. A part without mates and joints still
   * cannot be dragged.
   */
  static function checkMateDrag(session:ProjectDocumentSession, metresPerUnit:Float):Void {
    var definition = session.projectAssemblyDefinition, record = session.projectAssemblyState;
    check(definition != null && record != null, "the mated assembly has a state");
    var pose = new AssemblyState(definition, record).worldPose("pin");
    var grab = materia.assembly.AssemblyFrames.transformPoint(pose, 5, 0, 15);
    var drag = session.beginAssemblyDrag("project:pin", [grab.x * metresPerUnit, grab.y * metresPerUnit, grab.z * metresPerUnit]);
    check(drag != null, "a mated pin can be dragged");
    var dx = grab.x - 30, dy = grab.y - 20, turn = 0.8;
    var target = [30 + dx * Math.cos(turn) - dy * Math.sin(turn), 20 + dx * Math.sin(turn) + dy * Math.cos(turn), grab.z];
    var steps = 4;
    for (step in 1...steps + 1) {
      var angle = turn * step / steps;
      drag.update((30 + dx * Math.cos(angle) - dy * Math.sin(angle)) * metresPerUnit,
        (20 + dx * Math.sin(angle) + dy * Math.cos(angle)) * metresPerUnit, grab.z * metresPerUnit);
    }
    check(drag.following(), 'pulled around the bore, the pin follows: ${drag.message()}');
    check(drag.commit(), "the drag commits");
    expectPin(session, 30, 20, 10, "the dragged pin stays in the bore");
    var turned = new AssemblyState(definition, session.projectAssemblyState).worldPose("pin");
    var moved = materia.assembly.AssemblyFrames.transformPoint(turned, 5, 0, 15);
    check(Math.abs(moved.x - target[0]) < 0.5 && Math.abs(moved.y - target[1]) < 0.5,
      'the grabbed point went round to the cursor: ${moved.x}, ${moved.y} for ${target[0]}, ${target[1]}');
    check(session.document.undo(), "the drag undoes");
    var back = new AssemblyState(definition, session.projectAssemblyState).worldPose("pin");
    check(Math.abs(back.qz - pose.qz) < 1e-9 && Math.abs(back.qw - pose.qw) < 1e-9, "undo turns the pin back");
    check(session.document.redo(), "and redoes");
    check(session.beginAssemblyDrag("project:plate", [0.03, 0.02, 0.01]) == null, "the grounded plate is not dragged");
  }

  /** The index of the described face of `feature` on `component`, for planes the one whose normal's z has `normalZ`'s sign. */
  static function describedFace(descriptors:Map<String, String>, component:String, feature:String, normalZ:Int):Int {
    var records:Array<Dynamic> = Json.parse(descriptors.get(component));
    for (record in records) {
      var direction:Array<Float> = Reflect.field(record, "direction");
      if (Reflect.field(record, "feature") == feature && (normalZ == 0 || direction[2] * normalZ > 0.5))
        return Reflect.field(record, "index");
    }
    throw 'No $feature face on $component';
  }

  static function expectPin(session:ProjectDocumentSession, x:Float, y:Float, z:Float, label:String):Void {
    var definition = session.projectAssemblyDefinition, record = session.projectAssemblyState;
    check(definition != null && record != null, label + ": the assembly has a state");
    var pose = new AssemblyState(definition, record).worldPose("pin");
    check(Math.abs(pose.x - x) < 1e-3 && Math.abs(pose.y - y) < 1e-3 && Math.abs(pose.z - z) < 1e-3,
      '$label: pin at ${pose.x}, ${pose.y}, ${pose.z}');
  }

  static function checkBackgroundLaunch(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-arm/materia.project.json");
    var editor = new ReferenceEditorApp();
    check(!editor.openingProject() && editor.scene.items().length == 0, "an editor starts with an empty scene");
    editor.openProjectInBackground(manifest);
    check(editor.openingProject() && editor.scene.items().length == 0,
      "a launch project is queued and nothing is built before the first frame");
    check(editor.workspace.isOpen("start"), "the Start page shows the build's progress");
    var deadline = Sys.time() + 600.0;
    while (editor.openingProject() && Sys.time() < deadline) {
      editor.tick();
      Sys.sleep(0.01);
    }
    check(!editor.openingProject(), "the launch project finishes building");
    check(editor.session.projectReference != null && editor.scene.items().length > 0,
      "tick() opens the launch project once its build ends");
    check(editor.session.missions != null, "the launch project brings its mission");
    check(!editor.workspace.isOpen("start"), "the Start page closes once the launch project opens");
    editor.dispose();
    var missing = new ReferenceEditorApp();
    missing.openProjectInBackground(FileSystem.fullPath(root) + "/no-such-project/materia.project.json");
    var failedBy = Sys.time() + 60.0;
    while (missing.openingProject() && Sys.time() < failedBy) {
      missing.tick();
      Sys.sleep(0.01);
    }
    check(!missing.openingProject() && missing.scene.items().length == 0,
      "a launch project that cannot build ends the wait and leaves the empty scene");
    check(missing.workspace.isOpen("start"), "a failed launch keeps the Start page open to explain");
    missing.dispose();
  }

  /** The press preview is a mechanical assembly whose coupled ram and gauges remain editable. */
  /** Unloaded flow reference with the backend's declared finite velocity response. */
  static function checkPressController(root:String):Void {
    PressBrakeSimulationChecks.run(root, false);
  }

  static function checkPressBrake(root:String):Void {
    var manifest = root + "/machinekit/examples/press-brake/materia.project.json";
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition = generated.assemblyDefinition;
    if (definition == null) throw "Press preview lost its assembly definition";
    check(generated.objects.length == definition.occurrences.length, "press preview includes every physical member");
    for (part in generated.objects) check(part.mass > 0, 'press part ${part.id} has material-derived mass');
    var session = new ProjectDocumentSession(null, false);
    try {
      session.openGeneratedScene(generated.objects, manifest, generated.geometryBySnapshot, definition,
        generated.assemblyState, generated.localCentersByDefinition, generated.metresPerUnit,
        generated.physical, generated.recipeDocument);
      var initial = new AssemblyState(definition, session.projectAssemblyState);
      var ramZ = initial.worldConnector("ram", "tool-mount").z;
      check(session.setAssemblyJointCoordinate("Y1", 25), "press ram can move through the editor");
      check(session.setAssemblyJointCoordinate("X", 125), "press X backgauge can move through the editor");
      check(session.setAssemblyJointCoordinate("R", 20), "press R backgauge can move through the editor");
      check(session.setAssemblyJointCoordinate("depth", 30), "press depth stop can move independently");
      var moved = new AssemblyState(definition, session.projectAssemblyState);
      check(Math.abs(moved.worldConnector("ram", "tool-mount").z - (ramZ - 25)) < 1e-6,
        "editor ram coordinate moves its installed tool stack");
      check(Math.abs(moved.worldPose("guideLeft").z - moved.worldPose("guideRight").z) < 1e-6,
        "editor keeps the torsion-bar ram guides together");
      var finger = moved.worldConnector("fingerLeft", "contact");
      check(Math.abs(finger.y - 125) < 1e-6 && Math.abs(finger.z - 850) < 1e-6,
        "editor X/R coordinates move the physical finger contact");
    } catch (error:Dynamic) { session.dispose(); throw error; }
    session.dispose();
    Sys.println("press brake project: coupled ram, independent depth stop and X/R editor axes passed");
    checkPressBrakePhysics(generated, manifest);
  }

  /** The generated CAD hulls, hydraulic circuit and passive contacts run at the normal app tick. */
  static function checkPressBrakePhysics(generated:app.GeneratedAssemblyScene, manifest:String):Void {
    var definition = generated.assemblyDefinition, physical = generated.physical;
    if (definition == null || physical == null) throw "Press simulation lost its assembly or physical data";
    var initial = new AssemblyState(definition, generated.assemblyState);
    var nominalContact = initial.worldConnector("fingerLeft", "contact");
    var stockHeight = nominalContact.z * generated.metresPerUnit;
    var gaugeFace = nominalContact.y * generated.metresPerUnit;
    var punchTip = Math.POSITIVE_INFINITY, dieTop = Math.NEGATIVE_INFINITY;
    for (occurrence in definition.occurrences) {
      if (occurrence.id.indexOf("-punch-") >= 0 && !StringTools.endsWith(occurrence.id, "-clamp"))
        punchTip = Math.min(punchTip, initial.worldConnector(occurrence.id, "contact").z);
      if (occurrence.id.indexOf("-die-") >= 0)
        dieTop = Math.max(dieTop, initial.worldConnector(occurrence.id, "contact").z);
    }
    var toolGap = (punchTip - dieTop) * generated.metresPerUnit;
    check(Math.isFinite(toolGap) && toolGap > 0, "press test has installed tool contact datums");
    var model = AssemblySimulationBridge.toRobotModel(definition, physical, generated.assemblyState).model;
    var indices = new Map<String, Int>();
    for (i in 0...model.joints.length) indices.set(model.joints[i].id, i);
    var ram = indices.get("Y1"), follower = indices.get("Y2");
    var left = indices.get("fingerLeft-hinge"), right = indices.get("fingerRight-hinge");
    var gaugeX = indices.get("X"), gaugeR = indices.get("R");
    if (ram == null || follower == null || left == null || right == null) throw "Press simulation lost a ram or contact hinge";
    if (gaugeX == null || gaugeR == null) throw "Press simulation lost a backgauge axis";
    var sheet:SceneObjectData = {id: "contact-sheet", label: "Contact test stock", type: "rectangle",
      x: 0, y: gaugeFace - 0.052, z: stockHeight + 0.00075, width: 0.7, height: 0.1, depth: 0.0015,
      collisionEnabled: true, dynamicBody: false, mass: 0.7 * 0.1 * 0.0015 * 7850,
      red: 0.3, green: 0.5, blue: 0.6, visible: true};
    generated.objects.push(sheet);
    var session = new ProjectDocumentSession(null, false);
    var world = new RobotWorld();
    var simulation = new ApplicationSimulation(world);
    try {
      session.openGeneratedProject(generated, manifest);
      simulation.setBackend(ApplicationSimulation.MUJOCO);
      check(simulation.rebuild(session.sensors, session.scene, session), 'press simulation builds: ${simulation.error}');
      var player = simulation.digitalProgramPlayer();
      if (player == null) throw "Press simulation lost its timed hydraulic program";
      var robot:AssemblyRobot = @:privateAccess player.robot;
      var unknown:materia.project.SceneArtifact.SceneArtifactDigitalProgram = {durationSec: 1, loop: false,
        events: [{timeSec: 0, channel: "absent", value: false}]};
      rejectDigital(() -> { var invalid = new MachineDigitalProgramPlayer(unknown, robot); }, "unknown output channel");
      var unsafe:materia.project.SceneArtifact.SceneArtifactDigitalProgram = {durationSec: 1, loop: false,
        events: [{timeSec: 0, channel: "ramValve/coilA", value: true}]};
      rejectDigital(() -> { var invalid = new MachineDigitalProgramPlayer(unsafe, robot); }, "unsafe terminal coil output");
      robot.blueprint.channels.push(new robotkit.execution.ProcessChannelDeclaration("test-analog", ProcessEventValue.Analog(0)));
      var analog:materia.project.SceneArtifact.SceneArtifactDigitalProgram = {durationSec: 1, loop: false,
        events: [{timeSec: 0, channel: "test-analog", value: false}]};
      rejectDigital(() -> { var invalid = new MachineDigitalProgramPlayer(analog, robot); }, "non-digital output channel");
      robot.blueprint.channels.pop();
      robot.blueprint.channels.push(new robotkit.execution.ProcessChannelDeclaration("test-held", ProcessEventValue.Digital(false), true));
      var held:materia.project.SceneArtifact.SceneArtifactDigitalProgram = {durationSec: 1, loop: false,
        events: [{timeSec: 0, channel: "test-held", value: false}]};
      rejectDigital(() -> { var invalid = new MachineDigitalProgramPlayer(held, robot); }, "output retained on stop");
      robot.blueprint.channels.pop();
      var stock = simulation.environmentObject("contact-sheet");
      if (stock == null) throw "Press simulation lost its test stock";
      var peak = 0.0, changed = false;
      for (_ in 0...100) {
        simulation.step();
        var observed = world.snapshot().robots()[0];
        peak = Math.max(peak, observed.positions.get(ram));
        if (pressSignal(observed, "ram-speed-change") == 1) changed = true;
      }
      var resting = world.snapshot().robots()[0];
      check(resting.positions.get(left) < -0.006 && resting.positions.get(right) < -0.006 &&
        pressSignal(resting, "fingerLeft-contact") == 0 && pressSignal(resting, "fingerRight-contact") == 0,
        "both CAD gauge fingers drop under gravity and open their measured switches");
      for (advance in 0...20) {
        var measured = world.snapshot().robots()[0];
        // Place stock against the measured carrier; the passive finger's angle is never prescribed.
        var tip = gaugeFace + measured.positions.get(gaugeX) - 0.002 + 0.002 * (advance + 1) / 20;
        simulation.stop();
        stock.teleport(new nativekit.sim.SimPose(0, tip - 0.050, stockHeight + measured.positions.get(gaugeR) + 0.00075));
        for (_ in 0...2) {
          simulation.step();
          var observed = world.snapshot().robots()[0];
          peak = Math.max(peak, observed.positions.get(ram));
          if (pressSignal(observed, "ram-speed-change") == 1) changed = true;
        }
      }
      for (_ in 0...100) {
        var measured = world.snapshot().robots()[0];
        simulation.stop();
        stock.teleport(new nativekit.sim.SimPose(0, gaugeFace + measured.positions.get(gaugeX) - 0.050,
          stockHeight + measured.positions.get(gaugeR) + 0.00075));
        simulation.step();
      }
      check(player.failure == null, 'press program remains healthy while stock settles: ${player.failure}');
      var touching = world.snapshot().robots()[0];
      var leftContact = pressSignal(touching, "fingerLeft-contact"), rightContact = pressSignal(touching, "fingerRight-contact");
      var contactDatum = initial.worldConnector("fingerLeft", "contact");
      check(touching.positions.get(left) > -0.001 && touching.positions.get(right) > -0.001 &&
        leftContact == 1 && rightContact == 1,
        'stock contact closes both measured CAD finger switches: q=${touching.positions.get(left)}, ${touching.positions.get(right)}; signals=$leftContact, $rightContact; X=${touching.positions.get(gaugeX)} R=${touching.positions.get(gaugeR)}; initial contact=${contactDatum.x}, ${contactDatum.y}, ${contactDatum.z}');
      simulation.stop();
      stock.teleport(new nativekit.sim.SimPose(0, -1, stockHeight + 0.00075));
      for (_ in 0...100) simulation.step();
      var released = world.snapshot().robots()[0];
      check(released.positions.get(left) < -0.006 && released.positions.get(right) < -0.006 &&
        pressSignal(released, "fingerLeft-contact") == 0 && pressSignal(released, "fingerRight-contact") == 0,
        "removing stock lets both CAD contact fingers reset by gravity");
      check(simulation.reset(), "press resets after independent stock contact checks");
      peak = 0; changed = false;
      var ticks = 0;
      while (player.passes == 0 && player.failure == null && ticks++ < 1000) {
        simulation.step();
        var observed = world.snapshot().robots()[0];
        peak = Math.max(peak, observed.positions.get(ram));
        if (pressSignal(observed, "ram-speed-change") == 1) changed = true;
        check(Math.abs(observed.positions.get(ram) - observed.positions.get(follower)) < 0.0005,
          "both hydraulic ram guides remain coupled");
        check(pressSignal(observed, "ram-bottom") == 0, "empty tooling stroke stays away from nominal mechanical bottom");
      }
      check(player.failure == null && player.passes == 1, 'press completes its native timed valve plan: ${player.failure}');
      check(peak > 0 && peak < toolGap && changed, 'press crosses its speed-change switch with installed tooling still clear: peak=$peak gap=$toolGap');
      var returned = world.snapshot().robots()[0];
      check(Math.abs(returned.positions.get(ram)) < 0.0002 && pressSignal(returned, "ram-top") == 1,
        'ram returns to its measured top switch: q=${returned.positions.get(ram)} m, switch=${pressSignal(returned, "ram-top")}');
      var homeDeflection = Math.abs(returned.positions.get(ram));
      check(homeDeflection <= PressBrake.RAM_STOP_DEFLECTION_MM * 0.001,
        'settled ram home is within its declared cap deflection allowance: $homeDeflection m');
      Sys.println('press ram home: $homeDeflection m cap deflection');
      check(simulation.reset() && player.passes == 0, "press reset clears its timed program completion");
      for (_ in 0...25) simulation.step();
      check(player.failure == null && world.snapshot().robots()[0].trajectoryActive,
        "press timed valve program restarts after reset");
      robot.robot.stop(robotkit.core.StopMode.Emergency);
      simulation.step(); simulation.step();
      check(player.failure != null, "press timed program reports native emergency-stop failure");
      check(simulation.reset() && player.failure == null, "press reset clears its native fault diagnosis");
      Sys.println('press brake simulation: contacts, hydraulic stroke, home and lifecycle passed (peak=$peak m)');
      simulation.dispose(); session.dispose(); generated.objects.pop();
    } catch (error:Dynamic) {
      simulation.dispose(); session.dispose(); generated.objects.pop(); throw error;
    }
  }

  static function rejectDigital(action:Void->Void, label:String):Void {
    var refused = false;
    try action() catch (_:Dynamic) refused = true;
    check(refused, 'Machine digital program rejects $label');
  }

  static function pressSignal(snapshot:robotkit.core.RobotSnapshot, id:String):Float {
    if (snapshot.faultCode != 0 || snapshot.safety != RobotKitRuntimeConstants.RK_SAFETY_READY)
      throw 'Press runtime failed while observing "$id": fault ${snapshot.faultCode}, safety ${snapshot.safety}';
    for (index in 0...snapshot.sensors.length) {
      var sensor = snapshot.sensors.get(index);
      if (sensor.sensorId == id) return sensor.values.get(0);
    }
    var observed = [for (index in 0...snapshot.sensors.length) snapshot.sensors.get(index).sensorId];
    throw 'Press simulation lost measured switch "$id"; observed ${observed.join(", ")}; fault ${snapshot.faultCode}, safety ${snapshot.safety}, positions ${snapshot.positions.toArray()}';
  }

  public static function main():Int {
    var code = checks();
    // No worker may still be planning when the process exits: the native library's destructors would run under it.
    var planning = motionkit.robot.ProgramPlanner.active();
    if (planning > 0) Sys.println('$planning program planners still planning at the end; waiting for them');
    var running = motionkit.robot.ProgramPlanner.shutdown();
    if (running != 0) throw 'The process is ending with $running program planners still running';
    return code;
  }

  static function checkJointMotionPolicy():Void {
    var policy = new JointMotionPolicy(3.0, 12.0, 0.4, 1.5);
    var mechanical = new JointLimits(-1.0, 1.0, 0.7, 10.0, 0.2);
    var rotary = policy.rates(JointType.Revolute, mechanical);
    var linear = policy.rates(JointType.Prismatic, mechanical);
    if (rotary.velocity != 0.7 || rotary.acceleration != 0.2 || rotary.jerk != 12.0 ||
        linear.acceleration != 0.2 || linear.jerk != 1.5)
      throw "Joint-motion policy must respect hardware caps and distinguish angular from linear rates";
    var withoutAcceleration = new JointLimits(-1.0, 1.0, 0.7);
    if (policy.rates(JointType.Revolute, withoutAcceleration).acceleration != 3.0 ||
        policy.rates(JointType.Prismatic, withoutAcceleration).acceleration != 0.4)
      throw "Unrated acceleration must use the explicit requested motion policy";
    var zeroRejected = false;
    try {
      var invalid = policy.rates(JointType.Revolute, new JointLimits(-1.0, 1.0, 0.7, 10.0, 0.0));
    } catch (_:Dynamic) { zeroRejected = true; }
    if (!zeroRejected) throw "A zero hardware acceleration cap must forbid motion";
    var missingRejected = false;
    try {
      var invalid = policy.rates(JointType.Revolute, new JointLimits(-1.0, 1.0));
    } catch (_:Dynamic) { missingRejected = true; }
    if (!missingRejected) throw "Joint-motion policy must not fabricate a mechanical speed cap";
  }

  static function checks():Int {
    var root = Sys.getCwd();
    while (!FileSystem.exists(root + "/cadkit/examples/modeling/materia.project.json")) {
      var parent = haxe.io.Path.directory(root);
      if (parent == root || parent.length == 0) throw "Could not locate Materia repository";
      root = parent;
    }
    Sys.setCwd(root);
    return ProjectSourceSuite.run(root, cases());
  }

  /** Every project-source case: the default suite in order, then focused variants and groups. */
  static function cases():Array<ProjectSourceCase> {
    var suite = ProjectSourceCase.suite, focused = ProjectSourceCase.focused, group = ProjectSourceCase.group;
    return [
      suite("core", checkCore),
      suite("press-brake", checkPressBrake),
      suite("press-controller", checkPressController),
      suite("sheet-press", root -> PressBrakeSimulationChecks.run(root, true)),
      suite("screwdriver-simulation", _ -> ScrewdriverSimulationTests.run()),
      suite("screwdriver-equipment", checkScrewdrivingEquipment),
      suite("screwdriver-cad", root -> checkScrewdrivingOne(root)),
      suite("screwdriver-virtual", root -> checkScrewdrivingOne(root, true)),
      suite("screwdriver-assembly", checkScrewdrivingAssembly),
      suite("screwdriver-required", RequiredFasteningTests.run),
      suite("screwdriver-registration", checkScrewdriverTouchRegistration),
      suite("screwdriver-feeder", checkScrewFeeder),
      suite("screwdriver-blow-feed", checkBlowFeed),
      suite("screwdriver-force-search", checkScrewEntrySearch),
      suite("screwdriver-physical-thread", checkScrewPhysicalThread),
      suite("screwdriver-staged", checkScrewdrivingStaged),
      suite("screwdriver-loss", checkScrewdrivingLoss),
      suite("arm", checkRobotArm),
      suite("gripper", checkParallelGripperMission),
      suite("cobot", root -> checkCobotArms(root)),
      suite("gantry", root -> checkGantryPicker(root)),
      suite("gantry-yaw", root -> checkGantryPicker(root, true)),
      suite("gantry-welder", checkGantryWelder),
      suite("track-arm", checkTrackArm),
      suite("overhead-arm", OverheadArmTests.run),
      suite("clearance", root -> { checkJointSceneClearance(root); checkHeldJointSceneClearance(root); checkHeldJointSceneClearance(root,true); }),
      suite("welder", checkRobotWelder),
      suite("mates", checkMates),
      suite("bench-mill", checkBenchMill),
      suite("enclosed-mill", checkEnclosedMillProject),
      suite("router-device-home", checkVirtualRouterHoming),
      suite("cell", checkTendingControllers),
      suite("interface", checkTendingRobotInterface),
      suite("tending", root -> checkMachineTending(root, TendingCheck.Production)),
      suite("cnc-router", root -> checkCncRouter(root)),
      suite("belts", checkBeltRouter),
      suite("corexy", checkCoreXyPlotter),
      suite("printer-project", checkPrinter),
      suite("printer-cube-short", root -> checkPrinterCube(root, "cube-short", 4.0, true)),
      suite("printer-shaping", checkPrinterShaping),
      suite("printer-mirror", checkPrinterMirror),
      suite("mobile-base", checkMobileBase),
      suite("mobile-mission", checkMobileMission),
      suite("mobile-obstacle", checkMobileObstacle),
      suite("overlay-edge", _ -> checkMissionOverlayEdge()),
      suite("controls", checkCncControls),
      suite("background", checkBackgroundLaunch),
      focused("mobile-welder-registration-boundary", root -> MobileWelderTests.runRegistration(root, true)),
      focused("mobile-welder-registration-boundary-mujoco", root -> MobileWelderTests.runRegistration(root, true, false, ApplicationSimulation.MUJOCO)),
      focused("mobile-welder-registration", root -> MobileWelderTests.runRegistration(root)),
      focused("mobile-welder-voltage-cutoff", root -> MobileWelderTests.runRegistration(root, false, true)),
      focused("mobile-welder-wire-clearance", MobileWelderTests.runWireClearance),
      focused("mobile-welder-frames", MobileWelderTests.runFrames),
      focused("mobile-welder-mission", root -> MobileWelderTests.runMission(root)),
      focused("mobile-welder-mission-mujoco", root -> MobileWelderTests.runMission(root, ApplicationSimulation.MUJOCO)),
      focused("mobile-welder-energy", root -> MobileWelderTests.runMission(root, ApplicationSimulation.MUJOCO, true)),
      focused("mobile-welder-energy-test-backend", root -> MobileWelderTests.runMission(root, ApplicationSimulation.DETERMINISTIC, true)),
      focused("mobile-welder-joint-handoff", MobileWelderTests.runJointHandoff),
      focused("mobile-welder", MobileWelderTests.run),
      focused("printer-cube", root -> checkPrinterCube(root, "cube", 20.0, false)),
      focused("track-weld", root -> checkWholeWeldment(root, ApplicationSimulation.MUJOCO, "track MuJoCo", "track-arm")),
      focused("gantry-starts", root -> { checkGantryPicker(root, false, false, "far"); checkGantryPicker(root, false, false, "unreferenced"); }),
      focused("gantry-realtime", root -> checkGantryPicker(root, false, true)),
      focused("cobot-analytic", checkCobotAnalytic),
      focused("arm-analytic", checkRobotArmAnalytic),
      focused("gantry-analytic", checkCartesianAnalyticExamples),
      focused("welder-whole", root -> { checkWholeWeldment(root, ApplicationSimulation.MUJOCO, "MuJoCo"); checkWholeWeldment(root, ApplicationSimulation.DETERMINISTIC, "test backend"); }),
      focused("cobot850", root -> checkCobotArms(root, [850])),
      focused("brake", checkMillBrakeProject),
      focused("tending-plan", _ -> TendingCyclePlan.TendingCyclePlanChecks.run()),
      focused("tending-route", root -> checkMachineTending(root, TendingCheck.Route)),
      focused("tending-realtime-route", root -> checkMachineTending(root, TendingCheck.RealtimeRoute)),
      focused("tending-performance", root -> checkMachineTending(root, TendingCheck.Performance)),
      focused("tending-interactive", checkTendingInteractive),
      focused("tending-reach", root -> checkMachineTending(root, TendingCheck.Reach)),
      focused("tending-vise", root -> checkMachineTending(root, TendingCheck.Vise)),
      focused("tending-exchange", root -> checkMachineTending(root, TendingCheck.Exchange)),
      focused("identity", checkEnclosedCabinetIdentity),
      focused("clearance-retained", root -> checkHeldJointSceneClearance(root,true)),
      focused("welder-plate", checkFarPlateWeld),
      focused("welder-air", checkWeldInTheAir),
      focused("welder-quality", checkWeldQualities),
      focused("welder-seam", root -> { checkRobotWelderOn(root, ApplicationSimulation.MUJOCO, "MuJoCo"); checkRobotWelderOn(root, ApplicationSimulation.DETERMINISTIC, "test backend"); }),
      group("mill", ["bench-mill", "enclosed-mill", "interface"]),
      group("printer", ["printer-project", "printer-shaping", "printer-mirror"]),
      group("mobile", ["mobile-base", "mobile-mission", "mobile-obstacle", "overlay-edge"]),
      group("router", ["cnc-router", "belts"]),
      group("screwdriver-process", ["screwdriver-simulation", "screwdriver-equipment"]),
      group("screwdriver", ["screwdriver-simulation", "screwdriver-equipment", "screwdriver-cad", "screwdriver-virtual",
        "screwdriver-assembly", "screwdriver-required", "screwdriver-registration", "screwdriver-feeder", "screwdriver-blow-feed",
        "screwdriver-force-search", "screwdriver-physical-thread", "screwdriver-staged", "screwdriver-loss"])
    ];
  }

  /** The kit-level checks and the generated-project editor session round trip. */
  static function checkCore(root:String):Void {
    checkJointMotionPolicy();
    // Four joints and two segments distinguish per-joint stopping limits from
    // per-segment storage, including the constructor's default path.
    var stationary = [0.0, 0.0, 0.0, 0.0];
    var shortPlan = [for (i in 0...2) new TrajectorySegment(Int64.ofInt(i * 100000000),
      Int64.ofInt(100000000), [for (_ in stationary) [0.0]])];
    var defaultStops = new ExecutionPlanSubmission(Int64.ofInt(1), Int64.ofInt(1), Int64.ofInt(0),
      RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE, stationary, stationary, stationary, shortPlan);
    check(defaultStops.controlAcceleration.length == 4,
      "default stopping acceleration has one entry per joint, independent of segment count");
    var explicitStops = new ExecutionPlanSubmission(Int64.ofInt(2), Int64.ofInt(1), Int64.ofInt(0),
      RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE, stationary, stationary, stationary, shortPlan,
      null, null, null, null, null, true, null, false, null, [1.0, 2.0, 3.0, 4.0]);
    check(explicitStops.controlAcceleration.get(3) == 4.0,
      "explicit stopping acceleration accepts all four joints in a two-segment plan");
    var flat = Bytes.alloc(4 * 24);
    for (index in 0...4) {
      flat.setDouble(index * 24, (index & 1) == 0 ? 0.0 : 2.0);
      flat.setDouble(index * 24 + 8, (index & 2) == 0 ? 0.0 : 1.0);
      flat.setDouble(index * 24 + 16, 0.0);
    }
    var flatHull = cadkit.ConvexHullVertices.safeFromMesh(flat, 4, 0.5);
    check(flatHull.warning != null && flatHull.vertices.length == 24 &&
      flatHull.vertices[2] == -0.25 && flatHull.vertices[14] == 0.25,
      "flat collision part uses a 0.5-unit thickened box with a warning");
    checkCoupling(0);
    checkCoupling(1);
    checkLinkCollision(0, true);
    checkLinkCollision(0, false);
    checkLinkCollision(1, true);
    checkLinkCollision(1, false);
    checkHullCollision(0, true);
    checkHullCollision(0, false);
    checkHullCollision(1, true);
    checkHullCollision(1, false);
    checkWedgeContact();
    var manifest = FileSystem.fullPath(root + "/cadkit/examples/modeling/materia.project.json");
    var machineManifest = FileSystem.fullPath(root + "/machinekit/examples/materia.project.json");
    var requirement = MateriaProjectRunner.executionRequirement(machineManifest);
    check(requirement.kind == "requires-project-code" &&
      requirement.projectPath == FileSystem.fullPath(machineManifest) && requirement.module.length > 0,
      "project inspection identifies executable generator code without running it");
    var machineScene = MateriaProjectRunner.loadProject(machineManifest);
    var hasAluminium = false, hasSteel = false;
    for (part in machineScene.objects) {
      check(part.mass > 0.000001 && Math.abs(part.mass - 1.0) > 0.000001,
        "generated part mass comes from volume and material density");
      if (part.materialId == "aluminium") hasAluminium = true;
      if (part.materialId == "steel-c45") hasSteel = true;
    }
    check(hasAluminium && hasSteel, "machine preview carries distinct physical materials");
    check(machineScene.physical.parts.length > 0 && machineScene.assemblyDefinition != null,
      "machine preview keeps assembly mass properties without retaining mesh streams");
    for (part in machineScene.physical.parts)
      check(part.collisionErrorRatio != null && part.collisionErrorRatio <= 0.02,
        'part ${part.id} hull support error exceeds 2% of its diagonal: ${part.collisionErrorRatio}');
    checkMachinePartHullCollision(machineScene.physical.parts[0].collisionHull,
      machineScene.physical.metresPerUnit);
    var machineDefinition:AssemblyDefinition = cast(machineScene.assemblyDefinition, AssemblyDefinition);
    if (machineDefinition != null) {
      var translated = AssemblySimulationBridge.toRobotModel(machineDefinition,
        machineScene.physical);
      // Parts share their rigid body's link, so the links carry every part's material-derived mass,
      // plus the root link's 1 g placeholder.
      var parts = 0.0, links = 0.0;
      for (occurrence in machineDefinition.occurrences) {
        var record = [for (item in machineScene.objects) if (item.id == "project:" + occurrence.id) item][0];
        parts += record.mass;
      }
      for (link in translated.model.links) links += link.mass;
      check(Math.abs(links - 0.001 - parts) < 1e-6, 'assembly links keep the parts\' material-derived mass ($links, $parts)');
    }
    var machineSession = new ProjectDocumentSession(null, false);
    var machineWorld = new RobotWorld();
    var machineSimulation = new ApplicationSimulation(machineWorld);
      machineSession.openGeneratedScene(machineScene.objects, machineManifest,
        machineScene.geometryBySnapshot, machineScene.assemblyDefinition, machineScene.assemblyState,
        machineScene.localCentersByDefinition, machineScene.metresPerUnit,
        machineScene.physical, machineScene.recipeDocument);
      check(machineSession.sensors.robotModels().length == 0,
        "generated assembly starts without an unrelated sensor robot");
      check(machineSimulation.rebuild(machineSession.sensors, machineSession.scene, machineSession),
        "generated assembly builds in the shared simulation: " + machineSimulation.error);
      var machineAssemblyId = machineDefinition.id;
      check(machineSimulation.simulatedRobotIds().length == 1 &&
        machineSimulation.simulatedRobotIds()[0] == "assembly:" + machineAssemblyId,
        "assembly-only document attaches its simulated robot");
      machineSession.scene.setName(machineScene.objects[0].id, "Renamed generated part");
      machineSession.scene.setVisible(machineScene.objects[0].id, false);
      check(!machineSimulation.pending(machineSession.sensors, machineSession.scene),
        "generated part name and visibility leave the physics configuration current");
      machineSession.scene.select(machineScene.objects[0].id);
      var massProperty = [for (item in machineSession.scene.properties())
        if (StringTools.endsWith(item.id, ":mass")) item][0];
      var massEdit = new PropertyBinding(massProperty, machineSession.scene.context())
        .apply(PropertyValue.Float(machineScene.objects[0].mass * 1.1));
      check(massEdit == PropertyEditResult.Applied, "generated part mass edit applies: " + massEdit);
      check(machineSimulation.pending(machineSession.sensors, machineSession.scene),
        "generated part mass requires a physics rebuild");
      check(machineSimulation.rebuild(machineSession.sensors, machineSession.scene, machineSession),
        "material mass change rebuilds: " + machineSimulation.error);
      check(machineSession.setAssemblyJointCoordinate("coupling", 0.2) &&
        machineSimulation.pending(machineSession.sensors, machineSession.scene),
        "assembly joint placement requires a physics rebuild");
      check(machineSimulation.rebuild(machineSession.sensors, machineSession.scene, machineSession),
        "updated assembly placement rebuilds: " + machineSimulation.error);
      machineSimulation.step();
      var runningFrame = machineSimulation.capturePresentationSnapshot();
      check([for (pose in runningFrame.environment) if (pose.id == machineScene.objects[0].id) pose].length == 1,
        "stepped assembly publishes generated-part poses for the viewport");
      check(machineSimulation.reset(), "assembly simulation resets");
      var resetFrame = machineSimulation.capturePresentationSnapshot();
      check([for (pose in resetFrame.environment) if (pose.id == machineScene.objects[0].id) pose].length == 0,
        "reset returns generated parts to the saved editor pose");
      machineSimulation.start();
      var applied = machineSimulation.appliedRevision;
      machineScene.physical.parts[0].volume = -1;
      check(!machineSimulation.rebuild(machineSession.sensors, machineSession.scene, machineSession) &&
        machineSimulation.appliedRevision == applied && machineSimulation.isRunning(),
        "failed assembly bridge preserves the running simulation");
      machineScene.physical.parts[0].volume = Math.abs(machineScene.physical.parts[0].volume);
      var edge = machineDefinition.joints[0];
      machineDefinition.joints.push({id: "test-closure", type: edge.type,
        role: AssemblyJointRole.Closure, parent: edge.parent, parentConnector: edge.parentConnector,
        child: edge.child, childConnector: edge.childConnector, axis: edge.axis,
        limits: edge.limits, defaultValue: edge.defaultValue});
      check(!machineSimulation.rebuild(machineSession.sensors, machineSession.scene, machineSession) &&
        machineSimulation.error != null && machineSimulation.error.indexOf("closures") >= 0 &&
        machineSimulation.isRunning(), "unsupported closure reports a simulation diagnostic");
      machineDefinition.joints.pop();
      machineSimulation.stop();
      machineSimulation.setBackend(ApplicationSimulation.MUJOCO);
      check(machineSimulation.rebuild(machineSession.sensors, machineSession.scene, machineSession),
        "MachineKit assembly rebuilds on MuJoCo: " + machineSimulation.error);
      machineSimulation.step();
      var restBefore = [for (pose in machineSimulation.capturePresentationSnapshot().environment)
        if (pose.id == machineScene.objects[0].id) pose][0];
      for (index in 0...100) machineSimulation.step();
      var restFrame = machineSimulation.capturePresentationSnapshot();
      var restAfter = [for (pose in restFrame.environment)
        if (pose.id == machineScene.objects[0].id) pose][0];
      var restShift = 0.0;
      for (axis in 0...3) restShift += Math.pow(restAfter.position[axis] - restBefore.position[axis], 2);
      check(Math.sqrt(restShift) < 0.01,
        'MachineKit assembly stays stable at rest: ${Math.sqrt(restShift)} metres');
      machineSimulation.start();
      check(machineSimulation.isRunning() && machineSimulation.error == null,
        "MachineKit assembly starts without a Simulation panel error");
      machineSimulation.capturePresentationSnapshot();
      machineSimulation.stop();
      var target = machineScene.objects[0];
      var dropX = 0.0, dropY = 0.0, dropTop = Math.NEGATIVE_INFINITY;
      // Parts ride their rigid body's link, each at its own offset there.
      var partLinks = AssemblySimulationBridge.toRobotModel(machineDefinition, machineScene.physical).partLinks;
      for (occurrence in machineDefinition.occurrences) {
        var physical = [for (part in machineScene.physical.parts)
          if (part.id == occurrence.definition) part][0];
        var placed = partLinks.get(occurrence.id);
        if (placed == null) continue;
        var body = restFrame.robots[0].links[placed.link];
        var linkPose = AssemblyRobot.compose({position: body.position, rotation: body.rotation}, placed.offset);
        var hull = physical.collisionHull;
        if (hull == null) continue;
        var centroidX = 0.0, centroidY = 0.0;
        var top = Math.NEGATIVE_INFINITY;
        var count = Std.int(hull.length / 3);
        for (index in 0...count) {
          var point = rotateVector(hull[index * 3] * machineScene.physical.metresPerUnit,
            hull[index * 3 + 1] * machineScene.physical.metresPerUnit,
            hull[index * 3 + 2] * machineScene.physical.metresPerUnit, linkPose.rotation);
          centroidX += point[0]; centroidY += point[1];
          top = Math.max(top, linkPose.position[2] + point[2]);
        }
        if (top > dropTop) {
          target = [for (part in machineScene.objects)
            if (part.id == "project:" + occurrence.id) part][0];
          dropX = linkPose.position[0] + centroidX / count;
          dropY = linkPose.position[1] + centroidY / count;
          dropTop = top;
        }
      }
      check(machineSession.scene.createRectangle(), "create a falling box above a MachineKit part");
      var probeId = machineSession.scene.selectedId;
      var probe = machineSession.scene.object(probeId);
      check(probe != null, "falling box was added to the generated scene");
      machineSession.scene.setDimensions(probeId, 0.05, 0.05, 0.05);
      machineSession.scene.setPositionXY(probeId, dropX, dropY);
      probe.z = dropTop + 0.5;
      probe.dynamicBody = true;
      check(machineSimulation.rebuild(machineSession.sensors, machineSession.scene, machineSession),
        "MachineKit assembly rebuilds with a falling box: " + machineSimulation.error);
      for (index in 0...200) machineSimulation.step();
      var onPose = [for (pose in machineSimulation.capturePresentationSnapshot().environment)
        if (pose.id == probeId) pose][0];
      var targetObject = machineSession.scene.object(target.id);
      check(targetObject != null, "target MachineKit part remains in the scene");
      targetObject.collisionEnabled = false;
      check(machineSimulation.rebuild(machineSession.sensors, machineSession.scene, machineSession),
        "MachineKit assembly rebuilds with one part collision disabled: " + machineSimulation.error);
      for (index in 0...200) machineSimulation.step();
      var offPose = [for (pose in machineSimulation.capturePresentationSnapshot().environment)
        if (pose.id == probeId) pose][0];
      check(onPose.position[2] > offPose.position[2] + 0.001,
        'falling box passes through disabled MachineKit part: on=${onPose.position[2]}, off=${offPose.position[2]}');
      for (part in machineScene.objects) if (part.id != target.id) {
        var unchanged = machineSession.scene.object(part.id);
        check(unchanged != null && unchanged.collisionEnabled,
          'disabling ${target.id} preserves ${part.id} collision');
      }
    machineSimulation.dispose(); machineWorld.close(); machineSession.dispose();
    var generatedScene = MateriaProjectRunner.loadProject(manifest);
    var generated = generatedScene.objects;
    check(generated.length == 13, "project generates all excavator parts");
    var base = generated[0];
    check(generatedScene.geometryBySnapshot.exists(base.meshSnapshot) &&
      StringTools.startsWith(base.meshSnapshot, "materia.artifact-part/1:"),
      "generated parts carry direct runtime geometry and a compact source identity");
    var session = new ProjectDocumentSession();
    var output = "/tmp/materia-project-source-" + Sys.getPid() + ".materia.json";
    var stage = "open generated scene";
    try {
      session.openGeneratedScene(generated, manifest,
        generatedScene.geometryBySnapshot, generatedScene.assemblyDefinition,
        generatedScene.assemblyState, generatedScene.localCentersByDefinition,
        generatedScene.metresPerUnit, generatedScene.physical,
        generatedScene.recipeDocument);
      session.scene.select(base.id);
      check(!session.scene.canMoveObject(base.id), "assembly-owned part moves through its joints");
      session.scene.setName(base.id, "Edited generated base");
      session.scene.setVisible(base.id, false);
      check(session.scene.createRectangle(), "authored object can join project");
      stage = "transfer unsaved scene";
      var live = session.liveState();
      stage = "restore unsaved scene";
      check(live.indexOf(base.meshSnapshot) < 0,
        "live state retains the project reference instead of embedding generated geometry");
      var restored = new ProjectDocumentSession();
      try {
        restored.restoreLiveState(live);
        check(restored.isDirty() && restored.path == null,
          "unsaved project stays untitled and dirty after reload");
        check(restored.projectReference == manifest && restored.projectAssemblyDefinition != null,
          "reload keeps the generated project and assembly");
        check(restored.sensors.robotModels().length == 0,
          "reload keeps a generated assembly free of an implicit sensor robot");
        var restoredBase = [for (item in restored.scene.records()) if (item.id == base.id) item][0];
        check(restored.scene.items().length == 14 &&
          restoredBase.label == "Edited generated base" && !restoredBase.visible,
          "reload keeps generated field edits and authored objects");
        var previousScene = restored.scene;
        var invalid:Dynamic = Json.parse(live);
        Reflect.setField(invalid, "content", "invalid scene document");
        var failed = false;
        try restored.restoreLiveState(Json.stringify(invalid)) catch (_:Dynamic) failed = true;
        check(failed && restored.scene == previousScene && restored.isDirty(),
          "failed restore leaves the current document intact");
      } catch (error:Dynamic) {
        restored.dispose();
        throw error;
      }
      restored.dispose();
      stage = "save scene";
      session.save(output);
      var saved = File.getContent(output);
      var document:Dynamic = Json.parse(saved);
      var project:Dynamic = Reflect.field(document, "project");
      check(project != null && Reflect.field(project, "reference") != null,
        "saved scene keeps its project reference");
      check(saved.indexOf(base.meshSnapshot) < 0,
        "saved project excludes generated mesh buffers");
      var savedEdits:Array<Dynamic> = cast Reflect.field(project, "overrides");
      check(Reflect.field(project, "version") == 1 && savedEdits.length > 0,
        "project saves typed sparse edits");
      for (edit in savedEdits) check(Reflect.field(edit, "property") != "width" &&
        Reflect.field(edit, "property") != "height" && Reflect.field(edit, "property") != "depth",
        "generated bounds are never saved as overrides");
      var authored:Array<Dynamic> = cast Reflect.field(document, "objects");
      check(authored.length == 1, "saved scene keeps authored objects separately");
      stage = "reopen scene";
      session.open(output);
      stage = "check reopened scene";
      check(session.projectReference == manifest, "reopen restores the source manifest");
      check(session.scene.items().length == 14, "reopen restores generated and authored membership");
      var reopenedBase = [for (item in session.scene.records()) if (item.id == base.id) item][0];
      check(reopenedBase.label == "Edited generated base" && !reopenedBase.visible,
        "reopen restores generated field edits");
      check(!session.isDirty(), "reopened project starts clean");
      session.save(output);
      check(File.getContent(output) == saved, "open-save-open-save is byte identical");
      var stale:Dynamic = Json.parse(saved);
      var staleProject:Dynamic = Reflect.field(stale, "project");
      var staleRemoved:Array<Dynamic> = cast Reflect.field(staleProject, "removed");
      staleRemoved.push("project:removed-in-generator");
      File.saveContent(output, Json.stringify(stale));
      session.open(output);
      check(session.staleEdits().length == 1, "missing removed part is diagnostic");
      check(session.discardStaleEdits() && session.staleEdits().length == 0,
        "stale edit can be discarded through history");
      check(session.document.undo() && session.staleEdits().length == 1,
        "discarding stale edits is undoable");
    } catch (error:Dynamic) {
      session.dispose();
      if (FileSystem.exists(output)) FileSystem.deleteFile(output);
      Sys.println('Project source test failed during $stage');
      throw error;
    }
    session.dispose();
    if (FileSystem.exists(output)) FileSystem.deleteFile(output);
  }

  static function checkMillBrakeProject(root:String):Void {
    var generated = MateriaProjectRunner.loadProject(FileSystem.fullPath(root + "/machinekit/examples/bench-mill/materia.project.json"));
    var definition = generated.assemblyDefinition;
    if (definition == null) throw "Mill brake check has no assembly definition";
    checkMillBrake(AssemblySimulationBridge.toRobotModel(definition, generated.physical).model);
  }

  static function checkEnclosedCabinetIdentity(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/bench-mill/materia.enclosed.project.json");
    checkSingleCabinetMotion(MateriaProjectRunner.loadProject(manifest), manifest);
  }
}

/** How much of the physical tending check runs: the whole production, or a focused part of it. */
enum TendingCheck {
  /** Startup, fault recovery and the four-part production with its acceptance numbers. */
  Production;
  /** The certified routes only, without physics. */
  Route;
  /** The same physical startup route using the editor's paced simulation ticks. */
  RealtimeRoute;
  /** Separately measure generator loading, physics construction, ticks and presentation. */
  Performance;
  /** The playable cell builds and its handling orientations hold. */
  Reach;
  /** Seating a blank in the vise. */
  Vise;
  /** Production up to the first two-part exchange. */
  Exchange;
}
