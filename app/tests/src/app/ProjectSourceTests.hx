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
import app.MatePickTool;
import cadkit.modeling.AssemblyState;
import app.ApplicationSimulation;
import robotkit.world.RobotWorld;
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
import robotkit.execution.ExecutionPlanSubmission;
import robotkit.device.DeviceBinding;
import robotkit.device.DeviceLayout;
import motionkit.trajectory.PlanDiagnostic;
import sys.FileSystem;
import sys.io.File;
import haxe.io.Bytes;

/** Save and reopen a generated project without persisting its mesh buffers. */
@:access(app.MissionPlayer)
@:access(motionkit.robot.ManipulatorMotion)
@:access(motionkit.robot.PlanExecutor)
@:access(processkit.WeldingPlanRunner)
class ProjectSourceTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
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
    var simulationHarness = new SimulationHarness(0.01, 1, backend);

    var simulation = simulationHarness.simulation;
    simulation.addRobotAtPose(RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()), [0.0, 0.0, 0.0],
      [0.0, 0.0, 0.0, 1.0], null, null, [enabled ? vertices : null]);
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
    for (backend in [ApplicationSimulation.DETERMINISTIC, ApplicationSimulation.MUJOCO]) {
      var simulationHarness = new SimulationHarness(0.01, 1, backend);

      var simulation = simulationHarness.simulation;
      simulation.addRobotAtPose(RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()), [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0], null, null, [wedge]);
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
        var simulationHarness = new SimulationHarness(0.01, 1, backend);

        var simulation = simulationHarness.simulation;
        simulation.addRobotAtPose(RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()), [0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 1.0], null, null, [enabled ? hull : null]);
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
    check(generated.mission != null, "the mobile base cell ships a mission");
    generated.mission = null;
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
    var work:materia.project.SceneArtifact.SceneArtifactMission = cast generated.mission;
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
    var work:materia.project.SceneArtifact.SceneArtifactMission = cast generated.mission;
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
  static function checkCobotArms(root:String):Void {
    for (size in [500, 850, 900, 1300]) {
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
      if (generated.mission == null) throw "Picker startup test needs a mission";
      generated.mission.powerUpOffsets = [{joint: "x", offset: 0.3}, {joint: "y", offset: 0.2}];
    }
    var placement = new AssemblyState(definition, generated.assemblyState);
    var session = new ProjectDocumentSession(null, false);
    var simulation = new ApplicationSimulation(new RobotWorld());
    session.openGeneratedProject(generated, manifest);
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      check(simulation.rebuild(session.sensors, session.scene, session), "gantry picker builds: " + simulation.error);
      var mission = simulation.missionPlayer();
      if (mission == null || mission.handling == null) throw "gantry picker has no handling mission";
      var motion = mission.handling.motion;
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
        var currentHandling:Null<motionkit.robot.HandlingPlanRunner> = mission.handling;
        if (currentHandling == null) throw "Picker handling runner disappeared";
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
          // own authored pads; allow only shallow, vertical support there.
          var supported = false;
          var supportDepth = currentHandling.pressDepth + 0.002;
          if (contact.linkIndex == 0 && contact.otherKind == robotkit.runtime.RobotContactOtherKind.Object &&
              contact.distance >= -supportDepth && Math.abs(contact.normal.z) >= 0.99) {
            for (seat in supports) if (contact.otherObject == seat.object &&
                Math.abs(contact.position.x - seat.x) <= 0.0451 &&
                Math.abs(contact.position.y - seat.y) <= 0.0451 &&
                Math.abs(contact.position.z - seat.z) <= supportDepth) supported = true;
          }
          if (supported) continue;
          if (contact.linkIndex != mission.toolLink) {
            var body = mission.simulation.linkPose(0, contact.linkIndex);
            for (hull in mission.robot.hulls) if (hull.link == contact.linkIndex) {
              var lo = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
              var hi = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
              for (vertex in 0...Std.int(hull.vertices.length / 3)) {
                var point = rotateVector(hull.vertices[vertex * 3], hull.vertices[vertex * 3 + 1],
                  hull.vertices[vertex * 3 + 2], body.rotation);
                for (axis in 0...3) {
                  lo[axis] = Math.min(lo[axis], point[axis] + body.position[axis]);
                  hi[axis] = Math.max(hi[axis], point[axis] + body.position[axis]);
                }
              }
              var point = [contact.position.x, contact.position.y, contact.position.z];
              var near = true;
              for (axis in 0...3) if (point[axis] < lo[axis] - 0.002 || point[axis] > hi[axis] + 0.002) near = false;
              if (near) Sys.println("picker contact candidate: " + hull.part + "; bounds=" + lo.join(",") + " / " + hi.join(","));
            }
          }
          // The compliant cup deliberately presses at most 3 mm into its intended grasp surface.
          check(contact.linkIndex == mission.toolLink && contact.otherKind == robotkit.runtime.RobotContactOtherKind.Object &&
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
          var welding:processkit.WeldingPlanRunner = cast mission.welding;
          motion = welding.motion;
        } else {
          var handling:motionkit.robot.HandlingPlanRunner = cast mission.handling;
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
      if (mission == null || mission.handling == null) throw "analytic arm fixture has no handling group";
      var handling:motionkit.robot.HandlingPlanRunner = cast mission.handling;
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

  static function checkRobotArm(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-arm/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition:AssemblyDefinition = cast(generated.assemblyDefinition, AssemblyDefinition);
    var placement = new AssemblyState(definition, generated.assemblyState);
    check(generated.robotMotions == null || generated.robotMotions.length == 0, "the arm project ships no joint tracks");
    var tools = generated.robotTools;
    check(tools != null && tools.length == 1 && tools[0].sensor != null, "the arm's suction tool reports on a vacuum sensor");
    var work:materia.project.SceneArtifact.SceneArtifactMission = cast generated.mission;
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
        var handling:motionkit.robot.HandlingPlanRunner = cast mission.handling;
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
      if (mission == null || mission.handling == null) throw "Track arm has no handling mission";
      var indices = mission.handling.motion.jointIndices;
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
      var make = mission.newHandling;
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
      var tip = mission.toolContact();
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
    var end = mission.toolContact();
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
      var mission = cast(generated.mission, materia.project.SceneArtifact.SceneArtifactMission);
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
      var mission:materia.project.SceneArtifact.SceneArtifactMission = cast generated.mission;
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
  static function openWelder(root:String, manifestName:String, ?adjust:app.MateriaProjectRunner.GeneratedAssemblyScene -> Void,
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
    var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast cast(generated.mission, materia.project.SceneArtifact.SceneArtifactMission).steps[0].weld;
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
    var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast cast(generated.mission, materia.project.SceneArtifact.SceneArtifactMission).steps[0].weld;
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
    var work:materia.project.SceneArtifact.SceneArtifactMission = cast generated.mission;
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
    var work:materia.project.SceneArtifact.SceneArtifactMission = cast generated.mission;
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

  /** The moving-table mill machines its single-tool bearing block in the shared backend. */
  static function checkBenchMill(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/bench-mill/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition:AssemblyDefinition = cast(generated.assemblyDefinition, AssemblyDefinition);
    var model = AssemblySimulationBridge.toRobotModel(definition, generated.physical).model;
    var job = generated.cncJob;
    if (job == null) throw "Mill project has no CNC job";
    var axisNames = job.axes;
    var axes = [for (joint in model.joints) if (axisNames.indexOf(Std.string(joint.id)) >= 0) joint];
    check([for (joint in axes) Std.string(joint.id)].join(",") == "y,x,z", "mill has three mechanical axes");
    var blueprint = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    check(blueprint.fastestPositionLoopRate() == 4000, "mill driver loop rate survives project generation");
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
      generated.recipeDocument, generated.robotMotions, null, generated.cncJob);
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
      check(player.passes == 1, "mill completes one pass");
      var stock = simulation.machiningStock();
      if (stock == null) throw "Mill has no stock";
      var removal = (Math.PI * (121 * 7 + 2 * 25 * 5) + (96 - (4 - Math.PI) * 12.25) * 3) / 1e9;
      var deviation = stock.deviation();
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
      generated.recipeDocument, generated.robotMotions, null, generated.cncJob);
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
      check(stock.removed > 3.6e-6 && stock.rapidContacts == 0 && stock.collisions == 0,
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
  }

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
    viseJoint.limits = viseDrive.extendSign > 0.0 ?
      new JointLimits(0.0, viseDrive.stroke) : new JointLimits(-viseDrive.stroke, 0.0);
    viseJoint.limits.overtravel = viseCompliance;
    viseJoint.limitTimeConstant = viseDrive.limitTimeConstant(viseLink.mass, viseCompliance);
    viseJoint.limitDampingRatio = 1.0;
    fixture.addActuator(doorActuator);
    fixture.addActuator(viseActuator);
    var blueprint = RobotRuntimeCompiler.compile(fixture, new robotkit.profile.RobotProfile());
    var physical = new SimulationHarness(0.01, physicsSubsteps, SimulationSpace.MUJOCO);
    try {
      var runtime = physical.simulation.addRobot(blueprint);
      var stroke = new TrajectorySegment(Int64.ofInt(0), Int64.parseString("3500000000"),
        [[0.0, 0.0], [0.0, 0.0]]);
      var events = [
        new ProcessTimedEvent(Int64.ofInt(10000000), doorDrive.channelA, ProcessEventValue.Digital(true)),
        new ProcessTimedEvent(Int64.ofInt(10000000), viseDrive.channelA, ProcessEventValue.Digital(true)),
        new ProcessTimedEvent(Int64.ofInt(1600000000), doorDrive.channelA, ProcessEventValue.Digital(false)),
        new ProcessTimedEvent(Int64.ofInt(1600000000), doorChannelB, ProcessEventValue.Digital(true))
      ];
      runtime.submitPlan(new ExecutionPlanSubmission(Int64.ofInt(1202), Int64.ofInt(blueprint.revision),
        Int64.ofInt(blueprint.calibrationRevision), RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE,
        [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [stroke], null, null, null, null, null, true, events), 1);
      var doorOpenStart = -1, doorOpenReached = -1;
      var doorCloseStart = -1, doorCloseReached = -1;
      var viseClampStart = -1, viseClampReached = -1;
      var viseClampEffort = 0.0;
      var lastDoorPosition = 0.0;
      for (tick in 1...351) {
        physical.step(Int64.fromFloat(tick * 10000000.0));
        var snapshot = runtime.snapshot();
        if (snapshot.safety != RobotKitRuntimeConstants.RK_SAFETY_READY &&
            snapshot.safety != RobotKitRuntimeConstants.RK_SAFETY_STOPPING)
          throw 'Pneumatic runtime fault at tick $tick: safety=${snapshot.safety}, fault=${snapshot.faultCode}, q=${snapshot.q.get(0)},${snapshot.q.get(1)}, effort=${snapshot.effort.get(0)},${snapshot.effort.get(1)}';
        var doorPosition = snapshot.q.get(0) * doorDrive.extendSign;
        var visePosition = snapshot.q.get(1) * viseDrive.extendSign;
        lastDoorPosition = doorPosition;
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
        if (viseClampStart >= 0 && viseClampReached < 0 && visePosition >= viseDrive.stroke * 0.98) {
          viseClampReached = tick;
          viseClampEffort = Math.abs(snapshot.effort.get(1));
        }
      }
      var doorOpenSeconds = (doorOpenReached - doorOpenStart) * 0.01;
      var doorCloseSeconds = (doorCloseReached - doorCloseStart) * 0.01;
      var viseClampSeconds = (viseClampReached - viseClampStart) * 0.01;
      var expectedDoorSeconds = doorDrive.stroke * 0.98 / doorDrive.ratedSpeed;
      var expectedDoorCloseSeconds = expectedDoorSeconds *
        doorDrive.extensionForce() / doorDrive.retractionForce();
      var expectedViseSeconds = viseDrive.stroke * 0.98 / viseDrive.ratedSpeed;
      check(doorOpenStart > 0 && doorOpenReached > doorOpenStart &&
        Math.abs(doorOpenSeconds / expectedDoorSeconds - 1.0) <= 0.1,
        'MuJoCo door extension takes stroke / rated speed within 10%, measured $doorOpenSeconds s; start=$doorOpenStart reached=$doorOpenReached position=$lastDoorPosition sign=${doorDrive.extendSign} drives=${blueprint.pneumaticDrives.length}, trajectory=${runtime.snapshot().trajectoryTimeNs}, active=${runtime.snapshot().trajectoryActive}, safety=${runtime.snapshot().safety}');
      check(doorCloseStart > 0 && doorCloseReached > doorCloseStart &&
        Math.abs(doorCloseSeconds / expectedDoorCloseSeconds - 1.0) <= 0.1,
        'MuJoCo door retraction takes stroke / rated speed within 10%, measured $doorCloseSeconds s');
      check(viseClampStart > 0 && viseClampReached > viseClampStart &&
        Math.abs(viseClampSeconds / expectedViseSeconds - 1.0) <= 0.1,
        'MuJoCo vise clamp takes stroke / rated speed within 10%, measured $viseClampSeconds s');
      check(Math.abs(viseClampEffort - viseDrive.extensionForce()) <= viseDrive.extensionForce() * 0.05,
        'MuJoCo vise joint reports clamp force within 5% of p·A, measured $viseClampEffort N');
      Sys.println('pneumatic mill: door open $doorOpenSeconds s, close $doorCloseSeconds s, ' +
        'vise clamp $viseClampSeconds s at $viseClampEffort N');
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
    // Each axis is as fast as the slowest of: a 24 V NEMA 23 at half its holding torque (twice the
    // 68.6 rad/s where its winding's reactance takes the supply), what the controller's step tick can
    // generate at its microstepping (40 kHz over 3200 steps a turn: 78.5 rad/s), and the screw's
    // critical speed, all through the axis's ratio to the motor.
    var motorSpeed = 2 * 24 / (50 * 2.5e-3 * 2.8);
    var binding = routerDeviceBinding(model, controller.stepTickHz);
    var stepSpeed = controller.stepTickHz / binding.channels[0].stepsPerUnit;
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
      var expected = Math.min(Math.min(motorSpeed, stepSpeed) / motorRatio, critical);
      var limits = wired.coupledLimits(joint.id, steady);
      check(Math.abs(limits.requireVelocity() - expected) < 1e-9,
        '$kind axis ${joint.id} is as fast as its motor, controller and screw allow: ${limits.requireVelocity()} m/s, expected $expected');
      check(limits.requireAcceleration() > 0.5 && limits.requireAcceleration() < 50,
        '$kind axis ${joint.id} accelerates as its motors move it: ${limits.requireAcceleration()} m/s²');
      derived.push('${joint.id} ${Math.round(limits.requireVelocity() * 1e4) / 10} mm/s, ${Math.round(limits.requireAcceleration() * 100) / 100} m/s²' +
        (critical < Math.POSITIVE_INFINITY ? ' (screw held to ${Math.round(critical * (Math.abs(leadRatio) > 0 ? leadRatio : 1) * 60 / (2 * Math.PI))} rpm)' : ''));
      var bare = model.coupledLimits(joint.id);
      free.push('${joint.id} ${Math.round(bare.requireVelocity() * 1e4) / 10} mm/s, ${Math.round(bare.requireAcceleration() * 100) / 100} m/s²');
    }
    Sys.println('cnc $kind axes with the controller and steady loads: ${derived.join("; ")}');
    Sys.println('cnc $kind axes from their motors and screws alone: ${free.join("; ")}');
    check(job != null && job.loop && job.stock == "stock" && job.spindle == "spindle" && job.target != null &&
      job.loadedTool == 1 && [for (tool in job.tools) tool.number].join(",") == "1,2",
      "the router generates a looping job that machines its stock to a target part with an end mill and a drill");
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
  static function checkMateJoint(manifest:String, generated:MateriaProjectRunner.GeneratedAssemblyScene):Void {
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
  static function checkMatePick(manifest:String, generated:MateriaProjectRunner.GeneratedAssemblyScene):Void {
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
    check(editor.session.mission != null, "the launch project brings its mission");
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

  public static function main():Int {
    var code = checks();
    // No worker may still be planning when the process exits: the native library's destructors would run under it.
    var planning = motionkit.robot.ProgramPlanner.active();
    if (planning > 0) Sys.println('$planning program planners still planning at the end; waiting for them');
    var running = motionkit.robot.ProgramPlanner.shutdown();
    if (running != 0) throw 'The process is ending with $running program planners still running';
    return code;
  }

  static function checks():Int {
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
    var root = Sys.getCwd();
    while (!FileSystem.exists(root + "/cadkit/examples/modeling/materia.project.json")) {
      var parent = haxe.io.Path.directory(root);
      if (parent == root || parent.length == 0) throw "Could not locate Materia repository";
      root = parent;
    }
    Sys.setCwd(root);
    // PROJECT_SOURCE_ONLY=mobile, =arm or =welder runs just those example's checks, for iterating on them.
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "mobile-welder-registration-boundary") {
      MobileWelderTests.runRegistration(root, true);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "mobile-welder-registration") {
      MobileWelderTests.runRegistration(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "mobile-welder-wire-clearance") {
      MobileWelderTests.runWireClearance(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "mobile-welder-frames") {
      MobileWelderTests.runFrames(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "mobile-welder-mission") {
      MobileWelderTests.runMission(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "mobile-welder-mission-mujoco") {
      MobileWelderTests.runMission(root, ApplicationSimulation.MUJOCO);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "mobile-welder-joint-handoff") {
      MobileWelderTests.runJointHandoff(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "mobile-welder") {
      MobileWelderTests.run(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "mobile") {
      checkMobileBase(root);
      checkMobileMission(root);
      checkMobileObstacle(root);
      checkMissionOverlayEdge();
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "corexy") {
      checkCoreXyPlotter(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "track-weld") {
      checkWholeWeldment(root, ApplicationSimulation.MUJOCO, "track MuJoCo", "track-arm");
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "track-arm") {
      checkTrackArm(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "gantry-welder") {
      checkGantryWelder(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "gantry-yaw") {
      checkGantryPicker(root, true);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "gantry-starts") {
      checkGantryPicker(root, false, false, "far");
      checkGantryPicker(root, false, false, "unreferenced");
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "gantry-realtime") {
      checkGantryPicker(root, false, true);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "gantry") {
      checkGantryPicker(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "cobot-analytic") {
      checkCobotAnalytic(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "arm-analytic") {
      checkRobotArmAnalytic(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "gantry-analytic") {
      checkCartesianAnalyticExamples(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "arm") {
      checkRobotArm(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "welder-whole") {
      checkWholeWeldment(root, ApplicationSimulation.MUJOCO, "MuJoCo");
      checkWholeWeldment(root, ApplicationSimulation.DETERMINISTIC, "test backend");
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "welder") {
      checkRobotWelder(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "cobot") {
      checkCobotArms(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "mill") {
      checkBenchMill(root);
      checkEnclosedMillProject(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "pneumatic") {
      checkEnclosedMillProject(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "welder-plate") {
      checkFarPlateWeld(root);
      return 0;
    }
    // Resume the phase gate after the whole-weldment and seam checks.
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "after-welder-seam") {
      checkWeldInTheAir(root);
      checkWeldCraterFault(root);
      checkWeldFollowsWorkpiece(root);
      checkWeldPost(root);
      checkWeldQualities(root);
      checkMates(root);
      checkBenchMill(root);
      checkEnclosedMillProject(root);
      checkVirtualRouterHoming(root);
      checkCncRouter(root);
      checkBeltRouter(root);
      checkCoreXyPlotter(root);
      checkMobileBase(root);
      checkMobileMission(root);
      checkMobileObstacle(root);
      checkMissionOverlayEdge();
      checkCncControls(root);
      checkBackgroundLaunch(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "welder-air") {
      checkWeldInTheAir(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "welder-quality") {
      checkWeldQualities(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "welder-seam") {
      checkRobotWelderOn(root, ApplicationSimulation.MUJOCO, "MuJoCo");
      checkRobotWelderOn(root, ApplicationSimulation.DETERMINISTIC, "test backend");
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "router-device-home") {
      checkVirtualRouterHoming(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "remaining") {
      checkCncRouter(root);
      checkBeltRouter(root);
      checkCoreXyPlotter(root);
      checkMobileBase(root);
      checkMobileMission(root);
      checkMobileObstacle(root);
      checkMissionOverlayEdge();
      checkCncControls(root);
      checkBackgroundLaunch(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "router") {
      checkCncRouter(root);
      checkBeltRouter(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "belts") {
      checkBeltRouter(root);
      return 0;
    }
    if (Sys.getEnv("PROJECT_SOURCE_ONLY") == "controls") {
      checkCncControls(root);
      checkBackgroundLaunch(root);
      return 0;
    }
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
    checkRobotArm(root);
    checkCobotArms(root);
    checkGantryPicker(root);
    checkGantryPicker(root, true);
    checkGantryWelder(root);
    checkTrackArm(root);
    checkRobotWelder(root);
    checkMates(root);
    checkBenchMill(root);
    checkEnclosedMillProject(root);
    checkVirtualRouterHoming(root);
    checkCncRouter(root);
    checkBeltRouter(root);
    checkCoreXyPlotter(root);
    checkMobileBase(root);
    checkMobileMission(root);
    checkMobileObstacle(root);
    checkMissionOverlayEdge();
    checkCncControls(root);
    checkBackgroundLaunch(root);
    return 0;
  }
}
