package app;

import FontCollection;
import LayoutFrame;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.UiContext;
import nativekit.ui.theme.Theme;
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
import nativekit.ui.properties.PropertyBinding;
import nativekit.ui.properties.PropertyValue;
import nativekit.ui.properties.PropertyEditResult;
import haxe.Json;
import haxe.Int64;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import robotkit.model.JointCoupling;
import sys.FileSystem;
import sys.io.File;
import haxe.io.Bytes;

/** Save and reopen a generated project without persisting its mesh buffers. */
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
    var runtime = simulation.addRobot(RobotRuntimeCompiler.compile(model));
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
    simulation.addRobotAtPose(RobotRuntimeCompiler.compile(model), [0.0, 0.0, 0.0],
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
    simulation.addRobotAtPose(RobotRuntimeCompiler.compile(model), [0.0, 0.0, 0.0],
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
      simulation.addRobotAtPose(RobotRuntimeCompiler.compile(model), [0.0, 0.0, 0.0],
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
        simulation.addRobotAtPose(RobotRuntimeCompiler.compile(model), [0.0, 0.0, 0.0],
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
    check(tree.childCount("parts:pedestal") == 2 && tree.childCount("parts:toolFlange") == 6,
      "a body's Parts node holds the parts fixed to its root");
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

  static function rejectsGrips(raw:Dynamic, fragment:String):Void {
    var message = "";
    try RobotGripEvent.decode(raw) catch (error:Dynamic) message = Std.string(error);
    check(message.indexOf(fragment) >= 0, 'grip events "$fragment" rejected: $message');
  }

  static function checkGripEvents(definition:AssemblyDefinition, physical:AssemblyPhysicalData):Void {
    var good = RobotGripEvent.decode([{time: 1.0, link: "cup", action: "grip"}, {time: 2.5, link: "cup", action: "release"}]);
    check(good.length == 2 && good[0].grip && !good[1].grip && good[1].time == 2.5 && good[1].link == "cup",
      "grip events decode in order");
    check(RobotGripEvent.decode(null).length == 0, "a project without grips has none");
    rejectsGrips([{time: 1.0, link: "cup", action: "grip"}], "end with a release");
    rejectsGrips([{time: 1.0, link: "cup", action: "release"}], "alternate");
    rejectsGrips([{time: 2.0, link: "cup", action: "grip"}, {time: 1.0, link: "cup", action: "release"}], "never decrease");
    rejectsGrips([{time: 1.0, link: "cup", action: "squeeze"}], "grip or release");
    rejectsGrips([{time: 1.0, link: "cup", action: "grip", extra: 1}], "Unknown robot grip field");
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

  /** The arm example opens with its shipped motion and the simulation follows it. */
  static function checkRobotArm(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-arm/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    check(generated.robotMotions != null && generated.robotMotions.length == 6,
      "robot arm project ships one motion track for each joint");
    var definition:AssemblyDefinition = cast(generated.assemblyDefinition, AssemblyDefinition);
    var session = new ProjectDocumentSession(null, false);
    var armWorld = new RobotWorld();
    var armSimulation = new ApplicationSimulation(armWorld);
    session.openGeneratedScene(generated.objects, manifest, generated.assembly,
      generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
      generated.localCentersByDefinition, generated.metresPerUnit, generated.physical,
      generated.recipeDocument, generated.robotMotions, generated.robotGrips, null, generated.cncJob);
    check(session.robotMotions.length == 6, "opening the arm project installs its motion");
    checkArmHierarchy(session);
    checkArmDrag(session, generated.metresPerUnit);
    checkGripEvents(definition, generated.physical);
    armSimulation.setBackend(ApplicationSimulation.MUJOCO);
    check(armSimulation.rebuild(session.sensors, session.scene, session),
      "robot arm builds in the shared simulation: " + armSimulation.error);
    // The joint order of the simulated model, which leaves out the freed workpiece.
    var model = AssemblySimulationBridge.toRobotModel(definition, generated.physical, null, ["workpiece"]).model;
    var index = new Map<String, Int>();
    for (position in 0...model.joints.length) index.set(model.joints[position].id, position);
    var id = "assembly:" + definition.id;
    armSimulation.step();
    var startPoses = armSimulation.capturePresentationSnapshot().environment;
    var startCup = [for (pose in startPoses) if (pose.id == "project:tool/cup") pose];
    var startWork = [for (pose in startPoses) if (pose.id == "project:workpiece") pose];
    check(startCup.length == 1 && startWork.length == 1 && [for (pose in startPoses) if (StringTools.startsWith(pose.id, "project:")) pose].length ==
      definition.occurrences.length, "robot arm publishes a pose for every part, tool and free workpiece included");
    check(session.robotGrips.length == 4, "the arm project ships its four vacuum commands");
    // Run the whole authored cycle: the joints must follow their tracks the entire way, which also means
    // the arm never fights the table or the workpiece.
    var duration = 0.0;
    for (track in generated.robotMotions) duration = Math.max(duration, track.keys[track.keys.length - 1].time);
    var worst = 0.0, farthest = 0.0, nextSample = 0.0, steps = 0;
    var lift = 0.0, atPlace = false, everHeld = false;
    while (armSimulation.activeSession().simulationTime() < duration + 0.5 && steps++ < 100000) {
      armSimulation.step();
      var now = armSimulation.activeSession().simulationTime();
      if (now < nextSample) continue;
      nextSample = now + 0.25;
      var observed = armWorld.snapshot().robot(id);
      if (observed == null) throw "robot arm is missing from the world snapshot";
      // Track positions are relative to the initial pose, like the joint positions themselves.
      for (track in generated.robotMotions) {
        var slot = index.get(track.jointId);
        if (slot == null) throw 'robot arm has no joint ${track.jointId}';
        var error = Math.abs(observed.positions.get(slot) - track.sample(now));
        worst = Math.max(worst, error);
        check(error < 0.05, 'robot arm joint ${track.jointId} follows its track at $now s: off by $error rad');
      }
      for (pose in armSimulation.capturePresentationSnapshot().environment) {
        if (pose.id == "project:tool/cup") {
          var travel = 0.0;
          for (axis in 0...3) travel += Math.pow(pose.position[axis] - startCup[0].position[axis], 2);
          farthest = Math.max(farthest, Math.sqrt(travel));
        } else if (pose.id == "project:workpiece") {
          lift = Math.max(lift, pose.position[2] - startWork[0].position[2]);
          // The far pad is 0.3 m along x from the near one.
          if (Math.abs(pose.position[0] - startWork[0].position[0] - 0.3) < 0.015 &&
              Math.abs(pose.position[1] - startWork[0].position[1]) < 0.015) atPlace = true;
        }
      }
      var holding = armSimulation.heldObjectIds();
      if (holding.length > 0) {
        check(holding.join(",") == "project:workpiece", "only the workpiece is ever held (" + holding.join(",") + ")");
        everHeld = true;
      }
    }
    check(farthest > 0.2, "the suction cup travels to the workpiece and the pad (moved at most " + farthest + " m)");
    // The vacuum must really carry the workpiece: lifted clear of the pad, set down on the far pad, and
    // brought back and released at the near one by the end of the cycle.
    check(everHeld, "the suction cup grips the workpiece");
    check(lift > 0.08, "the workpiece is lifted off its pad (rose at most " + lift + " m)");
    check(atPlace, "the workpiece is carried to the far pad");
    var endWork = [for (pose in armSimulation.capturePresentationSnapshot().environment) if (pose.id == "project:workpiece") pose];
    check(endWork.length == 1 && Math.abs(endWork[0].position[0] - startWork[0].position[0]) < 0.02 &&
      Math.abs(endWork[0].position[1] - startWork[0].position[1]) < 0.02,
      "the workpiece is back on its first pad after the second leg");
    check(armSimulation.heldObjectIds().length == 0, "nothing is held once the cycle has released the workpiece");
    // A reset puts the workpiece back and re-arms the vacuum commands for the next run.
    check(armSimulation.reset(), "the arm simulation resets");
    check(armSimulation.heldObjectIds().length == 0, "a reset holds nothing");
    armSimulation.step();
    var restored = [for (pose in armSimulation.capturePresentationSnapshot().environment) if (pose.id == "project:workpiece") pose];
    check(restored.length == 1 && Math.abs(restored[0].position[0] - startWork[0].position[0]) < 0.002 &&
      Math.abs(restored[0].position[2] - startWork[0].position[2]) < 0.002, "a reset puts the workpiece back on its pad");
    var again = false;
    steps = 0;
    while (armSimulation.activeSession().simulationTime() < 4.0 && steps++ < 100000) {
      armSimulation.step();
      if (armSimulation.heldObjectIds().length > 0) again = true;
    }
    check(again, "the vacuum grips again after a reset");
    Sys.println('robot arm followed its motion track to within $worst rad over ${armSimulation.activeSession().simulationTime()} s');
  }

  /**
   * The router example opens, its axes simulate as prismatic joints in metres, and its CNC job, CAM
   * made from a NEMA 23 motor plate, mills and drills the plate out of the stock in real time, changing
   * tools on the way: the stock loses exactly the plate's recesses and holes, nothing is cut from the
   * part, and no rapid or holder touches stock.
   */
  static function checkCncRouter(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/cnc-router/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition:AssemblyDefinition = cast(generated.assemblyDefinition, AssemblyDefinition);
    var model = AssemblySimulationBridge.toRobotModel(definition, generated.physical).model;
    var axes = [for (joint in model.joints) if (joint.type == JointType.Prismatic) joint];
    check([for (joint in axes) Std.string(joint.id)].join(",") == "y,x,z", "the router simulates axes y, x and z");
    // Four rigid bodies and four lead screws, each screw turning with its coupling on the motor shaft.
    check(model.joints.length == 7 && model.links.length == 8,
      'the router simulates as four rigid bodies, four screws, three axes and four screw joints, got ' +
      '${model.links.length} links and ${model.joints.length} joints');
    var leads = [for (coupling in model.couplings) '${coupling.leader}:${Math.round(coupling.ratio)}'];
    leads.sort(Reflect.compare);
    // A 2 mm lead turns its screw pi radians per millimetre: 3142 per metre.
    check(leads.join(",") == "x:3142,y:3142,y:3142,z:3142", 'each axis turns its screws by their lead, got $leads');
    for (joint in axes) {
      var travel = joint.limits.upper - joint.limits.lower;
      check(Math.abs(travel - (Std.string(joint.id) == "z" ? 0.08 : 0.3)) < 1e-9,
        'router axis ${joint.id} travel is in metres, got $travel');
      check(joint.limits.overtravel > 0, 'router axis ${joint.id} carries its overtravel');
    }
    // Each axis is as fast as a 24 V NEMA 23 turns its Tr10 x 2 screw at half its holding torque:
    // twice the 68.6 rad/s where its winding's reactance takes the supply, over pi rad per mm.
    var turns = 2 * 24 / (50 * 2.5e-3 * 2.8) / (Math.PI * 1000);
    var derived:Array<String> = [];
    for (joint in axes) {
      var limits = model.coupledLimits(joint.id);
      check(Math.abs(limits.velocity - turns) < 1e-9,
        'router axis ${joint.id} is as fast as its motor turns its screw: ${limits.velocity} m/s, expected $turns');
      check(limits.maxAcceleration > 0.5 && limits.maxAcceleration < 50,
        'router axis ${joint.id} accelerates as its motors move it: ${limits.maxAcceleration} m/s²');
      derived.push('${joint.id} ${Math.round(limits.velocity * 1e4) / 10} mm/s, ${Math.round(limits.maxAcceleration * 100) / 100} m/s²');
    }
    Sys.println('cnc router axes from their motors: ${derived.join("; ")}');
    var job = generated.cncJob;
    check(job != null && job.loop && job.stock == "stock" && job.spindle == "spindle" && job.target != null &&
      job.loadedTool == 1 && [for (tool in job.tools) tool.number].join(",") == "1,2",
      "the router generates a looping job that machines its stock to a target part with an end mill and a drill");
    var session = new ProjectDocumentSession(null, false);
    var simulation = new ApplicationSimulation(new RobotWorld());
    session.openGeneratedScene(generated.objects, manifest, generated.assembly,
      generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
      generated.localCentersByDefinition, generated.metresPerUnit, generated.physical,
      generated.recipeDocument, generated.robotMotions, generated.robotGrips, null, generated.cncJob);
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
    var screwStart = [partRotation("screwXCoupling"), partRotation("screwZCoupling")];
    var lowest = 0.0, steps = 0, stepping = 0.0, tools:Array<Int> = [player.loadedTool];
    // Allocation is counted, not timed, so it holds whatever else the machine is doing.
    var allocatedBefore = hl.Gc.totalAllocated(), collectionsBefore = hl.Gc.collections();
    while (player.passes == 0 && simulation.activeSession().simulationTime() < 600.0 && steps++ < 1000000) {
      var before = Sys.time();
      simulation.step();
      stepping += Sys.time() - before;
      check(simulation.cncFailure() == null, 'the router program runs: ${simulation.cncFailure()}');
      if (steps % 10 == 0) lowest = Math.min(lowest, toolPosition()[2] - start[2]);
      if (player.loadedTool != tools[tools.length - 1]) tools.push(player.loadedTool);
      if (steps % 1000 == 0) {
        // The X and Z screws turn half a turn for every millimetre their axes move.
        var now = toolPosition();
        for (axis in [0, 2]) {
          var before = screwStart[axis == 0 ? 0 : 1], after = partRotation(axis == 0 ? "screwXCoupling" : "screwZCoupling");
          var dot = Math.abs(before[0] * after[0] + before[1] * after[1] + before[2] * after[2] + before[3] * after[3]);
          var turned = Math.PI * 1000 * (now[axis] - start[axis]);
          check(Math.abs(dot - Math.abs(Math.cos(turned / 2))) < 0.02,
            'the ${axis == 0 ? "X" : "Z"} screw turns with its axis: ${2 * Math.acos(Math.min(1.0, dot))} rad for $turned');
        }
      }
    }
    var allocatedPerTick = (hl.Gc.totalAllocated() - allocatedBefore) / steps;
    var collections = hl.Gc.collections() - collectionsBefore;
    // About 56 KB a tick when measured (2026-10-02): mostly robot snapshots, then the stock's cut moves.
    check(allocatedPerTick < 80000, 'the router allocates under 80 KB a simulated tick, got ${Math.round(allocatedPerTick)} bytes');
    // The pass ends with the drill; the next pass, started as this one is counted, loads the end mill again.
    var changes = tools.join(",");
    check(changes == "1,2" || changes == "1,2,1", 'the router starts with the end mill and changes to the drill, got $tools');
    var seconds = simulation.activeSession().simulationTime();
    check(player.passes == 1, 'the router finishes one pass of its program, at $seconds s');
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
    check(stock.rapidContacts == 0 && stock.collisions == 0,
      'no rapid runs through the stock and the holder never touches it (${stock.rapidContacts}, ${stock.collisions})');
    var deviation = stock.deviation();
    check(deviation.gouge < 1e-9, 'nothing is cut from the finished plate, gouge ${deviation.gouge} m³');
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
    Sys.println('cnc router milled the motor plate in ${Math.round(seconds * 10) / 10} s of machining: removed ' +
      '${Math.round(stock.removed * 1e10) / 10} mm³ of ${Math.round(recesses * 1e10) / 10}, leftover ' +
      '${Math.round(deviation.leftover * 1e10) / 10} mm³, gouge ${Math.round(deviation.gouge * 1e10) / 10} mm³; ' +
      '${Math.round(stepping / steps * 1e5) / 100} ms per simulated tick');
    Sys.println('cnc router per tick: motion ${Math.round(player.motionSeconds / steps * 1e5) / 100} ms, cutting ' +
      '${Math.round(player.cuttingSeconds / steps * 1e5) / 100} ms, meshing ${Math.round(player.meshingSeconds / steps * 1e5) / 100} ms; ' +
      'compile ${Math.round(player.runSeconds * 1000)} ms, slowest update ${Math.round(player.slowestUpdate * 1000)} ms; ' +
      '${Math.round(allocatedPerTick / 100) / 10} KB allocated a tick, $collections collections');
  }

  /**
   * The router's job answers the operator: it reports the line it runs, stops on a feed hold and
   * carries on, takes a speed override, and restarts at the drilling with the drill loaded.
   */
  static function checkCncControls(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/cnc-router/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var session = new ProjectDocumentSession(null, false);
    var simulation = new ApplicationSimulation(new RobotWorld());
    session.openGeneratedScene(generated.objects, manifest, generated.assembly,
      generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
      generated.localCentersByDefinition, generated.metresPerUnit, generated.physical,
      generated.recipeDocument, generated.robotMotions, generated.robotGrips, null, generated.cncJob);
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
    check(run(20.0, () -> player.currentLine > 0), "the player reports the line it runs");
    check(StringTools.trim(lines[player.currentLine - 1]).length > 0, "the running line is a line of the program");
    // The CNC panel, laid out on its own and operated by pointer.
    var fonts = FontCollection.create();
    fonts.add("uikit/vendor/harfbuzz/perf/fonts/Roboto-Regular.ttf");
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
      'the panel restarts the program at the picked line, now line ${player.currentLine}');
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
    session.openGeneratedScene(generated.objects, manifest, generated.assembly, generated.geometryBySnapshot,
      generated.assemblyDefinition, generated.assemblyState, generated.localCentersByDefinition, generated.metresPerUnit,
      generated.physical, generated.recipeDocument, generated.robotMotions, generated.robotGrips, descriptors);
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
    session.openGeneratedScene(generated.objects, manifest, generated.assembly, generated.geometryBySnapshot,
      generated.assemblyDefinition, generated.assemblyState, generated.localCentersByDefinition, generated.metresPerUnit,
      generated.physical, generated.recipeDocument, generated.robotMotions, generated.robotGrips, descriptors);
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
    session.openGeneratedScene(generated.objects, manifest, generated.assembly, generated.geometryBySnapshot,
      generated.assemblyDefinition, generated.assemblyState, generated.localCentersByDefinition, generated.metresPerUnit,
      generated.physical, generated.recipeDocument, generated.robotMotions, generated.robotGrips, descriptors);
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
    check(editor.session.robotMotions.length == 6, "the launch project brings its motion");
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
      machineSession.openGeneratedScene(machineScene.objects, machineManifest, machineScene.assembly,
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
      session.openGeneratedScene(generated, manifest, generatedScene.assembly,
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
        check(restored.projectReference == manifest && restored.projectAssembly != null,
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
    checkMates(root);
    checkCncRouter(root);
    checkCncControls(root);
    checkBackgroundLaunch(root);
    return 0;
  }
}
