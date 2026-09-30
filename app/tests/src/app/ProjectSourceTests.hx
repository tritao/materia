package app;

import app.MateriaProjectRunner;
import app.Main.ReferenceEditorApp;
import app.ProjectDocumentSession;
import app.ApplicationSimulation;
import robotkit.world.RobotWorld;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalData;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
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
    // A free part gets no link; a part that is joined cannot be freed.
    var whole = AssemblySimulationBridge.toRobotModel(definition, physical).model;
    var freed = AssemblySimulationBridge.toRobotModel(definition, physical, null, ["workpiece"]).model;
    check(freed.links.length == whole.links.length - 1 && freed.joints.length == whole.joints.length - 1,
      "a free part leaves the assembly robot without its link or its root joint");
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
      generated.recipeDocument, generated.robotMotions, generated.robotGrips, generated.cncJob);
    check(session.robotMotions.length == 6, "opening the arm project installs its motion");
    checkArmHierarchy(session);
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
   * The router example opens, its axes simulate as prismatic joints in metres, and its CNC job cuts
   * three slots in the stock in real time: the stock loses exactly the slots' volume as the machine
   * moves, with no rapid through stock and no holder contact.
   */
  static function checkCncRouter(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/cnc-router/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition:AssemblyDefinition = cast(generated.assemblyDefinition, AssemblyDefinition);
    var model = AssemblySimulationBridge.toRobotModel(definition, generated.physical).model;
    var axes = [for (joint in model.joints) if (joint.type == JointType.Prismatic) joint];
    check([for (joint in axes) Std.string(joint.id)].join(",") == "y,x,z", "the router simulates axes y, x and z");
    for (joint in axes) {
      var travel = joint.limits.upper - joint.limits.lower;
      check(Math.abs(travel - (Std.string(joint.id) == "z" ? 0.08 : 0.3)) < 1e-9,
        'router axis ${joint.id} travel is in metres, got $travel');
      check(joint.limits.overtravel > 0 && joint.limits.maxAcceleration > 0,
        'router axis ${joint.id} carries its overtravel and acceleration');
    }
    var job = generated.cncJob;
    check(job != null && job.loop && job.stockPart == "stock" && job.tools.length == 1,
      "the router ships a looping job that machines its stock");
    check(generated.robotMotions == null || generated.robotMotions.length == 0,
      "the router's motion comes from its program, not tracks");
    var session = new ProjectDocumentSession(null, false);
    var simulation = new ApplicationSimulation(new RobotWorld());
    session.openGeneratedScene(generated.objects, manifest, generated.assembly,
      generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
      generated.localCentersByDefinition, generated.metresPerUnit, generated.physical,
      generated.recipeDocument, generated.robotMotions, generated.robotGrips, generated.cncJob);
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    check(simulation.rebuild(session.sensors, session.scene, session),
      "the router builds in the shared simulation: " + simulation.error);
    function toolPosition():Array<Float> {
      var tool = [for (pose in simulation.capturePresentationSnapshot().environment) if (pose.id == "project:tool") pose];
      check(tool.length == 1, "the router publishes its tool pose");
      return tool[0].position;
    }
    simulation.step();
    var start = toolPosition();
    var lowest = 0.0, steps = 0, leftStart = false, backAt = -1.0, cutting = 0.0;
    while (backAt < 0 && simulation.activeSession().simulationTime() < 120.0 && steps++ < 100000) {
      var before = Sys.time();
      simulation.step();
      cutting += Sys.time() - before;
      check(simulation.cncFailure() == null, 'the router program runs: ${simulation.cncFailure()}');
      var position = toolPosition();
      var offset = [for (axis in 0...3) position[axis] - start[axis]];
      lowest = Math.min(lowest, offset[2]);
      var away = Math.sqrt(offset[0] * offset[0] + offset[1] * offset[1] + offset[2] * offset[2]);
      if (away > 0.01) leftStart = true;
      else if (leftStart && lowest < -0.05) backAt = simulation.activeSession().simulationTime();
    }
    check(backAt > 0, "the program returns the tool to where it started");
    // From 54 mm above the stock the tool goes 2 mm into it.
    check(Math.abs(lowest + 0.056) < 0.0005, 'the tool cuts 2 mm deep, lowest $lowest m');
    var stock = simulation.machiningStock();
    if (stock == null) throw "the router cuts no stock";
    // Each slot is a plunge and an 80 mm pass of a 6 mm flat end mill, 2 mm deep.
    var slots = 3 * (0.08 * 0.006 + Math.PI * 0.003 * 0.003) * 0.002;
    check(Math.abs(stock.removed - slots) < slots * 0.03,
      'the stock loses the three slots, ${stock.removed} m³ removed against $slots');
    check(stock.rapidContacts == 0 && stock.collisions == 0,
      'no rapid runs through the stock and the holder never touches it (${stock.rapidContacts}, ${stock.collisions})');
    check(stock.geometry().triangleCount() > 12, "the cut stock meshes with its slots");
    var player = simulation.cncPlayer();
    if (player != null) Sys.println('cnc router per tick: motion ${Math.round(player.motionSeconds / steps * 1e5) / 100} ms, cutting ' +
      '${Math.round(player.cuttingSeconds / steps * 1e5) / 100} ms, meshing ${Math.round(player.meshingSeconds / steps * 1e5) / 100} ms; compile ${Math.round(player.runSeconds * 1000)} ms, slowest update ${Math.round(player.slowestUpdate * 1000)} ms');
    session.dispose();
    Sys.println('cnc router cut three slots in ${Math.round(backAt * 10) / 10} s of machining, removing ' +
      '${Math.round(stock.removed * 1e10) / 10} mm³ of ${Math.round(slots * 1e10) / 10}; ' +
      '${Math.round(cutting / steps * 1e5) / 100} ms per simulated tick');
  }

  /** A project named at launch builds in the background: queued at once, opened by tick(). */
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
      for (occurrence in machineDefinition.occurrences) {
        var link = [for (item in translated.model.links) if (item.id == occurrence.id) item][0];
        var record = [for (item in machineScene.objects) if (item.id == "project:" + occurrence.id) item][0];
        check(Math.abs(link.mass - record.mass) < 1e-6,
          "assembly occurrence link keeps material-derived mass: " + occurrence.id);
      }
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
      for (occurrence in machineDefinition.occurrences) {
        var physical = [for (part in machineScene.physical.parts)
          if (part.id == occurrence.definition) part][0];
        var linkPose = [for (link in restFrame.robots[0].links)
          if (link.id == occurrence.id) link][0];
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
    checkCncRouter(root);
    checkBackgroundLaunch(root);
    return 0;
  }
}
