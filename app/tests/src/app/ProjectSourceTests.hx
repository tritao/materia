package app;

import app.MateriaProjectRunner;
import app.ProjectDocumentSession;
import app.ApplicationSimulation;
import robotkit.world.RobotWorld;
import cadbridge.AssemblySimulationBridge;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import nativekit.ui.properties.PropertyBinding;
import nativekit.ui.properties.PropertyValue;
import nativekit.ui.properties.PropertyEditResult;
import haxe.Json;
import haxe.Int64;
import robotkit.runtime.Simulation;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import robotkit.model.JointCoupling;
import sys.FileSystem;
import sys.io.File;

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
    var simulation = new Simulation(0.01, 1, backend);
    var runtime = simulation.addRobot(RobotRuntimeCompiler.compile(model));
    runtime.submitPosition(0, 0.3, 1);
    for (index in 0...200) simulation.step(Int64.ofInt(index));
    var q = runtime.snapshot().q;
    check(Math.abs(q.get(0)) > 0.1 &&
      Math.abs(q.get(1) + 2.0 * q.get(0)) < (backend == 0 ? 1e-9 : 0.02),
      'joint coupling tracks on backend $backend: ${q.get(0)}, ${q.get(1)}');
    simulation.dispose();
  }

  static function checkLinkCollision(backend:Int, enabled:Bool):Void {
    var model = new RobotModel("generated part collision probe");
    model.addLink(new Link("part"));
    model.collisionApproximation = robotkit.model.CollisionApproximation.None;
    var simulation = new Simulation(0.01, 1, backend);
    simulation.addRobotAtPose(RobotRuntimeCompiler.compile(model), [0.0, 0.0, 0.0],
      [0.0, 0.0, 0.0, 1.0], null, [enabled ? [0.5, 0.5, 0.5] : null]);
    var box = simulation.spawnBox([0.0, 0.0, 1.25], [0.1, 0.1, 0.1], true, 1.0);
    for (index in 0...200) simulation.step(Int64.ofInt(index));
    var height = simulation.objectPose(box).position[2];
    check(enabled ? height > 0.5 : (backend == 1 ? height < 0.25 : height < -1.0),
      'link collision ${enabled ? "on" : "off"} on backend $backend: $height');
    simulation.dispose();
  }

  public static function main():Int {
    checkCoupling(0);
    checkCoupling(1);
    checkLinkCollision(0, true);
    checkLinkCollision(0, false);
    checkLinkCollision(1, true);
    checkLinkCollision(1, false);
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
      requirement.projectPath == machineManifest && requirement.module.length > 0,
      "project inspection identifies executable generator code without running it: " +
      requirement.kind + " / " + requirement.projectPath + " / " + requirement.module);
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
        machineScene.localCentersByDefinition, machineScene.metresPerUnit, machineScene.physical);
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
        generatedScene.metresPerUnit, generatedScene.physical);
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
    return 0;
  }
}
