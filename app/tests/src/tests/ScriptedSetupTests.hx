package tests;

import app.ApplicationSimulation;
import app.SceneDocumentSession;
import app.ScriptOwnership;
import app.SceneCodec;
import app.Main.ReferenceEditorApp;
import app.SetupScript;
import app.ScriptedSetup;
import app.SetupScriptRegistry;
import app.examples.TwoRobotSetupScript;
import haxe.Json;
import LayoutFrame;
import FontCollection;
import nativekit.ui.core.RenderNode;
import robotkit.world.RobotWorld;
import robotkit.world.McapRobotRecording;
import robotkit.world.McapRecordingReader;
import robotkit.world.ReplayRobot;
import robotkit.behavior.HoldJointBehavior;
import robotkit.behavior.WorldBehaviorRunner;
import robotkit.model.Sensor;
import sys.FileSystem;
import sys.io.File;

private class FailingReloadScript implements SetupScript {
  public function new() {
  }
  public function reference():String return TwoRobotSetupScript.REFERENCE;
  public function version():Int return 2;
  public function evaluate(output:ScriptedSetup):Void throw "deliberate setup failure";
}

class ScriptedSetupTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;

  public static function main():Int {
    try {
      run();
      Sys.println("Scripted setup tests passed");
      return 0;
    } catch (error:Dynamic) {
      Sys.println("Scripted setup tests failed: " + Std.string(error));
      return 1;
    }
  }

  public static function run():Void {
    var session = new SceneDocumentSession();
    var setup = session.openScript(TwoRobotSetupScript.REFERENCE);
    check(setup.sensors.configuredRobotIds().length == 2, "script creates two stable robots");
    var lidar:Sensor = setup.sensors.selected();
    check(
      lidar != null && lidar.frame != null && lidar.frame.link.id == "arm",
      "script attaches LiDAR to its articulated link"
    );
    var ownership:ScriptOwnership = session.scriptOwnership;
    check(ownership != null && ownership.document == session.document &&
      setup.scene.document == session.document && setup.sensors.document == session.document,
      "script overrides, scene and sensors share the project history instance");
    check(
      ownership != null && ownership.sensorRateOrigin(setup.sensors.robotId, lidar.id) == "script",
      "inspector reports the script value origin"
    );
    ownership.setOverridesEnabled(true);
    ownership.setSensorRate(setup.sensors.robotId, lidar.id, 33.0);
    check(session.document.undo(), "project undo reaches the script sensor override");
    var undoneOverride = session.refreshScriptOverrides();
    var restoredSensor:Sensor = undoneOverride.sensors.selected();
    check(restoredSensor != null && restoredSensor.updateRate != 33.0,
      "undo rematerializes the script baseline without its last override");
    check(session.document.redo(), "project redo restores the script sensor override");
    var overridden = session.refreshScriptOverrides();
    var overriddenLidar:Sensor = overridden.sensors.selected();
    check(
      overriddenLidar.updateRate == 33.0 && ownership.sensorRateOrigin(
        overridden.sensors.robotId,
        overriddenLidar.id
      ) == "override",
      "stable-ID sensor rate override materializes independently of script data"
    );
    var sensorTarget = overridden.sensors.robotId + "/" + overriddenLidar.id;
    ownership.setOverride(sensorTarget, "noiseStddev", "number", 0.125);
    ownership.setOverride(sensorTarget, "rayCount", "integer", 17);
    ownership.setOverride(sensorTarget, "maxRange", "number", 8.5);
    ownership.setOverride(sensorTarget, "startAngleRadians", "number", -1.2);
    ownership.setOverride(sensorTarget, "fieldOfViewRadians", "number", 2.4);
    ownership.setOverride(sensorTarget, "mount.position", "vector", [0.6, 0.1, 0.2]);
    ownership.setOverride(sensorTarget, "mount.rotation", "vector", [0.0, 0.0, 0.0, 1.0]);
    ownership.setOverride("materia/robot", "position", "vector", [-2.0, 0.5, 0.0]);
    ownership.setOverride("moving-obstacle", "dimensions", "vector", [1.0, 1.5, 2.0]);
    ownership.setOverride("moving-obstacle", "collisionEnabled", "boolean", false);
    ownership.setOverride(ScriptOwnership.SIMULATION_TARGET, "timestep", "number", 0.02);
    ownership.setOverride(ScriptOwnership.SIMULATION_TARGET, "backend", "integer", ApplicationSimulation.DETERMINISTIC);
    overridden.scene.dispose();
    overridden.sensors.dispose();
    overridden = session.refreshScriptOverrides();
    overriddenLidar = overridden.sensors.selected();
    var obstacle = overridden.scene.records()[0];
    var overriddenFrame:robotkit.model.Frame = cast overriddenLidar.frame;
    check(
      overriddenLidar.noiseStddev == 0.125 && overriddenLidar.rayCount == 17
      && overriddenLidar.maxRange == 8.5 && overriddenLidar.startAngleRadians == -1.2
      && overriddenLidar.fieldOfViewRadians == 2.4
      && overriddenFrame != null && overriddenFrame.position[0] == 0.6,
      "typed sensor and mount overrides materialize"
    );
    check(overridden.sensors.robotPosition("materia/robot")[0] == -2.0, "stable robot pose override materializes");
    check(
      obstacle.width == 1.0 && obstacle.height == 1.5 && !obstacle.collisionEnabled,
      "typed environment overrides materialize"
    );
    check(
      overridden.backend == ApplicationSimulation.DETERMINISTIC && overridden.timestep == 0.02,
      "typed simulation overrides materialize"
    );
    check(
      ownership.origin(sensorTarget,
      "noiseStddev") == "override" && ownership.origin(sensorTarget, "noiseSeed") == "script",
      "origin is reported per property"
    );

    if (!FileSystem.exists("build")) FileSystem.createDirectory("build");
    var directory = "build/scripted-setup-" + Std.random(100000000);
    FileSystem.createDirectory(directory);
    var path = directory + "/scene.materia";
    session.save(path);
    var encoded:Dynamic = Json.parse(File.getContent(path));
    check(haxe.crypto.Sha256.encode("abc") ==
      "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
      "script content hash matches SHA-256 known vector");
    check(app.ScriptCanonicalJson.encode({b: 1, a: 2})
      == app.ScriptCanonicalJson.encode({a: 2, b: 1}),
      "script content identity ignores object key order");
    var identity:Dynamic = Reflect.field(encoded, "script");
    check(Reflect.field(identity, "packageId") == "materia.examples"
      && Reflect.field(identity, "sourceSha256") == SetupScriptRegistry.TWO_ROBOT_SOURCE_SHA256
      && StringTools.trim(Reflect.field(identity, "configurationSha256")).length == 64,
      "script documents persist package, source, and configuration identities");
    var encodedObjects:Array<Dynamic> = Reflect.field(encoded, "objects");
    check(
      encodedObjects.length == 0 && Reflect.field(encoded, "sensors") == null,
      "script document saves a reference and overrides without an editable configuration copy"
    );
    var reopened = new SceneDocumentSession();
    reopened.open(path);
    var reopenedLidar:Sensor = reopened.sensors.selected();
    check(
      reopened.scriptOwnership != null && reopenedLidar.updateRate == 33.0
      && reopenedLidar.noiseStddev == 0.125 && reopened.scene.records()[0].width == 1.0,
      "typed script overrides survive reopen"
    );
    check(reopenedLidar.frame != null && reopenedLidar.frame.link.id == "arm"
      && reopenedLidar.frame.position[0] == 0.6 && reopenedLidar.rayCount == 17,
      "scripted sensor identity, link mount, and ray count survive reopen");
    var incompatible:Dynamic = Json.parse(File.getContent(path));
    Reflect.setField(Reflect.field(incompatible, "script"), "configurationSha256",
      "0000000000000000000000000000000000000000000000000000000000000000");
    var incompatiblePath = directory + "/incompatible.materia";
    File.saveContent(incompatiblePath, Json.stringify(incompatible));
    var generationBeforeMismatch = reopened.generation;
    var rejectedMismatch = false;
    try reopened.open(incompatiblePath) catch (_:Dynamic) rejectedMismatch = true;
    check(rejectedMismatch && reopened.generation == generationBeforeMismatch
      && reopenedLidar.updateRate == 33.0,
      "script identity mismatch preserves the open project");

    var legacy = '{"format":"materia.scene","version":1,"objects":[],"sensors":null,"script":{'
      + '"reference":"${TwoRobotSetupScript.REFERENCE}","version":1,"overridesEnabled":true,'
      + '"overrides":[{"targetId":"$sensorTarget","property":"updateRate","value":44.0}]}}';
    var migrated:app.ScriptOwnershipRecord = cast SceneCodec.decodeScript(legacy);
    check(
      migrated.overrideVersion == ScriptOwnership.OVERRIDE_VERSION && migrated.overrides[0].kind == "number",
      "legacy numeric overrides migrate to the versioned typed contract"
    );

    var stale = new ScriptOwnership(TwoRobotSetupScript.REFERENCE, {reference: TwoRobotSetupScript.REFERENCE,
      version: 1, overrideVersion: ScriptOwnership.OVERRIDE_VERSION, overridesEnabled: true, overrides:[{
      targetId: "materia/robot/deleted-sensor",
      property: "updateRate",
      kind: "number",
      encodedValue: "7"
    }
    ]}
    );
    var staleSetup = stale.materialize();
    check(
      stale.diagnostics.length == 1 && stale.diagnostics[0].indexOf("deleted-sensor") >= 0,
      "deleted stable IDs are reported without applying an override elsewhere"
    );
    check(
      stale.removeStaleOverrides() && stale.staleOverrides.length == 0,
      "stale overrides can be explicitly removed with undo history"
    );
    staleSetup.scene.dispose();
    staleSetup.sensors.dispose();
    stale.dispose();

    var world = new RobotWorld(), simulation = new ApplicationSimulation(world);
    ownership.revertTarget(ScriptOwnership.SIMULATION_TARGET);
    session.refreshScriptOverrides();
    simulation.setBackend(ownership.backend());
    simulation.setTimestep(ownership.timestep());
    check(simulation.rebuild(session.sensors, session.scene), "script configuration rebuilds headlessly");
    check(
      simulation.backend == ApplicationSimulation.MUJOCO && simulation.timestep == 0.01,
      "headless instantiation reproduces the script simulation settings"
    );
    var recordingPath = directory + "/scripted.mcap";
    var writer = new McapRobotRecording(recordingPath, 1024 * 1024, false);
    var observed = simulation.step();
    for (index in 0...8) observed = simulation.step();
    for (robotId in ["materia/robot", "materia/robot-b"]) {
      var robot = observed.robot(robotId);
      check(robot != null && robot.sensors.length == 3, "scripted robots publish all configured sensors");
      if (robot != null) {
        writer.recordSnapshot(robot);
        for (frame in robot.sensors.toArray()) writer.recordSensor(robotId, frame);
      }
    }
    writer.close();
    var loaded = McapRecordingReader.load(recordingPath);
    for (robotId in ["materia/robot", "materia/robot-b"]) {
      var original = observed.robot(robotId);
      var replay = new ReplayRobot(robotId, loaded);
      check(original != null && replay.snapshot().id == robotId,
        "MCAP replay selects the scripted robot by stable ID");
      if (original != null) {
        var expected = original.sensors.toArray();
        var actual = replay.sensors();
        check(actual.length == expected.length, "MCAP replay preserves the scripted sensor count");
        for (index in 0...expected.length) check(actual[index].sensorId == expected[index].sensorId
          && actual[index].linkId == expected[index].linkId
          && actual[index].mountPosition.get(0) == expected[index].mountPosition.get(0)
          && haxe.Int64.compare(actual[index].sequence, expected[index].sequence) == 0
          && haxe.Int64.compare(actual[index].sourceTimestampNs, expected[index].sourceTimestampNs) == 0
          && actual[index].values.length == expected[index].values.length,
          "MCAP replay preserves sensor IDs, owning links, mounts, and measurements");
        for (index in 0...expected.length) for (sample in 0...expected[index].values.length)
          check(actual[index].values.get(sample) == expected[index].values.get(sample),
            "MCAP replay preserves each scripted sensor measurement");
      }
      var liveAdapter = world.robot(robotId);
      check(liveAdapter != null && new WorldBehaviorRunner(new HoldJointBehavior(0, 0.25)).update(liveAdapter) == 1,
        "scripted behavior produces a command from live simulation observations");
      check(new WorldBehaviorRunner(new HoldJointBehavior(0, 0.25)).update(replay) == 1
        && replay.generatedCommands.commands.length == 1 && loaded.commands.length == 0,
        "the same behavior consumes replay observations without altering historical commands");
      replay.close();
    }
    var keepRecording = Sys.getEnv("MATERIA_KEEP_SCRIPT_MCAP") == "1";
    if (keepRecording) Sys.println("Materia scripted MCAP fixture: " + recordingPath);
    else {
      FileSystem.deleteFile(recordingPath);
      var recordingStatusPath = recordingPath + ".incomplete.status";
      if (FileSystem.exists(recordingStatusPath)) FileSystem.deleteFile(recordingStatusPath);
    }
    simulation.start();
    var generation = session.generation;
    SetupScriptRegistry.register(TwoRobotSetupScript.REFERENCE, function() return new FailingReloadScript(),
      SetupScriptRegistry.identity(TwoRobotSetupScript.REFERENCE));
    var failed = false;
    try session.reloadScript() catch (_:Dynamic) failed = true;
    check(
      failed && session.generation == generation && simulation.isRunning(),
      "failed reload preserves both published configuration and running simulation"
    );
    SetupScriptRegistry.register(TwoRobotSetupScript.REFERENCE, function() return new TwoRobotSetupScript(),
      SetupScriptRegistry.identity(TwoRobotSetupScript.REFERENCE));

    simulation.dispose();
    world.close();
    reopened.dispose();
    session.dispose();
    inspectorInteraction(directory);
    FileSystem.deleteFile(path);
    if (!keepRecording && Sys.getEnv("MATERIA_KEEP_INSPECTOR_WORKSPACE") != "1")
      FileSystem.deleteDirectory(directory);
  }

  static function inspectorInteraction(directory:String):Void {
    var workspacePath = directory + "/workspace.json";
    var fontPath:Null<String> = null;
    for (candidate in ["../../uikit/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf",
      "../../../uikit/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf"])
      if (FileSystem.exists(candidate)) fontPath = candidate;
    check(fontPath != null, "scripted inspector test font is available");
    var fonts = FontCollection.create();
    fonts.add(cast fontPath);
    var editor = new ReferenceEditorApp(fonts, workspacePath, null, null, null,
      TwoRobotSetupScript.REFERENCE);
    editor.workspace.activate("sensors");
    var frame = new LayoutFrame(1320.0, 900.0);
    var root = editor.submit(frame);
    var sensorPanel:RenderNode = cast findByStyleKey(root, "sensor-panel");
    check(sensorPanel != null && sensorPanel.resolved != null, "Sensors pane is laid out");
    var sensorBounds:ResolvedLayoutItem = cast sensorPanel.resolved;
    var sensorRight = sensorBounds.x + sensorBounds.width;
    check(checkTextFieldsFit(root, sensorRight, frame.width) >= 3,
      "scripted scene property fields are present and fit the inspector");
    for (key in ["script-reload", "script-overrides", "script-revert-simulation",
      "script-remove-stale", "sensor-undo", "sensor-redo", "sensor-apply",
      "sensor-run", "sensor-pause", "sensor-reset", "sensor-design"]) {
      var button:RenderNode = findByStyleKey(root, key);
      check(button != null && button.resolved != null && button.resolved.x >= 0
        && button.resolved.x + button.resolved.width <= sensorRight + 0.01,
        "scripted inspector actions fit the default Sensors pane: " + key);
    }
    click(editor, frame, "script-overrides");
    var ownership:ScriptOwnership = cast editor.session.scriptOwnership;
    check(ownership.overridesEnabled, "inspector enables stable-ID overrides");
    click(editor, frame, "script-rate-increase");
    var selected:Sensor = cast editor.sensors.selected();
    check(selected.updateRate == 21.0 && ownership.sensorRateOrigin("materia/robot", selected.id)
      == "override", "inspector changes one scripted sensor rate");
    click(editor, frame, "sensor-apply");
    check(editor.simulation.isActive() && editor.simulation.simulatedRobotIds().length == 2,
      "inspector applies both scripted robots to the shared simulation");
    click(editor, frame, "sensor-run");
    check(editor.simulation.isRunning(), "inspector starts the scripted simulation");
    click(editor, frame, "sensor-pause");
    check(editor.simulation.isActive() && !editor.simulation.isRunning(),
      "inspector pause preserves the current simulation");
    click(editor, frame, "sensor-reset");
    check(editor.simulation.isActive() && !editor.simulation.isRunning(),
      "inspector reset restores the simulation without returning to design mode");
    editor.dispose();
    fonts.dispose();
    if (Sys.getEnv("MATERIA_KEEP_INSPECTOR_WORKSPACE") == "1")
      Sys.println("Materia scripted inspector workspace: " + FileSystem.fullPath(workspacePath));
    else if (FileSystem.exists(workspacePath)) FileSystem.deleteFile(workspacePath);
  }

  static function click(editor:ReferenceEditorApp, frame:LayoutFrame, key:String):Void {
    var button:RenderNode = cast findByStyleKey(editor.submit(frame), key);
    check(button != null && button.resolved != null, "inspector action exists: " + key);
    var bounds = button.resolved.clippedViewportBounds();
    for (attempt in 0...8) {
      if (bounds.width > 0 && bounds.height > 0) break;
      var location:ResolvedLayoutItem = cast button.resolved;
      editor.ui.scroll(120.0, 500.0, 0.0,
        location.y >= frame.height ? 250.0 : -250.0);
      button = cast findByStyleKey(editor.submit(frame), key);
      bounds = button.resolved.clippedViewportBounds();
    }
    check(bounds.width > 0 && bounds.height > 0, "inspector action is visible: " + key);
    var x = bounds.x + bounds.width / 2, y = bounds.y + bounds.height / 2;
    editor.ui.pointerDown(x, y, 0);
    editor.ui.pointerUp(x, y, 0);
    editor.submit(frame);
  }

  static function findByStyleKey(node:RenderNode, key:String):Null<RenderNode> {
    if (node.styleKey == key) return node;
    for (child in node.children) {
      var found = findByStyleKey(child, key);
      if (found != null) return found;
    }
    return null;
  }

  static function checkTextFieldsFit(node:RenderNode, sensorRight:Float, viewportWidth:Float):Int {
    var bounds = node.resolved;
    var count = 0;
    if (node.styleType == "text-field" && bounds != null && bounds.visible && bounds.width > 0) {
      var paneRight = bounds.x < sensorRight ? sensorRight : viewportWidth;
      check(bounds.x >= 0.0 && bounds.x + bounds.width <= paneRight + 0.01,
        "scripted inspector field fits its pane: " + node.styleKey);
      count++;
    }
    for (child in node.children) count += checkTextFieldsFit(child, sensorRight, viewportWidth);
    return count;
  }
}
