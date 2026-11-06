package tests;

import app.ApplicationSimulation;
import app.ProjectArtifactLoader;
import app.ProjectDocumentSession;
import materia.project.SceneArtifact;
import robotkit.world.RobotWorld;

/** One installed machine, measured guard feedback, real stock, three restartable scenarios. */
class UnifiedHobberTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void {
    checks++;
    if (!ok) throw message;
  }
  public static function main():Int {
    try {
      run();
      var selection = Sys.args().length > 2 ? " cutting" : "";
      Sys.println('Unified hobber$selection: $checks checks passed');
      return 0;
    } catch (e:Dynamic) {
      Sys.println('Unified hobber failed: $e');
      return 1;
    }
  }
  static function run():Void {
    SpindleDriveTests.run();
    HobbingSpindleSamplingTests.run();
    // An artifact produced by the app can be replayed without regenerating unchanged CAD. With no argument this
    // test still builds the machine from source, so both paths exercise the same typed scene and runtime.
    var args = Sys.args();
    if (args.length > 2 && args[2] != "cutting") throw "Expected optional scenario selection: cutting";
    var cuttingOnly = args.length > 2;

    var data = args.length > 0 ? SceneArtifact.decode(sys.io.File.getBytes(args[0])) : {
      var machine = new FlexsplineHobber(new gearkit.GearSpec(1, 40, 4, new gearkit.Involute(20 * Math.PI / 180)), 4);
      var fit = HobberEnclosurePreview.assembled(machine);
      HobberEnclosurePreview.runtimeScene(fit.assembly, machine, fit.shell.layout.stroke, true);
    };
    var definition = data.assemblyDefinition, motion = data.machineMotion;
    if (definition == null || motion == null || motion.enclosureGuard == null) throw "Guarded hobber scene required";
    var enclosure = motion.enclosureGuard;
    var initial = new cadkit.modeling.AssemblyState(definition);
    initial.setJoint(enclosure.doorJoint, enclosure.stroke * 1000);
    data.assemblyState = initial.record();
    var loader = new ProjectArtifactLoader();
    var loaded = loader.load(SceneArtifact.encode(data));
    var project = new ProjectDocumentSession(null, false);
    project.openGeneratedProject(loaded, "unified-hobber");
    check(project.hobbingScenario == "cutting", "Default scenario is Cutting");
    if (args.length > 1) writeTree(project, args[1]);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      check(simulation.rebuild(project.sensors, project.scene, project), "Build unified scene: " + simulation.error);
      var player = simulation.hobbingPlayer(), guard = simulation.enclosureController();
      if (player == null || guard == null) throw "Missing shared process/guard";
      var robot = @:privateAccess player.robot;
      var hobJoint = (@:privateAccess player.indices)[4];
      var spindle = toolpathkit.motion.SpindleDriveBinding.resolve(robot.blueprint, hobJoint);
      check(spindle.drive.joint != hobJoint, "Playback regression uses a belt-coupled spindle, with no velocity drive on B");
      Sys.println("HOBBER_SPINDLE_BINDING " + haxe.Json.stringify({
        spindle: robot.model.joints[hobJoint].id,
        drive: robot.model.joints[spindle.drive.joint].id,
        radiansPerSpeedUnit: spindle.radiansPerSpeedUnit
      }));
      Sys.stdout().flush();
      if (Sys.getEnv("HOBBER_TRACE") == "1") for (index in [hobJoint, spindle.drive.joint, (@:privateAccess player.indices)[5]]) {
        var joint = robot.blueprint.joints[index];
        Sys.println("HOBBER_DRIVE " + haxe.Json.stringify({
          id:robot.model.joints[index].id, passive:joint.passive, maxEffort:joint.maxEffort, maxRate:joint.maxRate,
          stiffness:joint.servoStiffness, damping:joint.servoDamping, inertia:joint.reflectedInertia,
          positionLoopHz:joint.positionLoopRate
        }));
      }
      var stages:Map<String, Bool> = [];
      function complete(label:String):Void {
        var ticks = 0;
        while (!player.completed && player.failure == null && !guard.status().fault && ticks++ < 500000) {
          simulation.step();
          stages.set(player.scenarioStage, true);
          if (Sys.getEnv("HOBBER_TRACE") == "1" && ticks % 10000 == 0) {
            var observation = robot.robot.snapshot(), status = guard.status();
            Sys.println("HOBBER_SCENARIO " + haxe.Json.stringify({
              ticks:ticks, stage:player.scenarioStage, initialized:status.initialized,
              phase:status.phase, axesStill:status.axesStandstill, spindleStill:status.spindleStandstill,
              positions:[for (i in (@:privateAccess player.indices)) observation.positions.get(i)],
              velocities:[for (i in (@:privateAccess player.indices)) observation.velocities.get(i)],
              motorPosition:observation.positions.get(spindle.drive.joint), motorVelocity:observation.velocities.get(spindle.drive.joint),
              motorEffort:observation.efforts.get(spindle.drive.joint)
            }));
            Sys.stdout().flush();
          }
          var s = guard.status();
          if (player.cutStarted && !player.cutCompleted) check(
            s.closed && s.locked,
            "Cutting always requires measured closed/locked feedback"
          );
          if (player.scenarioStage == "loading" || player.scenarioStage == "unloading") check(
            s.open && !s.locked && s.axesStandstill && s.spindleStandstill,
            "Transfer requires measured open/unlocked and standstill"
          );
        }
        Sys.println(label + " " + haxe.Json.stringify({
          stage: player.scenarioStage,
          completed: player.completed,
          failure: player.failure,
          guard: guard.status().message,
          ticks: ticks,
          windows: player.stock.stream.passes,
          seconds: player.elapsed,
          syncArcsec: player.syncError * 648000 / Math.PI,
          tangentialForce: player.stock.forces.maxTangential,
          linearAxesMm: [for (axis in 0...3) (robot.robot.snapshot().positions.get((@:privateAccess player.indices)[axis])
            + (@:privateAccess player.offsets)[axis]) * 1000],
          spindlePosition: robot.robot.snapshot().positions.get(hobJoint),
          spindleVelocity: robot.robot.snapshot().velocities.get(hobJoint)
        }));
        Sys.stdout().flush();
        check(player.failure == null && !guard.status().fault, label + " must not fault");
        check(player.completed, label + " completes within 125 simulated seconds");
        check(player.stock.forces.maxTangential > 0, label + " removes real stock");
        var tableJoint = robot.model.joints[(@:privateAccess player.indices)[5]].id;
        var tableEncoders = [for (encoder in robot.model.encoders) if (encoder.joint == tableJoint) encoder];
        check(tableEncoders.length > 0 && Math.isFinite(player.syncError)
          && player.syncError <= 2 * tableEncoders[0].resolution(), label + " stays within two table encoder quanta");
      }
      complete("Cutting from initially open doors");
      check(
        stages.exists("securing") && stages.exists("cutting"),
        "Automatic cut closes the initial open door before cutting"
      );
      if (!cuttingOnly) {
        simulation.selectHobbingScenario(project, "idle");
        for (i in 0...3000) simulation.step();
        check(!player.cutStarted && player.stock.stream.passes == 0, "Idle never executes the cutting program");
        check(!guard.status().canStart, "Idle disables manual cutting");
        simulation.selectHobbingScenario(project, "loading");
        stages.clear();
        complete("Load, cut and unload");
        for (stage in ["parking", "loading", "cutting", "unloading"]) check(
          stages.exists(stage),
          "Full cycle includes " + stage
        );
        check(
          player.headRetracted && player.cutCompleted && guard.status().open,
          "Full cycle retracts and opens before unloading"
        );
        check(player.sleeveMeshUploads == 1, "Both scenarios share one cached sleeve mesh");
        simulation.selectHobbingScenario(project, "cutting");
        guard.setBoundaryClear(false);
        for (i in 0...2000) simulation.step();
        check(guard.status().fault && !player.cutStarted, "Scenario cannot bypass a guard boundary fault");
      }
      simulation.dispose();
      project.dispose();
    } catch (e:Dynamic) {
      simulation.dispose();
      project.dispose();
      throw e;
    }
  }

  /** Inventory the actual editor hierarchy and view choices without invoking PNG/layout capture. */
  static function writeTree(project:ProjectDocumentSession, directory:String):Void {
    var tree = new app.EditorSceneTree(project.scene, project.projectAssemblyDefinition);
    var rows:Array<{key:String, parent:String, label:String, group:Bool}> = [];
    var groups:Array<{key:String, label:String, path:String, count:Int, shown:Int}> = [];
    function visit(key:String, parent:String):Void {
      rows.push({key:key, parent:parent, label:tree.visibilityLabel(key), group:tree.isGroup(key)});
      var group = tree.groupInfo(key);
      if (group != null) groups.push({key:key, label:group.label, path:group.path, count:group.count, shown:group.shown});
      for (i in 0...tree.childCount(key)) visit(tree.childKeyAt(key, i), key);
    }
    visit("scene", "");
    var presentation = project.presentation;
    var views = presentation == null ? [] : [for (view in presentation.views) {id:view.id, label:view.label}];
    sys.FileSystem.createDirectory(directory);
    sys.io.File.saveContent(directory + "/hobber-ui-tree.json", haxe.Json.stringify({tree:rows, groups:groups, views:views}, null, "  "));
    Sys.println('HOBBER_UI_TREE ${rows.length} rows, ${groups.length} groups, ${views.length} views: $directory/hobber-ui-tree.json');
  }
}
