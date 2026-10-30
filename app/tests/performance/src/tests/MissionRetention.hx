package tests;

import app.Main.ReferenceEditorApp;
import haxeon.ui.LayoutFrame;
import haxeon.ui.core.RenderNode;
import haxeon.ui.docking.DockNode;
import haxeon.ui.semantics.AccessibilityRole;
import materia.project.SceneArtifact.SceneArtifactMission;

/** A grounded box and a digital loopback exercise real mission execution without CAD or motion planning. */
@:access(app.Main.ReferenceEditorApp)
@:access(app.ProjectDocumentSession)
@:access(app.editor.MissionPanel)
@:access(tests.HeadlessEditorProfile) class MissionRetention {
  static function work(?controller:String, fail:Bool = false):SceneArtifactMission {
    var steps:Array<materia.project.SceneArtifact.SceneArtifactMissionStep> =
      [for (index in 0...24) {kind: "signal", signal: {name: "pulse", value: index % 2 == 0}}];
    if (fail) steps = [{kind: "waitFor", waitFor: {name: "unwired", value: true, timeout: 0.002}}];
    return {controllerId: controller, steps: steps};
  }

  static function install(editor:ReferenceEditorApp):Void {
    var model = new cadkit.modeling.AssemblyModel("m");
    model.add("base");
    model.ground("base");
    var centers:Map<String, Array<Float>> = ["base" => [0, 0, 0]];
    var data:Array<app.SceneObjectData> = [{
      id: "project:base", label: "Mission fixture", type: "rectangle", x: 0.0, y: 0.0, z: 0.0,
      width: 0.01, height: 0.01, depth: 0.01, collisionEnabled: false, dynamicBody: false,
      mass: 0.001, red: 0.5, green: 0.5, blue: 0.5, visible: true
    }];
    var physical:cadbridge.AssemblySimulationBridge.AssemblyPhysicalData = {metresPerUnit: 1, parts: [{
      id: "base", materialId: "steel", volume: 0.000001, centerOfMass: [0, 0, 0],
      inertia: [0.0000000000167, 0, 0, 0, 0.0000000000167, 0, 0, 0, 0.0000000000167], density: 1000
    }]};
    editor.session.openGeneratedScene(data, null, null, model.definition("mission-retention"), null, centers, 1, physical);
    editor.session.missions = [work("alpha")];
    editor.session.controllers = [{id: "alpha", kind: "robot", joints: [], sensors: [], channels: ["pulse"]}];
    editor.session.signals = [{name: "pulse", from: {controller: "alpha", port: "pulse"}, to: {controller: "alpha", port: "feedback"}}];
    editor.documentChanged();
  }

  public static function run(fonts:haxeon.ui.FontCollection, output:String, cycles:Int, frames:Array<String>):Void {
    var editor = new ReferenceEditorApp(fonts, output + "/mission-workspace.json", null, null, null, null, null, null, true);
    var frame = new LayoutFrame(900, 620);
    function step(action:String, cycle:Int):Void {
      editor.invalidateView();
      try HeadlessEditorProfile.submit(editor, frame, frames, action, cycle)
      catch (error:Dynamic) throw "Mission retention " + action + "/" + cycle + ": " + Std.string(error);
    }
    function text(value:String):Null<RenderNode> {
      var root = editor.ui.root, found:Null<RenderNode> = null;
      if (root != null) root.walk(node -> { if (node.layout.text == value) found = node; });
      return found;
    }
    function select(index:Int):Void {
      var root = editor.ui.root, found:Null<RenderNode> = null;
      if (root != null) root.walk(node -> {
        var semantics = node.semantics;
        if (semantics != null && semantics.role == AccessibilityRole.CollectionItem && semantics.positionInSet == index + 1)
          found = node;
      });
      var row = found;
      if (row == null) throw "Mission row is not visible: " + index;
      var bounds = row.globalBounds(), x = bounds.x + bounds.width / 2, y = bounds.y + bounds.height / 2;
      var hit = false;
      for (node in editor.ui.events.hitTestPath(x, y)) if (node == row) hit = true;
      if (!hit) throw "Mission row hit-test is wrong";
      editor.ui.pointerDown(x, y, 0);
      editor.ui.pointerUp(x, y, 0);
    }
    try {
      // A new caller-provided card updates optional actions even when execution state has not changed.
      var called = 0, tokens = editor.appearance.theme.tokens;
      function card(?action:Void->Void):Void editor.ui.submit(
        app.editor.MissionStatusView.build(editor.missionController, tokens, "mission-card", action), frame);
      card();
      card(() -> called = 1);
      HeadlessEditorProfile.click(editor, "mission-card-steps", false);
      if (called != 1) throw "Mission card did not add its caller action";
      card(() -> called = 2);
      HeadlessEditorProfile.click(editor, "mission-card-steps", false);
      if (called != 2) throw "Mission card retained an old caller callback";
      card();
      var cardRoot = editor.ui.root;
      if (cardRoot != null && HeadlessEditorProfile.findByStyleKey(cardRoot, "mission-card-steps", false) != null)
        throw "Mission card did not remove its caller action";
      editor.workspace.setRoot(DockNode.Panel("mission"));
      step("mission-empty", 0);
      if (text("This project has no mission.") == null) throw "Missing empty mission message";
      for (cycle in 0...cycles) {
        editor.session.missions = [work("alpha"), work("beta")];
        step("mission-authored", cycle);
        var before = text("MISSION");
        step("mission-unchanged", cycle);
        if (before == null || editor.ui.verifyRetentionEvery == 0 && text("MISSION") != before)
          throw "Unchanged mission panel rebuilt";
        HeadlessEditorProfile.click(editor, "mission-program-beta", false);
        step("mission-program", cycle);
        if (editor.missionController.snapshot().selectedProgram != "beta") throw "Mission program selection did not change";
        select(1);
        step("mission-select", cycle);
        if (!HeadlessEditorProfile.showsText(editor, "Selected step 2")) throw "Mission selected detail is stale";
        install(editor);
        editor.workspace.setRoot(DockNode.Panel("mission"));
        step("mission-replaced", cycle);
        if (HeadlessEditorProfile.showsText(editor, "Selected step")) throw "Replacement mission kept old selection";
        if (!editor.simulation.rebuild(editor.session.sensors, editor.scene, editor.session))
          throw "Mission fixture simulation failed: " + editor.simulation.error;
        step("mission-prepared", cycle);
        var player = editor.simulation.missionPlayer("alpha");
        if (player == null) throw "Mission fixture player was not created";
        var ticks = 0;
        while (player.stepIndex < 14 && !player.finished && player.failure == null && ticks++ < 200) {
          editor.simulation.step();
          step("mission-progress", cycle);
        }
        if (player.failure != null || player.stepIndex < 14) throw "Mission fixture failed to advance: " + player.failure;
        if (editor.missionPanel.scroll.offsetY <= 0) throw "Mission did not follow the current step";
        var root = editor.ui.root, row:Null<RenderNode> = null;
        if (root != null) root.walk(node -> {
          if (row == null && node.semantics != null && node.semantics.role == AccessibilityRole.CollectionItem && node.resolved != null) {
            var clipped = node.resolved.clippedViewportBounds();
            if (clipped.width > 0 && clipped.height > 4) row = node;
          }
        });
        var scrollRow = row;
        if (scrollRow == null) throw "Mission has no visible scroll row";
        var resolved = scrollRow.resolved;
        if (resolved == null) throw "Mission scroll row has no geometry";
        var bounds = resolved.clippedViewportBounds();
        editor.ui.scroll(bounds.x + bounds.width / 2, bounds.y + bounds.height / 2, 0, -60);
        step("mission-manual-scroll", cycle);
        if (text("Follow current step") == null) throw "Mission manual scroll did not pause following";
        HeadlessEditorProfile.click(editor, "mission-follow", false);
        step("mission-follow", cycle);
        if (text("Follow current step") != null || !editor.missionPanel.following) throw "Mission did not resume following";
        while (!player.finished && player.failure == null && ticks++ < 300) {
          editor.simulation.step();
          step("mission-progress", cycle);
        }
        if (!player.finished || player.failure != null || !HeadlessEditorProfile.showsText(editor, "Mission complete"))
          throw "Mission completion is stale: " + player.failure;
        editor.simulation.reset();
        step("mission-reset", cycle);
        if (HeadlessEditorProfile.showsText(editor, "Mission complete")) throw "Reset mission kept completion status";
        editor.simulation.clear();
        editor.session.missions = [work("alpha", true)];
        if (!editor.simulation.rebuild(editor.session.sensors, editor.scene, editor.session))
          throw "Mission failure fixture did not prepare: " + editor.simulation.error;
        var failing = editor.simulation.missionPlayer("alpha");
        if (failing == null) throw "Mission failure fixture has no player";
        ticks = 0;
        while (failing.failure == null && ticks++ < 20) {
          editor.simulation.step();
          step("mission-timeout", cycle);
        }
        if (failing.failure == null || !HeadlessEditorProfile.showsText(editor, "Mission failed"))
          throw "Mission timeout failure is stale";
        editor.simulation.reset();
        step("mission-failure-reset", cycle);
        if (HeadlessEditorProfile.showsText(editor, "Mission failed")) throw "Reset mission kept its failure status";
        editor.simulation.clear();
        editor.session.missions = null;
        step("mission-cleared", cycle);
        if (text("This project has no mission.") == null) throw "Cleared mission kept old content";
      }
    } catch (error:Dynamic) {
      editor.dispose();
      throw error;
    }
    editor.dispose();
  }
}
