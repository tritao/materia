package tests;

import app.Main.ReferenceEditorApp;
import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames;
import haxeon.ui.LayoutFrame;
import haxeon.ui.core.RenderNode;
import haxeon.ui.docking.DockNode;
import haxeon.ui.semantics.AccessibilityRole;

/** Three small Cartesian slides run a finite program without CAD, stock or a project generator. */
@:access(app.Main.ReferenceEditorApp)
@:access(app.ProjectDocumentSession)
@:access(app.editor.CncPanel)
@:access(tests.HeadlessEditorProfile) class CncRetention {
  static function install(editor:ReferenceEditorApp):Void {
    var model = new AssemblyModel("m"), ids = ["base", "x", "y", "z"];
    var centers:Map<String, Array<Float>> = [];
    for (id in ids) {
      model.add(id);
      model.connector(id, "axis", AssemblyFrames.identity());
      centers.set(id, [0, 0, 0]);
    }
    model.ground("base");
    for (index in 0...3) model.mateOnAxis(ids[index + 1], "prismatic", ids[index], "axis", ids[index + 1], "axis",
      {x: index == 0 ? 1 : 0, y: index == 1 ? 1 : 0, z: index == 2 ? 1 : 0}, 0,
      {lower: -0.1, upper: 0.1, velocity: 0.1, effort: 100, acceleration: 1});
    var data:Array<app.SceneObjectData> = [for (id in ids) {
      id: "project:" + id, label: id, type: "rectangle", x: 0.0, y: 0.0, z: 0.0,
      width: 0.01, height: 0.01, depth: 0.01, collisionEnabled: false, dynamicBody: false,
      mass: 0.001, red: 0.5, green: 0.5, blue: 0.5, visible: true
    }];
    var physical:cadbridge.AssemblySimulationBridge.AssemblyPhysicalData = {metresPerUnit: 1, parts: [for (id in ids) {
      id: id, materialId: "steel", volume: 0.000001, centerOfMass: [0, 0, 0],
      inertia: [0.0000000000167, 0, 0, 0, 0.0000000000167, 0, 0, 0, 0.0000000000167], density: 1000,
      collisionHull: [-0.005, -0.005, -0.005, -0.005, -0.005, 0.005, -0.005, 0.005, -0.005, -0.005, 0.005, 0.005,
        0.005, -0.005, -0.005, 0.005, -0.005, 0.005, 0.005, 0.005, -0.005, 0.005, 0.005, 0.005]
    }]};
    editor.session.openGeneratedScene(data, null, null, model.definition("cnc-retention"), null, centers, 1, physical);
    editor.session.cncJob = {
      source: "G21 G90\n" + [for (index in 0...32) 'G53 G0 X${index % 2}\nG4 P0.01'].join("\n") + "\nM2",
      axes: ["x", "y", "z"], spindle: "z", channelScope: "fixture", workOffset: [0, 0, 0],
      tools: [], loop: false, stocks: [], stockSources: [], kinematicPreview: true
    };
    editor.documentChanged();
  }

  public static function run(fonts:haxeon.ui.FontCollection, output:String, cycles:Int, frames:Array<String>):Void {
    var editor = new ReferenceEditorApp(fonts, output + "/cnc-workspace.json", null, null, null, null, null, null, true);
    var frame = new LayoutFrame(900, 620);
    function step(action:String, cycle:Int):Void {
      editor.invalidateView();
      try HeadlessEditorProfile.submit(editor, frame, frames, action, cycle)
      catch (error:Dynamic) throw "CNC retention " + action + "/" + cycle + ": " + Std.string(error);
    }
    function text(value:String):Null<RenderNode> {
      var found:Null<RenderNode> = null, root = editor.ui.root;
      if (root != null) root.walk(node -> { if (node.layout.text == value) found = node; });
      return found;
    }
    function select(index:Int):Void {
      var found:Null<RenderNode> = null, root = editor.ui.root;
      if (root != null) root.walk(node -> {
        if (node.semantics != null && node.semantics.role == AccessibilityRole.CollectionItem
          && node.semantics.positionInSet == index + 1) found = node;
      });
      var row = found;
      if (row == null || row.resolved == null) throw "Missing visible CNC row " + index;
      var bounds = row.resolved.clippedViewportBounds();
      editor.ui.pointerDown(bounds.x + bounds.width / 2, bounds.y + bounds.height / 2, 0);
      editor.ui.pointerUp(bounds.x + bounds.width / 2, bounds.y + bounds.height / 2, 0);
    }
    try {
      editor.workspace.setRoot(DockNode.Panel("cnc"));
      step("cnc-empty", 0);
      if (text("This project has no machining or print program.") == null) throw "Missing empty CNC message";
      for (cycle in 0...cycles) {
        install(editor);
        editor.workspace.setRoot(DockNode.Panel("cnc"));
        step("cnc-preview", cycle);
        var before = text("CNC PROGRAM");
        step("cnc-unchanged", cycle);
        if (before == null || editor.ui.verifyRetentionEvery == 0 && text("CNC PROGRAM") != before)
          throw "Unchanged CNC preview rebuilt";
        select(2);
        step("cnc-select", cycle);
        if (editor.cncPanel.selectedLine != 3) throw "CNC preview selection did not change";
        var job = editor.session.cncJob;
        if (job == null) throw "Missing CNC fixture job";
        job.source = "; replacement\n" + job.source;
        step("cnc-source", cycle);
        if (editor.cncPanel.selectedLine != 0 || text("    1  ; replacement") == null)
          throw "CNC source replacement kept old rows or selection";
        if (!editor.simulation.rebuild(editor.session.sensors, editor.scene, editor.session))
          throw "CNC simulation failed: " + editor.simulation.error;
        step("cnc-prepared", cycle);
        var player = editor.simulation.toolpathPlayer();
        if (player == null) throw "Missing CNC fixture player";
        if (text("Ready · Play to start") != null) throw "Live CNC kept preview controls";
        var ticks = 0;
        while (player.currentLine < 22 && !player.finished() && player.failure == null && ticks++ < 1500) {
          editor.simulation.step();
          step("cnc-progress", cycle);
        }
        if (player.failure != null || player.currentLine < 22) throw "CNC fixture did not advance: " + player.failure;
        if (editor.cncPanel.scroll.offsetY <= 0) throw "CNC did not follow its executing line";
        HeadlessEditorProfile.click(editor, "cnc-hold", false);
        step("cnc-hold", cycle);
        var stopping = 0;
        while (!player.held() && player.failure == null && stopping++ < 200) {
          editor.simulation.step();
          step("cnc-stopping", cycle);
        }
        if (!player.held() || player.failure != null) throw "CNC feed hold did not settle: " + player.failure;
        HeadlessEditorProfile.click(editor, "cnc-resume", false);
        step("cnc-resume", cycle);
        if (player.held()) throw "CNC did not resume";
        var root = editor.ui.root;
        var speed = root == null ? null : HeadlessEditorProfile.findByStyleKey(root, "cnc-speed", false);
        if (speed == null || !editor.ui.accessibilityAction(speed.id.value, 4, "0.5", 0, 0, 0))
          throw "CNC speed override rejected SetValue";
        step("cnc-speed", cycle);
        if (player.speedOverride != 0.5) throw "CNC speed override is stale";
        editor.workspace.setRoot(DockNode.Panel("hierarchy"));
        step("cnc-hidden", cycle);
        for (_ in 0...10) editor.simulation.step();
        editor.workspace.setRoot(DockNode.Panel("cnc"));
        step("cnc-returned", cycle);
        if (text("CNC PROGRAM") == null) throw "CNC failed to remount";
        while (!player.finished() && player.failure == null && ticks++ < 3000) {
          editor.simulation.step();
          step("cnc-progress", cycle);
        }
        if (!player.finished() || player.failure != null || !HeadlessEditorProfile.showsText(editor, "Completed"))
          throw "CNC completion is stale: " + player.failure;
        editor.simulation.reset();
        step("cnc-reset", cycle);
        editor.simulation.clear();
        editor.session.cncJob = null;
        step("cnc-cleared", cycle);
        if (text("This project has no machining or print program.") == null) throw "Cleared CNC kept old source";
      }
    } catch (error:Dynamic) {
      editor.dispose();
      throw error;
    }
    editor.dispose();
  }
}
