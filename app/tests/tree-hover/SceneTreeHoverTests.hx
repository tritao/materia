import app.editor.SceneTreeRow;
import haxeon.platform.Runtime;
import haxeon.ui.FontCollection;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutFrame;
import haxeon.ui.LayoutStyle;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiContext;
import haxeon.ui.core.View;
import haxeon.ui.icons.IconName;
import haxeon.ui.semantics.AccessibilityRole;
import haxeon.ui.widgets.collections.TreeRootMetadata;
import haxeon.ui.widgets.collections.TreeView;
import haxeon.ui.widgets.collections.TreeViewModel;

private class SceneRows implements TreeViewModel {
  public function new() {}
  public function rootCount():Int return 1000;
  public function rootRange(start:Int, count:Int):Array<TreeRootMetadata>
    return [for (i in start...start + count) new TreeRootMetadata("group-" + i, false)];
  public function childCount(key:String):Int return 0;
  public function childKeyAt(key:String, index:Int):String return "";
  public function initiallyExpanded(key:String):Bool return false;
  public function estimatedExtent():Float return 28;
  public function extentIsUniform():Bool return true;
  public function extentAt(key:String):Float return 28;
  public function revision():Int return 1;
  public function buildItem(key:String):View return new SceneTreeRow(key, "Hobber " + key, IconName.Hierarchy,
    "Part occurrence: " + key, true, true, false, () -> {}, () -> {});
}

@:access(app.Main.ReferenceEditorApp)
class SceneTreeHoverTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;
  public static function main():Int {
    var runtime = Runtime.start();
    var fonts = FontCollection.create();
    fonts.addSystemFallbacks();
    var ui = new UiContext(null, fonts);
    ui.buildContext.environment.reducedMotion = true;
    var style = new LayoutStyle();
    style.width = LayoutAxis.fixed(420);
    style.height = LayoutAxis.fixed(420);
    var tree = new TreeView("scene", new SceneRows(), style);
    var frame = new LayoutFrame(420, 420);
    var builds = 0;
    var build = () -> { builds++; return tree; };
    var submit = () -> ui.submitCached(build, frame, "scene");
    var editor:Null<app.Main.ReferenceEditorApp> = null;
    for (arg in Sys.args()) if (StringTools.startsWith(arg, "--artifact=")) {
      var path = arg.substr(11);
      var host = new app.Main.ReferenceEditorApp(fonts, "/tmp/materia-hover-workspace-" + Std.random(1000000) + ".json",
        null, null, null, null, path);
      if (!host.workspace.isOpen("hierarchy")) host.workspace.open("hierarchy", "inspector");
      host.workspace.activate("hierarchy");
      host.treeModel.selectKey(host.scene.items()[0].id);
      host.invalidateView();
      ui.dispose();
      ui = host.ui;
      frame = new LayoutFrame(1320, 900);
      submit = () -> host.submit(frame);
      editor = host;
    }
    var dragHost = editor;
    if (Sys.args().indexOf("--drag") >= 0 && dragHost != null) {
      submit();
      frame.deltaSeconds = 1 / 60;
      var item = dragHost.scene.object("project:machine/Xbridge");
      if (item == null) item = dragHost.scene.object("project:Xbridge");
      if (item == null) throw "The drag benchmark requires a hobber artifact containing Xbridge";
      var origin = [item.x, item.y, item.z];
      var drag = dragHost.scene.beginAssemblyDrag(item.id, origin);
      if (drag == null) throw "Xbridge cannot be dragged";
      for (i in 0...40) {
        var start = Sys.time();
        drag.update(origin[0] + 0.01 * Math.sin(i * 0.2), origin[1], origin[2]);
        var moved = Sys.time();
        submit();
        var end = Sys.time();
        Sys.println('DRAG step=$i update_ms=${(moved-start)*1000} submit_ms=${(end-moved)*1000}');
      }
      drag.cancel();
      var restored = dragHost.scene.object(item.id);
      check(restored != null && Math.abs(restored.x - origin[0]) < 1e-9, "cancel restores Xbridge");
      var definition = dragHost.session.projectAssemblyDefinition;
      var record = dragHost.session.projectAssemblyState;
      if (definition == null || record == null) throw "Hobber has no assembly state";
      var state = new cadkit.modeling.AssemblyState(definition, record);
      var model = state.kinematicModel().model;
      var jointId = item.id == "project:Xbridge" ? "X" : "machine/X";
      var snapshot = kinematicskit.KinematicSnapshot.of(state.kinematicSeed());
      var axis = snapshot.jointWorldAxis(model.jointIndex(jointId));
      var committedDrag:Null<app.SceneAssemblyDrag> = null;
      var attempts:Array<String> = [];
      for (direction in [-1, 1]) {
        if (committedDrag != null) continue;
        var candidate = dragHost.scene.beginAssemblyDrag(item.id, origin);
        if (candidate == null) throw "Cannot restart Xbridge drag";
        candidate.update(origin[0] + direction * 0.001 * axis.x,
          origin[1] + direction * 0.001 * axis.y, origin[2] + direction * 0.001 * axis.z);
        attempts.push(candidate.message());
        if (candidate.following()) committedDrag = candidate;
        else candidate.cancel();
      }
      if (committedDrag == null) throw "No reachable Xbridge pull: " + attempts.join("; ");
      var heldItem = dragHost.scene.object(item.id);
      if (heldItem == null) throw "Xbridge disappeared during drag";
      var held = [heldItem.x, heldItem.y, heldItem.z];
      check(committedDrag.commit(), "release commits Xbridge as an edit");
      dragHost.session.document.undo();
      restored = dragHost.scene.object(item.id);
      check(restored != null && Math.abs(restored.x - origin[0]) < 1e-9, "undo restores Xbridge");
      dragHost.session.document.redo();
      restored = dragHost.scene.object(item.id);
      check(restored != null && Math.abs(restored.x - held[0]) < 1e-6
        && Math.abs(restored.y - held[1]) < 1e-6 && Math.abs(restored.z - held[2]) < 1e-6,
        "redo restores the dragged Xbridge pose");
      Sys.println("DRAG cancel/commit/undo/redo passed");
      dragHost.dispose(); fonts.dispose(); runtime.dispose();
      return 0;
    }
    var expansionOnly = Sys.args().indexOf("--expand") >= 0;
    var host = editor;
    if (expansionOnly && host != null) host.treeModel.selectKey("scene");
    frame.deltaSeconds = expansionOnly ? 1 / 60 : 0;
    var root = submit();
    if (expansionOnly && host != null) {
      var key = host.treeModel.assemblyKey();
      for (cycle in 0...12) {
        var target:Null<RenderNode> = null;
        root.walk(row -> {
          if (row.semantics != null && row.semantics.role == AccessibilityRole.TreeItem && row.semantics.label == key)
            row.walk(node -> {
              if (node.semantics != null && node.semantics.role == AccessibilityRole.Button
                  && (node.semantics.label == "Expand" || node.semantics.label == "Collapse")) target = node;
            });
        });
        var arrow = target;
        if (arrow == null) throw "Missing assembly disclosure";
        var bounds = arrow.globalBounds();
        var started = Sys.time();
        ui.pointerMove(bounds.x + 8, bounds.y + bounds.height / 2);
        ui.pointerDown(bounds.x + 8, bounds.y + bounds.height / 2, 0);
        ui.pointerUp(bounds.x + 8, bounds.y + bounds.height / 2, 0);
        var clicked = Sys.time();
        root = submit();
        var ended = Sys.time();
        var metrics = ui.frameMetrics;
        if (metrics == null) throw "missing expansion metrics";
        Sys.println('EXPANSION cycle=$cycle input_ms=${(clicked-started)*1000} submit_ms=${(ended-clicked)*1000} view_ms=${metrics.viewSeconds*1000} tree_ms=${metrics.treeAndStyleSeconds*1000} layout_ms=${metrics.nativeLayoutSeconds*1000} reconcile_ms=${metrics.reconcileSeconds*1000} nodes=${metrics.nodeCount} styles=${metrics.styleResolutions}');
        for (_ in 0...12) root = submit();
      }
      host.dispose(); fonts.dispose(); runtime.dispose();
      return 0;
    }
    var rows:Array<RenderNode> = [];
    root.walk(node -> {
      if (node.semantics != null && node.semantics.role == AccessibilityRole.TreeItem
          && node.resolved != null && node.globalBounds().y >= 0 && node.globalBounds().y < frame.height - 40) rows.push(node);
    });
    check(rows.length >= 8, "not enough scene rows: " + rows.length);
    var layouts = 0, reused = 0, resolutions = 0;
    var samples:Array<Float> = [];
    var before = builds;
    for (i in 0...160) {
      var bounds = rows[i % 8].globalBounds();
      var started = Sys.time();
      ui.pointerMove(bounds.x + Math.min(140, bounds.width / 2), bounds.y + 12);
      root = submit();
      if (i < 16) continue;
      samples.push((Sys.time() - started) * 1000);
      var metrics = ui.frameMetrics;
      if (metrics == null) throw "missing frame metrics";
      if (metrics.nativeLayoutSubmitted) layouts++;
      if (metrics.reusedSubmission) reused++;
      resolutions += metrics.styleResolutions;
    }
    samples.sort((a,b) -> a < b ? -1 : a > b ? 1 : 0);
    var metrics = ui.frameMetrics;
    if (metrics == null) throw "missing frame metrics";
    Sys.println('SCENE HOVER: nodes=${metrics.nodeCount} builds=${builds-before} layout=$layouts reused=$reused styleResolutions=$resolutions p50_ms=${samples[72]} p95_ms=${samples[136]}');
    if (Sys.args().indexOf("--baseline") < 0) {
      check(layouts == 0, "hovering scene rows submitted native layout");
      check(reused == samples.length, "hovering scene rows left the paint-only submission path");
    }
    if (editor != null) editor.dispose(); else ui.dispose();
    fonts.dispose(); runtime.dispose();
    return 0;
  }
}
