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
    var root = submit();
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
