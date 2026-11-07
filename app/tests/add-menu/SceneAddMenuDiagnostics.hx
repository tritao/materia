import app.Main.ReferenceEditorApp;
import haxeon.platform.Runtime;
import haxeon.ui.FontCollection;
import haxeon.ui.LayoutFrame;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiEventKind;
import haxeon.ui.core.UiKey;
import sys.FileSystem;
import sys.io.File;

/** Repeated real Add-button input with hit paths, visibility and frame timings. */
@:access(app.Main.ReferenceEditorApp)
class SceneAddMenuDiagnostics {
  static function find(root:RenderNode, key:String):Null<RenderNode> {
    if (root.styleKey == key) return root;
    for (child in root.children) { var found = find(child, key); if (found != null) return found; }
    return null;
  }
  static function menuCount(root:RenderNode):Int {
    var count = 0;
    root.walk(node -> {if (node.semantics != null && node.semantics.label == "Menu") count++;});
    return count;
  }
  public static function main():Int {
    var runtime = Runtime.start();
    var rootDir = Sys.getCwd();
    while (!FileSystem.exists(rootDir + "/app/src/Main.hx")) rootDir = haxe.io.Path.directory(rootDir);
    var fonts = FontCollection.create();
    fonts.add(cast (rootDir + "/haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf"));
    var output = "/tmp/materia-add-menu-" + Sys.getPid();
    FileSystem.createDirectory(output);
    var rows:Array<Dynamic> = [];
    for (size in [[1320.0, 900.0], [1000.0, 650.0], [1920.0, 1080.0]]) {
      var editor = new ReferenceEditorApp(fonts, output + '/workspace-${size[0]}.json', null, null, null, null, null, null, true);
      editor.workspace.close("start");
      editor.workspace.activate("hierarchy");
      var frame = new LayoutFrame(size[0], size[1]);
      var root = editor.submit(frame);
      var trigger = find(root, "hierarchy-add");
      if (trigger == null || trigger.resolved == null) throw "Missing Add trigger";
      var bounds = trigger.resolved.clippedViewportBounds();
      var x = bounds.x + bounds.width / 2, y = bounds.y + bounds.height / 2;
      // Repeated clicks on the same Add button with and without a frame between press/release.
      for (mode in ["repeat", "split-press", "escape-reopen", "burst"]) {
        editor.hierarchyAddVisible = false;
        editor.submit(frame);
        for (index in 0...8) {
          if (mode == "escape-reopen") {editor.ui.key(UiEventKind.KeyDown, UiKey.Escape); editor.submit(frame);}
          var before = editor.hierarchyAddVisible;
          var path = editor.ui.events.hitTestPath(x, y);
          var hits = [for (node in path) node.styleKey == null ? node.styleType : node.styleKey];
          var t0 = Sys.time();
          editor.ui.pointerDown(x, y, 0);
          var t1 = Sys.time();
          if (mode == "split-press") editor.submit(frame);
          var t2 = Sys.time();
          editor.ui.pointerUp(x, y, 0);
          var t3 = Sys.time();
          var submitted = mode != "burst" || index % 2 == 1;
          if (submitted) root = editor.submit(frame);
          var t4 = Sys.time();
          var metrics = editor.ui.lastFrameMetrics;
          var row = {size: size, mode: mode, index: index, x: x, y: y, hit: hits,
            before: before, after: editor.hierarchyAddVisible, menuCount: menuCount(root),
            reused: metrics == null ? false : metrics.reusedSubmission,
            downMs: (t1-t0)*1000, intermediateFrameMs: (t2-t1)*1000, upMs: (t3-t2)*1000,
            frameMs: (t4-t3)*1000, inputAndFrameMs: (t4-t0)*1000};
          if (submitted) {
            var expectedMenus = editor.hierarchyAddVisible ? 1 : 0;
            if (row.menuCount != expectedMenus) throw 'Popup/state mismatch at ${size[0]} / $mode / $index';
            if (!editor.hierarchyAddVisible) {
              var closedHit = editor.ui.events.hitTestPath(x, y);
              var reachesAdd = false;
              for (node in closedHit) if (node.styleKey == "hierarchy-add") reachesAdd = true;
              if (!reachesAdd) throw 'Dismissed menu still intercepts Add at ${size[0]} / $mode / $index';
            }
          }
          rows.push(row);
          Sys.println(haxe.Json.stringify(row));
        }
      }
      editor.dispose();
    }
    File.saveContent(output + "/measurements.json", haxe.Json.stringify(rows));
    Sys.println('Scene Add: ${rows.length} clicks passed; measurements: $output/measurements.json');
    fonts.dispose();
    runtime.dispose();
    return 0;
  }
}
