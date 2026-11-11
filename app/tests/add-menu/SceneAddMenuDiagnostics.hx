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
  /** Capture the real popup in an isolated temporary workspace, in either theme. */
  static function capture():Int {
    var options = new haxeon.ui.host.DesktopUiHostOptions();
    var dark = Sys.args().indexOf("--dark") >= 0;
    var output = "/tmp/materia-add-menu-" + (dark ? "dark-" : "light-") + Sys.getPid();
    FileSystem.createDirectory(output);
    options.title = "Materia Add menu capture " + Sys.getPid();
    options.width = 1000;
    options.height = 650;
    var app:Null<ReferenceEditorApp> = null;
    var menuFrames = 0;
    var host:Null<haxeon.ui.host.DesktopUiHostContext> = null;
    options.onFrameRendered = function() {
      var editor = app;
      if (editor == null) return;
      var root = editor.ui.root;
      if (root == null) return;
      if (editor.hierarchyAddVisible && menuCount(root) == 1) {
        // The host callback precedes presentation; let the first menu frame
        // reach the window before asking the window system for its pixels.
        menuFrames++;
        if (menuFrames < 2) {editor.invalidateView(); return;}
        var screenshot = output + "/frame.png";
        if (Sys.command("import", ["-window", options.title, screenshot]) != 0) throw "Menu screenshot failed";
        Sys.println("Menu screenshot: " + screenshot);
        var context = host;
        if (context != null) context.requestClose();
        return;
      }
      var trigger = find(root, "hierarchy-add");
      if (trigger == null || trigger.resolved == null) return;
      var bounds = trigger.globalBounds();
      editor.hierarchyAddX = bounds.x;
      editor.hierarchyAddY = bounds.y + bounds.height + 4;
      editor.hierarchyAddVisible = true;
      editor.invalidateView();
    };
    return haxeon.ui.host.DesktopUiHost.run(options, context -> {
      host = context;
      var editor = new ReferenceEditorApp(context.fonts, output + "/workspace.json",
        dark ? haxeon.ui.theme.Theme.dark() : haxeon.ui.theme.Theme.light(), null, context,
        null, null, null, true);
      editor.workspace.close("start");
      editor.workspace.activate("hierarchy");
      app = editor;
      return editor;
    });
  }
  public static function main():Int {
    if (Sys.args().indexOf("--capture-menu") >= 0) return capture();
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
          var metrics = editor.ui.frameMetrics;
          var row = {size: size, mode: mode, index: index, x: x, y: y, hit: hits,
            before: before, after: editor.hierarchyAddVisible, menuCount: menuCount(root),
            reused: metrics == null ? false : metrics.reusedSubmission,
            downMs: (t1-t0)*1000, intermediateFrameMs: (t2-t1)*1000, upMs: (t3-t2)*1000,
            frameMs: (t4-t3)*1000, inputAndFrameMs: (t4-t0)*1000};
          if (submitted) {
            var expectedMenus = editor.hierarchyAddVisible ? 1 : 0;
            if (row.menuCount != expectedMenus) throw 'Popup/state mismatch at ${size[0]} / $mode / $index';
            if (editor.hierarchyAddVisible) {
              var labels:Array<String> = [];
              root.walk(node -> {
                if (node.semantics != null && node.semantics.role == haxeon.ui.semantics.AccessibilityRole.MenuItem)
                  labels.push(node.semantics.label);
              });
              var expected = ["Rectangle", "Empty part", "Mounting plate", "L bracket", "Stock simulation", "Worker", "Import STEP part…"];
              if (labels.join("|") != expected.join("|")) throw "Unexpected creation menu: " + labels.join("|");
              var dividers = 0;
              root.walk(node -> {
                if (node.styleKey != null && StringTools.endsWith(node.styleKey, "-separator-spacing")) {
                  dividers++;
                  var line = node.children[0];
                  if (line.resolved == null || line.globalBounds().width < 100) throw "Menu divider collapsed";
                }
              });
              if (dividers != 2) throw "Expected two menu dividers";
            }
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
