package app;

import app.editor.ExampleCatalog;

import haxeon.ui.FontFamily;
import app.Main.ReferenceEditorApp;
import haxe.CallStack;
import haxeon.ui.host.BrowserUiHost;
import haxeon.ui.host.BrowserUiHostOptions;
import haxeon.ui.host.BrowserUiHostOptions.BrowserUiFontAsset;
import haxeon.ui.host.BrowserUiHostSession;
import haxeon.ui.host.DesktopUiHostContext;
import haxeon.ui.host.UiHostSession.UiHostLifecycle;
import haxeon.ui.theme.Theme;

/**
 * Browser entry point for the reference editor. The page calls the exposed `configure`, then `main` once, then
 * `frame` from each requestAnimationFrame tick; the editor itself is the one `Main.open` hosts on the desktop.
 */
class MainWeb {
  static var width = 1280;
  static var height = 800;
  static var darkTheme = false;
  static var session:Null<BrowserUiHostSession>;
  static var editor:Null<ReferenceEditorApp>;

  /** Sets the initial canvas size in CSS pixels and the theme (0 = light, 1 = dark) before `main`. */
  @:expose public static function configure(canvasWidth:Int, canvasHeight:Int, theme:Int):Int {
    if (session != null || canvasWidth <= 0 || canvasHeight <= 0) return 1;
    width = canvasWidth;
    height = canvasHeight;
    darkTheme = theme == 1;
    return 0;
  }

  @:expose public static function main():Int {
    try {
      #if wasm
      // Files from earlier sessions first: the editor reads its settings and workspace while it starts.
      BrowserFiles.start();
      BrowserExamples.configure();
      var identity = MateriaWebFiles.materia_build_identity();
      if (identity.status != 0) throw "Browser build identity is unavailable";
      PreparedSceneCaches.configureBrowser(identity.data.toString());
      #end
      var options = new BrowserUiHostOptions();
      options.title = "Materia";
      options.onFrameRendered = function() { if (editor != null) editor.projectFrameRendered(); };
      options.width = width;
      options.height = height;
      options.fonts = [
        new BrowserUiFontAsset("IBMPlexSans-Regular", "assets/IBMPlexSans-Regular.ttf",
          "/assets/IBMPlexSans-Regular.ttf", FontFamily.Default),
        new BrowserUiFontAsset("NotoEmoji-Regular", "assets/NotoEmoji-Regular.ttf",
          "/assets/NotoEmoji-Regular.ttf", FontFamily.Emoji)
      ];
      var started = BrowserUiHost.start(options, function(context) {
        var host:DesktopUiHostContext = cast context;
        var created = new ReferenceEditorApp(host.fonts, null, darkTheme ? Theme.dark() : Theme.light(), null, host);
        if (created.preferences.showStartPage) created.showStartPage();
        editor = created;
        return created;
      });
      session = started;
      return started.state == UiHostLifecycle.Failed ? 2 : 0;
    } catch (error:Dynamic) {
      record(error);
      return 1;
    }
  }

  /** Advances the host and the editor; returns 1 while running, 0 once stopped, and a negative value on failure. */
  @:expose public static function frame(time:Float):Int {
    var active = session;
    if (active == null) return 0;
    try {
      var advanced = active.advance(time);
      if (advanced < 0) {
        var failure = active.error;
        Sys.println("materia: " + (failure == null ? "the UI host failed" : failure.toString() + "\n" + failure.stack));
        return advanced;
      }
      if (editor != null) editor.tick();
      return advanced;
    } catch (error:Dynamic) {
      record(error);
      return -1;
    }
  }

  /** Opens host-supplied artifact bytes through the ordinary dirty-document and progress workflow. */
  @:expose public static function openArtifact():Int {
    #if wasm
    var app = editor;
    if (app == null || app.documents.blocked()) return 1;
    try {
      var name = MateriaWebFiles.materia_artifact_name();
      var content = MateriaWebFiles.materia_artifact_content();
      if (name.status != 0 || content.status != 0 || content.data.length > 150000000) return 2;
      if (!ProjectSourceLoader.isPrebuilt(name.data.toString())) return 3;
      var path = BrowserFiles.storeArtifact(name.data.toString(), content.data);
      app.documents.requestOpenPath(path);
      return 0;
    } catch (error:Dynamic) { record(error); return 4; }
    #else
    return 1;
    #end
  }

  /** Commands whose availability the browser tests check. */
  static final REPORTED_COMMANDS = ["editor.new", "editor.undo", "editor.redo", "scene.create", "scene.delete", "sim.play", "sim.step",
    "sim.stop", "sim.reset"];

  /**
   * Prints one `materia-report` line of JSON to the page's console: the editor's mode, scene objects, command
   * availability and diagnostic state, and every visible widget with a style key or accessibility label, with its
   * accessibility role (haxeon.ui.semantics.AccessibilityRole) and value and its bounds in CSS pixels. Browser tests (`web/tools/tour.py`) find controls through it and check each step.
   */
  @:expose public static function report():Int {
    var app = editor;
    if (app == null) return 1;
    try {
      var widgets:Array<Dynamic> = [];
      var root = app.ui.root;
      if (root != null) collectWidgets(root, widgets);
      var state:Dynamic = null, stateError:Null<String> = null;
      try {
        state = app.diagnosticState();
      } catch (error:Dynamic) {
        stateError = Std.string(error);
      }
      var stored:Array<String> = [];
      #if wasm
      stored = storedFiles("/");
      #end
      Sys.println("materia-report " + haxe.Json.stringify({
        mode: app.mode.id,
        examples: [for (entry in ExampleCatalog.available()) entry.id],
        exampleFailure: @:privateAccess app.startFailure,
        projectJob: app.session.projectJob,
        robotMotionCount: app.session.robotMotions.length,
        dynamicParts: [for (record in app.scene.records()) if (record.dynamicBody == true) record.id],
        simulationRunning: app.simulation.isRunning(),
        simulationActive: app.simulation.isActive(),
        simulationError: app.simulation.error,
        collisions: app.collisionSummary,
        mission: app.simulation.missionPlayer() == null ? null : app.simulation.missionPlayer().progress(),
        objects: [for (record in app.scene.records()) {id: record.id, label: record.label, type: record.type, x: record.x, y: record.y}],
        selected: app.scene.selectedId,
        commands: [for (id in REPORTED_COMMANDS) {id: id, enabled: commandEnabled(app, id)}],
        state: state,
        stateError: stateError,
        files: stored,
        widgets: widgets
      }));
      return 0;
    } catch (error:Dynamic) {
      record(error);
      return 2;
    }
  }

  #if wasm
  /** Every file below `directory` in the guest's filesystem, which BrowserFiles keeps across page loads. */
  static function storedFiles(directory:String):Array<String> {
    var result:Array<String> = [];
    var names = runtime.MemoryFileSystem.readDirectory(directory);
    if (names == null) return result;
    for (name in names) {
      var path = (directory == "/" ? "" : directory) + "/" + name;
      if (runtime.MemoryFileSystem.isDirectory(path)) result = result.concat(storedFiles(path));
      else result.push(path);
    }
    return result;
  }
  #end

  static function commandEnabled(app:ReferenceEditorApp, id:String):Bool {
    var command = app.commands.get(id);
    if (command == null) return false;
    return command.isEnabled();
  }

  static function collectWidgets(node:haxeon.ui.core.RenderNode, into:Array<Dynamic>):Void {
    var resolved = node.resolved, semantics = node.semantics;
    var label:Null<String> = null, value:Null<String> = null, role:Null<Int> = null;
    if (semantics != null) {
      label = semantics.label;
      value = semantics.value;
      role = semantics.role;
    }
    if (resolved != null && (node.styleKey != null || label != null)) {
      var bounds = resolved.clippedViewportBounds();
      if (bounds.width > 0 && bounds.height > 0)
        into.push({key: node.styleKey, label: label, role: role, value: value, x: bounds.x, y: bounds.y,
          width: bounds.width, height: bounds.height, enabled: node.enabled});
    }
    for (child in node.children) collectWidgets(child, into);
  }

  /** Failures go to the page's console; `main` and `frame` also report them through their result. */
  static function record(error:Dynamic):Void {
    Sys.println("materia: " + Std.string(error) + "\n" + CallStack.toString(CallStack.exceptionStack(true)));
  }
}
