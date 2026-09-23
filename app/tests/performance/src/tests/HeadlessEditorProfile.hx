package tests;

import app.Main.ReferenceEditorApp;
import haxe.Json;
import LayoutFrame;
import FontCollection;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.UiEventKind;
import nativekit.ui.core.UiKey;
import nativekit.ui.core.UiModifier;
import nativekit.ui.debug.UiFrameMetrics;
import sys.FileSystem;
import sys.io.File;

/** Replays real editor input through UiContext without a window or X server. */
class HeadlessEditorProfile {
  static function main():Int {
    try {
      if (Sys.args().length != 2) throw "Usage: headless-profile OUTPUT_DIR CYCLES";
      var output = Sys.args()[0];
      var cycles = Std.parseInt(Sys.args()[1]);
      if (cycles == null || cycles < 1) throw "CYCLES must be positive";
      if (!FileSystem.exists(output)) FileSystem.createDirectory(output);
      run(output, cycles);
      return 0;
    } catch (error:Dynamic) {
      Sys.println("Headless editor profile failed: " + Std.string(error));
      return 1;
    }
  }

  static function run(output:String, cycles:Int):Void {
    var fontPath = "../uikit/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf";
    if (!FileSystem.exists(fontPath)) throw "Benchmark font is unavailable: " + fontPath;
    var fonts = FontCollection.create();
    fonts.add(fontPath);
    var editor = new ReferenceEditorApp(fonts, output + "/workspace.json");
    var frame = new LayoutFrame(1320.0, 900.0);
    var frames:Array<String> = [];
    var actions:Array<String> = [];
    try {
      submit(editor, frame, frames, "initial");
      for (cycle in 0...cycles) {
        click(editor, "sensors");
        if (editor.workspace.activePanelId != "sensors") throw "Sensors tab did not activate";
        submit(editor, frame, frames, "sensors");
        action(actions, "sensors", cycle);
        click(editor, "hierarchy");
        if (editor.workspace.activePanelId != "hierarchy") throw "Hierarchy tab did not activate";
        submit(editor, frame, frames, "hierarchy");
        action(actions, "hierarchy", cycle);
      }
      var nameKey = "editor:" + editor.scene.selectedId + ":" +
        editor.scene.selectionRevision + ":name";
      click(editor, nameKey);
      submit(editor, frame, frames, "inspector-focus");
      action(actions, "inspector-focus", 0);
      editor.ui.key(UiEventKind.KeyDown, UiKey.A, UiModifier.Control);
      editor.ui.text(UiEventKind.TextInput, "Profile box");
      editor.ui.key(UiEventKind.KeyDown, UiKey.Enter);
      submit(editor, frame, frames, "rename");
      action(actions, "rename", 0);
      var box = editor.scene.object("box");
      if (box == null || box.label != "Profile box")
        throw "Inspector rename did not commit";
      File.saveContent(output + "/frame-timeline.jsonl", frames.join("\n") + "\n");
      File.saveContent(output + "/actions.jsonl", actions.join("\n") + "\n");
      File.saveContent(output + "/app-state.json", Json.stringify(editor.diagnosticState()));
    } catch (error:Dynamic) {
      editor.dispose();
      fonts.dispose();
      throw error;
    }
    editor.dispose();
    fonts.dispose();
  }

  static function submit(editor:ReferenceEditorApp, frame:LayoutFrame,
      output:Array<String>, actionName:String):Void {
    var subtrees:Array<Dynamic> = [];
    editor.ui.buildContext.buildProbe = function(name, preparationSeconds, buildSeconds, nodes) {
      subtrees.push({name: name, preparationSeconds: preparationSeconds,
        buildSeconds: buildSeconds, nodeCount: nodes});
    };
    var started = Sys.time();
    editor.submit(frame);
    editor.ui.buildContext.buildProbe = null;
    var elapsed = Sys.time() - started;
    var metrics:UiFrameMetrics = cast editor.ui.frameMetrics;
    if (metrics == null) throw "UI frame metrics are unavailable";
    output.push(Json.stringify({
      frame: metrics.frameNumber, action: actionName, startedAtSeconds: started,
      frameSeconds: elapsed, submitSeconds: metrics.submitSeconds,
      viewSeconds: metrics.viewSeconds,
      treeAndStyleSeconds: metrics.treeAndStyleSeconds,
      nativeLayoutSeconds: metrics.nativeLayoutSeconds,
      reconcileSeconds: metrics.reconcileSeconds,
      nodeCount: metrics.nodeCount,
      styleResolutions: metrics.styleResolutions,
      styleCacheHits: metrics.styleCacheHits,
      styleCacheMisses: metrics.styleCacheMisses,
      styleChangedNodes: metrics.styleChangedNodes,
      subtrees: subtrees
    }));
  }

  static function click(editor:ReferenceEditorApp, key:String):Void {
    var root = editor.ui.root;
    if (root == null) throw "UI tree is not ready";
    var node = findByStyleKey(root, key);
    if (node == null || node.resolved == null)
      throw "Benchmark target is unavailable: " + key;
    var bounds = node.resolved.clippedViewportBounds();
    if (bounds.width <= 0 || bounds.height <= 0)
      throw "Benchmark target is outside the viewport: " + key;
    var x = bounds.x + bounds.width / 2;
    var y = bounds.y + bounds.height / 2;
    editor.ui.pointerDown(x, y, 0);
    editor.ui.pointerUp(x, y, 0);
  }

  static function findByStyleKey(node:RenderNode, key:String):Null<RenderNode> {
    if (node.styleKey == key) return node;
    for (child in node.children) {
      var found = findByStyleKey(child, key);
      if (found != null) return found;
    }
    return null;
  }

  static function action(output:Array<String>, name:String, cycle:Int):Void {
    output.push(Json.stringify({action: name, cycle: cycle,
      timeSeconds: Sys.time()}));
  }
}
