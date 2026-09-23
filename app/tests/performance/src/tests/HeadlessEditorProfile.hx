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
import nativekit.ui.semantics.AccessibilityRole;
import sys.FileSystem;
import sys.io.File;

/** Replays real editor input through UiContext without a window or X server. */
class HeadlessEditorProfile {
  static function main():Int {
    try {
      if (Sys.args().length < 2 || Sys.args().length > 4)
        throw "Usage: headless-profile OUTPUT_DIR CYCLES [tab-inspector|tab-matrix] [HEAP_DUMP_PATH]";
      var output = Sys.args()[0];
      var cycles = Std.parseInt(Sys.args()[1]);
      if (cycles == null || cycles < 1) throw "CYCLES must be positive";
      var scenario = Sys.args().length >= 3 && (Sys.args()[2] == "tab-inspector" ||
        Sys.args()[2] == "tab-matrix") ? Sys.args()[2] : "tab-inspector";
      if (Sys.args().length == 4 && scenario == "tab-inspector" && Sys.args()[2] != "tab-inspector")
        throw "Unknown headless scenario: " + Sys.args()[2];
      var heapDumpPath = Sys.args().length == 4 ? Sys.args()[3] :
        Sys.args().length == 3 && Sys.args()[2] != scenario ? Sys.args()[2] : null;
      if (!FileSystem.exists(output)) FileSystem.createDirectory(output);
      run(output, cycles, scenario, heapDumpPath);
      return 0;
    } catch (error:Dynamic) {
      Sys.println("Headless editor profile failed: " + Std.string(error));
      return 1;
    }
  }

  static function run(output:String, cycles:Int, scenario:String, heapDumpPath:Null<String>):Void {
    var fontPath = "../uikit/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf";
    if (!FileSystem.exists(fontPath)) throw "Benchmark font is unavailable: " + fontPath;
    var fonts = FontCollection.create();
    fonts.add(fontPath);
    var editor = new ReferenceEditorApp(fonts, output + "/workspace.json");
    var frame = new LayoutFrame(1320.0, 900.0);
    var frames:Array<String> = [];
    var actions:Array<String> = [];
    var retained:Array<String> = [];
    try {
      submit(editor, frame, frames, "initial");
      if (scenario == "tab-matrix") {
        var groups = [["hierarchy", "sensors"],
          ["viewport", "perspective", "console", "telemetry"]];
        for (cycle in 0...cycles) {
          for (group in groups) for (from in group) for (to in group) {
            if (from == to) continue;
            if (editor.workspace.activePanelId != from) {
              click(editor, from);
              submit(editor, frame, frames, "setup:" + from, cycle);
            }
            var allocatedBefore = hl.Gc.totalAllocated();
            var collectionsBefore = hl.Gc.collections();
            var markBefore = hl.Gc.markMicros();
            var inputStarted = Sys.time();
            click(editor, to);
            var inputSeconds = Sys.time() - inputStarted;
            if (editor.workspace.activePanelId != to) throw "Tab did not activate: " + to;
            var name = from + "->" + to;
            submit(editor, frame, frames, name, cycle, inputSeconds,
              allocatedBefore, collectionsBefore, markBefore);
            action(actions, name, cycle);
          }
          if ((cycle + 1) % 20 == 0) retained.push(retainedCounts(editor, cycle + 1));
        }
      } else {
        for (cycle in 0...cycles) {
          click(editor, "sensors");
          if (editor.workspace.activePanelId != "sensors") throw "Sensors tab did not activate";
          submit(editor, frame, frames, "sensors");
          action(actions, "sensors", cycle);
          click(editor, "hierarchy");
          if (editor.workspace.activePanelId != "hierarchy") throw "Hierarchy tab did not activate";
          submit(editor, frame, frames, "hierarchy");
          action(actions, "hierarchy", cycle);
          if ((cycle + 1) % 20 == 0) retained.push(retainedCounts(editor, cycle + 1));
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
      }
      File.saveContent(output + "/frame-timeline.jsonl", frames.join("\n") + "\n");
      File.saveContent(output + "/actions.jsonl", actions.join("\n") + "\n");
      File.saveContent(output + "/retained.jsonl", retained.join("\n") + "\n");
      File.saveContent(output + "/app-state.json", Json.stringify(editor.diagnosticState()));
      if (heapDumpPath != null) {
        frames.resize(0);
        actions.resize(0);
        retained.resize(0);
        hl.Gc.major();
        hl.Gc.dump(cast haxe.io.Bytes.ofString(heapDumpPath).getData());
      }
    } catch (error:Dynamic) {
      editor.dispose();
      fonts.dispose();
      throw error;
    }
    editor.dispose();
    fonts.dispose();
  }

  static function retainedCounts(editor:ReferenceEditorApp, cycle:Int):String {
    return Json.stringify({cycle: cycle, workspaceListeners: editor.workspace.listenerCount,
      state: editor.ui.stateStore.diagnosticCounts(),
      styles: editor.ui.buildContext.styleResolver.diagnosticCounts(),
      keys: editor.ui.buildContext.diagnosticKeyCounts()});
  }

  static function submit(editor:ReferenceEditorApp, frame:LayoutFrame,
      output:Array<String>, actionName:String, ?cycle:Int, ?inputSeconds:Float,
      ?allocatedBefore:Float, ?collectionsBefore:Float, ?markBefore:Float):Void {
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
    var allocatedBytes = allocatedBefore == null ? null : hl.Gc.totalAllocated() - allocatedBefore;
    var gcCollections = collectionsBefore == null ? null : hl.Gc.collections() - collectionsBefore;
    var gcMarkMicros = markBefore == null ? null : hl.Gc.markMicros() - markBefore;
    output.push(Json.stringify({
      frame: metrics.frameNumber, action: actionName, cycle: cycle,
      inputSeconds: inputSeconds,
      allocatedBytes: allocatedBytes, gcCollections: gcCollections,
      gcMarkMicros: gcMarkMicros,
      startedAtSeconds: started,
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
    var node = findByStyleKey(root, key, key.indexOf("editor:") != 0);
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

  static function findByStyleKey(node:RenderNode, key:String, tab:Bool):Null<RenderNode> {
    if (node.styleKey == key && (!tab || node.semantics != null &&
        node.semantics.role == AccessibilityRole.Tab)) return node;
    for (child in node.children) {
      var found = findByStyleKey(child, key, tab);
      if (found != null) return found;
    }
    return null;
  }

  static function action(output:Array<String>, name:String, cycle:Int):Void {
    output.push(Json.stringify({action: name, cycle: cycle,
      timeSeconds: Sys.time()}));
  }
}
