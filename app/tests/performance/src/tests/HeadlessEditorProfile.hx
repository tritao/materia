package tests;

import app.Main.ReferenceEditorApp;
import app.EditorScene;
import app.SceneObjectData;
import haxe.Json;
import LayoutFrame;
import FontCollection;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.EditOperation;
import nativekit.ui.core.UiEventKind;
import nativekit.ui.core.UiKey;
import nativekit.ui.core.UiModifier;
import nativekit.ui.debug.UiFrameMetrics;
import nativekit.ui.semantics.AccessibilityRole;
import sys.FileSystem;
import sys.io.File;

private typedef ClickPhases = {
  final lookupSeconds:Float;
  final boundsSeconds:Float;
  final pointerDownSeconds:Float;
  final pointerUpSeconds:Float;
  final eventPhases:PointerUpPhases;
}

private typedef PointerUpPhases = {
  var hitTestSeconds:Float;
  var pointerDispatchSeconds:Float;
  var clickDispatchSeconds:Float;
  var clickCaptureSeconds:Float;
  var clickTargetSeconds:Float;
  var clickBubbleSeconds:Float;
  var hoverSeconds:Float;
}

private typedef InputProbe = {
  final seconds:Float;
  final phases:ClickPhases;
  final allocatedBefore:Float;
  final allocatedAfter:Float;
  final collectionsBefore:Float;
  final collectionsAfter:Float;
  final markBefore:Float;
  final markAfter:Float;
}

/** Replays real editor input through UiContext without a window or X server. */
class HeadlessEditorProfile {
  static var profileSpans = false;
  static function main():Int {
    try {
      if (Sys.args().length < 2 || Sys.args().length > 4)
        throw "Usage: headless-profile OUTPUT_DIR CYCLES [tab-inspector|tab-matrix] [HEAP_DUMP_PATH]";
      var output = Sys.args()[0];
      var cycles = Std.parseInt(Sys.args()[1]);
      if (cycles == null || cycles < 1) throw "CYCLES must be positive";
      var scenario = Sys.args().length >= 3 && (Sys.args()[2] == "tab-inspector" ||
        Sys.args()[2] == "tab-matrix" || Sys.args()[2] == "architecture") ? Sys.args()[2] : "tab-inspector";
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
    profileSpans = Sys.getEnv("HAXEON_PROFILE_SPANS") == "1";
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
            var input = measuredClick(editor, to, from + "->" + to);
            if (editor.workspace.activePanelId != to) throw "Tab did not activate: " + to;
            var name = from + "->" + to;
            submit(editor, frame, frames, name, cycle, input);
            action(actions, name, cycle);
          }
          if ((cycle + 1) % 20 == 0) retained.push(retainedCounts(editor, cycle + 1));
        }
      } else if (scenario == "architecture") {
        runArchitectureScenario(editor, frame, output, cycles, frames, actions);
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

  /** Exercises the expensive editor boundaries called out in the architecture plan. */
  static function runArchitectureScenario(editor:ReferenceEditorApp, frame:LayoutFrame,
      output:String, cycles:Int, frames:Array<String>, actions:Array<String>):Void {
    var started = Sys.time();
    editor.scene.createBracket();
    var bracketId = editor.scene.selectedId;
    for (cycle in 0...cycles)
      editor.scene.setBracketWallThickness(bracketId, 0.003 + (cycle % 7) * 0.0001);
    action(actions, "cad-parameter-edits", 0);
    submit(editor, frame, frames, "architecture:cad-parameter-edits", 0);
    var cadSeconds = Sys.time() - started;

    started = Sys.time();
    var relationships = editor.bimModel.allRelationships();
    if (relationships.length > 0) {
      var opening = relationships[0];
      for (cycle in 0...cycles) editor.session.applyBimEdit("Profile opening move", function() {
        editor.bimModel.moveOpening(opening.openingId, 800 + cycle * 2, 900);
      });
    }
    action(actions, "bim-opening-edits", 0);
    submit(editor, frame, frames, "architecture:bim-opening-edits", 0);
    var bimSeconds = Sys.time() - started;
    var records:Array<SceneObjectData> = [];
    for (index in 0...10000) {
      var moving = index < 500;
      records.push({id:"stress-" + index, label:"Stress object " + index,
        type:"rectangle", x:(index % 100) * 0.15, y:Std.int(index / 100) * 0.15, z:0.0,
        width:0.1, height:0.1, depth:0.1, collisionEnabled:true,
        dynamicBody:moving, mass:1.0, red:0.3, green:0.5, blue:0.7, visible:true});
    }
    started = Sys.time();
    var candidate = new EditorScene(records);
    var sceneLoadSeconds = Sys.time() - started;
    action(actions, "load-10k-scene", 0);

    started = Sys.time();
    candidate.setPosition("stress-9999", 0, 0.25);
    var nudgeSeconds = Sys.time() - started;
    action(actions, "single-object-nudge", 0);

    started = Sys.time();
    for (cycle in 0...cycles) for (index in 0...500)
      candidate.setPositionXY("stress-" + index,
        (index % 100) * 0.15 + cycle * 0.001, Std.int(index / 100) * 0.15);
    var movingSeconds = Sys.time() - started;
    action(actions, "move-500-objects", 0);

    var position = candidate.object("stress-9999");
    if (position == null) throw "Architecture scenario lost its selected stress object";
    var previousX = position.x;
    for (cycle in 0...1000) {
      var fromX = previousX;
      var nextX = 0.25 + (cycle % 2) * 0.01;
      candidate.document.apply(new EditOperation("Stress move " + cycle,
        function() candidate.setPositionXY("stress-9999", nextX, position.y),
        function() candidate.setPositionXY("stress-9999", fromX, position.y)));
      previousX = nextX;
    }
    if (candidate.document.history.undoCount < 1000)
      throw 'Expected 1,000 distinct moves, found ${candidate.document.history.undoCount}';
    started = Sys.time();
    for (_ in 0...1000) if (!candidate.document.undo()) throw "Undo history ended during stress scenario";
    var undo1000Seconds = Sys.time() - started;
    candidate.dispose();
    action(actions, "undo-1000-edits", 0);

    for (index in 1...64) {
      var link = editor.sensors.model.addLink(new robotkit.model.Link("Profile link " + index,
        "profile/link-" + index));
      editor.sensors.model.addJoint(new robotkit.model.Joint("Profile joint " + index,
        robotkit.model.JointType.Fixed, editor.sensors.model.links[index - 1], link,
        "profile/joint-" + index));
    }
    if (!editor.simulation.rebuild(editor.sensors, editor.scene))
      throw "64-link simulation profile could not build";
    for (cycle in 0...cycles) {
      editor.simulation.step();
      editor.simulation.capturePresentationSnapshot();
      submit(editor, frame, frames, "architecture:simulation-presentation", cycle);
    }
    action(actions, "simulation-presentation-64-links", 0);
    editor.simulation.stop();
    File.saveContent(output + "/architecture-summary.json", haxe.Json.stringify({
      sceneObjects: records.length, movingObjects: 500, articulatedLinks: 64,
      cycles: cycles, cadParameterSeconds: cadSeconds, bimOpeningSeconds: bimSeconds,
      sceneLoadSeconds: sceneLoadSeconds,
      singleObjectNudgeSeconds: nudgeSeconds, movingObjectEditsSeconds: movingSeconds,
      undo1000Seconds: undo1000Seconds}));
  }

  static function retainedCounts(editor:ReferenceEditorApp, cycle:Int):String {
    return Json.stringify({cycle: cycle, workspaceListeners: editor.workspace.listenerCount,
      state: editor.ui.stateStore.diagnosticCounts(),
      styles: editor.ui.buildContext.styleResolver.diagnosticCounts(),
      keys: editor.ui.buildContext.diagnosticKeyCounts()});
  }

  static function submit(editor:ReferenceEditorApp, frame:LayoutFrame,
      output:Array<String>, actionName:String, ?cycle:Int, ?input:InputProbe):Void {
    var subtrees:Array<Dynamic> = [];
    editor.ui.buildContext.buildProbe = function(name, preparationSeconds, buildSeconds, nodes) {
      subtrees.push({name: name, preparationSeconds: preparationSeconds,
        buildSeconds: buildSeconds, nodeCount: nodes});
    };
    if (profileSpans && input != null) haxeon.ProfileSpan.begin("tab/" + actionName + "/frame");
    var started = Sys.time();
    editor.submit(frame);
    editor.ui.buildContext.buildProbe = null;
    var elapsed = Sys.time() - started;
    if (profileSpans && input != null) haxeon.ProfileSpan.end("tab/" + actionName + "/frame");
    var metrics:UiFrameMetrics = cast editor.ui.frameMetrics;
    if (metrics == null) throw "UI frame metrics are unavailable";
    var allocatedBytes:Null<Float> = null;
    var gcCollections:Null<Float> = null;
    var gcMarkMicros:Null<Float> = null;
    var inputAllocatedBytes:Null<Float> = null;
    var inputGcCollections:Null<Float> = null;
    var inputGcMarkMicros:Null<Float> = null;
    var submitAllocatedBytes:Null<Float> = null;
    var submitGcCollections:Null<Float> = null;
    var submitGcMarkMicros:Null<Float> = null;
    if (input != null) {
      var allocatedAfter = hl.Gc.totalAllocated();
      var collectionsAfter = hl.Gc.collections();
      var markAfter = hl.Gc.markMicros();
      allocatedBytes = allocatedAfter - input.allocatedBefore;
      gcCollections = collectionsAfter - input.collectionsBefore;
      gcMarkMicros = markAfter - input.markBefore;
      inputAllocatedBytes = input.allocatedAfter - input.allocatedBefore;
      inputGcCollections = input.collectionsAfter - input.collectionsBefore;
      inputGcMarkMicros = input.markAfter - input.markBefore;
      submitAllocatedBytes = allocatedAfter - input.allocatedAfter;
      submitGcCollections = collectionsAfter - input.collectionsAfter;
      submitGcMarkMicros = markAfter - input.markAfter;
    }
    output.push(Json.stringify({
      frame: metrics.frameNumber, action: actionName, cycle: cycle,
      inputSeconds: input == null ? null : input.seconds,
      inputPhases: input == null ? null : input.phases,
      allocatedBytes: allocatedBytes, gcCollections: gcCollections,
      gcMarkMicros: gcMarkMicros,
      inputAllocatedBytes: inputAllocatedBytes,
      inputGcCollections: inputGcCollections,
      inputGcMarkMicros: inputGcMarkMicros,
      submitAllocatedBytes: submitAllocatedBytes,
      submitGcCollections: submitGcCollections,
      submitGcMarkMicros: submitGcMarkMicros,
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

  static function measuredClick(editor:ReferenceEditorApp, key:String, actionName:String):InputProbe {
    var allocatedBefore = hl.Gc.totalAllocated();
    var collectionsBefore = hl.Gc.collections();
    var markBefore = hl.Gc.markMicros();
    if (profileSpans) haxeon.ProfileSpan.begin("tab/" + actionName + "/input");
    var started = Sys.time();
    var phases = click(editor, key);
    var seconds = Sys.time() - started;
    if (profileSpans) haxeon.ProfileSpan.end("tab/" + actionName + "/input");
    return {seconds: seconds, phases: phases,
      allocatedBefore: allocatedBefore, allocatedAfter: hl.Gc.totalAllocated(),
      collectionsBefore: collectionsBefore, collectionsAfter: hl.Gc.collections(),
      markBefore: markBefore, markAfter: hl.Gc.markMicros()};
  }

  static function click(editor:ReferenceEditorApp, key:String):ClickPhases {
    var lookupStarted = Sys.time();
    var root = editor.ui.root;
    if (root == null) throw "UI tree is not ready";
    var node = findByStyleKey(root, key, key.indexOf("editor:") != 0);
    if (node == null || node.resolved == null)
      throw "Benchmark target is unavailable: " + key;
    var lookupDone = Sys.time();
    var bounds = node.resolved.clippedViewportBounds();
    if (bounds.width <= 0 || bounds.height <= 0)
      throw "Benchmark target is outside the viewport: " + key;
    var x = bounds.x + bounds.width / 2;
    var y = bounds.y + bounds.height / 2;
    var boundsDone = Sys.time();
    editor.ui.pointerDown(x, y, 0);
    var downDone = Sys.time();
    var eventPhases:PointerUpPhases = {hitTestSeconds: 0.0,
      pointerDispatchSeconds: 0.0, clickDispatchSeconds: 0.0,
      clickCaptureSeconds: 0.0, clickTargetSeconds: 0.0,
      clickBubbleSeconds: 0.0,
      hoverSeconds: 0.0};
    editor.ui.events.pointerUpProbe = function(stage, seconds) {
      switch (stage) {
        case "hitTestSeconds": eventPhases.hitTestSeconds = seconds;
        case "pointerDispatchSeconds": eventPhases.pointerDispatchSeconds = seconds;
        case "clickDispatchSeconds": eventPhases.clickDispatchSeconds = seconds;
        case "clickCaptureSeconds": eventPhases.clickCaptureSeconds = seconds;
        case "clickTargetSeconds": eventPhases.clickTargetSeconds = seconds;
        case "clickBubbleSeconds": eventPhases.clickBubbleSeconds = seconds;
        case "hoverSeconds": eventPhases.hoverSeconds = seconds;
      }
    };
    editor.ui.pointerUp(x, y, 0);
    editor.ui.events.pointerUpProbe = null;
    var upDone = Sys.time();
    return {lookupSeconds: lookupDone - lookupStarted,
      boundsSeconds: boundsDone - lookupDone,
      pointerDownSeconds: downDone - boundsDone,
      pointerUpSeconds: upDone - downDone, eventPhases: eventPhases};
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
