package tests;

import app.Main.ReferenceEditorApp;
import app.MachineControlModel;
import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames;
import haxeon.ui.LayoutFrame;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiEventKind;
import haxeon.ui.core.UiKey;
import haxeon.ui.docking.DockNode;

/** Small kinematic fixture: no source generator, CAD build or simulation worker. */
@:access(app.Main.ReferenceEditorApp)
@:access(tests.HeadlessEditorProfile) class MachineControlRetention {
  static function install(editor:ReferenceEditorApp):Void {
    var model = new AssemblyModel("mm"), centers:Map<String, Array<Float>> = [];
    var ids = ["base", "slide", "rotor"];
    for (id in ids) {
      model.add(id);
      model.connector(id, "axis", AssemblyFrames.identity());
      centers.set(id, [0, 0, 0]);
    }
    model.mateOnAxis("x", "prismatic", "base", "axis", "slide", "axis", {x: 1, y: 0, z: 0}, 0,
      {lower: -100, upper: 100, velocity: null, effort: null});
    model.mateOnAxis("spindle/rotor-turn", "continuous", "base", "axis", "rotor", "axis", {x: 0, y: 0, z: 1}, 0,
      {lower: null, upper: null, velocity: null, effort: null, detents: [0, Math.PI / 2, Math.PI]});
    var data:Array<app.SceneObjectData> = [for (id in ids) {
      id: "project:" + id, label: id, type: "rectangle", x: 0.0, y: 0.0, z: 0.0, width: 0.01, height: 0.01,
      depth: 0.01, collisionEnabled: false, dynamicBody: false, mass: 1.0, red: 0.5, green: 0.5, blue: 0.5, visible: true
    }];
    editor.session.openGeneratedScene(data, null, null, model.definition("retained-machine"), null, centers, 0.001);
    editor.documentChanged();
  }

  public static function run(fonts:haxeon.ui.FontCollection, output:String, cycles:Int, frames:Array<String>):Void {
    var editor = new ReferenceEditorApp(fonts, output + "/machine-workspace.json", null, null, null, null, null, null, true);
    var frame = new LayoutFrame(900, 1100);
    function step(action:String, cycle:Int):Void {
      editor.invalidateView();
      try HeadlessEditorProfile.submit(editor, frame, frames, action, cycle)
      catch (error:Dynamic) throw "Machine retention " + action + "/" + cycle + ": " + Std.string(error);
    }
    function target(key:String):RenderNode {
      var root = editor.ui.root;
      var found = root == null ? null : HeadlessEditorProfile.findByStyleKey(root, key, false);
      if (found == null) throw "Missing machine target: " + key;
      return found;
    }
    function angle():RenderNode {
      var found:Null<RenderNode> = null, root = editor.ui.root;
      if (root != null) root.walk(node -> {
        if (node.semantics != null && node.semantics.label == "Spindle rotation scrubber") found = node;
      });
      if (found == null) throw "Missing machine scrubber";
      return found;
    }
    function position(id:String):Float {
      for (axis in MachineControlModel.axes(editor.session)) if (axis.id == id) return axis.value;
      throw "Missing machine axis " + id;
    }
    function near(actual:Float, expected:Float, message:String):Void {
      if (!Math.isFinite(actual) || Math.abs(actual - expected) > 0.01) throw message + ": " + actual + " != " + expected;
    }
    function fieldValue():Float {
      var semantics = target("machine-axis-value:x").semantics;
      if (semantics == null || semantics.value == null) throw "Missing machine position field value";
      return Std.parseFloat(semantics.value);
    }
    function enter(value:String, cycle:Int):Void {
      var field = target("machine-axis-value:x");
      if (!editor.ui.accessibilityAction(field.id.value, 4, value, 0, 0, 0)) throw "Machine field rejected SetValue";
      step("machine-draft", cycle);
      editor.ui.focusWidget(target("machine-axis-value:x").id);
      editor.ui.key(UiEventKind.KeyDown, UiKey.Enter);
      editor.ui.key(UiEventKind.KeyUp, UiKey.Enter);
      step("machine-enter", cycle);
    }
    try {
      install(editor);
      editor.workspace.setRoot(DockNode.Panel("machine-control"));
      step("machine-initial", 0);
      for (cycle in 0...cycles) {
        var before = target("machine-save-initial-pose");
        step("machine-unchanged", cycle);
        if (editor.ui.verifyRetentionEvery == 0 && target("machine-save-initial-pose") != before)
          throw "Unchanged machine controls rebuilt";
        enter("not-a-number", cycle);
        if (!HeadlessEditorProfile.showsText(editor, "Enter a numeric axis position")) throw "Machine entry error is missing";
        enter(Std.string(10 + cycle), cycle);
        near(position("x"), 10 + cycle, "Machine numeric edit did not apply");
        near(fieldValue(), 10 + cycle, "Machine numeric field is stale");
        if (HeadlessEditorProfile.showsText(editor, "Enter a numeric axis position")) throw "Machine error did not clear";
        var history = editor.session.document.history.undoCount, start = position("x");
        var external = editor.session.beginAssemblyJointEdit("x");
        external.update(25);
        step("machine-external-preview", cycle);
        near(fieldValue(), 25, "External joint preview did not reach retained controls");
        external.cancel();
        step("machine-external-cancel", cycle);
        near(fieldValue(), start, "External preview cancellation is stale");
        var slider = target("machine-axis-slider:x"), bounds = slider.globalBounds();
        var x = bounds.x + 10, y = bounds.y + bounds.height / 2, width = bounds.width - 20;
        editor.ui.pointerDown(x + width * 0.65, y, 0);
        step("machine-slider-down", cycle);
        editor.ui.pointerMove(x + width * 0.75, y);
        step("machine-slider-move", cycle);
        if (editor.session.document.history.undoCount != history) throw "Machine slider preview added history";
        near(position("x"), 50, "Machine slider stopped following its retained listener");
        editor.ui.pointerUp(x + width * 0.75, y, 0);
        step("machine-slider-up", cycle);
        if (editor.session.document.history.undoCount != history + 1) throw "Machine slider did not commit exactly once";
        editor.session.document.undo();
        step("machine-undo", cycle);
        near(fieldValue(), start, "Machine undo field is stale");
        editor.session.document.redo();
        step("machine-redo", cycle);
        near(fieldValue(), 50, "Machine redo field is stale");
        HeadlessEditorProfile.click(editor, "machine-axis-free:spindle/rotor-turn", false);
        step("machine-free", cycle);
        var rotary = angle(), r = rotary.globalBounds(), initialAngle = position("spindle/rotor-turn");
        var rx = r.x + r.width / 3, ry = r.y + r.height / 2;
        editor.ui.pointerDown(rx, ry, 0);
        step("machine-angle-down", cycle);
        editor.ui.pointerMove(rx + 20, ry);
        step("machine-angle-move", cycle);
        editor.ui.pointerUp(rx + 20, ry, 0);
        step("machine-angle-up", cycle);
        near(position("spindle/rotor-turn"), initialAngle + 20, "Retained angle gesture did not turn freely");
        editor.ui.focusWidget(angle().id);
        editor.ui.key(UiEventKind.KeyDown, UiKey.Right);
        editor.ui.key(UiEventKind.KeyUp, UiKey.Right);
        step("machine-angle-key", cycle);
        near(position("spindle/rotor-turn"), initialAngle + 21, "Retained angle keyboard edit is stale");
        HeadlessEditorProfile.click(editor, "machine-axis-free:spindle/rotor-turn", false);
        step("machine-detents", cycle);
        HeadlessEditorProfile.click(editor, "machine-save-initial-pose", false);
        step("machine-save-pose", cycle);
        near(editor.session.initialJointCoordinate("x", 0), 50, "Initial pose did not update");
        // Locking a document cancels an active gesture before deciding whether the panel can be reused.
        slider = target("machine-axis-slider:x");
        bounds = slider.globalBounds();
        x = bounds.x + bounds.width * 0.6; y = bounds.y + bounds.height / 2;
        editor.ui.pointerDown(x, y, 0);
        step("machine-gesture-before-lock", cycle);
        var busy = editor.documents.busy;
        editor.documents.busy = () -> true;
        step("machine-locked", cycle);
        if (editor.machineControlPanel.dragging() || target("machine-axis-slider:x").enabled)
          throw "Locked machine kept its gesture or enabled controls";
        near(position("x"), 50, "Locking a machine did not cancel its preview");
        editor.documents.busy = busy;
        step("machine-unlocked", cycle);
        if (!target("machine-axis-slider:x").enabled) throw "Unlocked machine did not enable its controls";
        install(editor);
        editor.workspace.setRoot(DockNode.Panel("machine-control"));
        step("machine-replaced", cycle);
        near(fieldValue(), 0, "Machine document replacement retained its old draft or pose");
      }
    } catch (error:Dynamic) {
      editor.dispose();
      throw error;
    }
    editor.dispose();
  }
}
