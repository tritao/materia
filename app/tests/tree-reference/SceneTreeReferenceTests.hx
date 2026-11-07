import app.EditorScene;
import app.EditorSceneTree;
import app.SceneObjectData;
import app.editor.SceneTreeReference;
import cadkit.modeling.AssemblyModel;
import haxeon.platform.Runtime;
import haxeon.ui.FontCollection;
import haxeon.ui.LayoutFrame;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiContext;
import haxeon.ui.semantics.AccessibilityRole;
import haxeon.ui.widgets.overlays.Menu;

class SceneTreeReferenceTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void { checks++; if (!ok) throw message; }
  static function find(root:RenderNode, label:String):RenderNode {
    var found:Null<RenderNode> = null;
    root.walk(node -> {
      if (node.semantics != null && node.semantics.role == AccessibilityRole.MenuItem && node.semantics.label == label) found = node;
    });
    if (found == null) throw "Missing menu action: " + label;
    return found;
  }
  public static function main():Int {
    var runtime = Runtime.start();
    var records:Array<SceneObjectData> = [for (id in ["bolt-a", "bolt-b"]) {
      id:"project:" + id, label:"M6 bolt", type:"rectangle", x:0.0, y:0.0, z:0.0, width:1.0, height:1.0,
      depth:1.0, collisionEnabled:false, dynamicBody:false, mass:1.0, red:0.5, green:0.5, blue:0.5, visible:true
    }];
    var scene = new EditorScene(records);
    var model = new AssemblyModel("m");
    model.add("bolt-a"); model.add("bolt-b");
    var definition = model.definition("fixture");
    for (occurrence in definition.occurrences) occurrence.includePath = "Hardware";
    scene.sourceMap = {features:[{id:"bolt-seat", label:"Bolt seat", occurrence:"bolt-a", faces:[], whole:true}], members:[]};
    var tree = new EditorSceneTree(scene, definition);
    tree.canChangeVisibility = () -> false;
    tree.selectKey("project:bolt-a");
    var before = haxe.Json.stringify(scene.records()), history = scene.document.history.undoCount;
    var selection = tree.selectionKey();
    var reference = SceneTreeReference.capture(tree, selection, Sys.getCwd() + "/machinekit/example/materia.project.json", "machine");
    var duplicate = SceneTreeReference.capture(tree, "project:bolt-b");
    check(reference.name == "M6 bolt" && duplicate.name == reference.name, "fixture must contain repeated names");
    check(reference.key != duplicate.key && reference.text().indexOf("Key: project:bolt-a") >= 0, "reference lost occurrence identity");
    check(reference.text().indexOf("Path: Scene / Fixture / Hardware / M6 bolt") >= 0, "reference lost authored hierarchy");
    check(reference.text().indexOf("Project: machinekit/example/materia.project.json") >= 0, "project path is not portable within this checkout");
    check(reference.text().indexOf("Variant: machine") >= 0, "reference lost the selected project variant");
    var group = SceneTreeReference.capture(tree, tree.assemblyKey("Hardware"));
    check(group.name == "Hardware" && group.key == "assembly:fixture:Hardware", "group reference is not exact");
    var source = SceneTreeReference.capture(tree, "source:bolt-seat");
    check(source.name == "Bolt seat" && source.path.join(" / ") == "Scene / Fixture / Hardware / M6 bolt / Bolt seat", "source feature lost its owning occurrence");
    tree.setFilter("not-a-match");
    check(SceneTreeReference.capture(tree, selection).path.join(" / ") == reference.path.join(" / "), "search changed the copied path");
    var root = SceneTreeReference.capture(tree, "scene");
    check(root.name == "Scene" && root.path.join(" / ") == "Scene", "scene root cannot be referenced");
    check(duplicate.text().indexOf("Project:") < 0 && duplicate.text().indexOf("Variant:") < 0, "absent context became invented metadata");
    check(haxe.Json.stringify(scene.records()) == before && scene.document.history.undoCount == history && tree.selectionKey() == selection,
      "capturing references changes selection, document data or undo history");
    // The menu uses a snapshot, so a later selection must not redirect its copy actions.
    tree.selectKey("project:bolt-b");
    var copied:Array<String> = [];
    var actions = SceneTreeReference.copyItems(reference, text -> copied.push(text));
    check(actions[0].enabled && actions[1].enabled, "read-only copy actions are disabled");
    var fonts = FontCollection.create(); fonts.addSystemFallbacks();
    var ui = new UiContext(null, fonts);
    var menu = new Menu("reference-menu", actions, 10, 10);
    var menuRoot = ui.submit(menu, new LayoutFrame(400, 240));
    var copy = find(menuRoot, "Copy reference").globalBounds();
    ui.pointerDown(copy.x + 8, copy.y + 8, 0); ui.pointerUp(copy.x + 8, copy.y + 8, 0);
    check(copied.length == 1 && copied[0] == reference.text(), "routed menu click copied a different selection");
    actions[1].onSelect();
    check(copied.length == 2 && copied[1] == "M6 bolt", "Copy name included IDs or display decorations");
    check(scene.document.history.undoCount == history, "copy actions added undo entries");
    ui.dispose(); fonts.dispose(); scene.dispose(); runtime.dispose();
    Sys.println('PASS: $checks scene tree reference checks, including routed copy actions, duplicate names, groups and source features');
    return 0;
  }
}
