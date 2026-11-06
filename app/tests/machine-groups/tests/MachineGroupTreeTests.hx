package tests;

import app.EditorScene;
import app.EditorSceneTree;
import app.SceneObjectData;
import app.SceneCodec;
import machinekit.assembly.MachineAssembly;
import FlexsplineHobber;
import BenchMill;
import BenchMillFrame;
import MillFramePreview;

/** Real generated membership through the editor, without tessellating machine geometry. */
class MachineGroupTreeTests {
  static var checks = 0;
  static final evidence:Array<String> = [];
  static function expect(ok:Bool, message:String):Void {
    checks++;
    if (!ok) throw message;
  }
  static function records(machine:MachineAssembly):Array<SceneObjectData> return [for (member in machine.components()) {
    id:"project:" + member.id, label:member.id, type:"rectangle", x:0.0, y:0.0, z:0.0,
    width:1.0, height:1.0, depth:1.0, collisionEnabled:false, dynamicBody:false, mass:1.0,
    red:0.5, green:0.5, blue:0.5, visible:true
  }];
  static function isolatedRoundTrip(scene:EditorScene, definition:materia.assembly.AssemblyDefinition, members:Array<String>, label:String):Void {
    var saved = new EditorScene(SceneCodec.decode(SceneCodec.encode(scene)));
    try {
      var savedTree = new EditorSceneTree(saved, definition);
      for (item in saved.items()) expect(saved.isVisible(item.id) == (members.indexOf(item.id) >= 0), label + " persists " + item.id);
    } catch (error:Dynamic) {saved.dispose(); throw error;}
    saved.dispose();
  }
  static function verify(machine:MachineAssembly, name:String, group:String, fixed:String, moving:String):Void {
    expect(machine.checkGroups().items.length == 0, name + " every member has functional ownership");
    var model = new cadkit.modeling.AssemblyModel("mm");
    machine.addTo(model, "");
    var definition = model.definition(name.toLowerCase().split(" ").join("-"));
    machinekit.assembly.AssemblyPreview.preserveOwnership(definition, machine.definition());
    var scene = new EditorScene(records(machine)), tree = new EditorSceneTree(scene, definition);
    try {
      var key = tree.assemblyKey(group), members = tree.visibilityMembers(key);
      expect(members.indexOf("project:" + fixed) >= 0 && members.indexOf("project:" + moving) >= 0, name + " group contains fixed and moving parts");
      tree.toggleVisibility(key);
      expect(!scene.isVisible("project:" + fixed) && !scene.info("project:" + moving).visible(), name + " group hide reaches viewport visibility");
      var restored = new EditorScene(SceneCodec.decode(SceneCodec.encode(scene)));
      try {
        var restoredTree = new EditorSceneTree(restored, definition);
        expect(!restored.isVisible("project:" + moving) && !restoredTree.visibilityState(key).own, name + " hidden group persists");
        restoredTree.toggleVisibility(key);
        expect(restored.isVisible("project:" + fixed) && restored.isVisible("project:" + moving), name + " group is independently showable after load");
      } catch (error:Dynamic) {restored.dispose(); throw error;}
      restored.dispose();
      tree.showAll(); tree.toggleVisibility(tree.assemblyKey()); tree.selectKey(key); tree.isolateSelected();
      for (item in scene.items()) expect(scene.isVisible(item.id) == (members.indexOf(item.id) >= 0), name + " isolate " + item.id);
      isolatedRoundTrip(scene, definition, members, name + " group isolate through hidden root");
      evidence.push(name + " functional tree");
      function list(parent:String, depth:Int):Void {
        for (index in 0...tree.childCount(parent)) {
          var child = tree.childKeyAt(parent, index);
          if (!tree.isGroup(child)) continue;
          evidence.push(StringTools.lpad("", " ", depth * 2) + tree.visibilityLabel(child) + " (" + tree.visibilityMembers(child).length + ")");
          list(child, depth + 1);
        }
      }
      list("scene", 0);
      for (selection in machine.sets()) {
        var set = tree.setKey(selection.id);
        expect(tree.visibilityMembers(set).length == selection.members.length, name + " editor preserves derived membership " + selection.id);
        evidence.push("  " + selection.label + ": " + selection.members.join(", "));
      }
      var selection = machine.sets().filter(s -> s.id == "kind:Purchased")[0], set = tree.setKey(selection.id);
      tree.showAll(); tree.toggleVisibility(tree.setKey("all")); tree.selectKey(set); tree.isolateSelected();
      isolatedRoundTrip(scene, definition, tree.visibilityMembers(set), name + " set isolate through hidden root");
    } catch (error:Dynamic) {scene.dispose(); throw error;}
    scene.dispose();
  }
  public static function main():Void {
    verify(new FlexsplineHobber(), "Hobber", "X axis", "railXLeft", "Xbridge");
    var reference = new BenchMill(false, false);
    verify(reference, "Legacy mill", "X axis", "railXFront", "table");
    var layout = MillFramePreview.layout(reference), z = MillFramePreview.zLayout(reference, layout);
    var xy = new machinekit.milling.MillXYStageLayout(layout, z.spindleY, BenchMill.RAIL);
    var frame = BenchMillFrame.create(layout, z, xy);
    verify(MillFramePreview.concept(reference, frame), "Framed mill", "XY stage/xyStage/X axis", "xyStage/railXFront", "xyStage/table");
    var report = evidence.join("\n") + "\n";
    if (Sys.args().length > 0) sys.io.File.saveContent(Sys.args()[0], report);
    else Sys.println(report);
    Sys.println("Real-machine editor tree: " + checks + " checks passed");
  }
}
