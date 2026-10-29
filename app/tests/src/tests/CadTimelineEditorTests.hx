package tests;

import app.EditorScene;
import app.SceneCodec;
import cadkit.parametric.DocumentCodec;

/** A recorded pendant plate opens as an editable CAD part whose history rows can be rolled back and forward. */
class CadTimelineEditorTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void {
    checks++;
    if (!ok) throw message;
  }
  static function depth(scene:EditorScene):Float {
    var item = scene.object("plate");
    if (item == null) throw "The plate object is missing";
    return item.depth;
  }
  public static function main():Int {
    run();
    return 0;
  }
  public static function run():Void {
    var recorded = machinekit.pendant.PendantCaseDocument.plate(machinekit.pendant.ReferencePendant.pendantCase());
    var graph = DocumentCodec.encode(recorded.document);
    recorded.document.close();
    var scene = new EditorScene([{
      id: "plate",
      label: "Face plate",
      type: "cad-part",
      x: 0.0,
      y: 0.0,
      z: 0.0,
      width: 0.114,
      height: 0.21,
      depth: 0.003,
      collisionEnabled: false,
      dynamicBody: false,
      mass: 1.0,
      red: 0.8,
      green: 0.8,
      blue: 0.8,
      visible: true,
      cadGraph: graph
    }]);
    try {
      var session = scene.cadSession("plate"), document = session.document, count = document.featureCount();
      check(scene.cadFeatureCount("plate") == count && count > 100, "The plate's recorded history is its feature rows");
      var base = -1, bosses = -1;
      for (index in 0...count) {
        var feature = document.featureAt(index);
        if (base < 0 && feature.label == "Face plate") base = index;
        if (bosses < 0 && feature.group == "Bosses") bosses = index;
      }
      check(base >= 0 && bosses > base, "The plate has a base step and later grouped steps");
      check(scene.cadFeatureNameAt("plate", base) == "Plate · Face plate", "A row shows its group and label");
      check(scene.cadFeatureNameAt("plate", bosses).indexOf("Bosses · ") == 0, "Grouped rows lead with their group");

      var undoCount = scene.document.history.undoCount;
      check(!scene.canRollForward() && !scene.canRollBackToSelectedFeature(), "Nothing to roll without a selected row");
      check(scene.selectTreeKey("plate:feature:" + base), "A history row can be selected");
      check(scene.canRollBackToSelectedFeature(), "A row before the end can be rolled back to");
      check(scene.rollBackToSelectedFeature(), "Rolling back succeeds");
      check(document.rollbackIndex == base && scene.canRollForward(), "The marker sits at the selected row");
      var rolledDepth = depth(scene);
      // Kernel bounds are slightly loose around the rounded corners, so compare with the 3 mm base by range.
      var shown = document.displayedFeature();
      check(shown != null && shown.label == "Face plate", "The marked feature is what is shown");
      check(rolledDepth < 0.0035, "The shown plate is the bare base, not the bosses and tongue: " + rolledDepth);
      check(scene.cadFeatureNameAt("plate", bosses).indexOf("Rolled back · ") == 0, "Later rows are marked rolled back");
      check(scene.cadFeatureNameAt("plate", base) == "Plate · Face plate", "The marked row is not rolled back");
      check(scene.document.history.undoCount == undoCount, "Rolling back adds no undo step");

      var reopened = new EditorScene(SceneCodec.decode(SceneCodec.encode(scene)));
      try {
        check(
          reopened.cadSession("plate").document.rollbackIndex == -1
          && reopened.cadFeatureCount("plate") == count,
          "The marker is view state: a saved scene keeps the whole history"
        );
      } catch (error:Dynamic) {
        reopened.dispose();
        throw error;
      }
      reopened.dispose();

      check(scene.rollForward(), "Rolling forward succeeds");
      check(document.rollbackIndex == -1 && !scene.canRollForward(), "The marker is cleared");
      check(depth(scene) > 0.005, "The full plate with bosses and tongue is shown again: " + depth(scene));
      check(scene.cadFeatureNameAt("plate", bosses).indexOf("Rolled back") < 0, "No row is marked rolled back");
    } catch (error:Dynamic) {
      scene.dispose();
      throw error;
    }
    scene.dispose();
    Sys.println("CAD timeline editor: " + checks + " checks passed");
  }
}
