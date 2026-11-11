import cadkit.parametric.DocumentCodec;
import cadkit.parametric.InstanceElement;
import machinekit.pendant.*;
import materia.project.SceneArtifact.SceneArtifactData;

/** The case's size is an editable recipe: a saved edit regenerates the whole pendant, and a bad one is refused. */
class PendantRecipeTests {
  static function check(ok:Bool, message:String):Void if (!ok) throw message;

  static function instanceOf(document:cadkit.parametric.Document):InstanceElement {
    for (element in document.allElements()) if (element.kind == "instance") {
      var property = element.property("machinekit.occurrence");
      if (property != null && property.value == PendantScene.RECIPE_OCCURRENCE) return cast element;
    }
    throw "The recipe has no instance for the plate";
  }

  /** Extent of an artifact part along x (0), y (1) or z (2), from its tessellation. */
  static function extent(scene:SceneArtifactData, id:String, axis:Int):Float {
    for (part in scene.parts) {
      if (part.id != id) continue;
      var low = 1e300, high = -1e300;
      for (index in 0...part.vertexCount) {
        var value = part.vertices.getDouble(index * 24 + axis * 8);
        low = Math.min(low, value);
        high = Math.max(high, value);
      }
      return high - low;
    }
    throw "No part " + id;
  }

  /** The recipe as the editor would save it after the user sets one of the plate's inputs. */
  static function editedRecipe(scene:SceneArtifactData, name:String, value:Float):String {
    var text = scene.recipeDocument;
    if (text == null) throw "The scene carries no recipe";
    var document = DocumentCodec.decode(text);
    var instance = instanceOf(document);
    document.definition(instance.definitionId).setUserEditedDefault(name, value);
    var result = DocumentCodec.encode(document);
    document.close();
    return result;
  }

  public static function main():Void {
    var scene = PendantScene.build();
    var text = scene.recipeDocument;
    check(text != null, "The scene carries its recipe");
    var recipe = DocumentCodec.decode(text);
    var instance = instanceOf(recipe);
    check(
      instance.resolved("width") == 114 && instance.resolved("height") == 210 && instance.resolved("thickness") == 3
      && instance.resolved("wall") == 3 && instance.resolved("depth") == 48,
      "The recipe starts at the reference case"
    );
    recipe.close();
    check(Math.abs(extent(scene, "case/plate", 0) - 114) < 1e-6, "The plate is 114 mm wide");
    check(Math.abs(extent(scene, "case/shell", 0) - 114) < 1e-6, "The shell is 114 mm wide");

    // An edit saved by the editor regenerates the plate, the shell, the board's room and the recorded history.
    var wider = PendantScene.build(editedRecipe(scene, "width", 120));
    check(Math.abs(extent(wider, "case/plate", 0) - 120) < 1e-6, "The edited plate is 120 mm wide");
    check(Math.abs(extent(wider, "case/shell", 0) - 120) < 1e-6, "The shell follows the plate's width");
    check(Math.abs(extent(wider, "case/plate", 1) - 210) < 1e-6, "Untouched inputs keep their values");
    var project = wider.project;
    if (project == null) throw "The edited scene carries a project record";
    var histories = project.histories;
    if (histories == null) throw "The edited scene carries histories";
    var plateHistory = DocumentCodec.decode(histories[0].document);
    check(plateHistory.parameter("width").value == 120, "The plate's recorded history is the edited one");
    plateHistory.close();
    var again = DocumentCodec.decode(wider.recipeDocument);
    check(instanceOf(again).resolved("width") == 120, "The edit survives into the next saved recipe");
    again.close();

    var deeper = PendantScene.build(editedRecipe(scene, "depth", 52));
    check(extent(deeper, "case/shell", 2) > extent(scene, "case/shell", 2) + 3, "A deeper case gives a deeper shell");

    // A case too small for its controls is refused, so the editor keeps the scene it has.
    var refused = false;
    try PendantScene.build(editedRecipe(scene, "width", 60)) catch (error:Dynamic) refused = true;
    check(refused, "A case too narrow for its controls is rejected");
    Sys.println("Pendant recipe: ok");
  }
}
