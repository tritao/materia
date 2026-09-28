package tests;

import app.EditorScene;
import app.SceneCodec;
import app.StockSimulationSession;
import nativekit.ui.properties.PropertyBinding;
import nativekit.ui.properties.PropertyValue;

/** The editor's stock-simulation object: scrubbing, colouring, and picking back to G-code. */
class StockSimulationTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;

  static function apply(scene:EditorScene, label:String, value:PropertyValue):Void {
    for (descriptor in scene.properties()) if (descriptor.label == label) {
      switch new PropertyBinding(descriptor, scene.context()).apply(value) {
        case Rejected(message): throw 'Stock simulation property "$label" rejected the edit: $message';
        case Applied, Unchanged:
      }
      return;
    }
    throw 'Missing stock simulation property: $label';
  }

  static function text(scene:EditorScene, label:String):String {
    for (descriptor in scene.properties()) if (descriptor.label == label)
      return switch descriptor.readValue(scene.context()) {
        case PropertyValue.Text(value): value;
        case PropertyValue.Int(value): Std.string(value);
        case PropertyValue.Enum(value): value;
        default: throw 'Unexpected value for "$label"';
      };
    throw 'Missing stock simulation property: $label';
  }

  public static function main():Int {
    var scene = new EditorScene([]);
    try {
      check(scene.createStockSimulation(), "editor adds a stock simulation");
      var id = scene.selectedId;
      var created = scene.object(id);
      check(created != null && created.kind == StockSimulationSession.KIND, "the new object is a stock simulation");
      var simulation = scene.stockSimulation(id);
      check(simulation.moveCount() > 50, "the demo program has moves to simulate");
      check(text(scene, "Moves cut") == Std.string(simulation.moveCount()),
        "the simulation starts at the program's end");
      check(text(scene, "Rapids through stock") == "0", "the demo program never rapids through stock");
      check(text(scene, "Deepest gouge") == "0 mm", "the demo program does not gouge its part");

      // The block is 60 x 40 mm centred on the object; the pocket floor lies 15 mm left of centre.
      check(scene.selectAtRay(-0.015, 0, 1, 0, 0, -1) == id, "a ray onto the pocket floor hits the stock");
      var picked = text(scene, "Picked surface");
      check(StringTools.startsWith(picked, "Operation 1 · line "), 'the pocket floor names its operation: $picked');
      var line = Std.parseInt(picked.substring("Operation 1 · line ".length, picked.indexOf(":")));
      check(line != null && line > 1 && line <= simulation.gcode.length, "the picked line is in the program");
      check(picked.indexOf("G") > 0, "the picked surface shows its G-code line");
      check(scene.selectAtRay(0, 0, 1, 0, 0, -1) == id && text(scene, "Picked surface").indexOf("Click") == 0,
        "the boss top is untouched stock");

      // Scrub back to the start: the same point is uncut.
      apply(scene, "Moves cut", PropertyValue.Int(0));
      check(simulation.position() == 0, "the timeline moves to the start");
      scene.selectAtRay(-0.015, 0, 1, 0, 0, -1);
      check(text(scene, "Picked surface").indexOf("Click") == 0, "before any move the pocket is uncut stock");
      apply(scene, "Moves cut", PropertyValue.Int(simulation.moveCount()));

      apply(scene, "Colour by", PropertyValue.Enum("deviation"));
      check(simulation.colorBy == "deviation", "colouring switches to deviation");
      apply(scene, "Colour by", PropertyValue.Enum("operation"));

      var reopened = new EditorScene(SceneCodec.decode(SceneCodec.encode(scene)));
      try {
        var loaded = reopened.object(id);
        check(loaded != null && loaded.kind == StockSimulationSession.KIND
          && reopened.stockSimulation(id).moveCount() == simulation.moveCount(),
          "a saved stock simulation reopens with the same program");
        reopened.dispose();
      } catch (error:Dynamic) {
        reopened.dispose();
        throw error;
      }
      scene.dispose();
    } catch (error:Dynamic) {
      scene.dispose();
      Sys.println("Stock simulation test failed: " + Std.string(error));
      return 1;
    }
    Sys.println("Stock simulation tests passed");
    return 0;
  }
}
