package app.editor;

import app.CadDocumentSession;
import app.EditorScene;
import app.SceneObjectData;
import app.StockSimulationSession;
import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.properties.PropertyDescriptorOptions;
import nativekit.ui.properties.PropertyOption;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;

/**
  A block of stock cut by a machining program with StockKit. Its inspector
  scrubs the program, switches between colouring by operation and by
  deviation from the finished part, and names the operation and G-code line
  of the last surface picked in the viewport.
**/
class StockSimulationKind implements ObjectKindProvider {
  public function new() {}

  public function id():String return StockSimulationSession.KIND;
  public function addSection():String return "Machining";
  public function addLabel():String return "Stock simulation";
  public function addCommand():String return "scene.create-stock-simulation";
  public function isCad():Bool return false;
  public function supportsFaceHover():Bool return false;
  public function supportsEdgeHover():Bool return false;
  public function hasGeneratedGeometry():Bool return false;
  public function supportsSketchEdit():Bool return false;
  public function isPrimitive():Bool return false;

  public function createDefaultRecord(id:String):SceneObjectData return {
    id: id, label: "Stock simulation", type: StockSimulationSession.KIND, x: 0.0, y: 0.0, z: 0.0,
    width: 0.06, height: 0.04, depth: 0.02, collisionEnabled: false, dynamicBody: false, mass: 1.0,
    // White, so the per-vertex simulation colours show unchanged.
    red: 1.0, green: 1.0, blue: 1.0, visible: true
  };

  public function createCadSession(graph:Null<String>, width:Float, height:Float,
      depth:Float):Null<CadDocumentSession> return null;

  public function commands(scene:EditorScene, id:String):Array<String> return [];

  public function properties(scene:EditorScene, id:String, prefix:String):Array<PropertyDescriptor> {
    var result:Array<PropertyDescriptor> = [];
    var session = scene.stockSimulation(id);

    var timeline = options(false);
    timeline.minimum = 0;
    timeline.maximum = session.moveCount();
    timeline.step = 1;
    timeline.validator = function(_, value) return switch value {
      case PropertyValue.Int(_): null;
      default: "Timeline position requires a whole number of moves";
    };
    result.push(new PropertyDescriptor(prefix + "stock-timeline", "Moves cut", PropertyType.Int,
      function(_) return PropertyValue.Int(scene.stockSimulation(id).position()),
      function(_, value) switch value {
        case PropertyValue.Int(position): scene.setStockSimulationPosition(id, position);
        default: throw "Timeline position requires a whole number of moves";
      }, timeline));

    var colouring = options(false);
    colouring.options = [new PropertyOption("operation", "Operation"),
      new PropertyOption("deviation", "Deviation from part")];
    colouring.validator = function(_, value) return switch value {
      case PropertyValue.Enum("operation" | "deviation"): null;
      default: "Colour by operation or deviation";
    };
    result.push(new PropertyDescriptor(prefix + "stock-colouring", "Colour by", PropertyType.Enum,
      function(_) return PropertyValue.Enum(scene.stockSimulation(id).colorBy),
      function(_, value) switch value {
        case PropertyValue.Enum(mode): scene.setStockSimulationColouring(id, mode);
        default: throw "Colour by operation or deviation";
      }, colouring));

    result.push(readOnly(prefix + "stock-picked", "Picked surface", function() {
      var simulation = scene.stockSimulation(id);
      var move = simulation.picked;
      return move == null ? "Click the stock to name the move that cut it" : simulation.describe(move);
    }));
    result.push(readOnly(prefix + "stock-program", "Program", function() {
      var simulation = scene.stockSimulation(id);
      return '${simulation.moveCount()} moves, ${simulation.gcode.length} G-code lines';
    }));
    result.push(readOnly(prefix + "stock-rapids", "Rapids through stock", function()
      return Std.string(scene.stockSimulation(id).rapidContacts())));
    result.push(readOnly(prefix + "stock-contact", "Shank or holder contact", function()
      return '${scene.stockSimulation(id).collisions()} moves'));
    result.push(readOnly(prefix + "stock-gouge", "Deepest gouge", function()
      return '${Math.round(scene.stockSimulation(id).deepestGouge() * 1e6) / 1000} mm'));
    return ScenePropertyProvider.common(scene, id, prefix, result);
  }

  static function options(history:Bool):PropertyDescriptorOptions {
    var result = new PropertyDescriptorOptions();
    result.category = "Simulation";
    result.recordHistory = history;
    return result;
  }

  static function readOnly(id:String, label:String, text:Void->String):PropertyDescriptor {
    var settings = options(false);
    settings.readOnly = true;
    return new PropertyDescriptor(id, label, PropertyType.Text,
      function(_) return PropertyValue.Text(text()),
      function(_, _) throw label + " is read-only", settings);
  }
}
