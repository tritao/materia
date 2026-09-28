package app.editor;

import app.CadBracketModel;
import app.CadDocumentSession;
import app.CadImportedModel;
import app.CadPartModel;
import app.CadPlateModel;
import app.CadSessionModel;
import app.EditorScene;
import app.SceneObjectData;
import nativekit.ui.properties.PropertyDescriptor;

/** The single list of kinds known to the editor. */
class ObjectKindRegistry {
  static final providers:Array<ObjectKindProvider> = [
    new BuiltinObjectKind("rectangle", "Primitive", "Rectangle", "scene.create", false, false),
    new BuiltinObjectKind("cad-part", "CAD", "Empty CAD part", "scene.create-part", true, true),
    new BuiltinObjectKind("cad-plate", "CAD", "Mounting plate", "scene.create-plate", true, true),
    new BuiltinObjectKind("cad-bracket", "CAD", "L bracket", "scene.create-bracket", true, true),
    new BuiltinObjectKind("cad-step", "Import", "STEP part", "scene.import-step", true, true),
    new BuiltinObjectKind("cad-preview", "", "Generated CAD preview", "", false, true),
    new StockSimulationKind()
  ];

  public static function all():Array<ObjectKindProvider> return providers.copy();

  public static function find(id:String):Null<ObjectKindProvider> {
    for (provider in providers) if (provider.id() == id) return provider;
    return null;
  }

  public static function require(id:String):ObjectKindProvider {
    var provider = find(id);
    if (provider == null) throw "Unknown scene object kind: " + id;
    return provider;
  }

  public static function addMenuCommands(section:String):Array<String> {
    var result:Array<String> = [];
    for (provider in providers) if (provider.addSection() == section && provider.addCommand() != "")
      result.push(provider.addCommand());
    return result;
  }
}

@:access(app.EditorScene)
private class BuiltinObjectKind implements ObjectKindProvider {
  final kind:String;
  final section:String;
  final label:String;
  final command:String;
  final cad:Bool;
  final faceHover:Bool;

  public function new(kind:String, section:String, label:String, command:String,
      cad:Bool, faceHover:Bool) {
    this.kind = kind; this.section = section; this.label = label; this.command = command;
    this.cad = cad; this.faceHover = faceHover;
  }

  public function id():String return kind;
  public function addSection():String return section;
  public function addLabel():String return label;
  public function addCommand():String return command;
  public function isCad():Bool return cad;
  public function supportsFaceHover():Bool return faceHover;
  public function supportsEdgeHover():Bool return faceHover;
  public function hasGeneratedGeometry():Bool return kind == "cad-preview";
  public function supportsSketchEdit():Bool return kind == "cad-part";
  public function isPrimitive():Bool return kind == "rectangle";

  public function createDefaultRecord(id:String):SceneObjectData {
    switch (kind) {
      case "rectangle": return record(id, "Rectangle", 1.6, 1.2, 0.1, true, 0.22, 0.52, 0.85);
      case "cad-part": return record(id, "Part", 0.05, 0.05, 0.01, false, 0.66, 0.68, 0.72);
      case "cad-plate":
        var model = CadPlateModel.create(0.08, 0.05, 0.006, 0.012);
        var graph = model.encode(); model.close();
        var result = record(id, "Mounting plate", 0.08, 0.05, 0.006, false, 0.34, 0.62, 0.78);
        result.cadGraph = graph;
        return result;
      case "cad-bracket": return record(id, "L bracket", 0.06, 0.04, 0.03, false, 0.64, 0.66, 0.70);
      default: throw 'Object kind "$kind" requires an external source';
    }
  }

  function record(id:String, name:String, width:Float, height:Float, depth:Float,
      collision:Bool, red:Float, green:Float, blue:Float):SceneObjectData return {
    id:id, label:name, type:kind, x:0.0, y:0.0, z:0.0,
    width:width, height:height, depth:depth, collisionEnabled:collision,
    dynamicBody:false, mass:1.0, red:red, green:green, blue:blue, visible:true
  };

  public function createCadSession(graph:Null<String>, width:Float, height:Float,
      depth:Float):Null<CadDocumentSession> {
    if (!cad) return null;
    var model:CadSessionModel = switch (kind) {
      case "cad-step":
        if (graph == null) throw "Imported STEP object has no persisted source graph";
        CadImportedModel.decode(graph);
      case "cad-part": graph == null ? CadPartModel.createEmpty() : CadPartModel.decode(graph);
      case "cad-bracket": graph == null ? CadBracketModel.create(width, height, depth) : CadBracketModel.decode(graph);
      case "cad-plate": graph == null ? CadPlateModel.create(width, height, depth,
        Math.min(0.012, Math.min(width, height) * 0.5)) : CadPlateModel.decode(graph);
      default: throw "Unknown CAD object kind: " + kind;
    };
    try { return new CadDocumentSession(model); }
    catch (error:Dynamic) { model.close(); throw error; }
  }

  public function properties(scene:EditorScene, id:String, prefix:String):Array<PropertyDescriptor> {
    var result:Array<PropertyDescriptor> = [];
    switch (kind) {
      case "cad-plate":
        result.push(scene.cadProperty(id, CadPlateModel.HOLE_DIAMETER, "Hole diameter", false, prefix));
        result.push(scene.cadProperty(id, CadPlateModel.HOLE_X, "Hole X", true, prefix));
        result.push(scene.cadProperty(id, CadPlateModel.HOLE_Y, "Hole Y", true, prefix));
      case "cad-bracket":
        result.push(scene.bracketProperty(id, "wall", "Wall thickness", function(model) return model.wallThickness(),
          function(value) scene.setBracketWallThickness(id, value), prefix));
        result.push(scene.bracketProperty(id, "hole-radius", "Hole radius", function(model) return model.holeRadius(),
          function(value) scene.setBracketHoleRadius(id, value), prefix));
      default:
    }
    return ScenePropertyProvider.common(scene, id, prefix, result);
  }

  public function commands(scene:EditorScene, id:String):Array<String> {
    var result:Array<String> = [];
    if (kind == "cad-part") {
      if (scene.canCreateSketch()) result.push("scene.create-sketch");
      if (scene.canCreateFaceSketch()) result.push("scene.create-face-sketch");
      if (scene.canCreateExtrusion()) result.push("scene.create-extrusion");
      if (scene.canCreatePocket()) result.push("scene.create-pocket");
      if (scene.canCreateVerticalFillet()) result.push("scene.create-vertical-fillet");
    }
    if (kind == "cad-plate" && scene.canAddHoleOnSelectedFace()) result.push("scene.add-face-hole");
    return result;
  }
}
