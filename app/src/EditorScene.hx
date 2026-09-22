package app;

import nativekit.scene.Scene;
import app.SceneCodec.SceneObjectData;
import nativekit.scene.Snapshot;
import nativekit.scene.SpatialIndex;
import nativekit.scene.Occurrence;
import nativekit.scene.OccurrenceInfo;
import nativekit.scene.GeometryData;
import nativekit.scene.MaterialData;
import nativekit.scene.Transform;
import nativekit.ui.core.EditorDocument;
import nativekit.ui.core.CommandContext;
import nativekit.ui.core.PropertyDescriptor;
import nativekit.ui.core.PropertyDescriptorOptions;
import nativekit.ui.core.PropertyType;
import nativekit.ui.core.PropertyValue;

/** One scene and one document shared by the hierarchy, inspector and viewport. */
class EditorScene {
  // Retained UI caches survive document replacement, so revisions must too.
  static var nextRevision:Int = 0;
  public final document:EditorDocument;
  final scene:Scene;
  final objects:Array<EditorSceneObject>;
  var snapshot:Snapshot;
  var spatial:SpatialIndex;
  public var selectedId(default, null):String = "box";
  public var revision(default, null):Int;
  public var selectionRevision(default, null):Int = 1;
  var disposed:Bool = false;

  public function new(?data:Array<SceneObjectData>) {
    nextRevision++;
    revision = nextRevision;
    document = new EditorDocument("scene");
    scene = Scene.create();
    objects = [];
    try {
      if (data == null) {
        addObject("box", "Blue box", -1.5, 0.0, 0.0, 1.6, 1.2, 0.22, 0.52, 0.85);
        addObject("tower", "Orange tower", 1.1, 0.0, 0.1, 1.2, 1.8, 0.92, 0.48, 0.22);
      } else {
        for (item in data) addObject(item.id, item.label, item.x, item.y, item.z,
          item.width, item.height, item.red, item.green, item.blue, item.visible);
        selectedId = data.length == 0 ? "scene" : data[0].id;
      }
      snapshot = scene.snapshot();
      try spatial = SpatialIndex.create(snapshot)
      catch (error:Dynamic) { snapshot.dispose(); throw error; }
    } catch (error:Dynamic) { scene.dispose(); throw error; }
  }

  function addObject(id:String, label:String, x:Float, y:Float, z:Float,
      width:Float, height:Float, red:Float, green:Float, blue:Float, visible:Bool = true):Void {
    var geometry = scene.createGeometry();
    var mesh = new GeometryData();
    mesh.addVertex(-width / 2, -height / 2, 0);
    mesh.addVertex(width / 2, -height / 2, 0);
    mesh.addVertex(width / 2, height / 2, 0);
    mesh.addVertex(-width / 2, height / 2, 0);
    mesh.addTriangle(0, 1, 2);
    mesh.addTriangle(0, 2, 3);
    mesh.setBounds(-width / 2, -height / 2, 0, width / 2, height / 2, 0);
    scene.setGeometryData(geometry, mesh);
    var material = scene.createMaterial();
    scene.setMaterialData(material, MaterialData.opaque(red, green, blue));
    var transaction = scene.beginTransaction();
    try {
      var occurrence = transaction.createOccurrence();
      transaction.setName(occurrence, label);
      transaction.setVisibility(occurrence, visible);
      transaction.setGeometry(occurrence, geometry);
      transaction.setMaterial(occurrence, material);
      transaction.setTransform(occurrence, Transform.identity().translated(x, y, z));
      transaction.commit();
      objects.push(new EditorSceneObject(id, label, occurrence, width, height, red, green, blue));
    } catch (error:Dynamic) {
      transaction.dispose();
      throw error;
    }
  }

  public function items():Array<EditorSceneObject> return objects.copy();

  public function object(id:String):Null<EditorSceneObject> {
    for (item in objects) if (item.id == id) return item;
    return null;
  }

  public function info(id:String):OccurrenceInfo {
    var item = object(id);
    if (item == null) throw "Unknown scene object: " + id;
    var value = snapshot.find(item.occurrence);
    if (value == null) throw "Missing scene occurrence: " + id;
    return value;
  }

  public function select(id:String):Bool {
    if (id != "scene" && object(id) == null) return false;
    if (id == selectedId) return false;
    selectedId = id;
    selectionRevision++;
    nextRevision++;
    revision = nextRevision;
    return true;
  }

  public function context():CommandContext {
    return new CommandContext(document, selectedId == "scene" ? [] : [selectedId],
      "scene-viewport", null, "scene-editor");
  }

  /** Orthographic world-space picking against the same published geometry. */
  public function pick(x:Float, y:Float):String {
    var hit = spatial.pickRay(x, y, 1000001.0, 0.0, 0.0, -1.0);
    for (item in objects) if (hit.occurrence().equals(item.occurrence)) return item.id;
    return "scene";
  }

  public function setPosition(id:String, axis:Int, value:Float):Void {
    if (axis < 0 || axis > 1 || value != value || value - value != 0.0 || Math.abs(value) > 1000000)
      throw "Position requires a finite X or Y coordinate";
    var item = object(id);
    if (item == null) throw "Unknown scene object: " + id;
    var before = info(id).localTransform();
    var current = Transform.identity().translated(before.element(12), before.element(13), before.element(14));
    current.set(12 + axis, value);
    var transaction = scene.beginTransaction();
    try {
      transaction.setTransform(item.occurrence, current);
      transaction.commit();
    } catch (error:Dynamic) { transaction.dispose(); throw error; }
    publish();
  }

  public function setVisible(id:String, visible:Bool):Void {
    var item = object(id);
    if (item == null) throw "Unknown scene object: " + id;
    var transaction = scene.beginTransaction();
    try {
      transaction.setVisibility(item.occurrence, visible);
      transaction.commit();
    } catch (error:Dynamic) { transaction.dispose(); throw error; }
    publish();
  }

  function publish():Void {
    var next = scene.snapshot();
    var nextSpatial:SpatialIndex;
    try nextSpatial = SpatialIndex.create(next)
    catch (error:Dynamic) { next.dispose(); throw error; }
    spatial.dispose();
    snapshot.dispose();
    snapshot = next;
    spatial = nextSpatial;
    nextRevision++;
    revision = nextRevision;
  }

  public function properties():Array<PropertyDescriptor> {
    var id = selectedId;
    if (object(id) == null) return [];
    // Capture identity in each binding: undo must still target this object after selection changes.
    var prefix = id + ":" + selectionRevision + ":";
    var result:Array<PropertyDescriptor> = [];
    for (axis in 0...2) result.push(positionProperty(id, axis, prefix));
    var visibility = new PropertyDescriptorOptions();
    visibility.category = "Rendering";
    result.push(new PropertyDescriptor(prefix + "visible", "Visible", PropertyType.Bool,
      function(_) return PropertyValue.Bool(info(id).visible()), function(_, value) {
        switch (value) {
          case PropertyValue.Bool(next): setVisible(id, next);
          default: throw "Visibility requires a boolean";
        }
      }, visibility));
    return result;
  }

  function positionProperty(id:String, axis:Int, prefix:String):PropertyDescriptor {
    var settings = new PropertyDescriptorOptions();
    settings.category = "Transform";
    settings.unit = "m";
    settings.step = 0.1;
    return new PropertyDescriptor(prefix + "position-" + axis, axis == 0 ? "Position X" : "Position Y",
      PropertyType.Float, function(_) return PropertyValue.Float(info(id).localTransform().element(12 + axis)),
      function(_, value) {
        switch (value) {
          case PropertyValue.Float(next): setPosition(id, axis, next);
          case PropertyValue.Int(next): setPosition(id, axis, next);
          default: throw "Position requires a number";
        }
      }, settings);
  }

  public function records():Array<SceneObjectData> {
    var result:Array<SceneObjectData> = [];
    for (item in objects) {
      var state = info(item.id);
      var transform = state.localTransform();
      result.push({id: item.id, label: item.label, type: "rectangle",
        x: transform.element(12), y: transform.element(13), z: transform.element(14),
        width: item.width, height: item.height, red: item.red, green: item.green, blue: item.blue,
        visible: state.visible()});
    }
    return result;
  }

  public function diagnosticState():Dynamic {
    var values:Array<Dynamic> = [];
    for (item in objects) {
      var state = info(item.id);
      values.push({id: item.id, label: item.label, x: state.localTransform().element(12),
        y: state.localTransform().element(13), visible: state.visible()});
    }
    return {objects: values, selectedId: selectedId, revision: revision,
      canUndo: document.canUndo, canRedo: document.canRedo, dirty: document.isDirty};
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    spatial.dispose();
    snapshot.dispose();
    scene.dispose();
  }
}

class EditorSceneObject {
  public final id:String;
  public final label:String;
  public final occurrence:Occurrence;
  public final width:Float;
  public final height:Float;
  public final red:Float;
  public final green:Float;
  public final blue:Float;
  public function new(id:String, label:String, occurrence:Occurrence, width:Float, height:Float,
      red:Float, green:Float, blue:Float) {
    this.id = id; this.label = label; this.occurrence = occurrence;
    this.width = width; this.height = height;
    this.red = red; this.green = green; this.blue = blue;
  }
}
