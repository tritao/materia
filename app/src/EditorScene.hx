package app;

import nativekit.scene.Scene;
import app.SceneCodec.SceneObjectData;
import nativekit.scene.Snapshot;
import nativekit.scene.SpatialIndex;
import nativekit.scene.Occurrence;
import nativekit.scene.OccurrenceInfo;
import nativekit.scene.GeometryData;
import nativekit.scene.MaterialData;
import nativekit.scene.Material;
import nativekit.scene.SceneView;
import nativekit.scene.SelectionSet;
import nativekit.scene.Transform;
import nativekit.ui.core.EditorDocument;
import nativekit.ui.core.EditOperation;
import nativekit.ui.core.CommandContext;
import nativekit.ui.core.PropertyDescriptor;
import nativekit.ui.core.PropertyDescriptorOptions;
import nativekit.ui.core.PropertyType;
import nativekit.ui.core.PropertyValue;

/** One scene and one document shared by the hierarchy, inspector and viewport. */
class EditorScene {
  // Retained UI caches survive document replacement, so revisions must too.
  static var nextRevision:Int = 0;
  static var nextEnvironmentRevision:Int = 0;
  public final document:EditorDocument;
  var scene:Scene;
  var objects:Array<EditorSceneObject>;
  var nextObjectId:Int = 1;
  var snapshot:Snapshot;
  var spatial:SpatialIndex;
  var selectionMaterial:Material;
  public var selectedId(default, null):String = "box";
  public var revision(default, null):Int;
  /** Changes only when simulation-consumed scene content changes, not selection. */
  public var environmentRevision(default, null):Int;
  public var selectionRevision(default, null):Int = 1;
  var disposed:Bool = false;

  public function new(?data:Array<SceneObjectData>) {
    nextRevision++;
    revision = nextRevision;
    nextEnvironmentRevision++;
    environmentRevision = nextEnvironmentRevision;
    document = new EditorDocument("scene");
    scene = Scene.create();
    objects = [];
    try {
      if (data == null) {
        addObject("box", "Blue box", -1.5, 0.0, 0.0, 1.6, 1.2, 0.1, 0.22, 0.52, 0.85);
        addObject("tower", "Orange tower", 1.1, 0.0, 0.1, 1.2, 1.8, 0.2, 0.92, 0.48, 0.22);
      } else {
        for (item in data) addObject(item.id, item.label, item.x, item.y, item.z,
          item.width, item.height, item.depth, item.red, item.green, item.blue, item.visible,
          item.collisionEnabled,item.dynamicBody,item.mass);
        selectedId = data.length == 0 ? "scene" : data[0].id;
      }
      selectionMaterial = scene.createMaterial();
      scene.setMaterialData(selectionMaterial, MaterialData.opaque(1.0, 0.88, 0.35));
      snapshot = scene.snapshot();
      try spatial = SpatialIndex.create(snapshot)
      catch (error:Dynamic) { snapshot.dispose(); throw error; }
    } catch (error:Dynamic) { scene.dispose(); throw error; }
  }

  function addObject(id:String, label:String, x:Float, y:Float, z:Float,
      width:Float, height:Float, depth:Float, red:Float, green:Float, blue:Float,
      visible:Bool = true,collisionEnabled:Bool=true,dynamicBody:Bool=false,mass:Float=1.0):Void {
    var geometry = scene.createGeometry();
    scene.setGeometryData(geometry, boxGeometry(width, height, depth));
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
      objects.push(new EditorSceneObject(id, label, occurrence, width, height, depth,
        collisionEnabled,dynamicBody,mass,red,green,blue));
    } catch (error:Dynamic) {
      transaction.dispose();
      throw error;
    }
  }

  public function canCreate():Bool return objects.length < 10000;

  function allocateId():String {
    var id = "rectangle-" + nextObjectId;
    nextObjectId++;
    while (object(id) != null) {
      id = "rectangle-" + nextObjectId;
      nextObjectId++;
    }
    return id;
  }

  public function createRectangle():Bool {
    if (!canCreate()) return false;
    var data = records();
    var id = allocateId();
    data.push({id: id, label: "Rectangle", type: "rectangle", x: 0.0, y: 0.0, z: 0.0,
      width: 1.6, height: 1.2, depth:0.1,collisionEnabled:true,dynamicBody:false,mass:1.0,
      red: 0.22, green: 0.52, blue: 0.85, visible: true});
    return changeObjects("Create rectangle", data, id);
  }

  public function duplicateSelected():Bool {
    if (!canCreate() || object(selectedId) == null) return false;
    var data = records();
    var source:SceneObjectData = null;
    for (item in data) if (item.id == selectedId) source = item;
    var id = allocateId();
    data.push({id: id, label: source.label + " copy", type: source.type,
      x: Math.min(1000000, source.x + 0.25), y: Math.min(1000000, source.y + 0.25), z: source.z,
      width: source.width, height: source.height, red: source.red, green: source.green,
      blue: source.blue, visible: source.visible,depth:source.depth,
      collisionEnabled:source.collisionEnabled,dynamicBody:source.dynamicBody,mass:source.mass});
    return changeObjects("Duplicate object", data, id);
  }

  public function deleteSelected():Bool {
    if (object(selectedId) == null) return false;
    var data = records();
    var index = 0;
    while (data[index].id != selectedId) index++;
    data.splice(index, 1);
    var next = data.length == 0 ? "scene" : data[index < data.length ? index : data.length - 1].id;
    return changeObjects("Delete object", data, next);
  }

  function changeObjects(label:String, after:Array<SceneObjectData>, selection:String):Bool {
    var before = records();
    var previousSelection = selectedId;
    return document.apply(new EditOperation(label,
      function() replaceObjects(after, selection),
      function() replaceObjects(before, previousSelection)));
  }

  // Build before swapping so allocation failures preserve the live scene and history.
  // Rebuilding also releases removed geometry/materials instead of accumulating tombstones.
  function replaceObjects(data:Array<SceneObjectData>, selection:String):Void {
    var next = new EditorScene(data);
    var oldScene = scene;
    var oldSnapshot = snapshot;
    var oldSpatial = spatial;
    var oldSelectionMaterial = selectionMaterial;
    scene = next.scene;
    snapshot = next.snapshot;
    spatial = next.spatial;
    objects = next.objects;
    selectionMaterial = next.selectionMaterial;
    next.scene = oldScene;
    next.snapshot = oldSnapshot;
    next.spatial = oldSpatial;
    next.selectionMaterial = oldSelectionMaterial;
    next.dispose();
    selectedId = selection;
    selectionRevision++;
    nextRevision++;
    revision = nextRevision;
    nextEnvironmentRevision++;
    environmentRevision = nextEnvironmentRevision;
  }

  public function setName(id:String, label:String):Void {
    if (StringTools.trim(label).length == 0 || SceneCodec.containsNul(label))
      throw "Name cannot be empty or contain NUL";
    var item = object(id);
    if (item == null) throw "Unknown scene object: " + id;
    var transaction = scene.beginTransaction();
    try {
      transaction.setName(item.occurrence, label);
      transaction.commit();
    } catch (error:Dynamic) { transaction.dispose(); throw error; }
    item.label = label;
    publish();
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

  public function renderSnapshot():Snapshot return snapshot;

  public function configureRenderView(view:SceneView, viewProjection:Transform):SceneView {
    view.setViewProjection(viewProjection);
    var selected = object(selectedId);
    var selection = new SelectionSet();
    if (selected != null) selection.add(selected.occurrence);
    view.applySelection(selection, selectionMaterial);
    return view;
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
    var before = info(id).localTransform();
    setPositionXY(id, axis == 0 ? value : before.element(12), axis == 1 ? value : before.element(13));
  }

  /** Updates both planar coordinates atomically; used for live viewport previews. */
  public function setPositionXY(id:String, x:Float, y:Float):Void {
    if (!finiteCoordinate(x) || !finiteCoordinate(y))
      throw "Position requires finite X and Y coordinates";
    var item = object(id);
    if (item == null) throw "Unknown scene object: " + id;
    var before = info(id).localTransform();
    var current = Transform.identity().translated(x, y, before.element(14));
    var transaction = scene.beginTransaction();
    try {
      transaction.setTransform(item.occurrence, current);
      transaction.commit();
    } catch (error:Dynamic) { transaction.dispose(); throw error; }
    publish();
  }

  /** Records a move whose final position has already been applied as a drag preview. */
  public function recordMove(id:String, fromX:Float, fromY:Float, toX:Float, toY:Float):Bool {
    if (!finiteCoordinate(fromX) || !finiteCoordinate(fromY) ||
        !finiteCoordinate(toX) || !finiteCoordinate(toY))
      throw "Move requires finite planar coordinates";
    if (object(id) == null) throw "Unknown scene object: " + id;
    if (fromX == toX && fromY == toY) return false;
    return document.record(new EditOperation("Move object",
      function() setPositionXY(id, toX, toY),
      function() setPositionXY(id, fromX, fromY)));
  }

  public function nudgeSelected(deltaX:Float, deltaY:Float):Bool {
    var item = object(selectedId);
    if (item == null || !finiteCoordinate(deltaX) || !finiteCoordinate(deltaY) ||
        (deltaX == 0.0 && deltaY == 0.0)) return false;
    var transform = info(item.id).localTransform();
    var fromX = transform.element(12);
    var fromY = transform.element(13);
    var toX = Math.max(-1000000.0, Math.min(1000000.0, fromX + deltaX));
    var toY = Math.max(-1000000.0, Math.min(1000000.0, fromY + deltaY));
    if (fromX == toX && fromY == toY) return false;
    return document.apply(new EditOperation("Nudge object",
      function() setPositionXY(item.id, toX, toY),
      function() setPositionXY(item.id, fromX, fromY)));
  }

  static inline function finiteCoordinate(value:Float):Bool
    return value == value && value - value == 0.0 && Math.abs(value) <= 1000000;

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

  public function setDimensions(id:String, width:Float, height:Float,?depth:Float):Void {
    var chosenDepth=depth==null?requiredObject(id).depth:depth;
    if (!validDimension(width) || !validDimension(height)||!validDimension(chosenDepth))
      throw "Rectangle dimensions must be finite and positive";
    if (object(id) == null) throw "Unknown scene object: " + id;
    var data = records();
    for (item in data) if (item.id == id) { item.width=width;item.height=height;item.depth=chosenDepth; }
    replaceObjects(data, selectedId);
  }

  public function setColour(id:String, red:Float, green:Float, blue:Float):Void {
    if (!validColour(red) || !validColour(green) || !validColour(blue))
      throw "Rectangle colour channels must be finite values from 0 to 1";
    if (object(id) == null) throw "Unknown scene object: " + id;
    var data = records();
    for (item in data) if (item.id == id) { item.red = red; item.green = green; item.blue = blue; }
    replaceObjects(data, selectedId);
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
    nextEnvironmentRevision++;
    environmentRevision = nextEnvironmentRevision;
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
    var name = new PropertyDescriptorOptions();
    name.category = "Object";
    name.validator = function(_, value) return switch (value) {
      case PropertyValue.Text(text):
        StringTools.trim(text).length == 0 || SceneCodec.containsNul(text)
          ? "Name cannot be empty or contain NUL" : null;
      default: "Name requires text";
    };
    result.push(new PropertyDescriptor(prefix + "name", "Name", PropertyType.Text,
      function(_) {
        var item = object(id);
        if (item == null) throw "Unknown scene object: " + id;
        return PropertyValue.Text(item.label);
      }, function(_, value) {
        switch (value) {
          case PropertyValue.Text(text): setName(id, text);
          default: throw "Name requires text";
        }
      }, name));
    result.push(dimensionProperty(id, 0, prefix));
    result.push(dimensionProperty(id, 1, prefix));
    var colour = new PropertyDescriptorOptions();
    colour.category = "Rendering";
    colour.validator = function(_, value) return switch (value) {
      case PropertyValue.Text(text): decodeColour(text) == null ? "Colour requires #RRGGBB" : null;
      default: "Colour requires #RRGGBB";
    };
    // Property history stores the displayed text. Retain exact channel snapshots so
    // undoing a hex edit restores values loaded from a document without quantizing them.
    var colourSnapshots:Map<String, Array<Float>> = new Map();
    var initialColour = requiredObject(id);
    colourSnapshots.set(encodeColour(initialColour.red, initialColour.green, initialColour.blue),
      [initialColour.red, initialColour.green, initialColour.blue]);
    result.push(new PropertyDescriptor(prefix + "colour", "Colour", PropertyType.Text,
      function(_) {
        var item = requiredObject(id);
        return PropertyValue.Text(encodeColour(item.red, item.green, item.blue));
      }, function(_, value) {
        switch (value) {
          case PropertyValue.Text(text):
            var key = text.toUpperCase();
            var channels = colourSnapshots.get(key);
            if (channels == null) channels = decodeColour(key);
            if (channels == null) throw "Colour requires #RRGGBB";
            var current = requiredObject(id);
            colourSnapshots.set(encodeColour(current.red, current.green, current.blue),
              [current.red, current.green, current.blue]);
            setColour(id, channels[0], channels[1], channels[2]);
          default: throw "Colour requires #RRGGBB";
        }
      }, colour));
    result.push(dimensionProperty(id, 2, prefix));
    result.push(boolProperty(id,"collision","Collision enabled",function(item)return item.collisionEnabled,
      "Physics",prefix));
    result.push(boolProperty(id,"dynamic","Dynamic body",function(item)return item.dynamicBody,
      "Physics",prefix));
    result.push(numberProperty(id,"mass","Mass",function(item)return item.mass,
      0.000001,1000000.0,"kg","Physics",prefix));
    return result;
  }

  function dimensionProperty(id:String, axis:Int, prefix:String):PropertyDescriptor {
    var settings = new PropertyDescriptorOptions();
    settings.category = "Geometry";
    settings.unit = "m";
    settings.minimum = 0.000001;
    settings.maximum = 1000000.0;
    settings.step = 0.1;
    settings.validator = function(_, value) {
      var number:Null<Float> = switch (value) {
        case PropertyValue.Float(next): next;
        case PropertyValue.Int(next): next;
        default: null;
      };
      return number == null || !validDimension(number) ? "Dimension must be finite and positive" : null;
    };
    var key=["width","height","depth"][axis],label=["Width","Height","Depth"][axis];
    return new PropertyDescriptor(prefix + key, label,
      PropertyType.Float, function(_) {
        var item = requiredObject(id);
        return PropertyValue.Float(axis==0?item.width:axis==1?item.height:item.depth);
      }, function(_, value) {
        var item = requiredObject(id);
        var number:Float = switch (value) {
          case PropertyValue.Float(next): next;
          case PropertyValue.Int(next): next;
          default: throw "Dimension requires a number";
        };
        setDimensions(id,axis==0?number:item.width,axis==1?number:item.height,axis==2?number:item.depth);
      }, settings);
  }

  function boolProperty(id:String,key:String,label:String,read:EditorSceneObject->Bool,
      category:String,prefix:String):PropertyDescriptor {
    var options=new PropertyDescriptorOptions();options.category=category;
    return new PropertyDescriptor(prefix+key,label,PropertyType.Bool,
      function(_)return PropertyValue.Bool(read(requiredObject(id))),function(_,value)switch value {
        case Bool(next):
          var data=records();for(item in data)if(item.id==id)Reflect.setField(item,key=="collision"?"collisionEnabled":"dynamicBody",next);
          replaceObjects(data,selectedId);
        default:throw label+" requires a boolean";
      },options);
  }
  function numberProperty(id:String,key:String,label:String,read:EditorSceneObject->Float,
      min:Float,max:Float,unit:String,category:String,
      prefix:String):PropertyDescriptor {
    var options=new PropertyDescriptorOptions();options.category=category;options.minimum=min;
    options.maximum=max;options.unit=unit;options.step=0.1;
    return new PropertyDescriptor(prefix+key,label,PropertyType.Float,
      function(_)return PropertyValue.Float(read(requiredObject(id))),function(_,value){
        var next:Float=switch value{case Float(v):v;case Int(v):v;default:throw label+" requires a number";};
        if(!Math.isFinite(next)||next<min||next>max)throw label+" is out of range";
        var data=records();for(item in data)if(item.id==id)item.mass=next;
        replaceObjects(data,selectedId);
      },options);
  }

  function requiredObject(id:String):EditorSceneObject {
    var item = object(id);
    if (item == null) throw "Unknown scene object: " + id;
    return item;
  }

  static function boxGeometry(width:Float, height:Float, depth:Float):GeometryData {
    var mesh = new GeometryData();
    var halfWidth = width / 2, halfHeight = height / 2, halfDepth = depth / 2;
    mesh.addVertex(-halfWidth, -halfHeight, -halfDepth);
    mesh.addVertex(halfWidth, -halfHeight, -halfDepth);
    mesh.addVertex(halfWidth, halfHeight, -halfDepth);
    mesh.addVertex(-halfWidth, halfHeight, -halfDepth);
    mesh.addVertex(-halfWidth, -halfHeight, halfDepth);
    mesh.addVertex(halfWidth, -halfHeight, halfDepth);
    mesh.addVertex(halfWidth, halfHeight, halfDepth);
    mesh.addVertex(-halfWidth, halfHeight, halfDepth);
    for (triangle in [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
        [0, 1, 5], [0, 5, 4], [3, 7, 6], [3, 6, 2],
        [0, 4, 7], [0, 7, 3], [1, 2, 6], [1, 6, 5]])
      mesh.addTriangle(triangle[0], triangle[1], triangle[2]);
    mesh.setBounds(-halfWidth, -halfHeight, -halfDepth, halfWidth, halfHeight, halfDepth);
    return mesh;
  }

  static inline function validDimension(value:Float):Bool
    return value == value && value - value == 0.0 && value >= 0.000001 && value <= 1000000.0;

  static inline function validColour(value:Float):Bool
    return value == value && value - value == 0.0 && value >= 0.0 && value <= 1.0;

  static function encodeColour(red:Float, green:Float, blue:Float):String
    return "#" + StringTools.hex(Math.round(red * 255), 2) +
      StringTools.hex(Math.round(green * 255), 2) + StringTools.hex(Math.round(blue * 255), 2);

  static function decodeColour(value:String):Null<Array<Float>> {
    if (value == null || value.length != 7 || value.charAt(0) != "#") return null;
    var channels:Array<Float> = [];
    for (offset in [1, 3, 5]) {
      var pair = value.substr(offset, 2);
      for (index in 0...2) {
        var code = pair.charCodeAt(index);
        if (!((code >= 48 && code <= 57) || (code >= 65 && code <= 70) ||
            (code >= 97 && code <= 102))) return null;
      }
      var parsed = Std.parseInt("0x" + pair);
      if (parsed == null) return null;
      channels.push(parsed / 255.0);
    }
    return channels;
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
        width:item.width,height:item.height,depth:item.depth,collisionEnabled:item.collisionEnabled,
        dynamicBody:item.dynamicBody,mass:item.mass,red:item.red,green:item.green,blue:item.blue,
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
  public var label:String;
  public final occurrence:Occurrence;
  public final width:Float;
  public final height:Float;
  public final depth:Float;
  public final collisionEnabled:Bool;
  public final dynamicBody:Bool;
  public final mass:Float;
  public final red:Float;
  public final green:Float;
  public final blue:Float;
  public function new(id:String,label:String,occurrence:Occurrence,width:Float,height:Float,depth:Float,
      collisionEnabled:Bool,dynamicBody:Bool,mass:Float,red:Float,green:Float,blue:Float) {
    this.id = id; this.label = label; this.occurrence = occurrence;
    this.width=width;this.height=height;this.depth=depth;this.collisionEnabled=collisionEnabled;
    this.dynamicBody=dynamicBody;this.mass=mass;
    this.red = red; this.green = green; this.blue = blue;
  }
}
