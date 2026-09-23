package app;

import nativekit.scene.Scene;
import nativekit.scene.SceneSnapshot;
import nativekit.scene.SpatialIndex;
import nativekit.scene.NodeId;
import nativekit.scene.SceneNode;
import nativekit.scene.GeometryData;
import nativekit.scene.Geometry;
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
import nativekit.scene.PickResult;
import cadkit.parametric.TopologyFingerprint;
import app.CadPlateModel.CadPlateParameters;
import app.CadPlateModel.CadPlateHoleEdit;
import app.CadBracketModel;

/** One scene and one document shared by the hierarchy, inspector and viewport. */
class EditorScene {
  // Retained UI caches survive document replacement, so revisions must too.
  static var nextRevision:Int = 0;
  static var nextEnvironmentRevision:Int = 0;
  public final document:EditorDocument;
  var bridge:SceneBridge;
  var scene(get, never):Scene;
  var objects:Array<EditorSceneObject>;
  var cadSessions:Map<String, CadDocumentSession>;
  var nextObjectId:Int = 1;
  var snapshot:SceneSnapshot;
  var spatial:SpatialIndex;
  var selectionMaterial:Material;
  public var selectedId(default, null):String = "box";
  public var selectedCadFaceIndex(default, null):Int = -1;
  public var selectedCadFaceX(default, null):Float = 0.0;
  public var selectedCadFaceY(default, null):Float = 0.0;
  var selectedCadFaceFingerprint:Null<TopologyFingerprint> = null;
  var selectedFeatureKey:Null<String> = null;
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
    objects = [];
    cadSessions = new Map();
    bridge = new SceneBridge();
    try {
      if (data == null) {
        addObject("box", "Blue box", -1.5, 0.0, 0.0, 1.6, 1.2, 0.1, 0.22, 0.52, 0.85);
        addObject("tower", "Orange tower", 1.1, 0.0, 0.1, 1.2, 1.8, 0.2, 0.92, 0.48, 0.22);
      } else {
        for (item in data) addObject(item.id, item.label, item.x, item.y, item.z,
          item.width, item.height, item.depth, item.red, item.green, item.blue, item.visible,
          item.collisionEnabled,item.dynamicBody,item.mass,item.type,item.cadGraph);
        selectedId = data.length == 0 ? "scene" : data[0].id;
      }
      selectionMaterial = scene.createMaterial();
      scene.setMaterialData(selectionMaterial, MaterialData.opaque(1.0, 0.88, 0.35));
      snapshot = scene.snapshot();
      try spatial = SpatialIndex.create(snapshot)
      catch (error:Dynamic) { snapshot.dispose(); throw error; }
    } catch (error:Dynamic) {
      for (session in cadSessions) session.close();
      cadSessions = new Map();
      bridge.dispose();
      throw error;
    }
  }

  function addObject(id:String, label:String, x:Float, y:Float, z:Float,
      width:Float, height:Float, depth:Float, red:Float, green:Float, blue:Float,
      visible:Bool = true,collisionEnabled:Bool=true,dynamicBody:Bool=false,mass:Float=1.0,
      kind:String="rectangle",?cadGraph:String):Void {
    var geometry = scene.createGeometry();
    var session:Null<CadDocumentSession> = null;
    var storedCadGraph = cadGraph;
    var geometryData:GeometryData;
    if (isCadKind(kind)) {
      session = createCadSession(storedCadGraph, width, height, depth, kind);
      if (storedCadGraph == null)
        storedCadGraph = session.encode();
      geometryData = session.geometry();
    } else {
      geometryData = boxGeometry(width, height, depth);
    }
    try {
      scene.setGeometryData(geometry, geometryData);
      var material = scene.createMaterial();
      scene.setMaterialData(material, MaterialData.opaque(red, green, blue));
      var transaction = scene.beginTransaction();
      try {
        var node = transaction.createNode();
        transaction.setName(node, label);
        transaction.setVisibility(node, visible);
        transaction.setGeometry(node, geometry);
        transaction.setMaterial(node, material);
        transaction.setTransform(node, Transform.identity().translated(x, y, z));
        transaction.commit();
        bridge.attach(id, node, geometry, material);
        objects.push(new EditorSceneObject(id, label, kind, width, height, depth,
          collisionEnabled,dynamicBody,mass,red,green,blue,
          storedCadGraph,
          x, y, z, visible));
        if (session != null)
          cadSessions.set(id, session);
      } catch (error:Dynamic) {
        transaction.dispose();
        throw error;
      }
    } catch (error:Dynamic) {
      if (session != null)
        session.close();
      throw error;
    }
  }

  public function canCreate():Bool return objects.length < 10000;

  function allocateId(prefix:String="rectangle"):String {
    var id = prefix + "-" + nextObjectId;
    nextObjectId++;
    while (object(id) != null) {
      id = prefix + "-" + nextObjectId;
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

  public function createMountingPlate():Bool {
    if (!canCreate()) return false;
    var data = records(), id = allocateId("plate");
    var model = CadPlateModel.create(0.08, 0.05, 0.006, 0.012);
    var graph = model.encode();
    model.close();
    data.push({id:id, label:"Mounting plate", type:"cad-plate", x:0.0, y:0.0, z:0.0,
      width:0.08, height:0.05, depth:0.006, collisionEnabled:false, dynamicBody:false, mass:1.0,
      red:0.34, green:0.62, blue:0.78, visible:true, cadGraph:graph});
    return changeObjects("Create mounting plate", data, id);
  }

  public function createBracket():Bool {
    if (!canCreate()) return false;
    var data=records(),id=allocateId("bracket");
    data.push({id:id,label:"L bracket",type:"cad-bracket",x:0.0,y:0.0,z:0.0,
      width:0.06,height:0.04,depth:0.03,collisionEnabled:false,dynamicBody:false,mass:1.0,
      red:0.64,green:0.66,blue:0.70,visible:true});
    return changeObjects("Create L bracket",data,id);
  }

  public function exportSelectedCad(path:String):Void {
    var item=requiredObject(selectedId);
    if(!isCadKind(item.kind))throw "Select a CAD part to export";
    requireCadSession(item.id).model.exportStep(path);
  }

  public function setBracketHoleRadius(id:String,value:Float):Void {
    if(requiredObject(id).kind!="cad-bracket")throw "Object is not a CAD bracket";
    var model=cadBracketModel(id),before=model.holeRadius();
    if(before==value)return;
    applyCadEdit(id,"Edit bracket hole radius",function(session) {
      var target:CadBracketModel=cast session.model;
      target.setHoleRadius(value);
    },function(session) {
      var target:CadBracketModel=cast session.model;
      target.setHoleRadius(before);
    });
  }

  public function setBracketWallThickness(id:String,value:Float):Void {
    if(requiredObject(id).kind!="cad-bracket")throw "Object is not a CAD bracket";
    var model=cadBracketModel(id),before=model.wallThickness();
    if(before==value)return;
    applyCadEdit(id,"Edit bracket wall thickness",function(session) {
      var target:CadBracketModel=cast session.model;
      target.setWallThickness(value);
    },function(session) {
      var target:CadBracketModel=cast session.model;
      target.setWallThickness(before);
    });
  }

  public function isCadPart(id:String):Bool {
    var item=object(id);
    return item!=null&&isCadKind(item.kind);
  }

  public function canAddHoleOnSelectedFace():Bool {
    var item=object(selectedId);
    return item!=null&&item.kind=="cad-plate"&&selectedCadFaceIndex>=0;
  }

  public function addHoleOnSelectedFace(diameter:Float=0.008):Bool {
    if(!canAddHoleOnSelectedFace())return false;
    var id=selectedId,faceIndex=selectedCadFaceIndex,x=selectedCadFaceX,y=selectedCadFaceY;
    var edit:Null<CadPlateHoleEdit> = null;
    return applyCadEdit(id,"Add through hole",function(session) {
      var model:CadPlateModel=cast session.model;
      if(edit==null)edit=model.addThroughHole(faceIndex,x,y,diameter);
      else model.setHoleEditActive(edit,true);
    },function(session) {
      if(edit==null)throw "CAD hole edit was not created";
      var model:CadPlateModel=cast session.model;
      model.setHoleEditActive(edit,false);
    });
  }

  public function duplicateSelected():Bool {
    if (!canCreate() || object(selectedId) == null) return false;
    var data = records();
    var source:SceneObjectData = null;
    for (item in data) if (item.id == selectedId) source = item;
    var id = allocateId();
    var cadGraph = isCadKind(source.type) ? currentCadGraph(source.id) : source.cadGraph;
    data.push({id: id, label: source.label + " copy", type: source.type,
      x: Math.min(1000000, source.x + 0.25), y: Math.min(1000000, source.y + 0.25), z: source.z,
      width: source.width, height: source.height, red: source.red, green: source.green,
      blue: source.blue, visible: source.visible,depth:source.depth,
      collisionEnabled:source.collisionEnabled,dynamicBody:source.dynamicBody,mass:source.mass,
      cadGraph:cadGraph});
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
    var afterIds:Map<String, Bool> = new Map();
    for (record in after) afterIds.set(record.id, true);
    for (record in before) if (isCadKind(record.type) && !afterIds.exists(record.id))
      record.cadGraph = currentCadGraph(record.id);
    var previousSelection = selectedId;
    return document.apply(new EditOperation(label,
      function() replaceObjects(after, selection),
      function() replaceObjects(before, previousSelection)));
  }

  function applyCadEdit(id:String, label:String, redo:CadDocumentSession->Void,
      undo:CadDocumentSession->Void):Bool {
    return document.apply(new EditOperation(label,
      function() runCadEdit(id, redo, undo),
      function() runCadEdit(id, undo, redo)));
  }

  function runCadEdit(id:String, edit:CadDocumentSession->Void,
      rollback:CadDocumentSession->Void):Void {
    var session = requireCadSession(id);
    session.perform(edit);
    try {
      syncCadSession(id);
    } catch (error:Dynamic) {
      try {
        session.perform(rollback);
        syncCadSession(id);
      } catch (_:Dynamic) {}
      throw error;
    }
  }

  function syncCadSession(id:String):Void {
    var item = requiredObject(id);
    var session = requireCadSession(id);
    scene.setGeometryData(runtimeFor(id).geometry, session.geometry());
    var values = session.model.sceneDimensions();
    item.width = values.width;
    item.height = values.height;
    item.depth = values.depth;
    if (id == selectedId && selectedCadFaceFingerprint != null) {
      var prior = selectedCadFaceFingerprint;
      var priorIndex = selectedCadFaceIndex;
      var index = session.model.remapFace(prior);
      selectedCadFaceIndex = index;
      if (index >= 0) {
        selectedCadFaceFingerprint = session.model.faceFingerprint(index);
        session.selectedTopology = selectedCadFaceFingerprint;
      } else {
        selectedCadFaceFingerprint = null;
        session.selectedTopology = null;
      }
      if (priorIndex != index) {
        selectionRevision++;
        nextRevision++;
        revision = nextRevision;
      }
    }
    publish();
  }

  // Reconcile document records into the runtime scene while preserving stable nodes.
  function replaceObjects(data:Array<SceneObjectData>, selection:String):Void {
    var previousFace=selectedCadFaceFingerprint;
    var previousFaceX=selectedCadFaceX,previousFaceY=selectedCadFaceY;
    var current:Map<String, EditorSceneObject> = new Map();
    for (item in objects) current.set(item.id, item);
    var nextObjects:Array<EditorSceneObject> = [];
    var nextCadSessions:Map<String, CadDocumentSession> = new Map();
    var stagedCadSessions:Array<CadDocumentSession> = [];
    var retained:Map<String, Bool> = new Map();
    var transaction = scene.beginTransaction();
    var changed = false;
    try {
      for (record in data) {
        var item = current.get(record.id);
        var session = item == null || !isCadKind(item.kind) || item.kind != record.type || item.cadGraph != record.cadGraph
          ? null : cadSessions.get(record.id);
        var storedGraph = record.cadGraph;
        if (isCadKind(record.type) && session == null) {
          session = createCadSession(storedGraph, record.width, record.height, record.depth, record.type);
          stagedCadSessions.push(session);
          storedGraph = session.encode();
        }
        if (session != null)
          nextCadSessions.set(record.id, session);
        if (item == null) {
          var geometry = scene.createGeometry();
          var geometryData = session != null
            ? session.geometry()
            : boxGeometry(record.width, record.height, record.depth);
          scene.setGeometryData(geometry, geometryData);
          var material = scene.createMaterial();
          scene.setMaterialData(material, MaterialData.opaque(record.red, record.green, record.blue));
          var node = transaction.createNode();
          transaction.setName(node, record.label);
          transaction.setVisibility(node, record.visible);
          transaction.setGeometry(node, geometry);
          transaction.setMaterial(node, material);
          transaction.setTransform(node, Transform.identity().translated(record.x, record.y, record.z));
          bridge.attach(record.id, node, geometry, material);
          item = new EditorSceneObject(record.id, record.label, record.type,
            record.width, record.height, record.depth, record.collisionEnabled,
            record.dynamicBody, record.mass, record.red, record.green, record.blue,
            storedGraph,
            record.x, record.y, record.z, record.visible);
          changed = true;
        } else {
          var runtime = runtimeFor(record.id);
          if (item.label != record.label) transaction.setName(runtime.node, record.label);
          if (item.visible != record.visible) transaction.setVisibility(runtime.node, record.visible);
          if (item.x != record.x || item.y != record.y || item.z != record.z)
            transaction.setTransform(runtime.node, Transform.identity().translated(record.x, record.y, record.z));
          if (item.label != record.label || item.visible != record.visible || item.x != record.x ||
              item.y != record.y || item.z != record.z) changed = true;
          if (item.width != record.width || item.height != record.height || item.depth != record.depth ||
              item.kind != record.type || item.cadGraph != storedGraph) {
            var geometryData = session != null
              ? session.geometry()
              : boxGeometry(record.width, record.height, record.depth);
            scene.setGeometryData(runtime.geometry, geometryData);
            changed = true;
          }
          if (item.red != record.red || item.green != record.green || item.blue != record.blue) {
            scene.setMaterialData(runtime.material, MaterialData.opaque(record.red, record.green, record.blue));
            changed = true;
          }
          if (item.collisionEnabled != record.collisionEnabled || item.dynamicBody != record.dynamicBody ||
              item.mass != record.mass) changed = true;
          item.label = record.label; item.kind = record.type;
          item.width = record.width; item.height = record.height; item.depth = record.depth;
          item.collisionEnabled = record.collisionEnabled; item.dynamicBody = record.dynamicBody;
          item.mass = record.mass; item.red = record.red; item.green = record.green; item.blue = record.blue;
          item.cadGraph = storedGraph; item.x = record.x; item.y = record.y; item.z = record.z;
          item.visible = record.visible;
        }
        retained.set(record.id, true);
        nextObjects.push(item);
      }
      for (item in objects) if (!retained.exists(item.id)) {
        transaction.destroyNode(runtimeFor(item.id).node);
        changed = true;
      }
      transaction.commit();
    } catch (error:Dynamic) {
      transaction.dispose();
      for (session in stagedCadSessions) session.close();
      throw error;
    }
    for (id in cadSessions.keys()) {
      var previous = cadSessions.get(id);
      if (previous != null && nextCadSessions.get(id) != previous)
        previous.close();
    }
    cadSessions = nextCadSessions;
    // Release resources after nodes no longer reference them.
    for (item in objects) if (!retained.exists(item.id)) {
      var runtime = runtimeFor(item.id);
      runtime.geometry.dispose(); runtime.material.dispose();
      bridge.detach(item.id);
    }
    objects = nextObjects;
    if (changed) publish();
    selectedId = selection;
    selectedFeatureKey=null;
    selectedCadFaceIndex=-1;selectedCadFaceFingerprint=null;
    if(previousFace!=null){
      var selected=object(selection);
      if(selected!=null&&isCadKind(selected.kind)){
        var session=cadSessions.get(selected.id);
        if(session!=null)try {
          selectedCadFaceIndex=session.model.remapFace(previousFace);
          if(selectedCadFaceIndex>=0){selectedCadFaceFingerprint=session.model.faceFingerprint(selectedCadFaceIndex);
            selectedCadFaceX=previousFaceX;selectedCadFaceY=previousFaceY;
            session.selectedTopology=selectedCadFaceFingerprint;}
        } catch(error:Dynamic) { selectedCadFaceIndex=-1; selectedCadFaceFingerprint=null; }
      }
    }
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
      transaction.setName(runtimeFor(id).node, label);
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

  function runtimeFor(id:String):EditorSceneRuntimeObject {
    var value = bridge.runtime(id);
    if (value == null) throw "Missing runtime scene node: " + id;
    return value;
  }

  public function info(id:String):SceneNode {
    var item = object(id);
    if (item == null) throw "Unknown scene object: " + id;
    var value = snapshot.findNode(runtimeFor(id).node);
    if (value == null) throw "Missing scene node: " + id;
    return value;
  }

  public function renderSnapshot():SceneSnapshot return snapshot;

  public function configureRenderView(view:SceneView, viewProjection:Transform,
      ?poses:Array<SimulationPoseVisual>):SceneView {
    view.setViewProjection(viewProjection);
    var selected = object(selectedId);
    var selection = new SelectionSet();
    if (selected != null) selection.add(runtimeFor(selected.id).node);
    view.applySelection(selection, selectionMaterial);
    if (poses != null) for (pose in poses) {
      if (object(pose.id) != null)
        view.setPose(runtimeFor(pose.id).node, poseTransform(pose.position, pose.rotation));
    }
    return view;
  }

  public function selectRayWithView(view:SceneView, originX:Float, originY:Float, originZ:Float,
      directionX:Float, directionY:Float, directionZ:Float):String {
    var index = SpatialIndex.create(snapshot, view);
    try {
      var hit = index.pickRay(originX, originY, originZ, directionX, directionY, directionZ);
      index.dispose();
      return selectHit(hit);
    } catch (error:Dynamic) { index.dispose(); throw error; }
  }

  public function pickRayWithView(view:SceneView, originX:Float, originY:Float, originZ:Float,
      directionX:Float, directionY:Float, directionZ:Float):String {
    var index = SpatialIndex.create(snapshot, view);
    try {
      var hit = index.pickRay(originX, originY, originZ, directionX, directionY, directionZ);
      index.dispose();
      return idForHit(hit);
    } catch (error:Dynamic) { index.dispose(); throw error; }
  }

  static function poseTransform(position:Array<Float>, rotation:Array<Float>):Transform {
    if (position == null || position.length != 3 || rotation == null || rotation.length != 4)
      throw "Invalid presentation pose";
    var x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3];
    return Transform.identity()
      .set(0,1-2*(y*y+z*z)).set(1,2*(x*y+z*w)).set(2,2*(x*z-y*w))
      .set(4,2*(x*y-z*w)).set(5,1-2*(x*x+z*z)).set(6,2*(y*z+x*w))
      .set(8,2*(x*z+y*w)).set(9,2*(y*z-x*w)).set(10,1-2*(x*x+y*y))
      .translated(position[0],position[1],position[2]);
  }

  public function select(id:String):Bool {
    if (id != "scene" && object(id) == null) return false;
    if (id == selectedId && selectedFeatureKey==null) return false;
    var previousSession=cadSessions.get(selectedId);
    if(previousSession!=null)previousSession.selectedTopology=null;
    selectedId = id;
    selectedFeatureKey=null;selectedCadFaceIndex=-1;selectedCadFaceFingerprint=null;
    selectionRevision++;
    nextRevision++;
    revision = nextRevision;
    return true;
  }

  public function treeSelectionKey():String return selectedFeatureKey==null?selectedId:selectedFeatureKey;

  public function selectTreeKey(key:String):Bool {
    if(key=="scene"||object(key)!=null)return select(key);
    var marker=key.indexOf(":feature:");
    if(marker<0)return select(key);
    var id=key.substr(0,marker),item=object(id);
    if(item==null||item.kind!="cad-plate")return false;
    if(selectedId==id&&selectedFeatureKey==key)return false;
    selectedId=id;selectedFeatureKey=key;
    selectedCadFaceIndex=-1;selectedCadFaceFingerprint=null;
    selectionRevision++;nextRevision++;revision=nextRevision;
    return true;
  }

  public function cadFeatureNames(id:String):Array<String> {
    var item=object(id);
    if(item==null||!isCadKind(item.kind))return [];
    return requireCadSession(id).model.featureNames();
  }

  public function selectAtXY(x:Float,y:Float):String
    return selectHit(spatial.pickRay(x,y,1000001.0,0.0,0.0,-1.0));

  public function selectAtRay(originX:Float,originY:Float,originZ:Float,
      directionX:Float,directionY:Float,directionZ:Float):String
    return selectHit(spatial.pickRay(originX,originY,originZ,directionX,directionY,directionZ));

  function selectHit(hit:PickResult):String {
    var previousFace=selectedCadFaceIndex;
    var id=idForHit(hit);
    select(id);
    selectedCadFaceIndex=-1;selectedCadFaceFingerprint=null;
    var item=object(id);
    if(item!=null&&isCadKind(item.kind)&&hit.subelement()>=0){
      var session=requireCadSession(id);
      try {
        var index=hit.subelement();
        selectedCadFaceFingerprint=session.model.faceFingerprint(index);
        selectedCadFaceIndex=index;
        selectedCadFaceX=hit.worldX()-item.x;
        selectedCadFaceY=hit.worldY()-item.y;
        session.selectedTopology=selectedCadFaceFingerprint;
      } catch(error:Dynamic){selectedCadFaceIndex=-1;selectedCadFaceFingerprint=null;session.selectedTopology=null;}
    }
    if(previousFace!=selectedCadFaceIndex){selectionRevision++;nextRevision++;revision=nextRevision;}
    return id;
  }

  function idForHit(hit:PickResult):String {
    for (item in objects) if (hit.node().equals(runtimeFor(item.id).node)) return item.id;
    return "scene";
  }

  public function context():CommandContext {
    return new CommandContext(document, selectedId == "scene" ? [] : [selectedId],
      "scene-viewport", null, "scene-editor");
  }

  /** Orthographic world-space picking against the same published geometry. */
  public function pick(x:Float, y:Float):String {
    var hit = spatial.pickRay(x, y, 1000001.0, 0.0, 0.0, -1.0);
    for (item in objects) if (hit.node().equals(runtimeFor(item.id).node)) return item.id;
    return "scene";
  }

  public function pickRay(originX:Float,originY:Float,originZ:Float,
      directionX:Float,directionY:Float,directionZ:Float):String {
    var hit=spatial.pickRay(originX,originY,originZ,directionX,directionY,directionZ);
    for(item in objects)if(hit.node().equals(runtimeFor(item.id).node))return item.id;
    return "scene";
  }

  public function setPosition(id:String, axis:Int, value:Float):Void {
    if (axis < 0 || axis > 1 || value != value || value - value != 0.0 || Math.abs(value) > 1000000)
      throw "Position requires a finite X or Y coordinate";
    var item = requiredObject(id);
    setPositionXY(id, axis == 0 ? value : item.x, axis == 1 ? value : item.y);
  }

  /** Updates both planar coordinates atomically; used for live viewport previews. */
  public function setPositionXY(id:String, x:Float, y:Float):Void {
    if (!finiteCoordinate(x) || !finiteCoordinate(y))
      throw "Position requires finite X and Y coordinates";
    var item = object(id);
    if (item == null) throw "Unknown scene object: " + id;
    var current = Transform.identity().translated(x, y, item.z);
    var transaction = scene.beginTransaction();
    try {
      transaction.setTransform(runtimeFor(id).node, current);
      transaction.commit();
    } catch (error:Dynamic) { transaction.dispose(); throw error; }
    item.x = x; item.y = y;
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
    var fromX = item.x;
    var fromY = item.y;
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
      transaction.setVisibility(runtimeFor(id).node, visible);
      transaction.commit();
    } catch (error:Dynamic) { transaction.dispose(); throw error; }
    item.visible = visible;
    publish();
  }

  public function setDimensions(id:String, width:Float, height:Float,?depth:Float):Void {
    var chosenDepth=depth==null?requiredObject(id).depth:depth;
    if (!validDimension(width) || !validDimension(height)||!validDimension(chosenDepth))
      throw "Rectangle dimensions must be finite and positive";
    var target = object(id);
    if (target == null) throw "Unknown scene object: " + id;
    if (isCadKind(target.kind)) {
      var before = requireCadSession(id).model.sceneDimensions();
      if(before.width==width&&before.height==height&&before.depth==chosenDepth)return;
      applyCadEdit(id,"Edit CAD dimensions",
        function(session)session.model.setSceneDimensions(width,height,chosenDepth),
        function(session)session.model.setSceneDimensions(before.width,before.height,before.depth));
      return;
    }
    var data = records();
    for (item in data) if (item.id == id) {
      item.width=width; item.height=height; item.depth=chosenDepth;
    }
    replaceObjects(data, selectedId);
  }

  public function setCadParameter(id:String, name:String, value:Float):Void {
    if (!validDimension(value) && name != CadPlateModel.HOLE_X && name != CadPlateModel.HOLE_Y)
      throw "CAD dimensions must be finite and positive";
    if ((name == CadPlateModel.HOLE_X || name == CadPlateModel.HOLE_Y) && !finiteCoordinate(value))
      throw "Hole position must be finite";
    var target = requiredObject(id);
    if (target.kind != "cad-plate") throw "Object is not a CAD plate";
    var values=cadPlateModel(id).parameters();
    var before:Float=switch(name) {
      case CadPlateModel.WIDTH:values.width;
      case CadPlateModel.HEIGHT:values.height;
      case CadPlateModel.THICKNESS:values.thickness;
      case CadPlateModel.HOLE_DIAMETER:values.holeDiameter;
      case CadPlateModel.HOLE_X:values.holeX;
      case CadPlateModel.HOLE_Y:values.holeY;
      default:throw "Unknown CAD parameter";
    };
    var next=[{name:name,value:value}];
    var previous=[{name:name,value:before}];
    applyCadEdit(id,"Edit CAD parameter",function(session) {
      var model:CadPlateModel=cast session.model;
      model.setMetreValues(next);
    },function(session) {
      var model:CadPlateModel=cast session.model;
      model.setMetreValues(previous);
    });
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
      function(_) return PropertyValue.Bool(requiredObject(id).visible), function(_, value) {
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
    if (requiredObject(id).kind == "cad-plate") {
      result.push(cadProperty(id,CadPlateModel.HOLE_DIAMETER,"Hole diameter",false,prefix));
      result.push(cadProperty(id,CadPlateModel.HOLE_X,"Hole X",true,prefix));
      result.push(cadProperty(id,CadPlateModel.HOLE_Y,"Hole Y",true,prefix));
    } else if (requiredObject(id).kind == "cad-bracket") {
      result.push(bracketProperty(id,"wall","Wall thickness",function(model)return model.wallThickness(),
        function(value)setBracketWallThickness(id,value),prefix));
      result.push(bracketProperty(id,"hole-radius","Hole radius",function(model)return model.holeRadius(),
        function(value)setBracketHoleRadius(id,value),prefix));
    }
    result.push(boolProperty(id,"collision","Collision",function(item)return item.collisionEnabled,
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

  function cadProperty(id:String, name:String, label:String, signed:Bool,
      prefix:String):PropertyDescriptor {
    var options = new PropertyDescriptorOptions();
    options.category = "Geometry"; options.unit = "m"; options.step = 0.001;
    options.minimum = signed ? -1000000.0 : 0.000001; options.maximum = 1000000.0;
    options.validator = function(_, value) {
      var number:Null<Float> = switch value { case Float(v):v; case Int(v):v; default:null; };
      if (number == null || !Math.isFinite(number) || (!signed && number <= 0))
        return signed ? "Position must be finite" : "Dimension must be finite and positive";
      return null;
    };
    return new PropertyDescriptor(prefix+name,label,PropertyType.Float,function(_) {
      var values=cadPlateModel(id).parameters();
      return PropertyValue.Float(name==CadPlateModel.HOLE_DIAMETER?values.holeDiameter:
        name==CadPlateModel.HOLE_X?values.holeX:values.holeY);
    },function(_,value) {
      var number:Float=switch value {case Float(v):v;case Int(v):v;default:throw label+" requires a number";};
      setCadParameter(id,name,number);
    },options);
  }

  function bracketProperty(id:String,key:String,label:String,read:CadBracketModel->Float,
      write:Float->Void,prefix:String):PropertyDescriptor {
    var options=new PropertyDescriptorOptions();
    options.category="Geometry";options.unit="m";options.step=0.001;
    options.minimum=0.000001;options.maximum=1000000.0;
    options.validator=function(_,value) {
      var number:Null<Float> = switch value {case Float(v):v;case Int(v):v;default:null;};
      return number==null||!Math.isFinite(number)||number<=0
        ?"Dimension must be finite and positive":null;
    };
    return new PropertyDescriptor(prefix+key,label,PropertyType.Float,function(_) {
      return PropertyValue.Float(read(cadBracketModel(id)));
    },function(_,value) {
        var number:Float=switch value {case Float(v):v;case Int(v):v;default:throw label+" requires a number";};
        write(number);
      },options);
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
      PropertyType.Float, function(_) return PropertyValue.Float(axis == 0 ? requiredObject(id).x : requiredObject(id).y),
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
      result.push({id: item.id, label: item.label, type: item.kind,
        x: item.x, y: item.y, z: item.z,
        width:item.width,height:item.height,depth:item.depth,collisionEnabled:item.collisionEnabled,
        dynamicBody:item.dynamicBody,mass:item.mass,red:item.red,green:item.green,blue:item.blue,
        visible: item.visible,cadGraph:item.cadGraph});
    }
    return result;
  }

  /** Save boundary: serialize each live authored CAD document only when requested. */
  public function recordsForSave():Array<SceneObjectData> {
    var result=records();
    for(record in result)if(isCadKind(record.type))
      record.cadGraph=currentCadGraph(record.id);
    return result;
  }

  public function currentCadGraph(id:String):String
    return requireCadSession(id).encode();

  public function cadSession(id:String):CadDocumentSession
    return requireCadSession(id);

  public function cadParameters(id:String):CadPlateParameters
    return cadPlateModel(id).parameters();

  public function diagnosticState():Dynamic {
    var values:Array<Dynamic> = [];
    for (item in objects) {
      values.push({id: item.id, label: item.label, x: item.x, y: item.y, visible: item.visible});
    }
    return {objects: values, selectedId: selectedId, revision: revision,
      canUndo: document.canUndo, canRedo: document.canRedo, dirty: document.isDirty};
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    for (session in cadSessions) session.close();
    cadSessions = new Map();
    spatial.dispose();
    snapshot.dispose();
    bridge.dispose();
  }

  function get_scene():Scene return bridge.scene;

  function createCadSession(graph:Null<String>,width:Float,height:Float,depth:Float,
      kind:String="cad-plate"):CadDocumentSession {
    var model:CadSessionModel;
    if(kind=="cad-bracket")
      model=graph==null?CadBracketModel.create(width,height,depth):CadBracketModel.decode(graph);
    else
      model=graph==null?CadPlateModel.create(width,height,depth,
        Math.min(0.012,Math.min(width,height)*0.5)):CadPlateModel.decode(graph);
    try {
      return new CadDocumentSession(model);
    }
    catch(error:Dynamic){model.close();throw error;}
  }

  function cadPlateModel(id:String):CadPlateModel
    return cast requireCadSession(id).model;

  function cadBracketModel(id:String):CadBracketModel
    return cast requireCadSession(id).model;

  static function isCadKind(kind:String):Bool
    return kind=="cad-plate"||kind=="cad-bracket";

  function requireCadSession(id:String):CadDocumentSession {
    var result=cadSessions.get(id);
    if(result==null)throw "CAD document session is unavailable for: "+id;
    return result;
  }

}

class EditorSceneObject {
  public final id:String;
  public var label:String;
  public var kind:String;
  public var width:Float;
  public var height:Float;
  public var depth:Float;
  public var collisionEnabled:Bool;
  public var dynamicBody:Bool;
  public var mass:Float;
  public var red:Float;
  public var green:Float;
  public var blue:Float;
  public var cadGraph:Null<String>;
  public var x:Float;
  public var y:Float;
  public var z:Float;
  public var visible:Bool;
  public function new(id:String,label:String,kind:String,width:Float,height:Float,depth:Float,
      collisionEnabled:Bool,dynamicBody:Bool,mass:Float,red:Float,green:Float,blue:Float,
      ?cadGraph:String,x:Float=0,y:Float=0,z:Float=0,visible:Bool=true) {
    this.id = id; this.label = label; this.kind = kind;
    this.width=width;this.height=height;this.depth=depth;this.collisionEnabled=collisionEnabled;
    this.dynamicBody=dynamicBody;this.mass=mass;
    this.red = red; this.green = green; this.blue = blue;
    this.cadGraph=cadGraph;
    this.x=x;this.y=y;this.z=z;this.visible=visible;
  }
}

/** Runtime-only resources associated with one document object. */
private class SceneBridge {
  public final scene:Scene;
  final objects:Map<String, EditorSceneRuntimeObject> = new Map();

  public function new() scene = Scene.create();

  public function attach(id:String,node:NodeId,geometry:Geometry,material:Material):Void
    objects.set(id,new EditorSceneRuntimeObject(node,geometry,material));

  public function runtime(id:String):Null<EditorSceneRuntimeObject>
    return objects.get(id);

  public function detach(id:String):Void
    objects.remove(id);

  public function dispose():Void
    scene.dispose();
}

private class EditorSceneRuntimeObject {
  public final node:NodeId;
  public final geometry:Geometry;
  public final material:Material;
  public function new(node:NodeId, geometry:Geometry, material:Material) {
    this.node=node;this.geometry=geometry;this.material=material;
  }
}
