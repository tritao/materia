package app;

import nativekit.ui.widgets.controls.Select;
import nativekit.ui.widgets.text.Text;


import nativekit.scene.Scene;
import nativekit.scene.SceneSnapshot;
import nativekit.scene.SpatialIndex;
import nativekit.scene.NodeId;
import nativekit.scene.SceneNode;
import nativekit.scene.GeometryData;
import nativekit.scene.Geometry;
import nativekit.scene.MaterialData;
import nativekit.scene.Material;
import nativekit.scene.Transaction;
import nativekit.scene.SceneView;
import nativekit.scene.SelectionSet;
import nativekit.scene.Transform;
import nativekit.ui.editing.EditorDocument;
import nativekit.ui.editing.EditOperation;
import nativekit.ui.core.CommandContext;
import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.properties.PropertyDescriptorOptions;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;
import nativekit.scene.PickResult;
import CadKit;
import cadkit.Shape;
import cadkit.parametric.Feature;
import cadkit.parametric.ReferenceState;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.TopologyReference;
import cadkit.parametric.TopologyHistoryMap;
import cadkit.parametric.TopologyResolver;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.PocketFeature;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SolvedSketch;
import app.CadPlateModel.CadPlateParameters;
import app.CadPlateModel.CadPlateHoleEdit;
import app.CadBracketModel;
import app.SketchDraftCodec.SketchDraftRecord;
import haxe.io.Path as FilePath;

private typedef SceneRecordChange = {
  final id:String;
  final before:Null<SceneObjectData>;
  final after:Null<SceneObjectData>;
  final beforeIndex:Int;
  final afterIndex:Int;
}

/** One scene and one document shared by the hierarchy, inspector and viewport. */
@:allow(tests.SceneAtomicityTests)
class EditorScene {
  /** Opt-in constructor phase timings used by the headless architecture profile. */
  var loadProfilePhases:Null<Map<String, Float>>;
  /** Test-only synchronous fault injection for scene edit publication boundaries. */
  @:allow(tests.SceneAtomicityTests)
  var failureInjection:Null<String->Void> = null;
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
  var presentationStale:Bool = false;
  var selectionMaterial:Material;
  public var selectedId(default, null):String = "box";
  public var selectedCadFaceIndex(default, null):Int = -1;
  public var selectedCadFaceX(default, null):Float = 0.0;
  public var selectedCadFaceY(default, null):Float = 0.0;
  var selectedCadFaceFingerprint:Null<TopologyFingerprint> = null;
  var selectedCadFace:Null<Shape> = null;
  var selectedFeatureKey:Null<String> = null;
  var activeSketchEdit:Null<CadSketchEditSession> = null;
  var activeSketchObjectId:Null<String> = null;
  var sketchEditPlaneValue:Null<Plane> = null;
  var sketchDraftRevision:Int = 0;
  var savedSketchDraftRevision:Int = 0;
  var savedSketchDraftPresent:Bool = false;
  public var revision(default, null):Int;
  /** Changes only when simulation-consumed scene content changes, not selection. */
  public var environmentRevision(default, null):Int;
  public var selectionRevision(default, null):Int = 1;
  var disposed:Bool = false;

  function profileLoadStart():Float
    return loadProfilePhases == null ? -1.0 : Sys.time();

  function profileLoadEnd(phase:String, started:Float):Void {
    if (started < 0.0 || loadProfilePhases == null) return;
    var previous:Null<Float> = loadProfilePhases.get(phase);
    if (previous == null) previous = 0.0;
    loadProfilePhases.set(phase, previous + Sys.time() - started);
  }

  function loadProfileSummary():Dynamic {
    var phases = loadProfilePhases;
    if (phases == null) return {};
    return {
      geometryDataSeconds: phases.get("geometryData"),
      geometryBatchCreatePublishSeconds: phases.get("geometryBatchCreatePublish"),
      materialBatchCreatePublishSeconds: phases.get("materialBatchCreatePublish"),
      transactionPrepareSeconds: phases.get("transactionPrepare"),
      transactionCommitSeconds: phases.get("transactionCommit"),
      applicationBookkeepingSeconds: phases.get("applicationBookkeeping"),
      finalMaterialSeconds: phases.get("finalMaterial"),
      snapshotSeconds: phases.get("snapshot"),
      spatialIndexSeconds: phases.get("spatialIndex")
    };
  }

  public function new(?data:Array<SceneObjectData>, ?sharedDocument:EditorDocument,
      ?loadProfilePhases:Map<String, Float>) {
    this.loadProfilePhases = loadProfilePhases;
    nextRevision++;
    revision = nextRevision;
    nextEnvironmentRevision++;
    environmentRevision = nextEnvironmentRevision;
    document = sharedDocument == null ? new EditorDocument("scene") : sharedDocument;
    objects = [];
    cadSessions = new Map();
    bridge = new SceneBridge();
    try {
      if (data == null) {
        addObject("box", "Blue box", -1.5, 0.0, 0.0, 1.6, 1.2, 0.1, 0.22, 0.52, 0.85);
        addObject("tower", "Orange tower", 1.1, 0.0, 0.1, 1.2, 1.8, 0.2, 0.92, 0.48, 0.22);
      } else {
        addObjects(data);
        selectedId = data.length == 0 ? "scene" : data[0].id;
        var savedDraft:Null<SceneObjectData> = null;
        for (item in data) if (item.sketchDraft != null) {
          if (savedDraft != null)
            throw "scene contains more than one active sketch draft";
          savedDraft = item;
        }
        if (savedDraft != null)
          restoreSketchDraft(savedDraft);
      }
      var phaseStarted = profileLoadStart();
      selectionMaterial = scene.createMaterial();
      scene.setMaterialData(selectionMaterial, MaterialData.opaque(1.0, 0.88, 0.35));
      profileLoadEnd("finalMaterial", phaseStarted);
      phaseStarted = profileLoadStart();
      snapshot = scene.snapshot();
      profileLoadEnd("snapshot", phaseStarted);
      phaseStarted = profileLoadStart();
      try spatial = SpatialIndex.create(snapshot)
      catch (error:Dynamic) { snapshot.dispose(); throw error; }
      profileLoadEnd("spatialIndex", phaseStarted);
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
      kind:String="rectangle",?cadGraph:String,?meshSnapshot:String):Void {
    var phaseStarted = profileLoadStart();
    var geometry = scene.createGeometry();
    profileLoadEnd("geometryHandle", phaseStarted);
    var session:Null<CadDocumentSession> = null;
    var storedCadGraph = cadGraph;
    var geometryData:GeometryData;
    phaseStarted = profileLoadStart();
    if (isCadKind(kind)) {
      session = createCadSession(storedCadGraph, width, height, depth, kind);
      if (storedCadGraph == null)
        storedCadGraph = session.encode();
      geometryData = session.geometry();
    } else if (kind == "cad-preview") {
      if (meshSnapshot == null) throw "CAD preview object has no mesh snapshot";
      geometryData = CadPreviewGeometry.geometry(meshSnapshot);
    } else {
      geometryData = boxGeometry(width, height, depth);
    }
    profileLoadEnd("geometryData", phaseStarted);
    try {
      phaseStarted = profileLoadStart();
      scene.setGeometryData(geometry, geometryData);
      profileLoadEnd("geometryPublication", phaseStarted);
      phaseStarted = profileLoadStart();
      var material = scene.createMaterial();
      scene.setMaterialData(material, MaterialData.opaque(red, green, blue));
      profileLoadEnd("materialSetup", phaseStarted);
      phaseStarted = profileLoadStart();
      var transaction = scene.beginTransaction();
      try {
        var node = transaction.createNode();
        transaction.setName(node, label);
        transaction.setVisibility(node, visible);
        transaction.setGeometry(node, geometry);
        transaction.setMaterial(node, material);
        transaction.setTransform(node, Transform.identity().translated(x, y, z));
        profileLoadEnd("transactionPrepare", phaseStarted);
        phaseStarted = profileLoadStart();
        transaction.commit();
        profileLoadEnd("transactionCommit", phaseStarted);
        phaseStarted = profileLoadStart();
        bridge.attach(id, node, geometry, material);
        objects.push(new EditorSceneObject(id, label, kind, width, height, depth,
          collisionEnabled,dynamicBody,mass,red,green,blue,
          storedCadGraph,
          x, y, z, visible, meshSnapshot));
        if (session != null)
          cadSessions.set(id, session);
        profileLoadEnd("applicationBookkeeping", phaseStarted);
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

  /** Builds loaded resources in bulk, then publishes all loaded nodes in one transaction. */
  function addObjects(data:Array<SceneObjectData>):Void {
    if (data.length == 0) return;
    var geometryData:Array<GeometryData> = [];
    var materialData:Array<MaterialData> = [];
    var candidates:Array<EditorSceneObject> = [];
    var preparationStarted = profileLoadStart();
    for (item in data) {
      var session:Null<CadDocumentSession> = null;
      var storedCadGraph = item.cadGraph;
      var geometry:GeometryData;
      if (isCadKind(item.type)) {
        session = createCadSession(storedCadGraph, item.width, item.height, item.depth, item.type);
        if (storedCadGraph == null)
          storedCadGraph = session.encode();
        geometry = session.geometry();
        cadSessions.set(item.id, session);
      } else if (item.type == "cad-preview") {
        if (item.meshSnapshot == null) throw "CAD preview object has no mesh snapshot";
        geometry = CadPreviewGeometry.geometry(item.meshSnapshot);
      } else {
        geometry = boxGeometry(item.width, item.height, item.depth);
      }
      geometryData.push(geometry);
      materialData.push(MaterialData.opaque(item.red, item.green, item.blue));
      candidates.push(new EditorSceneObject(item.id, item.label, item.type,
        item.width, item.height, item.depth, item.collisionEnabled, item.dynamicBody,
        item.mass, item.red, item.green, item.blue, storedCadGraph,
        item.x, item.y, item.z, item.visible, item.meshSnapshot));
    }
    profileLoadEnd("geometryData", preparationStarted);

    var geometryStarted = profileLoadStart();
    var geometries = scene.createGeometryBatch(geometryData);
    profileLoadEnd("geometryBatchCreatePublish", geometryStarted);
    var materialStarted = profileLoadStart();
    var materials = scene.createMaterialBatch(materialData);
    profileLoadEnd("materialBatchCreatePublish", materialStarted);

    var transaction = scene.beginTransaction();
    var nodes:Array<NodeId> = [];
    preparationStarted = profileLoadStart();
    try {
      for (index in 0...data.length) {
        var item = data[index];
        var node = transaction.createNode();
        transaction.setName(node, item.label);
        transaction.setVisibility(node, item.visible);
        transaction.setGeometry(node, geometries[index]);
        transaction.setMaterial(node, materials[index]);
        transaction.setTransform(node, Transform.identity().translated(item.x, item.y, item.z));
        nodes.push(node);
      }
      profileLoadEnd("transactionPrepare", preparationStarted);
      var commitStarted = profileLoadStart();
      transaction.commit();
      profileLoadEnd("transactionCommit", commitStarted);
    } catch (error:Dynamic) {
      transaction.dispose();
      throw error;
    }

    var bookkeepingStarted = profileLoadStart();
    for (index in 0...data.length) {
      var item = data[index];
      bridge.attach(item.id, nodes[index], geometries[index], materials[index]);
      objects.push(candidates[index]);
    }
    profileLoadEnd("applicationBookkeeping", bookkeepingStarted);
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

  public function createCadPart():Bool {
    if (!canCreate()) return false;
    var data = records(), id = allocateId("part");
    data.push({id:id, label:"Part", type:"cad-part", x:0.0, y:0.0, z:0.0,
      width:0.05, height:0.05, depth:0.01, collisionEnabled:false, dynamicBody:false, mass:1.0,
      red:0.66, green:0.68, blue:0.72, visible:true});
    return changeObjects("Create CAD part", data, id);
  }

  public function canCreateSketch():Bool {
    var item = object(selectedId);
    return item != null && item.kind == "cad-part" && activeSketchEdit == null;
  }

  /** Start a blank sketch draft without adding an unevaluated feature to the document. */
  public function createSketch():Bool {
    if (!canCreateSketch())
      return false;
    var id = selectedId;
    var session = requireCadSession(id);
    clearSelectedCadFace();
    selectedFeatureKey = null;
    var plane = Plane.XY();
    sketchEditPlaneValue = plane;
    activeSketchEdit = session.beginNewSketchEdit(plane, "mm");
    activeSketchObjectId = id;
    sketchDraftRevision++;
    refreshSelectionRevision();
    return true;
  }

  public function canAddSketchDraftRectangle():Bool {
    var draft = activeSketchEdit;
    if (draft == null)
      return false;
    var sketch = draft.sketch.snapshot();
    return sketch.points().length == 0 && sketch.entities().length == 0 && sketch.constraints().length == 0;
  }

  /** Add a fully constrained 20 mm starter rectangle to an otherwise empty draft. */
  public function addSketchDraftRectangle():Bool {
    if (!canAddSketchDraftRectangle())
      return false;
    var template = starterSketch();
    editSketchDraft(function(sketch) {
      for (point in template.points()) sketch.addPoint(point);
      for (entity in template.entities()) sketch.addEntity(entity);
      for (constraint in template.constraints()) sketch.addConstraint(constraint);
    });
    return true;
  }

  /** Add a fully constrained rectangle in the current sketch plane. */
  public function addSketchDraftRectangleBetween(startX:Float, startY:Float,
      endX:Float, endY:Float):Bool {
    var draft = activeSketchEdit;
    if (draft == null || !Math.isFinite(startX) || !Math.isFinite(startY) ||
        !Math.isFinite(endX) || !Math.isFinite(endY))
      return false;
    var minX = Math.min(startX, endX), maxX = Math.max(startX, endX);
    var minY = Math.min(startY, endY), maxY = Math.max(startY, endY);
    if (maxX - minX < 0.000001 || maxY - minY < 0.000001 ||
        Math.max(Math.max(Math.abs(minX), Math.abs(maxX)),
          Math.max(Math.abs(minY), Math.abs(maxY))) > 1000000.0)
      return false;
    var sketch = draft.sketch.snapshot();
    var prefixIndex = 1;
    var prefix = "rect" + prefixIndex;
    while (sketchHasPrefix(sketch, prefix)) {
      prefixIndex++;
      prefix = "rect" + prefixIndex;
    }
    var p0 = prefix + ".p0", p1 = prefix + ".p1", p2 = prefix + ".p2", p3 = prefix + ".p3";
    editSketchDraft(function(value) {
      value.addPoint(new SketchPoint(p0, minX, minY));
      value.addPoint(new SketchPoint(p1, maxX, minY));
      value.addPoint(new SketchPoint(p2, maxX, maxY));
      value.addPoint(new SketchPoint(p3, minX, maxY));
      value.addEntity(SketchEntity.line(prefix + ".bottom", p0, p1));
      value.addEntity(SketchEntity.line(prefix + ".right", p1, p2));
      value.addEntity(SketchEntity.line(prefix + ".top", p2, p3));
      value.addEntity(SketchEntity.line(prefix + ".left", p3, p0));
      value.addConstraint(SketchConstraint.fixed(prefix + ".anchor", p0));
      value.addConstraint(SketchConstraint.horizontal(prefix + ".bottom-horizontal", prefix + ".bottom"));
      value.addConstraint(SketchConstraint.vertical(prefix + ".right-vertical", prefix + ".right"));
      value.addConstraint(SketchConstraint.horizontal(prefix + ".top-horizontal", prefix + ".top"));
      value.addConstraint(SketchConstraint.vertical(prefix + ".left-vertical", prefix + ".left"));
      value.addConstraint(SketchConstraint.distance(prefix + ".width", p0, p1, maxX - minX));
      value.addConstraint(SketchConstraint.distance(prefix + ".height", p1, p2, maxY - minY));
    });
    return true;
  }

  function sketchHasPrefix(sketch:ConstrainedSketch, prefix:String):Bool {
    var start = prefix + ".";
    for (point in sketch.points()) if (StringTools.startsWith(point.id, start)) return true;
    for (entity in sketch.entities()) if (StringTools.startsWith(entity.id, start)) return true;
    for (constraint in sketch.constraints()) if (StringTools.startsWith(constraint.id, start)) return true;
    return false;
  }

  public function canClearSketchDraft():Bool {
    var draft = activeSketchEdit;
    if (draft == null)
      return false;
    var sketch = draft.sketch.snapshot();
    return sketch.points().length > 0 || sketch.entities().length > 0 || sketch.constraints().length > 0;
  }

  /** Return a draft to the valid empty state so geometry can be redrawn. */
  public function clearSketchDraft():Bool {
    if (!canClearSketchDraft())
      return false;
    var draft = activeSketchEdit;
    if (draft == null)
      return false;
    var snapshot = draft.sketch.snapshot();
    editSketchDraft(function(sketch) {
      for (constraint in snapshot.constraints()) sketch.removeConstraint(constraint.id);
      for (entity in snapshot.entities()) sketch.removeEntity(entity.id);
      for (point in snapshot.points()) sketch.removePoint(point.id);
    });
    return true;
  }

  public function canCreateFaceSketch():Bool {
    if (activeSketchEdit != null || selectedCadFace == null || selectedCadFaceIndex < 0)
      return false;
    var item = object(selectedId);
    if (item == null || item.kind != "cad-part")
      return false;
    var output = requireCadSession(selectedId).document.outputFeatureOrNull();
    var shape = output == null ? null : output.currentShape();
    return selectedCadFace.surfaceKind() == CadKit.SurfaceKind.Plane && shape != null &&
      shape.subshapeCount(CadKit.ShapeKind.Solid) > 0;
  }

  /** Create a face-attached sketch using the exact face selected in the viewport. */
  public function createFaceSketch():Bool {
    if (!canCreateFaceSketch())
      return false;
    var id = selectedId;
    var session = requireCadSession(id);
    var support = session.document.outputFeatureOrNull();
    if (support == null)
      return false;
    var supportShape = support.currentShape();
    if (supportShape == null)
      return false;
    var normal = Vector.fromNative(selectedCadFace.faceNormal()).normalized();
    var xDirection = Math.abs(normal.dot(Vector.X())) > 0.99 ? Vector.Y() : Vector.X();
    var fallback = supportSelectionForFace(supportShape, selectedCadFace, normal);
    var feature = new ConstrainedSketchFeature(starterSketch(6), support, fallback,
      xDirection, 0, false, selectedCadFace);
    if (!addCadFeature(id, "Create face sketch", feature, false))
      return false;
    var featureIndex = selectCadFeature(id, feature);
    clearSelectedCadFace();
    activeSketchEdit = session.beginSketchEdit(featureIndex);
    sketchEditPlaneValue = feature.workplane();
    activeSketchObjectId = id;
    refreshSelectionRevision();
    return true;
  }

  function supportSelectionForFace(support:Shape, face:Shape, normal:Vector):Null<SelectionRecipe> {
    var recipe = new SelectionRecipe("face", "plane", normal, "max", normal, 1, 0.000001);
    var matches:Array<Shape>;
    try matches = recipe.resolve(support) catch (_:Dynamic) return null;
    var isSelectedFace = matches.length == 1 && matches[0].sameAs(face);
    for (match in matches)
      match.close();
    return isSelectedFace ? recipe : null;
  }

  public function canCreateExtrusion():Bool {
    if (activeSketchEdit != null)
      return false;
    var item = object(selectedId);
    if (item == null || item.kind != "cad-part")
      return false;
    var feature = selectedCadFeature(selectedId);
    return feature != null && feature.active && feature.currentShape() != null &&
      Std.isOfType(feature, ConstrainedSketchFeature);
  }

  /** Add a default Z extrusion from the selected solved sketch. */
  public function createExtrusion():Bool {
    if (!canCreateExtrusion())
      return false;
    var id = selectedId;
    var source = selectedCadFeature(id);
    var feature = ExtrudeFeature.along(cast source, 10, Vector.Z());
    if (!addCadFeature(id, "Create extrusion", feature))
      return false;
    selectCadFeature(id, feature);
    refreshSelectionRevision();
    return true;
  }

  public function setExtrusionDepth(id:String, featureId:Int, depth:Float):Void {
    var session = requireCadSession(id);
    var candidate = session.document.featureById(featureId);
    if (candidate == null || !Std.isOfType(candidate, ExtrudeFeature))
      throw "selected extrusion is no longer available";
    var feature:ExtrudeFeature = cast candidate;
    if (feature.amount == null)
      throw "extrusion has no editable depth parameter";
    var previous = feature.amount.value;
    if (depth == previous)
      return;
    if (!Math.isFinite(depth) || depth <= 0)
      throw "Extrusion depth must be finite and positive";
    applyCadEdit(id, "Edit extrusion depth", function(owner) {
      var current:ExtrudeFeature = cast owner.document.featureById(featureId);
      if (current.amount == null)
        throw "extrusion has no editable depth parameter";
      current.amount.set(depth);
      owner.document.recompute();
    }, function(owner) {
      var current:ExtrudeFeature = cast owner.document.featureById(featureId);
      if (current.amount == null)
        throw "extrusion has no editable depth parameter";
      current.amount.set(previous);
      owner.document.recompute();
    });
  }

  public function canCreatePocket():Bool {
    if (activeSketchEdit != null)
      return false;
    var item = object(selectedId);
    if (item == null || item.kind != "cad-part")
      return false;
    var feature = selectedCadFeature(selectedId);
    return feature != null && feature.active && Std.isOfType(feature, ConstrainedSketchFeature) &&
      (cast(feature, ConstrainedSketchFeature)).supportFaceReference != null && feature.currentShape() != null;
  }

  /** Describe a broken support-face identity on the selected sketch feature. */
  public function selectedSketchSupportStatus():Null<String> {
    var candidate = selectedCadFeature(selectedId);
    if (candidate == null || !Std.isOfType(candidate, ConstrainedSketchFeature))
      return null;
    var sketch:ConstrainedSketchFeature = cast candidate;
    var reference:TopologyReference = sketch.supportFaceReference;
    if (reference == null)
      return null;
    return switch (reference.state) {
      case ReferenceState.Deleted: "Sketch support face was deleted. Select a replacement planar face.";
      case ReferenceState.Ambiguous: "Sketch support face is ambiguous. Select a replacement planar face.";
      case ReferenceState.Unresolved: "Sketch support face is unresolved. Select a replacement planar face.";
      case ReferenceState.Resolved, ReferenceState.Remapped, ReferenceState.Closed: null;
    };
  }

  public function canRepairSelectedSketchSupportFace():Bool {
    if (activeSketchEdit != null || selectedCadFace == null ||
        selectedCadFace.surfaceKind() != CadKit.SurfaceKind.Plane)
      return false;
    var candidate = selectedCadFeature(selectedId);
    if (candidate == null || !Std.isOfType(candidate, ConstrainedSketchFeature))
      return false;
    var sketch:ConstrainedSketchFeature = cast candidate;
    var reference:TopologyReference = sketch.supportFaceReference;
    if (reference == null || (reference.state != ReferenceState.Deleted &&
        reference.state != ReferenceState.Ambiguous && reference.state != ReferenceState.Unresolved))
      return false;
    var fingerprint = selectedCadFaceFingerprint;
    if (fingerprint == null)
      return false;
    var replacement:Null<Shape> = null;
    var resolved = false;
    try {
      replacement = sketch.resolveSupportFace(fingerprint);
      resolved = true;
    } catch (_:Dynamic) {
    }
    if (replacement != null)
      replacement.close();
    return resolved;
  }

  /** Rebind the selected sketch to the explicitly picked support face. */
  public function repairSelectedSketchSupportFace():Bool {
    if (!canRepairSelectedSketchSupportFace())
      return false;
    var id = selectedId;
    var candidate = selectedCadFeature(id);
    var sketch:ConstrainedSketchFeature = cast candidate;
    var replacement:TopologyFingerprint = selectedCadFaceFingerprint;
    if (replacement == null)
      return false;
    var reference:TopologyReference = sketch.supportFaceReference;
    if (reference == null)
      return false;
    var beforeFingerprint = reference.fingerprintData();
    var beforeState = reference.state;
    var featureId = sketch.id.toInt();
    return applyCadEdit(id, "Repair sketch support face", function(owner) {
      var current = owner.document.featureById(featureId);
      if (current == null || !Std.isOfType(current, ConstrainedSketchFeature))
        throw "selected constrained sketch is no longer available";
      var currentSketch:ConstrainedSketchFeature = cast current;
      currentSketch.repairSupportFace(replacement);
      owner.document.recompute();
    }, function(owner) {
      var current = owner.document.featureById(featureId);
      if (current == null || !Std.isOfType(current, ConstrainedSketchFeature))
        throw "selected constrained sketch is no longer available";
      var currentSketch:ConstrainedSketchFeature = cast current;
      // Undo may restore an unresolved authored reference. Keep the prior published
      // result visible and leave recomputation to an explicit repair or later edit.
      currentSketch.restoreSupportFaceReference(beforeFingerprint, beforeState);
    });
  }

  /** Cut the selected face-supported sketch through its source solid. */
  public function createPocket():Bool {
    if (!canCreatePocket())
      return false;
    var id = selectedId;
    var sketch:ConstrainedSketchFeature = cast selectedCadFeature(id);
    var pocket = PocketFeature.throughAll(sketch.support, sketch);
    if (!addCadFeature(id, "Create pocket", pocket))
      return false;
    selectCadFeature(id, pocket);
    refreshSelectionRevision();
    return true;
  }

  public function canCreateVerticalFillet():Bool {
    if (activeSketchEdit != null)
      return false;
    var item = object(selectedId);
    if (item == null || item.kind != "cad-part")
      return false;
    var output = requireCadSession(selectedId).document.outputFeatureOrNull();
    var shape = output == null ? null : output.currentShape();
    return shape != null && shape.subshapeCount(CadKit.ShapeKind.Solid) > 0 &&
      verticalEdgeCount(shape) > 0;
  }

  /** Fillet every straight edge parallel to world Z, preserving that query in the feature. */
  public function createVerticalFillet(radius:Float = 0.5):Bool {
    if (!canCreateVerticalFillet())
      return false;
    if (!Math.isFinite(radius) || radius <= 0)
      throw "Fillet radius must be finite and positive";
    var session = requireCadSession(selectedId);
    var source = session.document.outputFeatureOrNull();
    if (source == null || source.currentShape() == null)
      return false;
    var count = verticalEdgeCount(source.currentShape());
    if (count == 0)
      return false;
    var query = new SelectionRecipe("edge", "line", Vector.Z(), "all", Vector.Z(), count);
    var feature = new FilletFeature(source, radius, null, null, query);
    var id = selectedId;
    if (!addCadFeature(id, "Fillet vertical edges", feature))
      return false;
    selectCadFeature(id, feature);
    refreshSelectionRevision();
    return true;
  }

  public function setFilletRadius(id:String, featureId:Int, radius:Float):Void {
    var session = requireCadSession(id);
    var candidate = session.document.featureById(featureId);
    if (candidate == null || !Std.isOfType(candidate, FilletFeature))
      throw "selected fillet is no longer available";
    var feature:FilletFeature = cast candidate;
    var previous = feature.radius.value;
    if (radius == previous)
      return;
    if (!Math.isFinite(radius) || radius <= 0)
      throw "Fillet radius must be finite and positive";
    applyCadEdit(id, "Edit fillet radius", function(owner) {
      var current:FilletFeature = cast owner.document.featureById(featureId);
      current.radius.set(radius);
      owner.document.recompute();
    }, function(owner) {
      var current:FilletFeature = cast owner.document.featureById(featureId);
      current.radius.set(previous);
      owner.document.recompute();
    });
  }

  function verticalEdgeCount(shape:Shape):Int {
    var count = 0;
    for (index in 0...shape.subshapeCount(CadKit.ShapeKind.Edge)) {
      var edge = shape.subshape(CadKit.ShapeKind.Edge, index);
      try {
        if (edge.curveKind() == CadKit.CurveKind.Line) {
          var direction = Vector.fromNative(edge.tangentAt()).normalized();
          if (1.0 - Math.abs(direction.dot(Vector.Z())) <= 0.000001)
            count++;
        }
        edge.close();
      } catch (error:Dynamic) {
        edge.close();
        throw error;
      }
    }
    return count;
  }

  function addCadFeature(id:String, label:String, feature:Feature, publishAsOutput:Bool = true):Bool {
    var session = requireCadSession(id);
    var previousOutput = session.document.outputFeatureOrNull();
    return applyCadEdit(id, label, function(owner) {
      var transaction = owner.document.beginTransaction();
      try {
        if (feature.document == null) {
          var attached:Feature = owner.document.add(feature);
          owner.document.trackFeatureCreation(feature);
        } else {
          owner.document.setFeatureActive(feature, true);
        }
        if (publishAsOutput)
          owner.document.setOutputTracked(feature);
        owner.document.recompute();
        transaction.commit();
      } catch (error:Dynamic) {
        transaction.cancel();
        throw error;
      }
    }, function(owner) {
      var transaction = owner.document.beginTransaction();
      try {
        owner.document.setFeatureActive(feature, false);
        owner.document.setOutputTracked(previousOutput);
        owner.document.recompute();
        transaction.commit();
      } catch (error:Dynamic) {
        transaction.cancel();
        throw error;
      }
    });
  }

  function selectedCadFeature(id:String):Null<Feature> {
    if (selectedId != id || selectedFeatureKey == null)
      return null;
    var marker = selectedFeatureKey.indexOf(":feature:");
    if (marker < 0 || selectedFeatureKey.substr(0, marker) != id)
      return null;
    var index = Std.parseInt(selectedFeatureKey.substr(marker + 9));
    if (index == null)
      return null;
    try {
      return requireCadSession(id).document.featureAt(index);
    } catch (_:Dynamic) {
      return null;
    }
  }

  function selectCadFeature(id:String, feature:Feature):Int {
    var session = requireCadSession(id);
    for (index in 0...session.document.featureCount()) {
      if (session.document.featureAt(index) != feature)
        continue;
      selectedFeatureKey = id + ":feature:" + index;
      return index;
    }
    throw "created CAD feature is missing from its document";
  }

  static function starterSketch(side:Float = 20):ConstrainedSketch {
    var sketch = new ConstrainedSketch(Plane.XY(), "mm");
    var halfSide = side / 2;
    sketch.addPoint(new SketchPoint("p0", -halfSide, -halfSide));
    sketch.addPoint(new SketchPoint("p1", halfSide, -halfSide));
    sketch.addPoint(new SketchPoint("p2", halfSide, halfSide));
    sketch.addPoint(new SketchPoint("p3", -halfSide, halfSide));
    sketch.addEntity(SketchEntity.line("bottom", "p0", "p1"));
    sketch.addEntity(SketchEntity.line("right", "p1", "p2"));
    sketch.addEntity(SketchEntity.line("top", "p2", "p3"));
    sketch.addEntity(SketchEntity.line("left", "p3", "p0"));
    sketch.addConstraint(SketchConstraint.fixed("anchor", "p0"));
    sketch.addConstraint(SketchConstraint.horizontal("bottom-horizontal", "bottom"));
    sketch.addConstraint(SketchConstraint.vertical("right-vertical", "right"));
    sketch.addConstraint(SketchConstraint.horizontal("top-horizontal", "top"));
    sketch.addConstraint(SketchConstraint.vertical("left-vertical", "left"));
    sketch.addConstraint(SketchConstraint.distance("width", "p0", "p1", side));
    sketch.addConstraint(SketchConstraint.distance("height", "p1", "p2", side));
    return sketch;
  }

  public function importStep(path:String):Bool {
    if (!canCreate()) return false;
    var model = CadImportedModel.create(path);
    var graph:String;
    var dimensions:CadModelDimensions;
    try {
      graph = model.encode();
      dimensions = model.sceneDimensions();
    } catch (error:Dynamic) {
      model.close();
      throw error;
    }
    model.close();
    var data = records();
    var id = allocateId("step");
    var fileName = FilePath.withoutDirectory(path);
    var extension = fileName.lastIndexOf(".");
    var sourceName = extension <= 0 ? fileName : fileName.substr(0, extension);
    data.push({id:id, label:sourceName, type:"cad-step", x:0.0, y:0.0, z:0.0,
      width:dimensions.width, height:dimensions.height, depth:dimensions.depth,
      collisionEnabled:false, dynamicBody:false, mass:1.0, red:0.72, green:0.74, blue:0.78,
      visible:true, cadGraph:graph});
    return changeObjects("Import STEP", data, id);
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
    var model=cadBracketModel(id),before=model.wallThickness(),beforeHoleRadius=model.holeRadius();
    if(before==value)return;
    applyCadEdit(id,"Edit bracket wall thickness",function(session) {
      var target:CadBracketModel=cast session.model;
      target.setWallThickness(value);
    },function(session) {
      var target:CadBracketModel=cast session.model;
      target.setWallThickness(before);
      target.setHoleRadius(beforeHoleRadius);
    });
  }

  public function isCadPart(id:String):Bool {
    var item=object(id);
    return item!=null&&isCadKind(item.kind);
  }

  public function hasCadOutput(id:String):Bool {
    var item = object(id);
    if (item == null || !isCadKind(item.kind))
      return false;
    var output = requireCadSession(id).document.outputFeatureOrNull();
    return output != null && output.currentShape() != null;
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
      cadGraph:cadGraph,meshSnapshot:source.meshSnapshot});
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
    // This helper is used by structural add/remove/duplicate/import operations.
    // Keep only the membership delta; common records stay in the live scene.
    var afterIds:Map<String, Bool> = new Map();
    var existingIds:Map<String, Bool> = new Map();
    for (index in 0...after.length) {
      afterIds.set(after[index].id, true);
    }
    for (item in objects) existingIds.set(item.id, true);
    var changes:Array<SceneRecordChange> = [];
    for (index in 0...objects.length) {
      var item = objects[index];
      if (!afterIds.exists(item.id)) {
        var previous = recordForObject(item);
        if (isCadKind(previous.type)) previous.cadGraph = currentCadGraph(previous.id);
        changes.push({id: previous.id, before: previous, after: null,
          beforeIndex: index, afterIndex: -1});
      }
    }
    for (index in 0...after.length) {
      var next = after[index];
      if (!existingIds.exists(next.id))
        changes.push({id: next.id, before: null, after: next,
          beforeIndex: -1, afterIndex: index});
    }
    var previousSelection = selectedId;
    var initialAfter:Null<Array<SceneObjectData>> = after;
    return document.apply(new EditOperation(label,
      function() {
        var initial = initialAfter;
        initialAfter = null;
        if (initial != null) replaceObjects(initial, selection);
        else applyObjectChanges(changes, true, selection);
      },
      function() applyObjectChanges(changes, false, previousSelection),
      null, null, null, estimateSceneChanges(changes)));
  }

  function applyObjectChanges(changes:Array<SceneRecordChange>, forward:Bool, selection:String):Void {
    var data = records();
    var removals:Array<SceneRecordChange> = [];
    for (change in changes) {
      var target = forward ? change.after : change.before;
      if (target == null) removals.push(change);
    }
    removals.sort(function(lhs, rhs) {
      return findRecordIndex(data, rhs.id) - findRecordIndex(data, lhs.id);
    });
    for (change in removals) {
      var index = findRecordIndex(data, change.id);
      if (index >= 0) data.splice(index, 1);
    }

    var additions:Array<SceneRecordChange> = [];
    for (change in changes) {
      var target = forward ? change.after : change.before;
      var source = forward ? change.before : change.after;
      if (target == null) continue;
      var index = findRecordIndex(data, change.id);
      if (index >= 0) {
        data[index] = target;
      } else if (source == null) {
        additions.push(change);
      }
    }
    additions.sort(function(lhs, rhs) {
      var left = forward ? lhs.afterIndex : lhs.beforeIndex;
      var right = forward ? rhs.afterIndex : rhs.beforeIndex;
      return left - right;
    });
    for (change in additions) {
      var target = forward ? change.after : change.before;
      var index = forward ? change.afterIndex : change.beforeIndex;
      if (index < 0) index = data.length;
      if (index > data.length) index = data.length;
      data.insert(index, target);
    }
    replaceObjects(data, selection);
  }

  static function findRecordIndex(data:Array<SceneObjectData>, id:String):Int {
    for (index in 0...data.length) if (data[index].id == id) return index;
    return -1;
  }

  static function recordForObject(item:EditorSceneObject):SceneObjectData {
    return {id: item.id, label: item.label, type: item.kind,
      x: item.x, y: item.y, z: item.z,
      width: item.width, height: item.height, depth: item.depth,
      collisionEnabled: item.collisionEnabled, dynamicBody: item.dynamicBody, mass: item.mass,
      red: item.red, green: item.green, blue: item.blue,
      visible: item.visible, cadGraph: item.cadGraph};
  }

  static function estimateSceneChanges(changes:Array<SceneRecordChange>):Int {
    var bytes = 96 + changes.length * 48;
    for (change in changes) {
      bytes += estimateSceneRecord(change.before);
      bytes += estimateSceneRecord(change.after);
    }
    return bytes;
  }

  static function estimateSceneRecord(record:Null<SceneObjectData>):Int {
    if (record == null) return 0;
    return 384 + estimatedStringBytes(record.id) + estimatedStringBytes(record.label) +
      estimatedStringBytes(record.type) + estimatedStringBytes(record.cadGraph) +
      estimatedStringBytes(record.sketchDraft);
  }

  static function estimatedStringBytes(value:Null<String>):Int
    return value == null ? 0 : value.length * 2;

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

  function applyConstrainedSketchSnapshot(session:CadDocumentSession, featureId:Int,
      sketch:ConstrainedSketch):Void {
    var candidate = session.document.featureById(featureId);
    if (candidate == null || !Std.isOfType(candidate, ConstrainedSketchFeature))
      throw "constrained sketch feature is no longer available";
    var feature:ConstrainedSketchFeature = cast candidate;
    feature.replaceSketch(sketch);
    session.document.recompute();
  }

  function syncCadSession(id:String):Void {
    var item = requiredObject(id);
    var session = requireCadSession(id);
    scene.setGeometryData(runtimeFor(id).geometry, session.geometry());
    var values = session.model.sceneDimensions();
    // Scene persistence requires strictly positive authored dimensions, while a
    // valid CAD result can be planar or have kernel-scale numerical thickness.
    item.width = Math.max(Math.abs(values.width), 0.000001);
    item.height = Math.max(Math.abs(values.height), 0.000001);
    item.depth = Math.max(Math.abs(values.depth), 0.000001);
    if (id == selectedId && selectedCadFaceFingerprint != null) {
      var priorIndex = selectedCadFaceIndex;
      var index = -1;
      try index = remapSelectedCadFace(session, selectedCadFace, selectedCadFaceFingerprint)
      catch (_:Dynamic) clearSelectedCadFace();
      if (index < 0) clearSelectedCadFace();
      if (priorIndex != index) {
        selectionRevision++;
        nextRevision++;
        revision = nextRevision;
      }
    }
    publish();
  }

  /** Refine preview meshes after they have had one frame to reach the viewport. */
  public function advanceCadMeshRefinement():Bool {
    var requestFrame = false;
    var refined:Array<String> = [];
    for (id in cadSessions.keys()) {
      var session = cadSessions.get(id);
      if (session == null || !session.needsGeometryRefinement())
        continue;
      if (session.requestRefinementFrame()) {
        requestFrame = true;
      } else if (session.refinePublishedGeometry()) {
        refined.push(id);
      }
    }
    for (id in refined)
      syncCadSession(id);
    return requestFrame;
  }

  /** Resolve a selected face through producer history before considering geometry. */
  function remapSelectedCadFace(session:CadDocumentSession, priorFace:Null<Shape>,
      fingerprint:TopologyFingerprint):Int {
    if (priorFace == null)
      return -1;
    var output = session.document.outputFeatureOrNull();
    if (output == null || output.currentShape() == null)
      return -1;
    var resultShape = output.currentShape();
    var index = -1;
    var provenance = output.provenance;
    if (provenance != null) {
      var remap = new TopologyHistoryMap(provenance).remap(priorFace, CadKit.ShapeKind.Face);
      if (remap.state == ReferenceState.Remapped && remap.shape != null) {
        index = faceIndexOf(resultShape, remap.shape);
      }
      if (remap.shape != null)
        remap.shape.close();
    }
    if (index < 0) {
      var resolution = TopologyResolver.resolve(resultShape, fingerprint, CadKit.ShapeKind.Face, priorFace);
      if (resolution.state == ReferenceState.Resolved)
        index = resolution.index;
    }
    if (index < 0)
      return -1;
    var currentFace = resultShape.subshape(CadKit.ShapeKind.Face, index);
    try {
      var currentFingerprint = session.model.faceFingerprint(index);
      if (selectedCadFace != null)
        selectedCadFace.close();
      selectedCadFace = currentFace;
      selectedCadFaceIndex = index;
      selectedCadFaceFingerprint = currentFingerprint;
      session.selectedTopology = currentFingerprint;
      return index;
    } catch (error:Dynamic) {
      currentFace.close();
      throw error;
    }
  }

  function faceIndexOf(source:Shape, candidate:Shape):Int {
    var count = source.subshapeCount(CadKit.ShapeKind.Face);
    for (index in 0...count) {
      var face = source.subshape(CadKit.ShapeKind.Face, index);
      var matches = candidate.sameAs(face);
      face.close();
      if (matches)
        return index;
    }
    return -1;
  }

  function installSelectedCadFace(session:CadDocumentSession, face:Shape, index:Int,
      x:Float, y:Float):Void {
    try {
      var fingerprint = session.model.faceFingerprint(index);
      if (selectedCadFace != null)
        selectedCadFace.close();
      selectedCadFace = face;
      selectedCadFaceIndex = index;
      selectedCadFaceFingerprint = fingerprint;
      selectedCadFaceX = x;
      selectedCadFaceY = y;
      session.selectedTopology = fingerprint;
    } catch (error:Dynamic) {
      face.close();
      throw error;
    }
  }

  function clearSelectedCadFace():Void {
    var session = cadSessions.get(selectedId);
    if (session != null)
      session.selectedTopology = null;
    if (selectedCadFace != null)
      selectedCadFace.close();
    selectedCadFace = null;
    selectedCadFaceIndex = -1;
    selectedCadFaceFingerprint = null;
    selectedCadFaceX = 0.0;
    selectedCadFaceY = 0.0;
  }

  function refreshSelectionRevision():Void {
    selectionRevision++;
    nextRevision++;
    revision = nextRevision;
  }

  // Reconcile document records into the runtime scene while preserving stable nodes.
  function replaceObjects(data:Array<SceneObjectData>, selection:String):Void {
    var existingById:Map<String, EditorSceneObject> = new Map();
    for (item in objects) existingById.set(item.id, item);
    var prepared = new PreparedSceneEdit(scene.beginTransaction(), [],
      bridge.copyEntries(), bridge.copyNodeEntries(), cadSessions.copy());
    var previousFace=selectedCadFaceFingerprint;
    var previousFaceShape=selectedCadFace;
    var previousFaceX=selectedCadFaceX,previousFaceY=selectedCadFaceY;
    var retained:Map<String, Bool> = new Map();
    try {
      for (record in data) {
        var item = existingById.get(record.id);
        var session = item == null || !isCadKind(item.kind) || item.kind != record.type || item.cadGraph != record.cadGraph
          ? null : cadSessions.get(record.id);
        var storedGraph = record.cadGraph;
        if (isCadKind(record.type) && session == null) {
          session = createCadSession(storedGraph, record.width, record.height, record.depth, record.type);
          prepared.stagedCadSessions.push(session);
          failIfInjected("prepare.cad-session");
          storedGraph = session.encode();
        }
        if (session != null)
          prepared.cadSessions.set(record.id, session);
        else
          prepared.cadSessions.remove(record.id);
        if (item == null) {
          var geometry = scene.createGeometry();
          prepared.createdGeometry.push(geometry);
          var geometryData = session != null
            ? session.geometry()
            : (record.type == "cad-preview"
              ? CadPreviewGeometry.geometry(record.meshSnapshot)
              : boxGeometry(record.width, record.height, record.depth));
          scene.setGeometryData(geometry, geometryData);
          failIfInjected("prepare.new-geometry");
          var material = scene.createMaterial();
          prepared.createdMaterials.push(material);
          scene.setMaterialData(material, MaterialData.opaque(record.red, record.green, record.blue));
          failIfInjected("prepare.new-material");
          var node = prepared.transaction.createNode();
          prepared.transaction.setName(node, record.label);
          prepared.transaction.setVisibility(node, record.visible);
          prepared.transaction.setGeometry(node, geometry);
          prepared.transaction.setMaterial(node, material);
          prepared.transaction.setTransform(node, Transform.identity().translated(record.x, record.y, record.z));
          prepared.bridgeEntries.set(record.id, new EditorSceneRuntimeObject(node, geometry, material));
          prepared.nodeEntries.set(SceneBridge.nodeKey(node), record.id);
          failIfInjected("prepare.bridge-attach");
          item = new EditorSceneObject(record.id, record.label, record.type,
            record.width, record.height, record.depth, record.collisionEnabled,
            record.dynamicBody, record.mass, record.red, record.green, record.blue,
            storedGraph,
            record.x, record.y, record.z, record.visible, record.meshSnapshot);
          prepared.objects.push(item);
          failIfInjected("prepare.new-object");
          prepared.changed = true;
        } else {
          var runtime = prepared.bridgeEntries.get(record.id);
          if (runtime == null) throw "Missing runtime scene node: " + record.id;
          var candidate = item;
          var objectChanged = item.label != record.label || item.visible != record.visible ||
            item.x != record.x || item.y != record.y || item.z != record.z ||
            item.width != record.width || item.height != record.height || item.depth != record.depth ||
            item.kind != record.type || item.cadGraph != storedGraph ||
            item.meshSnapshot != record.meshSnapshot ||
            item.collisionEnabled != record.collisionEnabled || item.dynamicBody != record.dynamicBody ||
            item.mass != record.mass || item.red != record.red || item.green != record.green ||
            item.blue != record.blue;
          if (objectChanged) candidate = copyEditorSceneObject(item);
          if (item.label != record.label) prepared.transaction.setName(runtime.node, record.label);
          if (item.visible != record.visible) prepared.transaction.setVisibility(runtime.node, record.visible);
          if (item.x != record.x || item.y != record.y || item.z != record.z) {
            prepared.transaction.setTransform(runtime.node, Transform.identity().translated(record.x, record.y, record.z));
            prepared.changedBounds.push(runtime.node);
          }
          if (item.label != record.label || item.visible != record.visible || item.x != record.x ||
              item.y != record.y || item.z != record.z) prepared.changed = true;
          if (item.width != record.width || item.height != record.height || item.depth != record.depth ||
              item.kind != record.type || item.cadGraph != storedGraph || item.meshSnapshot != record.meshSnapshot) {
            var geometryData = session != null
              ? session.geometry()
              : (record.type == "cad-preview"
                ? CadPreviewGeometry.geometry(record.meshSnapshot)
                : boxGeometry(record.width, record.height, record.depth));
            var geometry = scene.createGeometry();
            prepared.createdGeometry.push(geometry);
            scene.setGeometryData(geometry, geometryData);
            prepared.transaction.setGeometry(runtime.node, geometry);
            prepared.retiredGeometry.push(runtime.geometry);
            runtime = new EditorSceneRuntimeObject(runtime.node, geometry, runtime.material);
            prepared.bridgeEntries.set(record.id, runtime);
            failIfInjected("prepare.existing-geometry");
            prepared.changed = true;
          }
          if (item.red != record.red || item.green != record.green || item.blue != record.blue) {
            var material = scene.createMaterial();
            prepared.createdMaterials.push(material);
            scene.setMaterialData(material, MaterialData.opaque(record.red, record.green, record.blue));
            prepared.transaction.setMaterial(runtime.node, material);
            prepared.retiredMaterials.push(runtime.material);
            runtime = new EditorSceneRuntimeObject(runtime.node, runtime.geometry, material);
            prepared.bridgeEntries.set(record.id, runtime);
            failIfInjected("prepare.existing-material");
            prepared.changed = true;
          }
          if (item.collisionEnabled != record.collisionEnabled || item.dynamicBody != record.dynamicBody ||
              item.mass != record.mass) prepared.changed = true;
          candidate.label = record.label; candidate.kind = record.type;
          candidate.width = record.width; candidate.height = record.height; candidate.depth = record.depth;
          candidate.collisionEnabled = record.collisionEnabled; candidate.dynamicBody = record.dynamicBody;
          candidate.mass = record.mass; candidate.red = record.red; candidate.green = record.green; candidate.blue = record.blue;
          candidate.cadGraph = storedGraph; candidate.x = record.x; candidate.y = record.y; candidate.z = record.z;
          candidate.meshSnapshot = record.meshSnapshot;
          candidate.visible = record.visible;
          if (objectChanged) prepared.changed = true;
          prepared.objects.push(candidate);
          failIfInjected("prepare.existing-object");
        }
        retained.set(record.id, true);
      }
      for (item in objects) if (!retained.exists(item.id)) {
        var runtime = bridge.runtime(item.id);
        if (runtime == null) throw "Missing runtime scene node: " + item.id;
        prepared.transaction.destroyNode(runtime.node);
        prepared.bridgeEntries.remove(item.id);
        prepared.nodeEntries.remove(SceneBridge.nodeKey(runtime.node));
        prepared.retiredGeometry.push(runtime.geometry);
        prepared.retiredMaterials.push(runtime.material);
        prepared.cadSessions.remove(item.id);
        prepared.changed = true;
      }
      for (id in cadSessions.keys()) {
        var previous = cadSessions.get(id);
        if (previous != null && prepared.cadSessions.get(id) != previous)
          prepared.retiredCadSessions.push(previous);
      }
      failIfInjected("transaction.before-commit");
      prepared.transaction.commit();
    } catch (error:Dynamic) {
      prepared.abort();
      throw error;
    }
    objects = prepared.objects;
    cadSessions = prepared.cadSessions;
    bridge.replaceEntries(prepared.bridgeEntries, prepared.nodeEntries);
    selectedId = selection;
    selectedFeatureKey=null;
    if (prepared.changed) publish(prepared.changedBounds);
    var restoredFace = -1;
    if(previousFace!=null){
      var selected=object(selection);
      if(selected!=null&&isCadKind(selected.kind)){
        var session=cadSessions.get(selected.id);
        if(session!=null)try {
          restoredFace=remapSelectedCadFace(session,previousFaceShape,previousFace);
          if(restoredFace>=0){selectedCadFaceX=previousFaceX;selectedCadFaceY=previousFaceY;}
        } catch(error:Dynamic) { restoredFace=-1; }
      }
    }
    if(restoredFace<0)try clearSelectedCadFace() catch (_:Dynamic) {}
    prepared.retire();
    selectionRevision++;
    nextRevision++;
    revision = nextRevision;
    nextEnvironmentRevision++;
    environmentRevision = nextEnvironmentRevision;
  }

  static function copyEditorSceneObject(item:EditorSceneObject):EditorSceneObject {
    return new EditorSceneObject(item.id, item.label, item.kind, item.width, item.height,
      item.depth, item.collisionEnabled, item.dynamicBody, item.mass, item.red, item.green,
      item.blue, item.cadGraph, item.x, item.y, item.z, item.visible, item.meshSnapshot);
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
    refreshPresentationIfStale();
    var item = object(id);
    if (item == null) throw "Unknown scene object: " + id;
    var value = snapshot.findNode(runtimeFor(id).node);
    if (value == null) throw "Missing scene node: " + id;
    return value;
  }

  public function renderSnapshot():SceneSnapshot {
    refreshPresentationIfStale();
    return snapshot;
  }

  public function configureRenderView(view:SceneView, viewProjection:Transform,
      ?poses:Array<SimulationPoseVisual>):SceneView {
    view.setViewProjection(viewProjection);
    var selected = object(selectedId);
    var selection = new SelectionSet();
    if (selected != null) selection.add(runtimeFor(selected.id).node);
    view.applySelection(selection, selectionMaterial);
    if (poses != null) {
      var poseNodes:Array<NodeId> = [];
      var poseTransforms:Array<Transform> = [];
      for (pose in poses) {
        var runtime = bridge.runtime(pose.id);
        if (runtime != null) {
          poseNodes.push(runtime.node);
          poseTransforms.push(poseTransform(pose.position, pose.rotation));
        }
      }
      view.replacePoses(poseNodes, poseTransforms);
    }
    return view;
  }

  public function selectRayWithView(view:SceneView, originX:Float, originY:Float, originZ:Float,
      directionX:Float, directionY:Float, directionZ:Float):String {
    refreshPresentationIfStale();
    return selectHit(spatial.pickRayWithView(view, originX, originY, originZ,
      directionX, directionY, directionZ));
  }

  public function pickRayWithView(view:SceneView, originX:Float, originY:Float, originZ:Float,
      directionX:Float, directionY:Float, directionZ:Float):String {
    refreshPresentationIfStale();
    return idForHit(spatial.pickRayWithView(view, originX, originY, originZ,
      directionX, directionY, directionZ));
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
    if (activeSketchEdit != null && (id != activeSketchObjectId || selectedFeatureKey != null))
      cancelSelectedSketchEdit();
    clearSelectedCadFace();
    selectedId = id;
    selectedFeatureKey=null;
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
    if(item==null||!isCadKind(item.kind))return false;
    if (activeSketchEdit != null && (id != activeSketchObjectId || key != selectedFeatureKey))
      cancelSelectedSketchEdit();
    if(selectedId==id&&selectedFeatureKey==key)return false;
    clearSelectedCadFace();
    selectedId=id;selectedFeatureKey=key;
    selectionRevision++;nextRevision++;revision=nextRevision;
    return true;
  }

  public function cadFeatureNames(id:String):Array<String> {
    var item=object(id);
    if(item==null||!isCadKind(item.kind))return [];
    return requireCadSession(id).model.featureNames();
  }

  public function cadFeatureCount(id:String):Int {
    var item = object(id);
    return item == null || !isCadKind(item.kind) ? 0 : requireCadSession(id).document.featureCount();
  }

  public function cadFeatureNameAt(id:String, index:Int):String {
    var item = object(id);
    if (item == null || !isCadKind(item.kind))
      return "Feature";
    var feature = requireCadSession(id).document.featureAt(index);
    var name = feature.serializationType();
    return feature.active ? name : "Inactive · " + name;
  }

  public function canBeginSelectedSketchEdit():Bool {
    if (activeSketchEdit != null || selectedFeatureKey == null)
      return false;
    var marker = selectedFeatureKey.indexOf(":feature:");
    if (marker < 0)
      return false;
    var index = Std.parseInt(selectedFeatureKey.substr(marker + 9));
    if (index == null)
      return false;
    try {
      var feature = requireCadSession(selectedId).document.featureAt(index);
      return feature.active && Std.isOfType(feature, ConstrainedSketchFeature);
    } catch (_:Dynamic) {
      return false;
    }
  }

  public function beginSelectedSketchEdit():Bool {
    if (!canBeginSelectedSketchEdit())
      return false;
    var marker = selectedFeatureKey.indexOf(":feature:");
    var index = Std.parseInt(selectedFeatureKey.substr(marker + 9));
    activeSketchEdit = requireCadSession(selectedId).beginSketchEdit(index);
    var feature:ConstrainedSketchFeature = cast activeSketchEdit.feature;
    sketchEditPlaneValue = feature.workplane();
    activeSketchObjectId = selectedId;
    refreshSelectionRevision();
    return true;
  }

  public function hasActiveSketchEdit():Bool
    return activeSketchEdit != null;

  function restoreSketchDraft(owner:SceneObjectData):Void {
    var encoded = owner.sketchDraft;
    if (encoded == null)
      return;
    var restored:SketchDraftRecord = SketchDraftCodec.decode(encoded);
    var session = requireCadSession(owner.id);
    if (restored.featureIndex < 0) {
      if (owner.type != "cad-part")
        throw "new sketch drafts require a generic CAD part";
      activeSketchEdit = session.beginNewSketchEdit(restored.sketch.plane,
        restored.sketch.units, restored.sketch);
      sketchEditPlaneValue = restored.sketch.plane;
      selectedFeatureKey = null;
    } else {
      if (restored.featureIndex >= session.document.featureCount())
        throw "sketch draft references a missing feature";
      var feature = session.document.featureAt(restored.featureIndex);
      if (!Std.isOfType(feature, ConstrainedSketchFeature) || !feature.active)
        throw "sketch draft must reference an active constrained sketch feature";
      var sketchFeature:ConstrainedSketchFeature = cast feature;
      activeSketchEdit = session.beginSketchEdit(restored.featureIndex, restored.sketch);
      sketchEditPlaneValue = sketchFeature.workplane();
      selectedFeatureKey = owner.id + ":feature:" + restored.featureIndex;
    }
    activeSketchObjectId = owner.id;
    selectedId = owner.id;
    sketchDraftRevision = 1;
    savedSketchDraftRevision = sketchDraftRevision;
    savedSketchDraftPresent = true;
    refreshSelectionRevision();
  }

  function sketchDraftFeatureIndex(draft:CadSketchEditSession):Int {
    if (draft.feature == null)
      return -1;
    if (activeSketchObjectId == null)
      throw "sketch draft has no owning CAD object";
    var session = requireCadSession(activeSketchObjectId);
    for (index in 0...session.document.featureCount())
      if (session.document.featureAt(index) == draft.feature)
        return index;
    throw "sketch draft feature is no longer in its document";
  }

  public function sketchDraftPlane():Null<Plane>
    return activeSketchEdit == null ? null : sketchEditPlaneValue;

  public function sketchDraftSnapshot():Null<ConstrainedSketch> {
    var draft = activeSketchEdit;
    return draft == null ? null : draft.sketch.snapshot();
  }

  public function sketchDraftSolution():Null<SolvedSketch> {
    if (activeSketchEdit == null)
      return null;
    var session = activeSketchEdit.sketch;
    return session.solution == null ? session.lastValidSolution : session.solution;
  }

  public function canApplySelectedSketchEdit():Bool {
    var draft = activeSketchEdit;
    if (draft == null || !draft.sketch.isSolved)
      return false;
    try {
      var profile = draft.sketch.buildProfile();
      profile.close();
      return true;
    } catch (_:Dynamic) {
      return false;
    }
  }

  public function sketchEditSummary():Null<String> {
    var draft = activeSketchEdit;
    if (draft == null)
      return null;
    if (draft.sketch.snapshot().entities().length == 0)
      return "Sketch is empty · drag on the workplane to draw a rectangle";
    var diagnostic = draft.sketch.diagnostic;
    if (diagnostic == null)
      return "Sketch draft has not been solved";
    return diagnostic.message + " · " + draft.sketch.degreesOfFreedom
      + " degrees of freedom" + (diagnostic.constraintIds.length == 0
        ? "" : " · constraints: " + diagnostic.constraintIds.join(", "))
      + " · drag to add a rectangle";
  }

  public function cancelSelectedSketchEdit():Bool {
    if (activeSketchEdit == null)
      return false;
    activeSketchEdit.cancel();
    activeSketchEdit = null;
    activeSketchObjectId = null;
    sketchEditPlaneValue = null;
    refreshSelectionRevision();
    return true;
  }

  public function applySelectedSketchEdit():Bool {
    var draft = activeSketchEdit;
    if (draft == null)
      return false;
    var id = activeSketchObjectId;
    if (id == null || object(id) == null)
      throw "sketch draft owner is no longer in the scene";
    if (!canApplySelectedSketchEdit())
      throw "sketch draft needs a solved, closed profile before it can be applied";

    var existingFeature = draft.feature;
    if (existingFeature == null) {
      var feature = new ConstrainedSketchFeature(draft.sketch.snapshot());
      if (!addCadFeature(id, "Create constrained sketch", feature))
        return false;
      selectCadFeature(id, feature);
      draft.cancel();
      activeSketchEdit = null;
      activeSketchObjectId = null;
      sketchEditPlaneValue = null;
      refreshSelectionRevision();
      return true;
    }

    var beforeSketch = existingFeature.sketch();
    var afterSketch = draft.sketch.snapshot();
    var featureId = existingFeature.id.toInt();
    var operation = new EditOperation("Edit constrained sketch", function() {
      runCadEdit(id, function(session) applyConstrainedSketchSnapshot(session, featureId, afterSketch),
        function(session) applyConstrainedSketchSnapshot(session, featureId, beforeSketch));
    }, function() {
      runCadEdit(id, function(session) applyConstrainedSketchSnapshot(session, featureId, beforeSketch),
        function(session) applyConstrainedSketchSnapshot(session, featureId, afterSketch));
    });
    document.apply(operation);
    draft.cancel();
    activeSketchEdit = null;
    activeSketchObjectId = null;
    sketchEditPlaneValue = null;
    refreshSelectionRevision();
    return true;
  }

  public function selectAtXY(x:Float,y:Float):String {
    refreshPresentationIfStale();
    return selectHit(spatial.pickRay(x,y,1000001.0,0.0,0.0,-1.0));
  }

  public function selectAtRay(originX:Float,originY:Float,originZ:Float,
      directionX:Float,directionY:Float,directionZ:Float):String {
    refreshPresentationIfStale();
    return selectHit(spatial.pickRay(originX,originY,originZ,directionX,directionY,directionZ));
  }

  function selectHit(hit:PickResult):String {
    var previousFace=selectedCadFaceIndex;
    var id=idForHit(hit);
    // Keep a selected feature-tree row while picking a replacement face on the same part.
    if (id != selectedId || (activeSketchEdit != null && selectedFeatureKey != null))
      select(id);
    clearSelectedCadFace();
    var item=object(id);
    if(item!=null&&isCadKind(item.kind)&&hit.subelement()>=0){
      var session=requireCadSession(id);
      try {
        var index=hit.subelement();
        var output=session.document.outputFeatureOrNull();
        if(output==null||output.currentShape()==null)throw "CAD output has no evaluated faces";
        var face=output.currentShape().subshape(CadKit.ShapeKind.Face,index);
        installSelectedCadFace(session,face,index,hit.worldX()-item.x,hit.worldY()-item.y);
      } catch(error:Dynamic){clearSelectedCadFace();}
    }
    if(previousFace!=selectedCadFaceIndex){selectionRevision++;nextRevision++;revision=nextRevision;}
    return id;
  }

  function idForHit(hit:PickResult):String {
    var id = bridge.idForNode(hit.node());
    return id == null ? "scene" : id;
  }

  public function context():CommandContext {
    return new CommandContext(document, selectedId == "scene" ? [] : [selectedId],
      "scene-viewport", null, "scene-editor");
  }

  /** Orthographic world-space picking against the same published geometry. */
  public function pick(x:Float, y:Float):String {
    refreshPresentationIfStale();
    var hit = spatial.pickRay(x, y, 1000001.0, 0.0, 0.0, -1.0);
    return idForHit(hit);
  }

  public function pickRay(originX:Float,originY:Float,originZ:Float,
      directionX:Float,directionY:Float,directionZ:Float):String {
    refreshPresentationIfStale();
    var hit=spatial.pickRay(originX,originY,originZ,directionX,directionY,directionZ);
    return idForHit(hit);
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
    var runtime = runtimeFor(id);
    var current = Transform.identity().translated(x, y, item.z);
    var transaction = scene.beginTransaction();
    try {
      transaction.setTransform(runtime.node, current);
      transaction.commit();
    } catch (error:Dynamic) { transaction.dispose(); throw error; }
    item.x = x; item.y = y;
    publish([runtime.node]);
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

  function publish(?updatedNodes:Array<NodeId>):Void {
    nextRevision++;
    revision = nextRevision;
    nextEnvironmentRevision++;
    environmentRevision = nextEnvironmentRevision;
    rebuildPresentation(updatedNodes);
  }

  /** Derived presentation caches may lag a committed edit and retry on the next access/frame. */
  function refreshPresentationIfStale():Void {
    if (presentationStale) rebuildPresentation();
    if (presentationStale) throw "Scene presentation is unavailable until its derived caches rebuild";
  }

  function rebuildPresentation(?updatedNodes:Array<NodeId>):Void {
    var next:Null<SceneSnapshot> = null;
    var nextSpatial:Null<SpatialIndex> = null;
    try {
      failIfInjected("publish.snapshot");
      next = scene.snapshot();
      failIfInjected("publish.spatial-index");
      if (!spatial.updateNodes(next, updatedNodes == null ? [] : updatedNodes))
        nextSpatial = SpatialIndex.create(next);
    } catch (_:Dynamic) {
      if (nextSpatial != null) try nextSpatial.dispose() catch (_:Dynamic) {}
      if (next != null) try next.dispose() catch (_:Dynamic) {}
      presentationStale = true;
      return;
    }
    var previousSnapshot = snapshot;
    var previousSpatial = spatial;
    snapshot = next;
    if (nextSpatial != null) spatial = nextSpatial;
    presentationStale = false;
    if (nextSpatial != null) try previousSpatial.dispose() catch (_:Dynamic) {}
    try previousSnapshot.dispose() catch (_:Dynamic) {}
  }

  function failIfInjected(point:String):Void {
    var injector = failureInjection;
    if (injector != null) injector(point);
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
    var importedShape = requiredObject(id).kind == "cad-step";
    var genericPart = requiredObject(id).kind == "cad-part";
    if (!importedShape && !genericPart) {
      result.push(dimensionProperty(id, 0, prefix));
      result.push(dimensionProperty(id, 1, prefix));
    }
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
    if (!importedShape && !genericPart)
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
    var selectedFeature = selectedCadFeature(id);
    if (selectedFeature != null && selectedFeature.active && Std.isOfType(selectedFeature, ExtrudeFeature)) {
      var extrusion:ExtrudeFeature = cast selectedFeature;
      if (extrusion.amount != null)
        result.push(extrusionDepthProperty(id, extrusion.id.toInt(), prefix));
    }
    if (selectedFeature != null && selectedFeature.active && Std.isOfType(selectedFeature, FilletFeature))
      result.push(filletRadiusProperty(id, selectedFeature.id.toInt(), prefix));
    if (activeSketchEdit != null && activeSketchObjectId == id)
      appendSketchDraftProperties(result, activeSketchEdit, prefix);
    result.push(boolProperty(id,"collision","Collision",function(item)return item.collisionEnabled,
      "Physics",prefix));
    result.push(boolProperty(id,"dynamic","Dynamic body",function(item)return item.dynamicBody,
      "Physics",prefix));
    result.push(numberProperty(id,"mass","Mass",function(item)return item.mass,
      0.000001,1000000.0,"kg","Physics",prefix));
    return result;
  }

  function appendSketchDraftProperties(result:Array<PropertyDescriptor>, draft:CadSketchEditSession,
      prefix:String):Void {
    var authored = draft.sketch.snapshot();
    for (point in authored.points()) {
      var pointId = point.id;
      result.push(sketchDraftNumberProperty(prefix + "sketch-point-" + pointId + "-x",
        "Point " + pointId + " X", authored.units, false,
        function() return sketchPoint(pointId).x,
        function(value) editSketchDraft(function(sketch) {
          var current = sketchPoint(pointId);
          sketch.replacePoint(new SketchPoint(pointId, value, current.y));
        })));
      result.push(sketchDraftNumberProperty(prefix + "sketch-point-" + pointId + "-y",
        "Point " + pointId + " Y", authored.units, false,
        function() return sketchPoint(pointId).y,
        function(value) editSketchDraft(function(sketch) {
          var current = sketchPoint(pointId);
          sketch.replacePoint(new SketchPoint(pointId, current.x, value));
        })));
    }
    for (constraint in authored.constraints()) {
      if (constraint.kind != "distance" && constraint.kind != "radius" && constraint.kind != "angle")
        continue;
      var constraintId = constraint.id;
      var unit = constraint.kind == "angle" ? "rad" : authored.units;
      var positive = constraint.kind != "angle";
      result.push(sketchDraftNumberProperty(prefix + "sketch-dimension-" + constraintId,
        "Dimension " + constraintId, unit, positive,
        function() return sketchConstraint(constraintId).value,
        function(value) editSketchDraft(function(sketch) {
          var current = sketchConstraint(constraintId);
          sketch.replaceConstraint(SketchConstraint.raw(current.id, current.kind, current.first,
            current.second, current.third, value));
        })));
    }
  }

  function sketchDraftNumberProperty(key:String, label:String, unit:String, positive:Bool,
      read:Void->Float, write:Float->Void):PropertyDescriptor {
    var options = new PropertyDescriptorOptions();
    options.category = "Sketch draft";
    options.recordHistory = false;
    options.unit = unit;
    options.minimum = positive ? 0.000001 : null;
    options.step = 0.1;
    options.validator = function(_, value) {
      var number:Null<Float> = switch (value) {
        case PropertyValue.Float(next): next;
        case PropertyValue.Int(next): next;
        default: null;
      };
      return number == null || !Math.isFinite(number) || (positive && number <= 0)
        ? (positive ? "Value must be finite and positive" : "Value must be finite") : null;
    };
    var descriptor = new PropertyDescriptor(key, label, PropertyType.Float,
      function(_) return PropertyValue.Float(read()), function(_, value) {
        var number:Float = switch (value) {
          case PropertyValue.Float(next): next;
          case PropertyValue.Int(next): next;
          default: throw "Sketch values must be numeric";
        };
        if (!Math.isFinite(number) || (positive && number <= 0))
          throw (positive ? "Value must be finite and positive" : "Value must be finite");
        write(number);
      }, options);
    return descriptor;
  }

  function sketchPoint(id:String):SketchPoint {
    if (activeSketchEdit == null)
      throw "Sketch draft is no longer active";
    for (point in activeSketchEdit.sketch.snapshot().points())
      if (point.id == id) return point;
    throw "Sketch draft point no longer exists: " + id;
  }

  function sketchConstraint(id:String):SketchConstraint {
    if (activeSketchEdit == null)
      throw "Sketch draft is no longer active";
    for (constraint in activeSketchEdit.sketch.snapshot().constraints())
      if (constraint.id == id) return constraint;
    throw "Sketch draft constraint no longer exists: " + id;
  }

  function editSketchDraft(change:ConstrainedSketch->Void):Void {
    if (activeSketchEdit == null)
      throw "Sketch draft is no longer active";
    activeSketchEdit.edit(change);
    sketchDraftRevision++;
    refreshSelectionRevision();
  }

  function dimensionProperty(id:String, axis:Int, prefix:String):PropertyDescriptor {
    var settings = new PropertyDescriptorOptions();
    settings.category = "Geometry";
    settings.recordHistory = !isCadKind(requiredObject(id).kind);
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
    options.recordHistory = false;
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
    options.recordHistory=false;
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

  function extrusionDepthProperty(id:String, featureId:Int, prefix:String):PropertyDescriptor {
    var options = new PropertyDescriptorOptions();
    options.recordHistory = false;
    options.category = "Feature";
    options.unit = "mm";
    options.minimum = 0.000001;
    options.maximum = 1000000.0;
    options.step = 1.0;
    options.validator = function(_, value) {
      var number:Null<Float> = switch (value) {
        case PropertyValue.Float(next): next;
        case PropertyValue.Int(next): next;
        default: null;
      };
      return number == null || !Math.isFinite(number) || number <= 0
        ? "Extrusion depth must be finite and positive" : null;
    };
    return new PropertyDescriptor(prefix + "extrusion-depth", "Extrusion depth", PropertyType.Float,
      function(_) {
        var candidate = requireCadSession(id).document.featureById(featureId);
        if (candidate == null || !Std.isOfType(candidate, ExtrudeFeature))
          throw "selected extrusion is no longer available";
        var extrusion:ExtrudeFeature = cast candidate;
        if (extrusion.amount == null)
          throw "extrusion has no editable depth parameter";
        return PropertyValue.Float(extrusion.amount.value);
      }, function(_, value) {
        var depth:Float = switch (value) {
          case PropertyValue.Float(next): next;
          case PropertyValue.Int(next): next;
          default: throw "Extrusion depth requires a number";
        };
        setExtrusionDepth(id, featureId, depth);
      }, options);
  }

  function filletRadiusProperty(id:String, featureId:Int, prefix:String):PropertyDescriptor {
    var options = new PropertyDescriptorOptions();
    options.recordHistory = false;
    options.category = "Feature";
    options.unit = "mm";
    options.minimum = 0.000001;
    options.maximum = 1000000.0;
    options.step = 0.5;
    options.validator = function(_, value) {
      var number:Null<Float> = switch (value) {
        case PropertyValue.Float(next): next;
        case PropertyValue.Int(next): next;
        default: null;
      };
      return number == null || !Math.isFinite(number) || number <= 0
        ? "Fillet radius must be finite and positive" : null;
    };
    return new PropertyDescriptor(prefix + "fillet-radius", "Fillet radius", PropertyType.Float,
      function(_) {
        var candidate = requireCadSession(id).document.featureById(featureId);
        if (candidate == null || !Std.isOfType(candidate, FilletFeature))
          throw "selected fillet is no longer available";
        var fillet:FilletFeature = cast candidate;
        return PropertyValue.Float(fillet.radius.value);
      }, function(_, value) {
        var radius:Float = switch (value) {
          case PropertyValue.Float(next): next;
          case PropertyValue.Int(next): next;
          default: throw "Fillet radius requires a number";
        };
        setFilletRadius(id, featureId, radius);
      }, options);
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
    return GeometryData.box(width, height, depth);
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
        visible: item.visible,cadGraph:item.cadGraph,meshSnapshot:item.meshSnapshot});
    }
    return result;
  }

  /** Save boundary: serialize each live authored CAD document only when requested. */
  public function recordsForSave():Array<SceneObjectData> {
    var result=records();
    for(record in result)if(isCadKind(record.type))
      record.cadGraph=currentCadGraph(record.id);
    var draft = activeSketchEdit;
    if (draft != null) {
      var featureIndex = sketchDraftFeatureIndex(draft);
      var foundOwner = false;
      for (record in result) if (record.id == activeSketchObjectId) {
        record.sketchDraft = SketchDraftCodec.encode(draft.sketch.snapshot(), featureIndex);
        foundOwner = true;
      }
      if (!foundOwner)
        throw "sketch draft owner is no longer in the scene";
    }
    return result;
  }

  public function hasUnsavedSketchDraftChanges():Bool {
    var present = activeSketchEdit != null;
    return present != savedSketchDraftPresent ||
      (present && sketchDraftRevision != savedSketchDraftRevision);
  }

  public function markSaved():Void {
    savedSketchDraftPresent = activeSketchEdit != null;
    savedSketchDraftRevision = sketchDraftRevision;
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
      canUndo: document.canUndo, canRedo: document.canRedo,
      dirty: document.isDirty || hasUnsavedSketchDraftChanges()};
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    if (activeSketchEdit != null) {
      try activeSketchEdit.cancel() catch (_:Dynamic) {}
      activeSketchEdit = null;
      activeSketchObjectId = null;
      sketchEditPlaneValue = null;
    }
    if (selectedCadFace != null) {
      selectedCadFace.close();
      selectedCadFace = null;
    }
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
    if(kind=="cad-step") {
      if (graph == null) throw "Imported STEP object has no persisted source graph";
      model = CadImportedModel.decode(graph);
    } else if (kind == "cad-part") {
      model = graph == null ? CadPartModel.createEmpty() : CadPartModel.decode(graph);
    } else if(kind=="cad-bracket") {
      model=graph==null?CadBracketModel.create(width,height,depth):CadBracketModel.decode(graph);
    } else {
      model=graph==null?CadPlateModel.create(width,height,depth,
        Math.min(0.012,Math.min(width,height)*0.5)):CadPlateModel.decode(graph);
    }
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
    return kind=="cad-plate"||kind=="cad-bracket"||kind=="cad-step"||kind=="cad-part";

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
  public var meshSnapshot:Null<String>;
  public var x:Float;
  public var y:Float;
  public var z:Float;
  public var visible:Bool;
  public function new(id:String,label:String,kind:String,width:Float,height:Float,depth:Float,
      collisionEnabled:Bool,dynamicBody:Bool,mass:Float,red:Float,green:Float,blue:Float,
      ?cadGraph:String,x:Float=0,y:Float=0,z:Float=0,visible:Bool=true,?meshSnapshot:String) {
    this.id = id; this.label = label; this.kind = kind;
    this.width=width;this.height=height;this.depth=depth;this.collisionEnabled=collisionEnabled;
    this.dynamicBody=dynamicBody;this.mass=mass;
    this.red = red; this.green = green; this.blue = blue;
    this.cadGraph=cadGraph;
    this.meshSnapshot=meshSnapshot;
    this.x=x;this.y=y;this.z=z;this.visible=visible;
  }
}

/** Runtime-only resources associated with one document object. */
private class SceneBridge {
  public final scene:Scene;
  var objects:Map<String, EditorSceneRuntimeObject> = new Map();
  var nodeEntries:Map<String, String> = new Map();

  public function new() scene = Scene.create();

  public function attach(id:String,node:NodeId,geometry:Geometry,material:Material):Void {
    objects.set(id,new EditorSceneRuntimeObject(node,geometry,material));
    nodeEntries.set(nodeKey(node), id);
  }

  public function runtime(id:String):Null<EditorSceneRuntimeObject>
    return objects.get(id);

  public function idForNode(node:NodeId):Null<String>
    return nodeEntries.get(nodeKey(node));

  public static function nodeKey(node:NodeId):String
    return haxe.Int64.toStr(node.stableValue());

  public function copyEntries():Map<String, EditorSceneRuntimeObject> {
    var result:Map<String, EditorSceneRuntimeObject> = new Map();
    for (id in objects.keys()) {
      var value = objects.get(id);
      if (value != null) result.set(id, value);
    }
    return result;
  }

  public function copyNodeEntries():Map<String, String> {
    var result:Map<String, String> = new Map();
    for (key in nodeEntries.keys()) {
      var value = nodeEntries.get(key);
      if (value != null) result.set(key, value);
    }
    return result;
  }

  public function replaceEntries(entries:Map<String, EditorSceneRuntimeObject>,
      nodes:Map<String, String>):Void {
    objects = entries;
    nodeEntries = nodes;
  }

  public function detach(id:String):Void {
    var value = objects.get(id);
    if (value != null) nodeEntries.remove(nodeKey(value.node));
    objects.remove(id);
  }

  public function dispose():Void
    scene.dispose();
}

/** All fallible scene-edit work staged before the native transaction commits. */
private class PreparedSceneEdit {
  public final transaction:Transaction;
  public final objects:Array<EditorSceneObject>;
  public final bridgeEntries:Map<String, EditorSceneRuntimeObject>;
  public final nodeEntries:Map<String, String>;
  public final cadSessions:Map<String, CadDocumentSession>;
  public final stagedCadSessions:Array<CadDocumentSession> = [];
  public final changedBounds:Array<NodeId> = [];
  public final createdGeometry:Array<Geometry> = [];
  public final createdMaterials:Array<Material> = [];
  public final retiredCadSessions:Array<CadDocumentSession> = [];
  public final retiredGeometry:Array<Geometry> = [];
  public final retiredMaterials:Array<Material> = [];
  public var changed:Bool = false;

  public function new(transaction:Transaction, objects:Array<EditorSceneObject>,
      bridgeEntries:Map<String, EditorSceneRuntimeObject>,
      nodeEntries:Map<String, String>,
      cadSessions:Map<String, CadDocumentSession>) {
    this.transaction = transaction;
    this.objects = objects;
    this.bridgeEntries = bridgeEntries;
    this.nodeEntries = nodeEntries;
    this.cadSessions = cadSessions;
  }

  public function abort():Void {
    try transaction.dispose() catch (_:Dynamic) {}
    for (geometry in createdGeometry) try geometry.dispose() catch (_:Dynamic) {}
    for (material in createdMaterials) try material.dispose() catch (_:Dynamic) {}
    for (session in stagedCadSessions) try session.close() catch (_:Dynamic) {}
  }

  /** Retirement happens only after the native scene no longer references these handles. */
  public function retire():Void {
    for (geometry in retiredGeometry) try geometry.dispose() catch (_:Dynamic) {}
    for (material in retiredMaterials) try material.dispose() catch (_:Dynamic) {}
    for (session in retiredCadSessions) try session.close() catch (_:Dynamic) {}
  }
}

private class EditorSceneRuntimeObject {
  public final node:NodeId;
  public final geometry:Geometry;
  public final material:Material;
  public function new(node:NodeId, geometry:Geometry, material:Material) {
    this.node=node;this.geometry=geometry;this.material=material;
  }
}
