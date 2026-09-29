package app.editor;

import app.EditorScene;
import app.EditorScene.EditorSceneObject;
import app.EditorScene.EditorSceneRuntimeObject;
import app.EditorScene.SceneBridge;
import app.SceneObjectData;
import nativekit.scene.ChangeSet;
import nativekit.scene.Geometry;
import nativekit.scene.Material;
import nativekit.scene.NodeId;
import nativekit.scene.Transaction;
import app.CadDocumentSession;

/** Stages record reconciliation and commits scene, CAD, and selection changes atomically. */
@:access(app.EditorScene)
class SceneReconciler {
  static function sameWorker(a:Null<app.WorkerObjectData>, b:Null<app.WorkerObjectData>):Bool {
    if (a == null || b == null) return a == null && b == null;
    if (a.asset != b.asset || a.job != b.job || a.migrationNote != b.migrationNote ||
        a.zones.length != b.zones.length) return false;
    for (index in 0...a.zones.length) if (a.zones[index] != b.zones[index]) return false;
    return true;
  }

  public static function replace(owner:EditorScene, data:Array<SceneObjectData>, selection:String):Void {
    owner.fullReconciliationCount = owner.fullReconciliationCount + 1;
    var physicsChanged = owner.model.physicsRecordsChanged(data);
    var previousSelectedId = owner.selectedId;
    var previousSelectedFeatureKey = owner.selectedFeatureKey;
    var previousSelected = owner.object(previousSelectedId);
    var previousSelectedKind:Null<String> = previousSelected == null ? null : previousSelected.kind;
    var existingById:Map<String, EditorSceneObject> = new Map();
    for (item in owner.objects) existingById.set(item.id, item);
    if (!physicsChanged) for (record in data) if (owner.kinematicOccurrences.exists(record.id)) {
      var old = existingById.get(record.id);
      if (old == null || old.mass != record.mass || old.materialId != record.materialId || old.meshSnapshot != record.meshSnapshot ||
          old.kind != record.type) { physicsChanged = true; break; }
    }
    var staleFaceHoverIds:Array<String> = [];
    for (id in owner.faceHoverNodes.keys()) {
      var record:Null<SceneObjectData> = null;
      for (candidate in data) if (candidate.id == id) {
        record = candidate;
        break;
      }
      var existing = existingById.get(id);
      var sourceChanged = record != null && existing != null && record.type == "cad-preview"
        ? existing.meshSnapshot != record.meshSnapshot : false;
      if (record == null || existing == null || !EditorScene.isFaceHoverKind(record.type) ||
          existing.kind != record.type || existing.cadGraph != record.cadGraph || sourceChanged)
        staleFaceHoverIds.push(id);
    }
    var prepared = new PreparedSceneEdit(owner.scene.beginTransaction(), [],
      owner.bridge.copyEntries(), owner.bridge.copyNodeEntries(), owner.cadSessions.copy());
    var committedChanges:Null<ChangeSet> = null;
    for (id in staleFaceHoverIds) {
      var node = owner.faceHoverNodes.get(id);
      if (node != null) prepared.transaction.destroyNode(node);
    }
    var previousFace=owner.selectedCadFaceFingerprint;
    var previousFaceShape=owner.selectedCadFace;
    var previousFaceX=owner.selectedCadFaceX,previousFaceY=owner.selectedCadFaceY;
    var retained:Map<String, Bool> = new Map();
    try {
      for (record in data) {
        var item = existingById.get(record.id);
        var session = item == null || !EditorScene.isCadKind(item.kind) || item.kind != record.type || item.cadGraph != record.cadGraph
          ? null : owner.cadSessions.get(record.id);
        var storedGraph = record.cadGraph;
        if (EditorScene.isCadKind(record.type) && session == null) {
          session = owner.createCadSession(storedGraph, record.width, record.height, record.depth, record.type);
          prepared.stagedCadSessions.push(session);
          owner.failIfInjected("prepare.cad-session");
          storedGraph = session.encode();
        }
        if (session != null)
          prepared.cadSessions.set(record.id, session);
        else
          prepared.cadSessions.remove(record.id);
        if (item == null) {
          var geometry = record.type == "cad-preview"
            ? owner.sharedPreviewGeometry(record.meshSnapshot, prepared.bridgeEntries, owner.objects, prepared.objects)
            : null;
          if (geometry == null) {
            geometry = owner.scene.createGeometry();
            prepared.createdGeometry.push(geometry);
            var geometryData = session != null
              ? session.geometry()
              : (record.type == "cad-preview"
                ? owner.previewGeometry(record.meshSnapshot)
                : owner.plainGeometry(record.id, record.type, record.width, record.height, record.depth));
            owner.scene.setGeometryData(geometry, geometryData);
            owner.failIfInjected("prepare.new-geometry");
          }
          if (geometry == null) throw 'No geometry resource was prepared for "${record.id}"';
          var material = owner.scene.createMaterial();
          prepared.createdMaterials.push(material);
          owner.scene.setMaterialData(material, ScenePresentation.materialFor(record.red, record.green, record.blue, record.appearance, record.type));
          owner.failIfInjected("prepare.new-material");
          var node = prepared.transaction.createNode();
          prepared.transaction.setName(node, record.label);
          prepared.transaction.setVisibility(node, record.visible);
          prepared.transaction.setGeometry(node, geometry);
          prepared.transaction.setMaterial(node, material);
          prepared.transaction.setTransform(node, EditorScene.objectTransform(record.x, record.y, record.z, record.rotation));
          prepared.bridgeEntries.set(record.id, new EditorSceneRuntimeObject(node, geometry, material));
          prepared.nodeEntries.set(SceneBridge.nodeKey(node), record.id);
          owner.failIfInjected("prepare.bridge-attach");
          item = new EditorSceneObject(record.id, record.label, record.type,
            record.width, record.height, record.depth, record.collisionEnabled,
            record.dynamicBody, record.mass, record.red, record.green, record.blue,
            storedGraph,
            record.x, record.y, record.z, record.visible, record.meshSnapshot, record.rotation, record.appearance, record.materialId);
          item.worker = record.worker;
          prepared.objects.push(item);
          owner.failIfInjected("prepare.new-object");
          prepared.changed = true;
        } else {
          var runtime = prepared.bridgeEntries.get(record.id);
          if (runtime == null) throw "Missing runtime scene node: " + record.id;
          var candidate = item;
          var objectChanged = item.label != record.label || item.visible != record.visible ||
            item.x != record.x || item.y != record.y || item.z != record.z ||
            item.width != record.width || item.height != record.height || item.depth != record.depth ||
            item.kind != record.type || item.cadGraph != storedGraph ||
            item.meshSnapshot != record.meshSnapshot || !EditorScene.sameRotation(item.rotation, record.rotation) ||
            item.collisionEnabled != record.collisionEnabled || item.dynamicBody != record.dynamicBody ||
            item.mass != record.mass || item.materialId != record.materialId || item.red != record.red || item.green != record.green ||
            item.blue != record.blue || !EditorScene.sameFinish(item.appearance, record.appearance) ||
            !sameWorker(item.worker, record.worker);
          if (objectChanged) candidate = SceneModel.copyEditorSceneObject(item);
          if (item.label != record.label) prepared.transaction.setName(runtime.node, record.label);
          if (item.visible != record.visible) prepared.transaction.setVisibility(runtime.node, record.visible);
          if (item.x != record.x || item.y != record.y || item.z != record.z ||
              !EditorScene.sameRotation(item.rotation, record.rotation)) {
            prepared.transaction.setTransform(runtime.node,
              EditorScene.objectTransform(record.x, record.y, record.z, record.rotation));
            prepared.changedBounds.push(runtime.node);
          }
          if (item.label != record.label || item.visible != record.visible || item.x != record.x ||
              item.y != record.y || item.z != record.z || !EditorScene.sameRotation(item.rotation, record.rotation)) prepared.changed = true;
          if (item.width != record.width || item.height != record.height || item.depth != record.depth ||
              item.kind != record.type || item.cadGraph != storedGraph || item.meshSnapshot != record.meshSnapshot) {
            var geometry = record.type == "cad-preview"
              ? owner.sharedPreviewGeometry(record.meshSnapshot, prepared.bridgeEntries, owner.objects,
                prepared.objects, record.id)
              : null;
            if (geometry == null) {
              geometry = owner.scene.createGeometry();
              prepared.createdGeometry.push(geometry);
              var geometryData = session != null
                ? session.geometry()
                : (record.type == "cad-preview"
                  ? owner.previewGeometry(record.meshSnapshot)
                  : owner.plainGeometry(record.id, record.type, record.width, record.height, record.depth));
              owner.scene.setGeometryData(geometry, geometryData);
            }
            if (geometry == null) throw 'No geometry resource was prepared for "${record.id}"';
            prepared.transaction.setGeometry(runtime.node, geometry);
            prepared.retiredGeometry.push(runtime.geometry);
            runtime = new EditorSceneRuntimeObject(runtime.node, geometry, runtime.material);
            prepared.bridgeEntries.set(record.id, runtime);
            owner.failIfInjected("prepare.existing-geometry");
            prepared.changed = true;
          }
          if (item.red != record.red || item.green != record.green || item.blue != record.blue ||
              !EditorScene.sameFinish(item.appearance, record.appearance)) {
            var material = owner.scene.createMaterial();
            prepared.createdMaterials.push(material);
            owner.scene.setMaterialData(material, ScenePresentation.materialFor(record.red, record.green, record.blue, record.appearance, record.type));
            prepared.transaction.setMaterial(runtime.node, material);
            prepared.retiredMaterials.push(runtime.material);
            runtime = new EditorSceneRuntimeObject(runtime.node, runtime.geometry, material);
            prepared.bridgeEntries.set(record.id, runtime);
            owner.failIfInjected("prepare.existing-material");
            prepared.changed = true;
          }
          if (item.collisionEnabled != record.collisionEnabled || item.dynamicBody != record.dynamicBody ||
              item.mass != record.mass) prepared.changed = true;
          candidate.label = record.label; candidate.kind = record.type;
          candidate.width = record.width; candidate.height = record.height; candidate.depth = record.depth;
          candidate.collisionEnabled = record.collisionEnabled; candidate.dynamicBody = record.dynamicBody;
          candidate.mass = record.mass; candidate.materialId = record.materialId; candidate.red = record.red; candidate.green = record.green; candidate.blue = record.blue; candidate.appearance = record.appearance;
          candidate.cadGraph = storedGraph; candidate.x = record.x; candidate.y = record.y; candidate.z = record.z;
          candidate.meshSnapshot = record.meshSnapshot;
          candidate.rotation = record.rotation;
          candidate.worker = record.worker;
          candidate.visible = record.visible;
          if (objectChanged) prepared.changed = true;
          prepared.objects.push(candidate);
          owner.failIfInjected("prepare.existing-object");
        }
        retained.set(record.id, true);
      }
      for (item in owner.objects) if (!retained.exists(item.id)) {
        var runtime = owner.bridge.runtime(item.id);
        if (runtime == null) throw "Missing runtime scene node: " + item.id;
        prepared.transaction.destroyNode(runtime.node);
        prepared.bridgeEntries.remove(item.id);
        prepared.nodeEntries.remove(SceneBridge.nodeKey(runtime.node));
        prepared.retiredGeometry.push(runtime.geometry);
        prepared.retiredMaterials.push(runtime.material);
        prepared.cadSessions.remove(item.id);
        prepared.changed = true;
      }
      for (id in owner.cadSessions.keys()) {
        var previous = owner.cadSessions.get(id);
        if (previous != null && prepared.cadSessions.get(id) != previous)
          prepared.retiredCadSessions.push(previous);
      }
      owner.failIfInjected("transaction.before-commit");
      committedChanges = prepared.transaction.commitWithChanges();
    } catch (error:Dynamic) {
      prepared.abort();
      throw error;
    }
    for (id in staleFaceHoverIds) {
      owner.faceHoverNodes.remove(id);
      owner.faceHoverIndexes.remove(id);
      var geometry = owner.faceHoverGeometries.get(id);
      owner.faceHoverGeometries.remove(id);
      if (geometry != null) geometry.dispose();
    }
    owner.objects = prepared.objects;
    owner.cadSessions = prepared.cadSessions;
    owner.pruneStockSimulations();
    owner.bridge.replaceEntries(prepared.bridgeEntries, prepared.nodeEntries);
    owner.syncWorkerVisuals();
    owner.selection.selectedId = selection;
    owner.selection.selectedFeatureKey=null;
    if (prepared.changed) owner.publish(prepared.changedBounds, physicsChanged, committedChanges);
    else if (committedChanges != null) committedChanges.dispose();
    var restoredFace = -1;
    if(previousFace!=null){
      var selected=owner.object(selection);
      if(selected!=null&&EditorScene.isCadKind(selected.kind)){
        var session=owner.cadSessions.get(selected.id);
        if(session!=null)try {
          restoredFace=owner.remapSelectedCadFace(session,previousFaceShape,previousFace);
          if(restoredFace>=0){owner.selection.selectedCadFaceX=previousFaceX;owner.selection.selectedCadFaceY=previousFaceY;}
        } catch(error:Dynamic) { restoredFace=-1; }
      }
    }
    if(restoredFace<0)try owner.clearSelectedCadFace() catch (_:Dynamic) {}
    prepared.retire();
    var nextSelected = owner.object(selection);
    var propertySchemaChanged = previousSelectedId != selection || previousSelectedFeatureKey != null ||
      previousSelectedKind != (nextSelected == null ? null : nextSelected.kind) ||
      (previousSelected != null && nextSelected != null && nextSelected.kind == "human-worker" &&
        !sameWorker(previousSelected.worker, nextSelected.worker));
    if (propertySchemaChanged)
      owner.selection.changed(owner);
  }

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
    var activeGeometry:Array<Geometry> = [];
    for (runtime in bridgeEntries) if (activeGeometry.indexOf(runtime.geometry) < 0)
      activeGeometry.push(runtime.geometry);
    var disposedGeometry:Array<Geometry> = [];
    for (geometry in retiredGeometry) if (activeGeometry.indexOf(geometry) < 0 &&
        disposedGeometry.indexOf(geometry) < 0) {
      try geometry.dispose() catch (_:Dynamic) {}
      disposedGeometry.push(geometry);
    }
    for (material in retiredMaterials) try material.dispose() catch (_:Dynamic) {}
    for (session in retiredCadSessions) try session.close() catch (_:Dynamic) {}
  }
}
