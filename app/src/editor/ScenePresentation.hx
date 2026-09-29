package app.editor;

import app.EditorScene.SceneBridge;
import app.EditorScene;
import app.ApplicationSimulation.SimulationPoseVisual;
import nativekit.scene.SceneView;
import nativekit.scene.SelectionSet;
import app.EditorScene.EditorSceneObject;
import CadKit;
import cadkit.Shape;
import nativekit.scene.GeometryData;
import nativekit.scene.Transform;
import materia.project.Appearance;
import materia.project.Appearance.Appearances;
import nativekit.scene.SceneSnapshot;
import nativekit.scene.SpatialIndex;
import nativekit.scene.NodeId;
import nativekit.scene.Geometry;
import nativekit.scene.Material;
import nativekit.scene.MaterialData;
import nativekit.scene.ChangeSet;

/** SceneKit resources, renderer changes, and hover presentation state. */
@:access(app.EditorScene)
class ScenePresentation {
  public var bridge:SceneBridge;
  public var snapshot:SceneSnapshot;
  public var spatial:SpatialIndex;
  public var presentationStale:Bool = false;
  public var fullReconciliationCount:Int = 0;
  public var spatialFullRebuildCount:Int = 0;
  public var pendingRenderChanges:Null<ChangeSet> = null;
  public var renderNeedsRefresh:Bool = false;
  public var selectionMaterial:Material;
  public var hoverMaterial:Material;
  public final faceHoverNodes:Map<String, NodeId> = new Map();
  public final faceHoverGeometries:Map<String, Geometry> = new Map();
  public final faceHoverIndexes:Map<String, Int> = new Map();

  public function new() bridge = new SceneBridge();

  public function queueRenderChanges(changes:ChangeSet):Void {
    if (renderNeedsRefresh) { changes.dispose(); return; }
    if (pendingRenderChanges != null) {
      pendingRenderChanges.dispose();
      pendingRenderChanges = null;
      changes.dispose();
      renderNeedsRefresh = true;
    } else pendingRenderChanges = changes;
  }

  public function requireRenderRefresh():Void {
    if (pendingRenderChanges != null) pendingRenderChanges.dispose();
    pendingRenderChanges = null;
    renderNeedsRefresh = true;
  }

  /** Transfers the one pending change set to the viewport; null requests a refresh. */
  public function takeRenderChanges():Null<ChangeSet> {
    if (renderNeedsRefresh) {
      renderNeedsRefresh = false;
      return null;
    }
    var changes = pendingRenderChanges;
    pendingRenderChanges = null;
    return changes;
  }

  /** Derived presentation caches may lag a committed edit and retry on the next access/frame. */
  public function refreshIfStale(owner:EditorScene):Void {
    if (presentationStale) rebuild(owner);
    if (presentationStale) throw "Scene presentation is unavailable until its derived caches rebuild";
  }

  public function rebuild(owner:EditorScene, ?updatedNodes:Array<NodeId>):Void {
    var next:Null<SceneSnapshot> = null;
    var nextSpatial:Null<SpatialIndex> = null;
    try {
      owner.failIfInjected("publish.snapshot");
      next = owner.scene.snapshot();
      owner.failIfInjected("publish.spatial-index");
      if (!spatial.updateNodes(next, updatedNodes == null ? [] : updatedNodes)) {
        nextSpatial = SpatialIndex.create(next);
        spatialFullRebuildCount = spatialFullRebuildCount + 1;
      }
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

  /** Ensures a hidden child node contains exactly one CAD or preview face for hover rendering. */
  public function ensureFaceHover(owner:EditorScene, id:String, faceIndex:Int):Void {
    var item = owner.object(id);
    if (item == null || !EditorScene.isFaceHoverKind(item.kind) || faceIndex < 0) return;
    if (faceHoverIndexes.get(id) == faceIndex && faceHoverNodes.exists(id)) return;

    var geometryData = faceHoverGeometry(owner, id, item, faceIndex);
    if (geometryData == null) return;
    var geometry = faceHoverGeometries.get(id);
    if (geometry == null) {
      geometry = owner.scene.createGeometry();
      owner.scene.setGeometryData(geometry, geometryData);
      var transaction = owner.scene.beginTransaction();
      var node = transaction.createNode();
      transaction.setName(node, "Hover face " + (faceIndex + 1));
      transaction.setVisibility(node, false);
      transaction.setParent(node, owner.runtimeFor(id).node);
      transaction.setGeometry(node, geometry);
      transaction.setMaterial(node, hoverMaterial);
      transaction.setTransform(node, Transform.identity());
      transaction.commit();
      faceHoverNodes.set(id, node);
      faceHoverGeometries.set(id, geometry);
    } else {
      owner.scene.setGeometryData(geometry, geometryData);
    }
    faceHoverIndexes.set(id, faceIndex);
    owner.publish();
  }

  /** Common face-hover geometry boundary for live CAD and serialized preview objects. */
  function faceHoverGeometry(owner:EditorScene, id:String, item:EditorSceneObject,
      faceIndex:Int):Null<GeometryData> {
    if (EditorScene.isCadKind(item.kind)) {
      var session = owner.cadSessions.get(id);
      if (session == null) return null;
      var output = session.document.outputFeatureOrNull();
      if (output == null || output.currentShape() == null) return null;
      var face:Shape = output.currentShape().subshape(CadKit.ShapeKind.Face, faceIndex);
      try {
        var result = session.model.geometryFor(face, true);
        face.close();
        return result;
      } catch (error:Dynamic) {
        face.close();
        throw error;
      }
    }
    if (item.kind == "cad-preview" && item.meshSnapshot != null)
      return owner.previewGeometry(item.meshSnapshot).subelementGeometry(faceIndex);
    return null;
  }

  public function configureRenderView(owner:EditorScene, view:SceneView, viewProjection:Transform,
      ?poses:Array<SimulationPoseVisual>, ?hoveredId:String, ?hoveredFaceIndex:Int = -1):SceneView {
    view.setViewProjection(viewProjection);
    var selected = owner.object(owner.selectedId);
    var selection = new SelectionSet();
    if (selected != null) {
      var workerVisual = owner.workerVisuals.get(selected.id);
      if (workerVisual == null) selection.add(owner.runtimeFor(selected.id).node);
      else for (node in workerVisual.character.model.primitiveNodes) selection.add(node);
    }
    view.applySelection(selection, selectionMaterial);
    var faceHoverNode = hoveredId == null || hoveredFaceIndex == null || hoveredFaceIndex < 0
      ? null : faceHoverNodes.get(hoveredId);
    if (faceHoverNode != null && faceHoverIndexes.get(hoveredId) == hoveredFaceIndex) {
      // The face node is hidden in the scene and shown only in this presentation view.
      view.setVisibility(faceHoverNode, true);
      view.setMaterial(faceHoverNode, hoverMaterial);
      view.applyHover(null, hoverMaterial);
    } else {
      var hovered = hoveredId == null ? null : owner.object(hoveredId);
      view.applyHover(hovered == null ? null : owner.runtimeFor(hovered.id).node, hoverMaterial);
    }
    if (poses != null) {
      var poseNodes:Array<NodeId> = [];
      var poseTransforms:Array<Transform> = [];
      for (pose in poses) {
        var runtime = bridge.runtime(pose.id);
        if (runtime != null) {
          poseNodes.push(runtime.node);
          poseTransforms.push(EditorScene.poseTransform(pose.position, pose.rotation));
        }
      }
      view.replacePoses(poseNodes, poseTransforms);
    }
    return view;
  }

  public static function materialFor(red:Float, green:Float, blue:Float, appearance:Null<Appearance>,
      kind:String = ""):MaterialData {
    var finish = appearance == null ? Appearances.neutral() : appearance;
    var result = MaterialData.opaque(red, green, blue).setMetallic(finish.metallic).setRoughness(finish.roughness);
    if (kind == "human-worker") result.setOpacity(0.0).setOpaque(false);
    return result;
  }

}
