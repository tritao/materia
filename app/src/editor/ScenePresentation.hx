package app.editor;

import app.EditorScene.SceneBridge;
import app.EditorScene;
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

  public static function materialFor(red:Float, green:Float, blue:Float, appearance:Null<Appearance>):MaterialData {
    var finish = appearance == null ? Appearances.neutral() : appearance;
    return MaterialData.opaque(red, green, blue).setMetallic(finish.metallic).setRoughness(finish.roughness);
  }

}
