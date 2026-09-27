package app.editor;

import app.EditorScene.SceneBridge;
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

  public static function materialFor(red:Float, green:Float, blue:Float, appearance:Null<Appearance>):MaterialData {
    var finish = appearance == null ? Appearances.neutral() : appearance;
    return MaterialData.opaque(red, green, blue).setMetallic(finish.metallic).setRoughness(finish.roughness);
  }

}
