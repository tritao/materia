package app.editor;

import Observable;

import app.EditorScene;
import cadkit.Shape;
import cadkit.parametric.TopologyFingerprint;
import app.editor.SelectionReference.PickedFace;

/** Selection state has its own revision, independent of scene content. */
@:access(app.EditorScene) class SelectionModel implements Observable {
  public var selectedId:String = "box";
  public var selectedFeatureKey:Null<String> = null;
  public var selectedCadFaceIndex:Int = -1;
  public var selectedCadEdgeIndex:Int = -1;
  public var selectedCadFaceX:Float = 0.0;
  public var selectedCadFaceY:Float = 0.0;
  public var selectedCadFaceFingerprint:Null<TopologyFingerprint> = null;
  public var selectedCadFace:Null<Shape> = null;
  /** The face under the last plain click on the selected part (any kind of part), for the status bar and its copy text. */
  public var pickedFace:Null<PickedFace> = null;
  public var revision:Int = 1;

  public function new() {
  }

  public function changed(scene:EditorScene):Void {
    revision++;
    scene.markVisualChanged();
    var listener = scene.onSelectionChanged;
    if (listener != null) listener();
  }

  public function select(scene:EditorScene, id:String):Bool {
    if (id != "scene" && scene.object(id) == null) return false;
    if (id == selectedId && selectedFeatureKey == null) return false;
    if (scene.activeSketchEdit != null
      && (id != scene.activeSketchObjectId || selectedFeatureKey != null)) scene.cancelSelectedSketchEdit();
    scene.clearSelectedCadFace();
    selectedId = id;
    selectedFeatureKey = null;
    changed(scene);
    return true;
  }

  public function selectTreeKey(scene:EditorScene, key:String):Bool {
    if (key == "scene" || scene.object(key) != null) return select(scene, key);
    var marker = key.indexOf(":feature:");
    if (marker < 0) return select(scene, key);
    var id = key.substr(0, marker), item = scene.object(id);
    if (item == null || !EditorScene.isCadKind(item.kind)) return false;
    if (scene.activeSketchEdit != null
      && (id != scene.activeSketchObjectId || key != selectedFeatureKey)) scene.cancelSelectedSketchEdit();
    if (selectedId == id && selectedFeatureKey == key) return false;
    scene.clearSelectedCadFace();
    selectedId = id;
    selectedFeatureKey = key;
    changed(scene);
    return true;
  }
}
