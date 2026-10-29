package app.editor;

import app.EditorScene;
import nativekit.scene.Geometry;
import nativekit.scene.NodeId;
import nativekit.scene.SceneView;

private typedef SourceFaceEntry = {id:String, snapshot:String, node:NodeId, face:Int, owned:Bool, geometry:Geometry};

/** Display-only face overlays. They add no occurrence, mass or independent pick target. */
@:access(app.EditorScene) class SourceFacePresentation {
  public final entries:Map<String, SourceFaceEntry> = [];
  function entry(key:String):SourceFaceEntry {
    var found = entries.get(key);
    if (found == null) throw "Missing source highlight resource";
    return found;
  }
  public var active:Array<String> = [];
  public function new() {
  }
  public function prepare(owner:EditorScene):Void {
    var selected = owner.sourceSelection;
    var features = selected == null ? [] : selected();
    active = [];
    var stale:Array<String> = [];
    for (key => entry in entries) {
      var item = owner.object(entry.id);
      if (item == null || item.meshSnapshot != entry.snapshot) stale.push(key);
    }
    if (stale.length > 0) {
      var edit = owner.scene.beginTransaction();
      for (key in stale) edit.destroyNode(entry(key).node);
      edit.commit();
      for (key in stale) {
        if (entry(key).owned) entry(key).geometry.dispose();
        entries.remove(key);
      }
      owner.publish();
    }
    var changed = false;
    for (feature in features) {
      var id = "project:" + feature.occurrence, item = owner.object(id);
      if (item == null || item.kind != "cad-preview" || item.meshSnapshot == null) continue;
      for (face in (feature.whole ? [-1] : feature.faces)) {
        var key = id + ":" + face;
        if (active.indexOf(key) >= 0) continue;
        active.push(key);
        if (entries.exists(key)) continue;
        var data = face < 0 ? null : owner.previewGeometry(item.meshSnapshot).subelementGeometry(face);
        if (face >= 0 && data == null) continue;
        var geometry = face < 0 ? owner.runtimeFor(id).geometry : owner.scene.createGeometry();
        try {
          if (data != null) owner.scene.setGeometryData(geometry, data);
          var edit = owner.scene.beginTransaction();
          try {
            var node = edit.createNode();
            edit.setName(node, "Source surface " + feature.id);
            edit.setVisibility(node, false);
            edit.setGeometry(node, geometry);
            edit.setMaterial(node, owner.selectionMaterial);
            edit.setTransform(node, owner.info(id).worldTransform());
            edit.commit();
            entries.set(
              key,
              {id: id, snapshot: item.meshSnapshot, node: node, face: face, owned: face >= 0, geometry: geometry}
            );
          } catch (error:Dynamic) {
            edit.dispose();
            throw error;
          }
        } catch (error:Dynamic) {
          if (face >= 0) geometry.dispose();
          throw error;
        }
        changed = true;
      }
    }
    if (changed) owner.publish();
  }
  public function owner(node:NodeId):Null<String> {
    for (entry in entries) if (entry.node.equals(node)) return entry.id;
    return null;
  }
  public function face(node:NodeId):Int {
    for (entry in entries) if (entry.node.equals(node)) return entry.face;
    return -1;
  }
  public function apply(owner:EditorScene, view:SceneView):Void {
    for (key in active) {
      var entry = entries.get(key);
      if (entry == null) continue;
      // The overlay is forced visible in this view, so a hidden owner must keep it from showing: it would float where the part was.
      if (!owner.isVisible(entry.id)) continue;
      view.setPose(entry.node, owner.info(entry.id).worldTransform());
      view.setVisibility(entry.node, true);
      view.setDiagnosticTint(entry.node, owner.selectionMaterial);
    }
  }
  public function dispose(owner:EditorScene):Void {
    if (!entries.keys().hasNext()) return;
    var edit = owner.scene.beginTransaction();
    for (entry in entries) edit.destroyNode(entry.node);
    edit.commit();
    for (entry in entries) if (entry.owned) entry.geometry.dispose();
    entries.clear();
    active = [];
  }
}
