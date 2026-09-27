package app.editor;

import app.EditorScene.EditorSceneObject;
import app.SceneObjectData;
import app.EditorScene;
import nativekit.ui.editing.EditOperation;
import app.editor.ObjectKindRegistry;

typedef SceneRecordChange = {
  final id:String;
  final before:Null<SceneObjectData>;
  final after:Null<SceneObjectData>;
  final beforeIndex:Int;
  final afterIndex:Int;
}

/** Authored scene records and structural undo deltas. */
@:access(app.EditorScene)
class SceneModel {
  public var objects:Array<EditorSceneObject> = [];
  public var nextObjectId:Int = 1;

  public function allocateId(owner:EditorScene, prefix:String="rectangle"):String {
    var id = prefix + "-" + nextObjectId;
    nextObjectId++;
    while (owner.object(id) != null) {
      id = prefix + "-" + nextObjectId;
      nextObjectId++;
    }
    return id;
  }

  public function createDefault(owner:EditorScene, kind:String, prefix:String, label:String):Bool {
    if (!owner.canCreate()) return false;
    var data = records();
    var id = allocateId(owner, prefix);
    data.push(ObjectKindRegistry.require(kind).createDefaultRecord(id));
    return changeObjects(owner, label, data, id);
  }

  public function duplicateSelected(owner:EditorScene):Bool {
    if (!owner.canCreate() || owner.object(owner.selectedId) == null) return false;
    var data = records();
    var source:SceneObjectData = null;
    for (item in data) if (item.id == owner.selectedId) source = item;
    var id = allocateId(owner);
    var cadGraph = EditorScene.isCadKind(source.type) ? owner.currentCadGraph(source.id) : source.cadGraph;
    data.push({id: id, label: source.label + " copy", type: source.type,
      x: Math.min(1000000, source.x + 0.25), y: Math.min(1000000, source.y + 0.25), z: source.z,
      width: source.width, height: source.height, red: source.red, green: source.green,
      blue: source.blue, appearance: source.appearance, visible: source.visible,depth:source.depth,
      collisionEnabled:source.collisionEnabled,dynamicBody:source.dynamicBody,mass:source.mass,
      cadGraph:cadGraph,meshSnapshot:source.meshSnapshot,rotation:source.rotation});
    return changeObjects(owner, "Duplicate object", data, id);
  }

  public function deleteSelected(owner:EditorScene):Bool {
    if (owner.object(owner.selectedId) == null) return false;
    var data = records();
    var index = 0;
    while (data[index].id != owner.selectedId) index++;
    data.splice(index, 1);
    var next = data.length == 0 ? "scene" : data[index < data.length ? index : data.length - 1].id;
    return changeObjects(owner, "Delete object", data, next);
  }

  public function changeObjects(owner:EditorScene, label:String, after:Array<SceneObjectData>, selection:String):Bool {
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
        var previous = SceneModel.recordForObject(item);
        if (EditorScene.isCadKind(previous.type)) previous.cadGraph = owner.currentCadGraph(previous.id);
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
    var previousSelection = owner.selectedId;
    var initialAfter:Null<Array<SceneObjectData>> = after;
    return owner.document.apply(new EditOperation(label,
      function() {
        var initial = initialAfter;
        initialAfter = null;
        if (initial != null) owner.replaceObjects(initial, selection);
        else applyObjectChanges(owner, changes, true, selection);
      },
      function() applyObjectChanges(owner, changes, false, previousSelection),
      null, null, null, SceneModel.estimateSceneChanges(changes)));
  }

  public function applyObjectChanges(owner:EditorScene, changes:Array<SceneRecordChange>, forward:Bool, selection:String):Void {
    owner.replaceObjects(SceneModel.applyChanges(records(), changes, forward), selection);
  }

  public function records():Array<SceneObjectData> {
    var result:Array<SceneObjectData> = [];
    for (item in objects) {
      result.push({id: item.id, label: item.label, type: item.kind,
        x: item.x, y: item.y, z: item.z,
        width:item.width,height:item.height,depth:item.depth,collisionEnabled:item.collisionEnabled,
        dynamicBody:item.dynamicBody,mass:item.mass,red:item.red,green:item.green,blue:item.blue,
        appearance:item.appearance,
        visible: item.visible,cadGraph:item.cadGraph,meshSnapshot:item.meshSnapshot,
        rotation:item.rotation});
    }
    return result;
  }

  public static function applyChanges(data:Array<SceneObjectData>, changes:Array<SceneRecordChange>, forward:Bool):Array<SceneObjectData> {
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
    return data;
  }

  static function findRecordIndex(data:Array<SceneObjectData>, id:String):Int {
    for (index in 0...data.length) if (data[index].id == id) return index;
    return -1;
  }

  public static function recordForObject(item:EditorSceneObject):SceneObjectData {
    return {id: item.id, label: item.label, type: item.kind,
      x: item.x, y: item.y, z: item.z,
      width: item.width, height: item.height, depth: item.depth,
      collisionEnabled: item.collisionEnabled, dynamicBody: item.dynamicBody, mass: item.mass,
      red: item.red, green: item.green, blue: item.blue, appearance: item.appearance,
      visible: item.visible, cadGraph: item.cadGraph,
      meshSnapshot: item.meshSnapshot, rotation: item.rotation};
  }

  public static function estimateSceneChanges(changes:Array<SceneRecordChange>):Int {
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

}
