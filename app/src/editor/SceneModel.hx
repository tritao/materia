package app.editor;

import app.EditorScene.EditorSceneObject;
import app.SceneObjectData;

typedef SceneRecordChange = {
  final id:String;
  final before:Null<SceneObjectData>;
  final after:Null<SceneObjectData>;
  final beforeIndex:Int;
  final afterIndex:Int;
}

/** Authored scene records and structural undo deltas. */
class SceneModel {
  public var objects:Array<EditorSceneObject> = [];
  public var nextObjectId:Int = 1;

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
