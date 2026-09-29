package app.editor;

import app.SceneObjectData;
import humankit.HumanJobTargets;
import humankit.HumanTargetBox;

/** Resolves document object IDs to the boxes used by HumanKit jobs. */
class WorkerSceneTargets implements HumanJobTargets {
  final records:Array<SceneObjectData>;

  public function new(records:Array<SceneObjectData>) this.records = records;

  public function box(objectId:String):Null<HumanTargetBox> {
    for (record in records) if (record.id == objectId) {
      var yaw = 0.0;
      var q = record.rotation;
      if (q != null) yaw = Math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
        1 - 2 * (q[1] * q[1] + q[2] * q[2]));
      return {center:[record.x,record.y,record.z],
        halfExtents:[record.width*0.5,record.height*0.5,record.depth*0.5],yaw:yaw};
    }
    return null;
  }
}
