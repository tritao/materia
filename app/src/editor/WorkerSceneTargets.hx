package app.editor;

import app.SceneObjectData;
import app.EditorScene;
import humankit.job.HumanJobTargets;
import humankit.HumanTargetBox;

/** Resolves document object IDs to the boxes used by HumanKit jobs. */
class WorkerSceneTargets implements HumanJobTargets {
  final records:Array<SceneObjectData>;
  final scene:Null<EditorScene>;

  public function new(records:Array<SceneObjectData>, ?scene:EditorScene) {
    this.records = records;
    this.scene = scene;
  }

  public function box(objectId:String):Null<HumanTargetBox> {
    for (record in records) if (record.id == objectId) {
      var yaw = 0.0;
      var q = record.rotation;
      if (q != null) yaw = Math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
        1 - 2 * (q[1] * q[1] + q[2] * q[2]));
      var center = [record.x,record.y,record.z];
      var half = [record.width*0.5,record.height*0.5,record.depth*0.5];
      if (scene != null && scene.isCadPart(record.id)) {
        var session = scene.cadSession(record.id);
        var bounds = session == null ? null : session.collisionBounds;
        if (bounds != null) {
          var x = bounds.center.x, y = bounds.center.y, z = bounds.center.z;
          var q = record.rotation;
          if (q != null) {
            var tx = 2 * (q[1]*z-q[2]*y), ty = 2 * (q[2]*x-q[0]*z), tz = 2 * (q[0]*y-q[1]*x);
            x += q[3]*tx+q[1]*tz-q[2]*ty;
            y += q[3]*ty+q[2]*tx-q[0]*tz;
            z += q[3]*tz+q[0]*ty-q[1]*tx;
          }
          center = [record.x+x,record.y+y,record.z+z];
          half = [bounds.halfExtents.x,bounds.halfExtents.y,bounds.halfExtents.z];
        }
      }
      return {center:center, halfExtents:half,yaw:yaw};
    }
    return null;
  }
}
