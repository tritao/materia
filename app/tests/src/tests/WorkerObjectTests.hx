package tests;

import app.EditorScene;
import app.Main.ReferenceEditorApp;
import app.SceneCodec;
import app.WorkerObjectData;
import nativekit.ui.properties.PropertyBinding;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;

/** Document worker creation, inspection, persistence and undo. */
class WorkerObjectTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;

  static function property(scene:EditorScene, label:String):nativekit.ui.properties.PropertyDescriptor {
    for (descriptor in scene.properties()) if (descriptor.label == label) return descriptor;
    throw 'Missing worker property "$label"';
  }

  static function worker(scene:EditorScene, id:String):WorkerObjectData {
    var item = scene.object(id);
    if (item == null || item.worker == null) throw 'Missing worker "$id"';
    return item.worker;
  }

  public static function main():Int {
    try {
      var app = new ReferenceEditorApp();
      check(app.scene.createRectangle(), "create a target object");
      var targetId = app.scene.selectedId;
      check(app.commands.execute("scene.create-worker"), "create worker through command");
      var workerId = app.scene.selectedId;
      var created = app.scene.object(workerId);
      if (created == null || created.worker == null) throw "worker scene object is missing authored data";
      var createdWorker = created.worker;
      check(created.kind == "human-worker", "worker has the new scene object kind");
      check(created.z == 0.0 && created.depth > 1.5, "worker stands at floor origin with measured height");
      check(app.scene.hasWorkerVisual(workerId), "worker has an idle character preview");
      var job = '{"version":1,"loop":false,"steps":[{"action":"wait","seconds":1.0},{"action":"pick","object":"'+targetId+'"}]}';
      app.scene.setWorkerData(workerId, {asset:createdWorker.asset, job:job, zones:[targetId]});
      var seconds = property(app.scene,"Seconds");
      switch new PropertyBinding(seconds, app.scene.context()).apply(PropertyValue.Float(2.0)) {
        case Rejected(message): throw 'Step edit rejected: $message';
        case Applied, Unchanged:
      }
      check(worker(app.scene,workerId).job.indexOf('"seconds":2') >= 0,
        "step edit updates worker job");
      check(app.scene.document.undo() && worker(app.scene,workerId).job.indexOf('"seconds":1') >= 0,
        "undo restores the prior step");
      var choice = property(app.scene,"Pick object");
      check(choice.type == PropertyType.Enum && choice.option(targetId) != null,
        "inspector lists scene object IDs in the pick dropdown");
      var yaw = property(app.scene,"Yaw");
      switch new PropertyBinding(yaw, app.scene.context()).apply(PropertyValue.Float(90.0)) {
        case Rejected(message): throw 'Yaw edit rejected: $message';
        case Applied, Unchanged:
      }
      var turned = app.scene.object(workerId);
      if (turned == null) throw "Rotated worker disappeared";
      var rotation = turned.rotation;
      check(rotation != null && Math.abs(rotation[2] - Math.sin(Math.PI / 4)) < 0.001 && turned.z == 0.0,
        "yaw edit rotates around Z and keeps worker on the floor");
      check(app.scene.document.undo(), "yaw edit can be undone");
      var encoded = SceneCodec.encode(app.scene);
      var reopened = new EditorScene(SceneCodec.decode(encoded));
      var loaded = reopened.object(workerId);
      if (loaded == null || loaded.worker == null) throw "reopened worker is missing";
      var loadedWorker = loaded.worker;
      check(loadedWorker.zones[0] == targetId && loadedWorker.job.indexOf('"action":"pick"') >= 0,
        "worker survives document round trip");
      check(reopened.hasWorkerVisual(workerId), "reopened worker has an idle character preview");
      reopened.dispose();
      app.scene.setWorkerData(workerId, {asset:createdWorker.asset, job:"broken", zones:[]});
      var malformed = new EditorScene(SceneCodec.decode(SceneCodec.encode(app.scene)));
      malformed.select(workerId);
      var foundError = false;
      for (descriptor in malformed.properties()) if (descriptor.label == "Job error") foundError = true;
      check(foundError, "malformed job loads with an inspector error");
      malformed.dispose();
      app.dispose();
      Sys.println("Worker object tests passed");
      return 0;
    } catch (error:Dynamic) {
      Sys.println('Worker object tests failed: $error');
      return 1;
    }
  }
}
