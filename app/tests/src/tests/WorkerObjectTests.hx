package tests;

import app.EditorScene;
import app.Main.ReferenceEditorApp;
import app.SceneCodec;
import app.WorkerObjectData;
import haxeon.ui.properties.PropertyBinding;
import haxeon.ui.properties.PropertyType;
import haxeon.ui.properties.PropertyValue;

/** Document worker creation, inspection, persistence and undo. */
class WorkerObjectTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;

  static function property(scene:EditorScene, label:String):haxeon.ui.properties.PropertyDescriptor {
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
      for (field in app.scene.properties())
        if (field.label == "Collision" || field.label == "Mass" || field.label == "Colour")
          throw 'Worker exposes a physics or box colour field: ${field.label}';
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
      app.scene.setWorkerData(workerId, {asset:createdWorker.asset,
        job:'{"version":1,"loop":false,"steps":[{"action":"pick","object":"'+targetId+'","hand":"right"},{"action":"place","onto":"'+targetId+'","hand":"right"}]}',
        zones:[targetId]});
      var hand = property(app.scene,"Hand");
      switch new PropertyBinding(hand, app.scene.context()).apply(PropertyValue.Enum("left")) {
        case Rejected(message): throw 'Paired hand edit rejected: $message';
        case Applied, Unchanged:
      }
      var paired = humankit.job.HumanJobSpec.parse(worker(app.scene,workerId).job);
      check(Reflect.field(paired.steps[0],"hand") == "left" &&
        Reflect.field(paired.steps[1],"hand") == "left", "hand edit updates pick and place together");
      check(app.scene.document.undo() && worker(app.scene,workerId).job.indexOf('"hand":"right"') >= 0,
        "undo restores both paired hands");
      app.scene.setWorkerData(workerId, {asset:createdWorker.asset,
        job:'{"version":1,"loop":false,"steps":[{"action":"pick","object":"'+targetId+'","hand":"left"},{"action":"wait","seconds":1}]}',
        zones:[targetId]});
      app.editor.HumanWorkerKind.selectStep(workerId, 1);
      var action:Null<haxeon.ui.properties.PropertyDescriptor> = null;
      for (field in app.scene.properties())
        if (StringTools.endsWith(field.id, "worker-step-1-action")) action = field;
      if (action == null) throw "Missing second worker step action";
      switch new PropertyBinding(action, app.scene.context()).apply(PropertyValue.Enum("place")) {
        case Rejected(message): throw 'Changing wait to place rejected: $message';
        case Applied, Unchanged:
      }
      var defaulted = humankit.job.HumanJobSpec.parse(worker(app.scene,workerId).job);
      check(Reflect.field(defaulted.steps[1], "hand") == "left",
        "new place defaults to the preceding pick hand");
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
