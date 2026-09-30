package tests;

import app.Main.ReferenceEditorApp;
import humankit.sim.HumanWorkerSignals;

/** Runs the saved worker document in the application's shared SimKit session. */
class WorkerDemoTests {
  public static function main():Int {
    try { run(); Sys.println("Worker document tests passed"); return 0; }
    catch (error:Dynamic) { Sys.println('Worker document tests failed: $error'); return 1; }
  }

  static function run():Void {
    linkBounds();
    motionCodec();
    migration();
    var editor = new ReferenceEditorApp();
    editor.enableWorkerDemo();
    if (editor.sensors.model.links[1].collisionShapes.length != 1)
      throw "Saved arm collision shape did not survive document loading";
    var worker = editor.simulation.humanWorker("worker-demo");
    if (worker == null) throw "Saved document did not create a worker";
    if (editor.session.path != null) throw "Demo should open as an untitled copy";
    var saved = sys.io.File.getContent(app.editor.WorkerAssetPath.resolve(
      "app/examples/worker-rack-to-table.materia"));
    if (saved.indexOf('"human-worker"') < 0 || saved.indexOf('"humans"') >= 0)
      throw "Example does not use document workers exclusively";
    var example:Dynamic = haxe.Json.parse(saved);
    var motion:Array<Dynamic> = Reflect.field(example, "robotMotions");
    if (motion == null || motion.length != 1)
      throw "Example has no document-owned robot motion";
    var authored = [for (record in editor.scene.records()) if (record.id == "worker-demo") record][0];
    if (authored.worker == null || authored.worker.job.indexOf('"offset":[0.1,-0.05]') < 0)
      throw "Acceptance example does not exercise a placement offset";
    var sawRack = false, sawTable = false, leftRack = false, separationCount = 0;
    var inRackAtEnd = false, checkedJump = false;
    var minSeparation = Math.POSITIVE_INFINITY, maxSeparation = 0.0;
    var firstLink:Null<Array<Float>> = null, linkTravel = 0.0;
    var previousPart:Null<Array<Float>> = null;
    var sawHeld = false, sawRelease = false, releaseDistance = Math.POSITIVE_INFINITY;
    var partObject = editor.simulation.environmentObject("worker-demo-part");
    if (partObject == null) throw "Part has no simulation body";
    if (partObject.motion != nativekit.sim.MotionType.Dynamic)
      throw "Placed part is not dynamic";
    var noCarrier = editor.simulation.activeSession().objectCarrier(partObject);
    worker.onTick = function(_, signals:HumanWorkerSignals) {
      if (signals.zones.indexOf("worker-demo-rack") >= 0) sawRack = true;
      if (sawRack && signals.zones.indexOf("worker-demo-rack") < 0) leftRack = true;
      inRackAtEnd = signals.zones.indexOf("worker-demo-rack") >= 0;
      if (signals.zones.indexOf("worker-demo-table") >= 0) sawTable = true;
      var distance = signals.separation.get(editor.sensors.robotId);
      if (distance == null || !Math.isFinite(distance)) throw "Robot separation is unavailable";
      separationCount++;
      minSeparation = Math.min(minSeparation, distance);
      maxSeparation = Math.max(maxSeparation, distance);
    };
    for (tick in 0...1800) {
      editor.simulation.step();
      var carrier = editor.simulation.activeSession().objectCarrier(partObject);
      if (carrier != noCarrier) sawHeld = true;
      if (sawHeld && carrier == noCarrier && !sawRelease) {
        sawRelease = true;
        var root = worker.body.rootTransform();
        var partAtRelease = [for (item in editor.simulation.environmentVisualState())
          if (item.id == "worker-demo-part") item.position][0];
        releaseDistance = Math.sqrt(Math.pow(root[12] - partAtRelease[0], 2) +
          Math.pow(root[13] - partAtRelease[1], 2));
      }
      var link = editor.simulation.visualState()[0].links[1].rotation;
      if (firstLink == null) firstLink = link.copy();
      linkTravel = Math.max(linkTravel, Math.sqrt(Math.pow(link[0]-firstLink[0],2) +
        Math.pow(link[1]-firstLink[1],2) + Math.pow(link[2]-firstLink[2],2) +
        Math.pow(link[3]-firstLink[3],2)));
      var part = [for (item in editor.simulation.environmentVisualState())
        if (item.id == "worker-demo-part") item.position];
      if (part.length != 1) throw "Part is missing during simulation";
      var current = part[0];
      if (previousPart != null && current[1] > 1.2) {
        checkedJump = true;
        var jump = distance(current, previousPart);
        if (jump > 0.15) throw 'Part jumped $jump metres near the table at tick $tick';
      }
      previousPart = current;
      if (worker.currentJobDone() && tick > 900) break;
    }
    var restingSpeed = Math.POSITIVE_INFINITY, settled:Null<Array<Float>> = null;
    for (tick in 0...450) {
      editor.simulation.step();
      var now = [for (item in editor.simulation.environmentVisualState())
        if (item.id == "worker-demo-part") item.position][0];
      if (settled != null) restingSpeed = distance(now, settled) / editor.simulation.timestep;
      settled = now;
      if (tick >= 90 && restingSpeed < 0.001) break;
    }
    var pose = [for (item in editor.simulation.environmentVisualState())
      if (item.id == "worker-demo-part") item][0];
    var table = [for (item in editor.scene.records()) if (item.id == "worker-demo-table") item][0];
    var part = [for (item in editor.scene.records()) if (item.id == "worker-demo-part") item][0];
    var horizontal = Math.sqrt(Math.pow(pose.position[0] - (table.x + 0.1), 2) +
      Math.pow(pose.position[1] - (table.y - 0.05), 2));
    var vertical = Math.abs(pose.position[2] - (table.z + table.depth / 2 + part.depth / 2));
    var tilt = Math.sqrt(pose.rotation[0] * pose.rotation[0] + pose.rotation[1] * pose.rotation[1]);
    if (!sawRelease || releaseDistance >= 1.0 || !worker.currentJobDone() ||
        worker.currentJobFailure() != null ||
        horizontal > 0.02 || vertical > 0.01 || restingSpeed > 0.001 || tilt > 0.02)
      throw 'Worker missed target: failure=${worker.currentJobFailure()} pose=${pose.position} rotation=${pose.rotation} horizontal=$horizontal vertical=$vertical speed=$restingSpeed tilt=$tilt';
    if (!sawRack || !leftRack || inRackAtEnd || !sawTable || !checkedJump || separationCount < 100 ||
        !Math.isFinite(minSeparation) || linkTravel < 0.1 || maxSeparation-minSeparation < 0.1)
      throw 'Worker safety stream is incomplete: rack=$sawRack left=$leftRack table=$sawTable count=$separationCount linkTravel=$linkTravel range=${maxSeparation-minSeparation}';
    Sys.println('worker document placement: horizontal=$horizontal vertical=$vertical speed=$restingSpeed');
    stalledRealtime();
    var roundTrip = "build/worker-demo-motion-roundtrip.materia";
    editor.session.save(roundTrip);
    var savedMotion:Array<Dynamic> = Reflect.field(haxe.Json.parse(sys.io.File.getContent(roundTrip)),
      "robotMotions");
    if (savedMotion == null || savedMotion.length != 1)
      throw "Saving the document lost its robot motion";
    var scene = editor.scene.runtimeContentScene();
    var snapshot = scene.snapshot();
    var nodeCount = snapshot.nodeCount();
    snapshot.dispose();
    var bad = [for (item in editor.scene.records()) if (item.id == "worker-demo") item][0];
    var original = bad.worker;
    if (original == null) throw "Worker data missing";
    editor.scene.setWorkerData("worker-demo", {asset:original.asset,job:"{broken",zones:original.zones});
    for (_ in 0...2) {
      if (!editor.simulation.rebuild(editor.sensors, editor.scene, editor.session))
        throw "Malformed job prevented a recoverable simulation rebuild";
      if (editor.simulation.humanWorker("worker-demo").currentJobFailure() == null)
        throw "Malformed job has no failure signal";
      snapshot = scene.snapshot();
      var after = snapshot.nodeCount();
      snapshot.dispose();
      if (after != nodeCount) throw 'Rebuild leaked character nodes: $nodeCount -> $after';
    }
    editor.scene.setWorkerData("worker-demo", {asset:original.asset,
      job:original.job,zones:["missing-zone"]});
    for (_ in 0...2) {
      if (!editor.simulation.rebuild(editor.sensors, editor.scene, editor.session))
        throw "Unknown worker zone prevented simulation rebuild";
      if (editor.simulation.humanWorker("worker-demo") == null)
        throw "Worker was lost with its missing zone";
      if (editor.simulation.collisionWarnings.join(" ").indexOf("missing-zone") < 0)
        throw "Missing zone has no warning";
      if (editor.simulation.humanWarnings("worker-demo").join(" ").indexOf("missing-zone") < 0)
        throw "Missing zone warning is not attached to its worker";
      snapshot = scene.snapshot();
      var after = snapshot.nodeCount();
      snapshot.dispose();
      if (after != nodeCount) throw 'Rebuild leaked character nodes: $nodeCount -> $after';
    }
    editor.scene.select("worker-demo");
    var missingZoneField:Null<nativekit.ui.properties.PropertyDescriptor> = null;
    for (field in editor.scene.properties()) if (field.label == "Missing zone: missing-zone")
      missingZoneField = field;
    if (missingZoneField == null) throw "Inspector cannot remove a dangling zone";
    switch new nativekit.ui.properties.PropertyBinding(missingZoneField, editor.scene.context())
      .apply(nativekit.ui.properties.PropertyValue.Bool(false)) {
      case Rejected(message): throw 'Removing dangling zone failed: $message';
      case Applied, Unchanged:
    }
    var cleaned = editor.scene.object("worker-demo");
    var cleanedWorker = cleaned == null ? null : cleaned.worker;
    if (cleanedWorker == null || cleanedWorker.zones.indexOf("missing-zone") >= 0)
      throw "Dangling zone remained selected";
    try editor.scene.setWorkerData("worker-demo", {asset:"animkit/assets/props/wrench.glb",
      job:original.job,zones:original.zones})
    catch (error:Dynamic) throw 'Bad asset edit failed: $error';
    var failedPreview = scene.snapshot();
    var failedPreviewNodes = failedPreview.nodeCount();
    failedPreview.dispose();
    for (_ in 0...3) {
      try editor.scene.syncWorkerVisuals() catch (error:Dynamic)
        throw 'Bad asset preview refresh failed: $error';
      if (editor.simulation.rebuild(editor.sensors, editor.scene, editor.session))
        throw "Non-humanoid character unexpectedly rebuilt";
      snapshot = scene.snapshot();
      var after = snapshot.nodeCount();
      snapshot.dispose();
      if (after != failedPreviewNodes) throw 'Bad asset leaked scene nodes: $failedPreviewNodes -> $after';
    }
    editor.dispose();
  }

  static function migration():Void {
    var editor = new ReferenceEditorApp();
    editor.session.open("fixtures/worker-legacy.materia");
    var worker = [for (item in editor.scene.records()) if (item.id == "legacy-worker") item];
    if (worker.length != 1) throw "Legacy human was not migrated";
    var data = worker[0].worker;
    if (data == null || data.migrationNote == null ||
        data.migrationNote.indexOf("rack-to-table") < 0)
      throw "Legacy human did not migrate with its job name";
    var destination = "build/legacy-worker-migrated.materia";
    editor.session.save(destination);
    var saved = sys.io.File.getContent(destination);
    if (saved.indexOf('"humans"') >= 0 || saved.indexOf('"human-worker"') < 0)
      throw "Migrated document saved in the old format";
    editor.dispose();
    var source:Dynamic = haxe.Json.parse(sys.io.File.getContent("fixtures/worker-legacy.materia"));
    var legacy:Array<Dynamic> = Reflect.field(Reflect.field(source, "sensors"), "humans");
    var halfRoll = 0.1, halfYaw = 0.2;
    Reflect.setField(legacy[0], "rotation", [Math.sin(halfRoll) * Math.cos(halfYaw),
      Math.sin(halfRoll) * Math.sin(halfYaw), Math.cos(halfRoll) * Math.sin(halfYaw),
      Math.cos(halfRoll) * Math.cos(halfYaw)]);
    var objects:Array<Dynamic> = Reflect.field(source,"objects");
    var collision:Dynamic = haxe.Json.parse(haxe.Json.stringify(objects[0]));
    Reflect.setField(collision,"id","legacy-worker");
    objects.push(collision);
    var clashPath = "build/legacy-worker-id-clash.materia";
    sys.io.File.saveContent(clashPath,haxe.Json.stringify(source));
    var clash = new ReferenceEditorApp();
    clash.session.open(clashPath);
    var migrated = [for (item in clash.scene.records()) if (item.type == "human-worker") item];
    var migratedData = migrated.length == 1 ? migrated[0].worker : null;
    if (migrated.length != 1 || migrated[0].id == "legacy-worker" ||
        migratedData == null || migratedData.migrationNote == null)
      throw "Legacy human ID clash did not migrate with a note";
    var rotation = migrated[0].rotation;
    if (rotation == null || Math.abs(rotation[0]) > 1e-9 || Math.abs(rotation[1]) > 1e-9 ||
        Math.abs(2 * Math.atan2(rotation[2], rotation[3]) - 0.4) > 1e-6)
      throw "Migrated worker retained legacy roll or lost yaw";
    clash.dispose();
  }

  /**
   * The realtime session is pumped from the editor's loop, so frames that stall
   * for many ticks must still deliver the part: the worker is fed per tick on
   * the simulation clock, not per frame.
   */
  static function stalledRealtime():Void {
    var editor = new ReferenceEditorApp();
    editor.enableWorkerDemo(0, true);
    var worker = editor.simulation.humanWorker("worker-demo");
    if (worker == null) throw "Realtime demo created no worker";
    var deadline = Sys.time() + 90.0;
    var frame = 0;
    while (!worker.currentJobDone() && Sys.time() < deadline) {
      editor.tick();
      // Three quick frames, then one that stalls for about four ticks.
      Sys.sleep(frame++ % 4 == 3 ? 0.045 : 0.004);
    }
    if (!worker.currentJobDone() || worker.currentJobFailure() != null)
      throw 'Stalled realtime job did not finish: failure=${worker.currentJobFailure()}';
    var settleUntil = Sys.time() + 4.0;
    while (Sys.time() < settleUntil) { editor.tick(); Sys.sleep(0.004); }
    var pose = [for (item in editor.simulation.environmentVisualState())
      if (item.id == "worker-demo-part") item][0];
    var table = [for (item in editor.scene.records()) if (item.id == "worker-demo-table") item][0];
    var part = [for (item in editor.scene.records()) if (item.id == "worker-demo-part") item][0];
    var horizontal = Math.sqrt(Math.pow(pose.position[0] - (table.x + 0.1), 2) +
      Math.pow(pose.position[1] - (table.y - 0.05), 2));
    var vertical = Math.abs(pose.position[2] - (table.z + table.depth / 2 + part.depth / 2));
    if (horizontal > 0.02 || vertical > 0.01)
      throw 'Stalled realtime run missed the table: horizontal=$horizontal vertical=$vertical';
    Sys.println('stalled realtime placement: horizontal=$horizontal vertical=$vertical');
    editor.dispose();
  }

  static function distance(a:Array<Float>, b:Array<Float>):Float
    return Math.sqrt(Math.pow(a[0]-b[0],2)+Math.pow(a[1]-b[1],2)+Math.pow(a[2]-b[2],2));

  static function motionCodec():Void {
    var tracks = app.RobotMotionTrack.decode([{version:1, robotId:"arm", joint:0, loop:true,
      keys:[{time:0.0, position:0.0}, {time:1.0, position:0.6}, {time:2.0, position:0.0}]}]);
    if (tracks.length != 1 || Math.abs(tracks[0].sample(0.5) - 0.3) > 1e-9 ||
        Math.abs(tracks[0].sample(2.5) - 0.3) > 1e-9 ||
        app.RobotMotionTrack.decode([tracks[0].record()]).length != 1)
      throw "Robot motion interpolation or codec failed";
    var rejected = false;
    try app.RobotMotionTrack.decode([{version:1, robotId:"arm", joint:0, loop:true,
      keys:[{time:0.0, position:0.0}, {time:0.0, position:1.0}]}])
    catch (_:Dynamic) rejected = true;
    if (!rejected) throw "Robot motion accepted non-increasing key times";
  }

  static function linkBounds():Void {
    var quarter = Math.sqrt(0.5);
    var center = app.editor.RobotLinkBounds.worldCenter([1.0,2.0,1.3],
      [0.0,0.0,quarter,quarter],[0.45,0.0,0.0]);
    if (Math.abs(center.x-1.0)>0.000000001 || Math.abs(center.y-2.45)>0.000000001)
      throw "Link bound centre did not follow its rotation";
  }
}
