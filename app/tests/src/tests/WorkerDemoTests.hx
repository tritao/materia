package tests;

import app.Main.ReferenceEditorApp;
import app.SensorConfiguration;
import humankit.sim.HumanWorkerSignals;

/** The app runs the rack-to-table job with a robot arm in one shared session. */
class WorkerDemoTests {
  public static function main():Int {
    try { run(); Sys.println("Worker demo tests passed"); return 0; }
    catch (error:Dynamic) { Sys.println('Worker demo tests failed: $error'); return 1; }
  }

  static function run():Void {
    var editor = new ReferenceEditorApp();
    editor.enableWorkerDemo();
    var loaded = new SensorConfiguration(editor.sensors.records());
    if (loaded.humans().length != 1 || loaded.humans()[0].jobName != "rack-to-table")
      throw "Configured humans did not survive a sensor record round trip";
    var worker = editor.simulation.humanWorker("worker-demo");
    if (worker == null) throw "App did not register the configured worker";
    var sawRack = false, sawTable = false, leftRack = false;
    var separationCount = 0, separationMin = Math.POSITIVE_INFINITY,
      separationMax = 0.0;
    worker.onTick = function(_, signals:HumanWorkerSignals) {
      if (signals.zones.indexOf("rack") >= 0) sawRack = true;
      if (sawRack && signals.zones.indexOf("rack") < 0) leftRack = true;
      if (signals.zones.indexOf("table") >= 0) sawTable = true;
      var distance = signals.separation.get("demo-arm");
      if (distance == null || !Math.isFinite(distance)) throw "Worker arm separation is unavailable";
      separationCount++;
      separationMin = Math.min(separationMin, distance);
      separationMax = Math.max(separationMax, distance);
    };
    for (tick in 0...1800) {
      editor.simulation.step();
      if (worker.currentJobDone() && tick > 900) break;
    }
    var visual = editor.simulation.environmentVisualState();
    var part = [for (entry in visual) if (entry.id == "worker-demo-part") entry];
    if (part.length != 1) throw "Demo part is missing";
    var p = part[0].position;
    var error = Math.sqrt(Math.pow(p[0] - 3.65, 2) + Math.pow(p[1] - 2.5, 2) +
      Math.pow(p[2] - 1.0, 2));
    if (!worker.currentJobDone() || worker.currentJobFailure() != null || error > 0.01)
      throw 'Worker did not place the part within 1 cm: done=${worker.currentJobDone()} failure=${worker.currentJobFailure()} pose=$p error=$error';
    if (!sawRack || !leftRack || !sawTable || separationCount < 100 ||
        separationMax - separationMin < 0.1)
      throw 'Worker safety stream missed a zone or moving arm: rack=$sawRack left=$leftRack table=$sawTable count=$separationCount range=${separationMax-separationMin}';
    editor.dispose();
  }
}
