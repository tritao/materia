package tests;

import app.Main.ReferenceEditorApp;

/**
 * The worker gallery: every case the worker is built for, one lane each, run to the end in the application's shared
 * simulation. Each lane's job must finish without a failure and leave its part on its table, at the height of the
 * table's top: that a worker bent over, crouched, knelt, turned round or used both hands is judged by the humankit
 * sweeps; here it is that the saved document, with its characters, heights and jobs, still does it.
 */
class WorkerGalleryTests {
  static inline var LANES = 6;
  static inline var LIMIT = 2400;

  public static function main():Int {
    try { run(); Sys.println("Worker gallery tests passed"); return 0; }
    catch (error:Dynamic) { Sys.println('Worker gallery tests failed: $error'); return 1; }
  }

  static function run():Void {
    var editor = new ReferenceEditorApp();
    editor.enableWorkerDemo(0, false, ReferenceEditorApp.WORKER_GALLERY);
    var ids = editor.simulation.humanWorkerIds();
    if (ids.length != LANES) throw 'The gallery has ${ids.length} workers, not $LANES';
    var tick = 0;
    while (tick++ < LIMIT) {
      editor.simulation.step();
      var all = true;
      for (id in ids) {
        var worker = editor.simulation.humanWorker(id);
        if (worker == null) throw 'Worker $id was not created';
        if (worker.currentJobFailure() != null) throw '$id failed: ${worker.currentJobFailure()}';
        if (!worker.currentJobDone()) all = false;
      }
      if (all) break;
    }
    for (id in ids) if (!editor.simulation.humanWorker(id).currentJobDone()) throw '$id had not finished after $LIMIT ticks';
    var state = editor.simulation.environmentVisualState();
    var find = function(wanted:String):Array<Float> {
      for (entry in state) if (entry.id == wanted) return entry.position;
      throw 'The gallery has no object $wanted';
    };
    for (lane in 0...LANES) {
      var part = find('gallery-$lane-part'), table = find('gallery-$lane-table');
      // The table's centre is half its thickness (0.05 m) under its top, and the part's half height (0.04 m) above it.
      var across = Math.sqrt(Math.pow(part[0] - table[0], 2) + Math.pow(part[1] - table[1], 2));
      if (across > 0.15) throw 'Lane $lane: the part came to rest ${Math.round(across * 100) / 100} m from the middle of its table';
      if (Math.abs(part[2] - (table[2] + 0.09)) > 0.04)
        throw 'Lane $lane: the part is at height ${part[2]}, not resting on a table whose centre is at ${table[2]}';
    }
  }
}
