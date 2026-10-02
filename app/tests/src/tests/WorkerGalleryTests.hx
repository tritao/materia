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

  /**
   * What a frame of the running gallery may cost. A tick is a hundredth of a second, so the gallery keeps up with
   * the clock only while the process spends well under 10 ms of CPU on one, and the application runs up to five ticks
   * between frames it draws. The limit is a little above what it costs now, not a hope: a change that puts per-tick
   * work back (a mesh skinned for every tick, the physics model recompiled or stepped once per body) fails here.
   * CPU time is not wall time, so a busy machine does not fail it.
   */
  static inline var CPU_MS_PER_TICK_LIMIT = 5.0;
  static inline var COST_FRAMES = 40;
  static inline var TICKS_PER_FRAME = 5;

  static function measureCost(editor:ReferenceEditorApp):Void {
    editor.simulation.start();
    editor.simulation.runTicks(TICKS_PER_FRAME);
    var started = Sys.cpuTime();
    for (_ in 0...COST_FRAMES) editor.simulation.runTicks(TICKS_PER_FRAME);
    var perTick = (Sys.cpuTime() - started) * 1000.0 / (COST_FRAMES * TICKS_PER_FRAME);
    Sys.println('worker gallery: ${Math.round(perTick * 100) / 100} ms of CPU per tick (limit $CPU_MS_PER_TICK_LIMIT)');
    if (perTick > CPU_MS_PER_TICK_LIMIT)
      throw 'The gallery costs ${Math.round(perTick * 10) / 10} ms of CPU per tick, over the limit of $CPU_MS_PER_TICK_LIMIT ms';
    editor.simulation.stop();
  }

  static function run():Void {
    var editor = new ReferenceEditorApp();
    editor.enableWorkerDemo(0, false, ReferenceEditorApp.WORKER_GALLERY);
    measureCost(editor);
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
