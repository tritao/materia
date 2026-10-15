package app;

import sys.thread.Mutex;
#if !wasm
import sys.thread.Thread;
#end

/** Prepares an isolated simulation while the editor continues rendering and handling input. */
class SimulationBuildJob {
  public final control:ProjectLoadControl = new ProjectLoadControl();
  public final startedAt:Float = Sys.time();
  final mutex:Mutex = new Mutex();
  var finished = false;
  var result:Null<ApplicationSimulation>;
  var failure:Null<String>;

  public function new(source:ApplicationSimulation, configuration:SensorConfiguration,
      scene:EditorScene, session:ProjectDocumentSession) {
    var self = this;
    function prepare():Void {
      var target = source.preparationTarget();
      var problem:Null<String> = null;
      try {
        if (!target.rebuild(configuration, scene, session, false, self.control)) throw target.error;
        self.control.throwIfCancelled();
        target.detachPreparedRobots();
      } catch (error:Dynamic) {
        problem = Std.string(error);
        target.dispose();
      }
      self.mutex.acquire();
      self.result = problem == null ? target : null;
      self.failure = problem;
      self.finished = true;
      self.mutex.release();
    }
    #if wasm
    prepare();
    #else
    Thread.create(prepare);
    #end
  }

  public function isFinished():Bool {
    mutex.acquire();
    var value = finished;
    mutex.release();
    return value;
  }

  public function take():ApplicationSimulation {
    mutex.acquire();
    var target = result, problem = failure;
    result = null;
    mutex.release();
    if (problem != null) throw problem;
    if (target == null) throw "Simulation preparation produced no result";
    if (control.isCancelled()) { target.dispose(); throw ProjectLoadControl.CANCELLED; }
    return target;
  }

  /** Closing waits for the builder before releasing the document it reads. */
  public function close():Void {
    control.cancel();
    while (!isFinished()) Sys.sleep(0.001);
    try take().dispose() catch (_:Dynamic) {}
  }
}
