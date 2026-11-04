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

  public function new(
    source:ApplicationSimulation,
    configuration:SensorConfiguration,
    scene:EditorScene,
    session:ProjectDocumentSession
  ) {
    var self = this;
    // Collect editable models on the owner thread: robotModels updates the configuration's
    // current selection and caches while resolving the complete set of robots.
    var models:Null<Array<{id:String, model:robotkit.model.RobotModel, profile:robotkit.profile.RobotProfile,
      position:Array<Float>, rotation:Array<Float>}>> = null;
    var captureFailure:Null<String> = null;
    try models = configuration.robotModels() catch (error:Dynamic) captureFailure = Std.string(error);
    function prepare():Void {
      var target = source.preparationTarget();
      var problem:Null<String> = null;
      try {
        if (captureFailure != null) throw captureFailure;
        if (!target.rebuild(configuration, scene, session, false, self.control, models)) throw target.error;
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
    if (control.isCancelled()) {
      target.dispose();
      throw ProjectLoadControl.CANCELLED;
    }
    return target;
  }

  /** Closing waits for the builder before releasing the document it reads. */
  public function close():Void {
    control.cancel();
    while (!isFinished()) Sys.sleep(0.001);
    try take().dispose() catch (_:Dynamic) {
    }
  }
}
