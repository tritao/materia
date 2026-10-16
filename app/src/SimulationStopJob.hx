package app;

import sys.thread.Mutex;
#if !wasm
import sys.thread.Thread;
#end

/** Joins playback, detaches on the UI owner, then releases native resources on a worker. */
class SimulationStopJob {
  final simulation:ApplicationSimulation;
  final mutex = new Mutex();
  var ready = false;
  var detached = false;
  var failure:Null<String>;

  public function new(simulation:ApplicationSimulation) {
    this.simulation = simulation;
    simulation.beginStop();
    launch(function() simulation.quiesceForStop());
  }

  function launch(action:Void->Void):Void {
    mutex.acquire();
    ready = false;
    mutex.release();
    function work():Void {
      var problem:Null<String> = null;
      try action() catch (error:Dynamic) problem = Std.string(error);
      mutex.acquire();
      if (problem != null) failure = problem;
      ready = true;
      mutex.release();
    }
    #if wasm
    work();
    #else
    Thread.create(work);
    #end
  }

  /** Called only on the scene/RobotWorld owner. Never waits for a worker. */
  public function poll():Bool {
    mutex.acquire();
    var complete = ready, problem = failure;
    mutex.release();
    if (!complete) return false;
    if (problem != null) throw problem;
    if (detached) return true;
    var releases:Array<Void->Void> = [];
    var detachFailure:Null<String> = null;
    try simulation.clearWithReleases(releases) catch (error:Dynamic) detachFailure = Std.string(error);
    detached = true;
    launch(function() {
      var first = detachFailure;
      for (release in releases) {
        try release() catch (error:Dynamic) if (first == null) first = Std.string(error);
      }
      if (first != null) throw first;
    });
    return false;
  }

  /** App close waits before freeing the document or process-wide runtime. */
  public function close():Void {
    while (!poll()) Sys.sleep(0.001);
  }
}
