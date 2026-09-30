package app;

import app.MateriaProjectRunner.GeneratedAssemblyScene;
import sys.thread.Mutex;
#if !wasm
import sys.thread.Thread;
#end

/** Builds and decodes a Materia project on a worker thread so the editor stays responsive meanwhile. */
class ProjectLoadJob {
  public final control:ProjectLoadControl = new ProjectLoadControl();
  public final startedAt:Float;
  final mutex:Mutex = new Mutex();
  var finished:Bool = false;
  var result:Null<GeneratedAssemblyScene> = null;
  var failure:Null<String> = null;

  public function new(projectPath:String) {
    startedAt = Sys.time();
    var self = this;
    #if wasm
    // The browser build has no worker threads yet, so the load runs before the first frame after it.
    try result = MateriaProjectRunner.loadProject(projectPath, null, control)
    catch (error:Dynamic) failure = Std.string(error);
    finished = true;
    #else
    Thread.create(function() {
      var loaded:Null<GeneratedAssemblyScene> = null;
      var problem:Null<String> = null;
      try loaded = MateriaProjectRunner.loadProject(projectPath, null, self.control)
      catch (error:Dynamic) problem = Std.string(error);
      self.mutex.acquire();
      self.result = loaded;
      self.failure = problem;
      self.finished = true;
      self.mutex.release();
    });
    #end
  }

  public function isFinished():Bool {
    mutex.acquire();
    var value = finished;
    mutex.release();
    return value;
  }

  public function wasCancelled():Bool return failure == ProjectLoadControl.CANCELLED;

  public function elapsedSeconds():Float return Sys.time() - startedAt;

  /** The loaded scene, or throws the worker's failure. Call only after isFinished(). */
  public function take():GeneratedAssemblyScene {
    mutex.acquire();
    var loaded = result;
    var problem = failure;
    mutex.release();
    if (problem != null) throw problem;
    if (loaded == null) throw "Project load finished without a result";
    return loaded;
  }
}
