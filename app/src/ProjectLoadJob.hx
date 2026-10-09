package app;

import app.GeneratedAssemblyScene;
import sys.thread.Mutex;
#if !wasm
import sys.thread.Thread;
#end

/** Builds and decodes a Materia project on a worker thread so the editor stays responsive meanwhile. */
class ProjectLoadJob {
  public final control:ProjectLoadControl;
  public final startedAt:Float;
  final mutex:Mutex = new Mutex();
  var finished:Bool = false;
  var result:Null<GeneratedAssemblyScene> = null;
  var failure:Null<String> = null;

  #if wasm
  var workerId:Int = 0;
  var artifactBytes:Null<haxe.io.Bytes>;
  var workerPhase:String = "";
  #end

  public function new(projectPath:String, ?jobId:String, ?loadControl:ProjectLoadControl) {
    control = loadControl == null ? new ProjectLoadControl() : loadControl;
    startedAt = Sys.time();
    var self = this;
    #if wasm
    try {
      if (jobId != null) throw "Browser loading cannot select a source project job";
      if (!ProjectSourceLoader.isPrebuilt(projectPath)) throw "Browser loading requires a prebuilt .mtrg artifact";
      var metadata = sys.FileSystem.metadata(projectPath);
      if (metadata == null || sys.FileSystem.isDirectory(projectPath) || metadata.size > 150000000)
        throw "Project artifact is missing or too large";
      control.phase("Starting preparation worker");
      var bytes = sys.io.File.getBytes(projectPath);
      artifactBytes = bytes;
      var cachedScene = new ProjectArtifactLoader(PreparedSceneCaches.current()).tryLoadCached(bytes, control);
      if (cachedScene != null) {
        result = cachedScene;
        artifactBytes = null;
        finished = true;
        return;
      }
      control.phase("Starting preparation worker");
      workerId = MateriaWebFiles.materia_worker_start(bytes);
      if (workerId <= 0) throw "Could not start preparation worker";
      control.attachCancellation(function() {
        MateriaWebFiles.materia_worker_cancel(self.workerId);
        self.artifactBytes = null;
        self.failure = ProjectLoadControl.CANCELLED;
        self.control.detach();
        self.finished = true;
      });
    } catch (error:Dynamic) { failure = Std.string(error); finished = true; }
    #else
    Thread.create(function() {
      var loaded:Null<GeneratedAssemblyScene> = null;
      var problem:Null<String> = null;
      try loaded = ProjectSourceLoader.load(projectPath, null, self.control, jobId)
      catch (error:Dynamic) problem = Std.string(error);
      self.control.phase("Waiting for the editor");
      self.mutex.acquire();
      self.result = loaded;
      self.failure = problem;
      self.finished = true;
      self.mutex.release();
    });
    #end
  }

  public function isFinished():Bool {
    #if wasm
    pollWorker();
    #end
    mutex.acquire();
    var value = finished;
    mutex.release();
    return value;
  }

  #if wasm
  function pollWorker():Void {
    if (finished || workerId <= 0) return;
    if (control.isCancelled()) {
      MateriaWebFiles.materia_worker_cancel(workerId);
      artifactBytes = null;
      failure = ProjectLoadControl.CANCELLED;
      finished = true;
      return;
    }
    var phase = MateriaWebFiles.materia_worker_phase(workerId);
    if (phase.status == 0) {
      var text = phase.data.toString();
      if (text != workerPhase) { workerPhase = text; control.phase(text); }
    }
    var status = MateriaWebFiles.materia_worker_status(workerId);
    if (status == 0) return;
    try {
      var response = MateriaWebFiles.materia_worker_result(workerId);
      if (status != 1 || response.status != 0) throw response.data.toString();
      control.throwIfCancelled();
      var bytes = artifactBytes;
      if (bytes == null) throw "Missing artifact input";
      result = new ProjectArtifactLoader(PreparedSceneCaches.current()).load(bytes, control, response.data, true);
    } catch (error:Dynamic) failure = Std.string(error);
    MateriaWebFiles.materia_worker_cancel(workerId);
    artifactBytes = null;
    control.detach();
    finished = true;
  }
  #end

  public function wasCancelled():Bool return control.isCancelled() || failure == ProjectLoadControl.CANCELLED;

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
