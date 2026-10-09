package app;

import app.editor.ExampleCatalog.ExampleEntry;

/** Opening an example may acquire resources first, then use the ordinary project loader. */
class ExampleLoadJob {
  public final control:ProjectLoadControl = new ProjectLoadControl();
  public final startedAt:Float = Sys.time();
  final entry:ExampleEntry;
  var project:Null<ProjectLoadJob>;
  var failure:Null<String>;
  var ready:Bool = false;
  #if wasm
  var download:Null<BrowserExampleDownload>;
  #end
  public function new(entry:ExampleEntry) {
    this.entry = entry;
    try {
      #if wasm
      if (BrowserExamples.has(entry.id)) download = new BrowserExampleDownload(entry.id, control);
      else startProject();
      #else
      startProject();
      #end
    } catch (error:Dynamic) { failure = Std.string(error); ready = true; }
  }
  function startProject():Void {
    switch entry.kind {
      case Project(path):
        #if wasm
        var source = BrowserExamples.has(entry.id) ? BrowserExamples.source(entry.id) : path;
        project = new ProjectLoadJob(source, null, control);
        #else
        project = new ProjectLoadJob(path, entry.jobId, control);
        #end
      default: ready = true;
    }
  }
  public function isFinished():Bool {
    if (ready) return true;
    #if wasm
    var pending = download;
    if (pending != null) {
      if (!pending.isFinished()) return false;
      download = null;
      try { pending.take(); control.throwIfCancelled(); startProject(); }
      catch (error:Dynamic) { failure = Std.string(error); ready = true; }
    }
    #end
    var running = project;
    return ready || (running != null && running.isFinished());
  }
  public function wasCancelled():Bool return control.isCancelled() || failure == ProjectLoadControl.CANCELLED || (project != null && project.wasCancelled());
  public function elapsedSeconds():Float return Sys.time() - startedAt;
  public function take():Null<GeneratedAssemblyScene> {
    if (failure != null) throw failure;
    control.throwIfCancelled();
    return project == null ? null : project.take();
  }
}
