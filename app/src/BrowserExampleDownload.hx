package app;

#if wasm
import haxe.Json;
import haxe.io.Bytes;
import haxe.crypto.Sha256;
import app.BrowserExamples.BrowserExampleFile;

/** Cancellable download stage. Publishes only complete, hash-verified resources to browser storage. */
class BrowserExampleDownload {
  final control:ProjectLoadControl;
  final files:Array<BrowserExampleFile>;
  var id:Int = 0;
  var finished:Bool = false;
  var failure:Null<String>;
  var phase:String = "";
  public function new(example:String, control:ProjectLoadControl) {
    this.control = control;
    files = BrowserExamples.missing(example);
    if (files.length == 0) { finished = true; return; }
    control.phase("Downloading example");
    id = MateriaWebFiles.materia_example_start(example, Json.stringify([for (file in files) file.path]));
    if (id <= 0) throw "Could not start example download";
    var self = this;
    control.attachCancellation(function() {
      MateriaWebFiles.materia_example_cancel(self.id);
      self.failure = ProjectLoadControl.CANCELLED;
      self.finished = true;
      self.control.detach();
    });
  }
  public function isFinished():Bool {
    if (finished) return true;
    var text = MateriaWebFiles.materia_example_phase(id);
    if (text.status == 0 && text.data.toString() != phase) { phase = text.data.toString(); control.phase(phase); }
    var status = MateriaWebFiles.materia_example_status(id);
    if (status == 0) return false;
    try {
      if (status != 1) {
        var error = MateriaWebFiles.materia_example_error(id);
        throw error.status == 0 ? error.data.toString() : "Example download failed";
      }
      var received:Array<Bytes> = [];
      for (i in 0...files.length) {
        control.throwIfCancelled();
        var response = MateriaWebFiles.materia_example_content(id, i);
        if (response.status != 0 || response.data.length != files[i].bytes ||
            PreparedProjectCache.hex(Sha256.make(response.data)) != files[i].sha256)
          throw "Downloaded example failed its integrity check";
        received.push(response.data);
      }
      control.throwIfCancelled();
      for (i in 0...files.length) BrowserFiles.store(files[i].path, received[i]);
    } catch (error:Dynamic) failure = Std.string(error);
    MateriaWebFiles.materia_example_cancel(id);
    control.detach();
    finished = true;
    return true;
  }
  public function take():Void { if (failure != null) throw failure; }
}
#end
