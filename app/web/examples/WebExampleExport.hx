import app.editor.ExampleCatalog;
import app.MateriaProjectRunner;
import app.PreparedProjectCache;
import haxe.io.Bytes;
import haxe.crypto.Sha256;
import haxe.Json;
import sys.io.File;
import sys.io.AtomicFile;
import sys.FileSystem;

/** Publish the desktop catalog as immutable, independently downloadable example resources. */
class WebExampleExport {
  static var destination:String;
  static function asset(bytes:Bytes, path:String, extension:String):Dynamic {
    if (bytes.length == 0 || bytes.length > 150000000) throw "Example resource exceeds its size limit";
    var digest = PreparedProjectCache.hex(Sha256.make(bytes));
    var object = "objects/" + digest + extension;
    AtomicFile.writeBytes(destination + "/" + object, bytes);
    return {url: object, sha256: digest, bytes: bytes.length,
      path: path == "" ? "/files/examples/" + digest + extension : path};
  }

  public static function main():Int {
    var args = Sys.args();
    if (args.length < 1) throw "Usage: WebExampleExport OUTPUT_DIRECTORY [ID,...]";
    FileSystem.createDirectory(args[0]);
    destination = FileSystem.fullPath(args[0]);
    FileSystem.createDirectory(destination + "/objects");
    var only = args.length > 1 ? args[1].split(",") : null;
    var published:Array<Dynamic> = [];
    for (entry in ExampleCatalog.entries) {
      if (only != null && only.indexOf(entry.id) < 0) continue;
      Sys.println("Publishing browser example: " + entry.id);
      var files:Array<Dynamic> = [];
      var source = "";
      switch entry.kind {
        case Project(path):
          var record = asset(MateriaProjectRunner.exportArtifact(path, entry.jobId), "", ".mtrg");
          files.push(record);
          source = record.path;
        case Script(_):
          // This setup is already compiled into the guest and needs no downloads.
        case WorkerRackToTable | WorkerGallery:
          var path = entry.id == "worker-gallery" ? "app/examples/worker-gallery.materia" : "app/examples/worker-rack-to-table.materia";
          source = "/" + path;
          var content = File.getBytes(path);
          files.push(asset(content, source, ".materia"));
          var root:Dynamic = Json.parse(content.toString());
          var assets = new Map<String, Bool>();
          for (object in (cast Reflect.field(root, "objects"):Array<Dynamic>)) {
            // Character data is nested under the human component in saved documents.
            collectAssets(object, assets);
          }
          for (path in assets.keys()) files.push(asset(File.getBytes(path), "/" + path, ".glb"));
      }
      published.push({id: entry.id, source: source, files: files});
    }
    if (only != null && published.length != only.length) throw "Unknown example selection";
    AtomicFile.write(destination + "/catalog.json", Json.stringify({version: 1, examples: published}));
    Sys.println('Published ${published.length} browser examples to $destination');
    return 0;
  }

  static function collectAssets(value:Dynamic, assets:Map<String, Bool>):Void {
    if (value == null || Std.isOfType(value, String) || Std.isOfType(value, Float) || Std.isOfType(value, Int) || Std.isOfType(value, Bool)) return;
    if (Std.isOfType(value, Array)) {
      for (item in (cast value:Array<Dynamic>)) collectAssets(item, assets);
      return;
    }
    for (name in Reflect.fields(value)) {
      var item:Dynamic = Reflect.field(value, name);
      if (name == "asset" && Std.isOfType(item, String) && StringTools.startsWith(item, "animkit/assets/")) assets.set(item, true);
      else collectAssets(item, assets);
    }
  }
}
