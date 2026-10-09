package app;

#if wasm
import haxe.Json;
import haxe.io.Bytes;
import haxe.crypto.Sha256;
import sys.FileSystem;
import sys.io.File;

typedef BrowserExampleFile = {var url:String; var sha256:String; var bytes:Int; var path:String;};
typedef BrowserExample = {var id:String; var source:String; var files:Array<BrowserExampleFile>;};

/** Browser host policy: a tiny index selects immutable resources, never executable project sources. */
class BrowserExamples {
  static final entries:Map<String, BrowserExample> = [];
  public static function configure():Void {
    var response = MateriaWebFiles.materia_examples_catalog();
    if (response.status != 0) return;
    var root:Dynamic = Json.parse(response.data.toString());
    if (root.version != 1 || !Std.isOfType(root.examples, Array)) throw "Unsupported browser example catalog";
    for (value in (cast root.examples:Array<Dynamic>)) {
      if (!Std.isOfType(value.id, String) || !Std.isOfType(value.source, String) || !Std.isOfType(value.files, Array))
        throw "Invalid browser example";
      var entry:BrowserExample = {id: value.id, source: value.source, files: []};
      if (app.editor.ExampleCatalog.find(entry.id) == null || entries.exists(entry.id))
        throw "Unknown or duplicate browser example";
      for (raw in (cast value.files:Array<Dynamic>)) {
        if (!Std.isOfType(raw.url, String) || !Std.isOfType(raw.sha256, String) || !Std.isOfType(raw.bytes, Int) ||
            !Std.isOfType(raw.path, String)) throw "Invalid browser resource fields";
        var file:BrowserExampleFile = {url: raw.url, sha256: raw.sha256, bytes: raw.bytes, path: raw.path};
        if (!validDigest(file.sha256) || file.bytes <= 0 || file.bytes > 150000000 ||
            !StringTools.startsWith(file.url, "objects/" + file.sha256) ||
            !(StringTools.startsWith(file.path, "/files/examples/") || StringTools.startsWith(file.path, "/app/examples/") ||
              StringTools.startsWith(file.path, "/animkit/assets/")) || file.path.indexOf("..") >= 0)
          throw "Invalid browser example resource";
        entry.files.push(file);
      }
      if (entry.source != "" && [for (file in entry.files) if (file.path == entry.source) file].length != 1)
        throw "Browser example source is missing";
      entries.set(entry.id, entry);
    }
  }
  static function validDigest(value:String):Bool {
    if (value == null || value.length != 64) return false;
    for (i in 0...value.length) if ("0123456789abcdef".indexOf(value.charAt(i)) < 0) return false;
    return true;
  }
  public static function has(id:String):Bool return entries.exists(id);
  public static function source(id:String):String {
    var entry = entries.get(id);
    if (entry == null) throw "Browser example is not published";
    return entry.source;
  }
  public static function missing(id:String):Array<BrowserExampleFile> {
    var entry = entries.get(id);
    if (entry == null) throw "Browser example is not published";
    return [for (file in entry.files) if (!stored(file)) file];
  }
  static function stored(file:BrowserExampleFile):Bool {
    try {
      var metadata = FileSystem.metadata(file.path);
      if (metadata == null || metadata.size != file.bytes) return false;
      return PreparedProjectCache.hex(Sha256.make(File.getBytes(file.path))) == file.sha256;
    } catch (_:Dynamic) return false;
  }
}
#end
