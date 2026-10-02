package app;

#if wasm
import haxe.io.Bytes;
import nativekit.ffi.NativeKit;
import nativekit.ffi.NativeKitTypes;
import runtime.MemoryFileSystem;
import runtime.MemoryFileSystem.MemoryFileSystemObserver;

/**
 * Keeps the browser editor's files across page loads. Haxeon's Wasm filesystem lives in module memory, so `start`
 * fills it from what the page stored (MateriaWebFiles, in the origin private file system) before the editor reads
 * settings or documents, then mirrors every change back. Scenes, settings, recent files and the workspace layout
 * all persist without their code knowing.
 *
 * Files the user picks or saves through the browser's chooser live in /files. A save target is linked to its path,
 * and each write there is also written to the user's file: every write for a retained file handle (the File System
 * Access API), and only the next one for a download, which would otherwise start one per save.
 */
class BrowserFiles implements MemoryFileSystemObserver {
  static inline final DIRECTORY = "/files";
  /** Save targets by the path linked to them. */
  static final links:Map<String, String> = [];

  /** Where a chosen file named `name` lives: its file name only, in the chooser's directory. */
  public static function pathFor(name:Null<String>):String {
    var file = name == null ? "" : name.split("/").join("_").split("\\").join("_");
    if (file == "" || file == "." || file == "..") file = "Untitled.materia.json";
    return DIRECTORY + "/" + file;
  }

  /** Keeps a picked file's content at `path`. */
  public static function store(path:String, content:Bytes):Void {
    MemoryFileSystem.createDirectory(DIRECTORY, true);
    MemoryFileSystem.saveBytes(path, content);
  }

  /** Sends later writes to `path` to the chooser's save target `uri` as well. */
  public static function link(path:String, uri:String):Void {
    MemoryFileSystem.createDirectory(DIRECTORY, true);
    links.set(MemoryFileSystem.normalize(path), uri);
  }

  public static function start():Void {
    var count = MateriaWebFiles.materia_files_restore_count();
    for (index in 0...count) {
      var path = MateriaWebFiles.materia_files_restore_path(index);
      if (path.status < 0) continue;
      var name = path.data.toString();
      if (path.status == 0) {
        MemoryFileSystem.createDirectory(name, true);
        continue;
      }
      var content = MateriaWebFiles.materia_files_restore_content(index);
      if (content.status != 0) continue;
      var separator = name.lastIndexOf("/");
      if (separator > 0) MemoryFileSystem.createDirectory(name.substr(0, separator), true);
      MemoryFileSystem.saveBytes(name, content.data);
    }
    MateriaWebFiles.materia_files_restore_done();
    MemoryFileSystem.observer = new BrowserFiles();
  }

  function new() {}

  public function directoryCreated(path:String):Void
    MateriaWebFiles.materia_files_directory_created(path);

  public function fileWritten(path:String, content:Bytes):Void {
    MateriaWebFiles.materia_files_file_written(path, content);
    export(path, content);
  }

  public function removed(path:String):Void
    MateriaWebFiles.materia_files_removed(path);

  public function renamed(path:String, newPath:String):Void {
    MateriaWebFiles.materia_files_renamed(path, newPath);
    if (links.exists(newPath) && !MemoryFileSystem.isDirectory(newPath)) export(newPath, MemoryFileSystem.getBytes(newPath));
  }

  /** Writes `content` to the save target linked to `path`, if any. Failures go to the browser console. */
  static function export(path:String, content:Bytes):Void {
    var uri = links.get(path);
    if (uri == null) return;
    if (StringTools.startsWith(uri, "nativekit-download://")) links.remove(path);
    var resource = new Resource();
    resource.set_struct_size(Resource.size());
    resource.set_flags(ResourceFlags.Writable);
    resource.set_uri(uri);
    var opened = NativeKit.nk_resource_open(resource,
      ResourceOpenFlags.Write | ResourceOpenFlags.Create | ResourceOpenFlags.Truncate);
    if (opened.status != Result.Ok) {
      trace('Could not write ${path.substr(path.lastIndexOf("/") + 1)} to the chosen file: ${opened.status}');
      return;
    }
    var stream = opened.out_stream;
    var written = NativeKit.nk_resource_write(stream.borrow(), content);
    if (written.status != Result.Ok || haxe.Int64.toInt(written.out_written) != content.length)
      trace('Could not write ${path.substr(path.lastIndexOf("/") + 1)} to the chosen file: ${written.status}');
    // Committing hands the bytes to the file handle or starts the download; closing alone would only flush to a
    // file handle. NativeKit reports the outcome asynchronously.
    var committed = NativeKit.nk_resource_commit(stream.borrow());
    if (committed.status != Result.Ok)
      trace('Could not save ${path.substr(path.lastIndexOf("/") + 1)} to the chosen file: ${committed.status}');
    stream.close();
  }
}
#end
