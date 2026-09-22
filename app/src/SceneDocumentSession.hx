package app;

import sys.io.File;
import sys.io.AtomicFile;
import haxe.io.Path as FilePath;

/** Owns the current document; unsuccessful I/O leaves it and its history intact. */
class SceneDocumentSession {
  public var scene(default, null):EditorScene;
  public var path(default, null):Null<String> = null;
  public var generation(default, null):Int = 0;

  public function new() scene = new EditorScene();

  public function newDocument():Void replace(new EditorScene(), null);

  public function open(file:String):Void {
    var absolute = checkedPath(file);
    var data = SceneCodec.decode(File.getContent(absolute));
    var next = new EditorScene(data);
    replace(next, absolute);
  }

  public function save(?file:String):Void {
    var destination = file == null ? path : file;
    if (destination == null) throw "Choose a filename for this scene";
    var absolute = checkedPath(destination);
    AtomicFile.write(absolute, SceneCodec.encode(scene));
    // Do not move the savepoint or change the document path until publication succeeds.
    path = absolute;
    scene.document.markSaved();
  }

  public function label():String {
    var name = path == null ? "Untitled" : FilePath.withoutDirectory(path);
    return name + (scene.document.isDirty ? " *" : "");
  }

  function replace(next:EditorScene, file:Null<String>):Void {
    var previous = scene;
    scene = next;
    path = file;
    generation++;
    previous.dispose();
  }

  static function checkedPath(value:String):String {
    if (StringTools.trim(value).length == 0 || SceneCodec.containsNul(value))
      throw "Choose a valid scene filename";
    // fullPath/realpath requires an existing destination; Save As does not.
    return FilePath.isAbsolute(value) ? value : Sys.getCwd() + "/" + value;
  }

  public function dispose():Void scene.dispose();
}
