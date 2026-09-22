package app;

import sys.io.File;
import sys.io.AtomicFile;
import haxe.io.Path as FilePath;

/** Owns the current document; unsuccessful I/O leaves it and its history intact. */
class SceneDocumentSession {
  public var scene(default, null):EditorScene;
  public var sensors(default, null):SensorConfiguration;
  public var path(default, null):Null<String> = null;
  public var generation(default, null):Int = 0;
  /** Application-owned runtime cleanup invoked only after replacement data validates. */
  public var beforeReplace:Null<Void->Void> = null;

  public function new() { scene = new EditorScene(); sensors = new SensorConfiguration(); }

  public function newDocument():Void replace(new EditorScene(), new SensorConfiguration(), null);

  public function open(file:String):Void {
    var absolute = checkedPath(file);
    var text = File.getContent(absolute);
    var data = SceneCodec.decode(text);
    var next = new EditorScene(data);
    var nextSensors:SensorConfiguration;
    try nextSensors = new SensorConfiguration(SceneCodec.decodeSensors(text))
    catch (error:Dynamic) { next.dispose(); throw error; }
    replace(next, nextSensors, absolute);
  }

  public function save(?file:String):Void {
    var destination = file == null ? path : file;
    if (destination == null) throw "Choose a filename for this scene";
    var absolute = checkedPath(destination);
    AtomicFile.write(absolute, SceneCodec.encode(scene, sensors));
    // Do not move the savepoint or change the document path until publication succeeds.
    path = absolute;
    scene.document.markSaved();
    sensors.document.markSaved();
  }

  public function label():String {
    var name = path == null ? "Untitled" : FilePath.withoutDirectory(path);
    return name + (isDirty() ? " *" : "");
  }

  public function isDirty():Bool return scene.document.isDirty || sensors.document.isDirty;

  function replace(next:EditorScene, nextSensors:SensorConfiguration, file:Null<String>):Void {
    try {if(beforeReplace!=null)beforeReplace();}
    catch(failure:Dynamic){next.dispose();nextSensors.dispose();throw failure;}
    var previous = scene;
    var previousSensors = sensors;
    scene = next;
    sensors = nextSensors;
    path = file;
    generation++;
    previous.dispose();
    previousSensors.dispose();
  }

  static function checkedPath(value:String):String {
    if (StringTools.trim(value).length == 0 || SceneCodec.containsNul(value))
      throw "Choose a valid scene filename";
    // fullPath/realpath requires an existing destination; Save As does not.
    return FilePath.isAbsolute(value) ? value : Sys.getCwd() + "/" + value;
  }

  public function dispose():Void { scene.dispose(); sensors.dispose(); }
}
