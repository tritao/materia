package app;

import sys.io.File;
import sys.io.AtomicFile;
import haxe.io.Path as FilePath;
import app.ScriptOwnership.ScriptMaterialization;

/** Owns the current document; unsuccessful I/O leaves it and its history intact. */
class SceneDocumentSession {
  public var scene(default, null):EditorScene;
  public var sensors(default, null):SensorConfiguration;
  public var path(default, null):Null<String> = null;
  public var generation(default, null):Int = 0;
  public var scriptOwnership(default,null):Null<ScriptOwnership> = null;
  /** Application-owned runtime cleanup invoked only after replacement data validates. */
  public var beforeReplace:Null<Void->Void> = null;

  public function new() { scene = new EditorScene(); sensors = new SensorConfiguration(); }

  public function newDocument():Void replace(new EditorScene(), new SensorConfiguration(), null,null);

  public function open(file:String):Void {
    var absolute = checkedPath(file);
    var text = File.getContent(absolute);
    var script=SceneCodec.decodeScript(text);
    if(script!=null){
      var ownership=new ScriptOwnership(script.reference,script),materialized:ScriptMaterialization;
      try materialized=ownership.materialize() catch(error:Dynamic){ownership.dispose();throw error;}
      replace(materialized.scene,materialized.sensors,absolute,ownership);return;
    }
    var data = SceneCodec.decode(text);
    var next = new EditorScene(data);
    var nextSensors:SensorConfiguration;
    try nextSensors = new SensorConfiguration(SceneCodec.decodeSensors(text))
    catch (error:Dynamic) { next.dispose(); throw error; }
    replace(next, nextSensors, absolute,null);
  }

  public function openScript(reference:String):ScriptMaterialization {
    var ownership=new ScriptOwnership(reference),materialized:ScriptMaterialization;
    try materialized=ownership.materialize() catch(error:Dynamic){ownership.dispose();throw error;}
    replace(materialized.scene,materialized.sensors,null,ownership);
    return materialized;
  }

  /** Publishes a validated candidate without touching the currently running simulation. */
  public function reloadScript():ScriptMaterialization {
    var ownership=scriptOwnership;if(ownership==null)throw "This document is not script-owned";
    var materialized=ownership.reload();replace(materialized.scene,materialized.sensors,path,ownership,true);
    return materialized;
  }
  public function refreshScriptOverrides():ScriptMaterialization {
    var ownership=scriptOwnership;if(ownership==null)throw "This document is not script-owned";
    var materialized=ownership.materialize();replace(materialized.scene,materialized.sensors,path,ownership,true);
    return materialized;
  }

  public function save(?file:String):Void {
    var destination = file == null ? path : file;
    if (destination == null) throw "Choose a filename for this scene";
    var absolute = checkedPath(destination);
    AtomicFile.write(absolute, SceneCodec.encode(scene, sensors,
      scriptOwnership==null?null:scriptOwnership.record()));
    // Do not move the savepoint or change the document path until publication succeeds.
    path = absolute;
    scene.document.markSaved();
    scene.markSaved();
    sensors.document.markSaved();
    if(scriptOwnership!=null)scriptOwnership.markSaved();
  }

  public function label():String {
    var name = path == null ? "Untitled" : FilePath.withoutDirectory(path);
    return name + (isDirty() ? " *" : "");
  }

  public function isDirty():Bool return scene.document.isDirty || scene.hasUnsavedSketchDraftChanges() ||
    sensors.document.isDirty ||
    (scriptOwnership!=null&&scriptOwnership.document.isDirty);

  function replace(next:EditorScene, nextSensors:SensorConfiguration, file:Null<String>,
      nextOwnership:Null<ScriptOwnership>,?preserveRuntime:Bool=false):Void {
    try {if(!preserveRuntime&&beforeReplace!=null)beforeReplace();}
    catch(failure:Dynamic){next.dispose();nextSensors.dispose();
      if(nextOwnership!=null&&nextOwnership!=scriptOwnership)nextOwnership.dispose();throw failure;}
    var previous = scene;
    var previousSensors = sensors;
    var previousOwnership=scriptOwnership;
    scene = next;
    sensors = nextSensors;
    scriptOwnership=nextOwnership;
    path = file;
    generation++;
    previous.dispose();
    previousSensors.dispose();
    if(previousOwnership!=null&&previousOwnership!=nextOwnership)previousOwnership.dispose();
  }

  static function checkedPath(value:String):String {
    if (StringTools.trim(value).length == 0 || SceneCodec.containsNul(value))
      throw "Choose a valid scene filename";
    // fullPath/realpath requires an existing destination; Save As does not.
    return FilePath.isAbsolute(value) ? value : Sys.getCwd() + "/" + value;
  }

  public function dispose():Void { scene.dispose(); sensors.dispose();if(scriptOwnership!=null)scriptOwnership.dispose(); }
}
