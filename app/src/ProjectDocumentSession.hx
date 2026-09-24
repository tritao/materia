package app;

import sys.io.File;
import sys.io.AtomicFile;
import haxe.io.Path as FilePath;
import app.ScriptOwnership.ScriptMaterialization;
import nativekit.ui.core.EditorDocument;
import nativekit.ui.core.EditOperation;
import bimkit.BimDocument;

/** Owns the current document; unsuccessful I/O leaves it and its history intact. */
class ProjectDocumentSession {
  public var document(default, null):EditorDocument;
  public var scene(default, null):EditorScene;
  public var sensors(default, null):SensorConfiguration;
  public var bim(default, null):BimDocument;
  public var path(default, null):Null<String> = null;
  public var generation(default, null):Int = 0;
  public var scriptOwnership(default,null):Null<ScriptOwnership> = null;
  /** Application-owned runtime cleanup invoked only after replacement data validates. */
  public var beforeReplace:Null<Void->Void> = null;

  public function new(?initialBim:BimDocument) {
    document = new EditorDocument("project");
    scene = new EditorScene(null, document);
    sensors = new SensorConfiguration(null, document);
    bim = initialBim == null ? new BimDocument() : initialBim;
    bim.cad.clearHistory();
  }

  public function newDocument():Void {
    var nextDocument = new EditorDocument("project");
    replace(new EditorScene(null, nextDocument), new SensorConfiguration(null, nextDocument),
      null, null, new BimDocument(), nextDocument);
  }

  public function open(file:String):Void {
    var absolute = checkedPath(file);
    var text = File.getContent(absolute);
    var script=SceneCodec.decodeScript(text);
    if(script!=null){
      var nextBim = SceneCodec.decodeBim(text);
      var nextDocument = new EditorDocument("project");
      var ownership=new ScriptOwnership(script.reference,script,nextDocument),materialized:ScriptMaterialization;
      try materialized=ownership.materialize() catch(error:Dynamic){ownership.dispose();nextBim.close();throw error;}
      replace(materialized.scene,materialized.sensors,absolute,ownership,nextBim,nextDocument);return;
    }
    var data = SceneCodec.decode(text);
    var nextDocument = new EditorDocument("project");
    var next = new EditorScene(data, nextDocument);
    var nextSensors:SensorConfiguration = null;
    var nextBim:BimDocument;
    try {
      nextSensors = new SensorConfiguration(SceneCodec.decodeSensors(text), nextDocument);
      nextBim = SceneCodec.decodeBim(text);
    }
    catch (error:Dynamic) { next.dispose(); if (nextSensors != null) nextSensors.dispose(); throw error; }
    replace(next, nextSensors, absolute, null, nextBim, nextDocument);
  }

  public function openScript(reference:String):ScriptMaterialization {
    var nextDocument = new EditorDocument("project");
    var ownership=new ScriptOwnership(reference,null,nextDocument),materialized:ScriptMaterialization;
    try materialized=ownership.materialize() catch(error:Dynamic){ownership.dispose();throw error;}
    replace(materialized.scene,materialized.sensors,null,ownership,new BimDocument(),nextDocument);
    return materialized;
  }

  /** Publishes a validated candidate without touching the currently running simulation. */
  public function reloadScript():ScriptMaterialization {
    var ownership=scriptOwnership;if(ownership==null)throw "This document is not script-owned";
    var materialized=ownership.reload();replace(materialized.scene,materialized.sensors,path,ownership,bim,document,true);
    return materialized;
  }
  public function refreshScriptOverrides():ScriptMaterialization {
    var ownership=scriptOwnership;if(ownership==null)throw "This document is not script-owned";
    var materialized=ownership.materialize();replace(materialized.scene,materialized.sensors,path,ownership,bim,document,true);
    return materialized;
  }

  public function save(?file:String):Void {
    var destination = file == null ? path : file;
    if (destination == null) throw "Choose a filename for this scene";
    var absolute = checkedPath(destination);
    AtomicFile.write(absolute, SceneCodec.encode(scene, sensors,
      scriptOwnership==null?null:scriptOwnership.record(), bim));
    // Do not move the savepoint or change the document path until publication succeeds.
    path = absolute;
    document.markSaved();
    scene.markSaved();
    if(scriptOwnership!=null)scriptOwnership.markSaved();
  }

  public function label():String {
    var name = path == null ? "Untitled" : FilePath.withoutDirectory(path);
    return name + (isDirty() ? " *" : "");
  }

  public function isDirty():Bool return document.isDirty || scene.hasUnsavedSketchDraftChanges();

  function replace(next:EditorScene, nextSensors:SensorConfiguration, file:Null<String>,
      nextOwnership:Null<ScriptOwnership>, nextBim:BimDocument, nextDocument:EditorDocument,
      ?preserveRuntime:Bool=false):Void {
    if (next == null || nextSensors == null || nextBim == null || nextDocument == null ||
        next.document != nextDocument || nextSensors.document != nextDocument ||
        (nextOwnership != null && nextOwnership.document != nextDocument)) {
      if (next != null) next.dispose();
      if (nextSensors != null) nextSensors.dispose();
      if (nextBim != null && nextBim != bim) nextBim.close();
      if (nextOwnership != null && nextOwnership != scriptOwnership) nextOwnership.dispose();
      throw "Project state must share its owning EditorDocument";
    }
    try {if(!preserveRuntime&&beforeReplace!=null)beforeReplace();}
    catch(failure:Dynamic){next.dispose();nextSensors.dispose();
      if(nextBim!=bim)nextBim.close();
      if(nextOwnership!=null&&nextOwnership!=scriptOwnership)nextOwnership.dispose();throw failure;}
    var previous = scene;
    var previousSensors = sensors;
    var previousOwnership=scriptOwnership;
    var previousBim=bim;
    document = nextDocument;
    scene = next;
    sensors = nextSensors;
    scriptOwnership=nextOwnership;
    bim=nextBim;
    path = file;
    generation++;
    previous.dispose();
    previousSensors.dispose();
    if(previousOwnership!=null&&previousOwnership!=nextOwnership)previousOwnership.dispose();
    if(previousBim!=nextBim)previousBim.close();
  }

  /** Applies a BIM mutation and places its CAD undo record in project order. */
  public function applyBimEdit(label:String, change:Void->Void):Bool {
    if (label == null || label.length == 0 || change == null) throw "BIM edits require a label and mutation";
    change();
    try {
      document.record(new EditOperation(label, function() {
        if (!bim.redo()) throw "BIM redo history is out of sync with project history";
      }, function() {
        if (!bim.undo()) throw "BIM undo history is out of sync with project history";
      }));
    } catch (error:Dynamic) {
      bim.undo();
      throw error;
    }
    return true;
  }

  static function checkedPath(value:String):String {
    if (StringTools.trim(value).length == 0 || SceneCodec.containsNul(value))
      throw "Choose a valid scene filename";
    // fullPath/realpath requires an existing destination; Save As does not.
    return FilePath.isAbsolute(value) ? value : Sys.getCwd() + "/" + value;
  }

  public function dispose():Void { scene.dispose(); sensors.dispose();if(scriptOwnership!=null)scriptOwnership.dispose();bim.close(); }
}
