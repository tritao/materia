package app;

import sys.io.File;
import sys.io.AtomicFile;
import sys.FileSystem;
import haxe.io.Path as FilePath;
import haxe.Json;
import app.ScriptOwnership.ScriptMaterialization;
import nativekit.ui.editing.EditorDocument;
import nativekit.ui.editing.EditHistory;
import bimkit.BimDocument;
import app.ProjectSceneRecord.ProjectSceneInstance;

/** Owns the current document; unsuccessful I/O leaves it and its history intact. */
class ProjectDocumentSession {
  public static inline var MAX_HISTORY_OPERATIONS:Int = 1000;
  public static inline var MAX_HISTORY_ESTIMATED_BYTES:Int = 64 * 1024 * 1024;

  public var document(default, null):EditorDocument;
  public var edits(default, null):ProjectEditCoordinator;
  public var scene(default, null):EditorScene;
  public var sensors(default, null):SensorConfiguration;
  public var bim(default, null):BimDocument;
  public var path(default, null):Null<String> = null;
  public var generation(default, null):Int = 0;
  public var scriptOwnership(default,null):Null<ScriptOwnership> = null;
  public var projectReference(default,null):Null<String> = null;
  var projectBaseline:Null<Array<SceneObjectData>> = null;
  /** Application-owned runtime cleanup invoked only after replacement data validates. */
  public var beforeReplace:Null<Void->Void> = null;

  public function new(?initialBim:BimDocument) {
    document = createDocument();
    edits = new ProjectEditCoordinator(document);
    scene = new EditorScene(null, document);
    sensors = new SensorConfiguration(null, document);
    bim = initialBim == null ? new BimDocument() : initialBim;
    bim.cad.clearHistory();
  }

  public function newDocument():Void {
    var nextDocument = createDocument();
    replace(new EditorScene(null, nextDocument), new SensorConfiguration(null, nextDocument),
      null, null, new BimDocument(), nextDocument);
  }

  public function open(file:String):Void {
    var absolute = checkedPath(file);
    var text = File.getContent(absolute);
    var project = SceneCodec.decodeProject(text);
    if (project != null) {
      openProjectDocument(absolute, text, project);
      return;
    }
    var script=SceneCodec.decodeScript(text);
    if(script!=null){
      var nextBim = SceneCodec.decodeBim(text);
      var nextDocument = createDocument();
      var ownership:Null<ScriptOwnership> = null;
      var materialized:ScriptMaterialization;
      try {
        ownership = new ScriptOwnership(script.reference,script,nextDocument);
        materialized = ownership.materialize();
      } catch(error:Dynamic) {
        if (ownership != null) ownership.dispose();
        nextBim.close();
        throw error;
      }
      replace(materialized.scene,materialized.sensors,absolute,ownership,nextBim,nextDocument);return;
    }
    var data = SceneCodec.decode(text);
    var nextDocument = createDocument();
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
    var nextDocument = createDocument();
    var ownership=new ScriptOwnership(reference,null,nextDocument),materialized:ScriptMaterialization;
    try materialized=ownership.materialize() catch(error:Dynamic){ownership.dispose();throw error;}
    replace(materialized.scene,materialized.sensors,null,ownership,new BimDocument(),nextDocument);
    return materialized;
  }

  /** Open generated geometry while retaining its source manifest. */
  public function openGeneratedScene(data:Array<SceneObjectData>, ?manifestPath:String):Void {
    if (data == null || data.length == 0)
      throw "Generated project preview contains no scene objects";
    var reference = manifestPath == null ? null : FileSystem.fullPath(manifestPath);
    var nextDocument = createDocument();
    var next = new EditorScene(data, nextDocument);
    var nextSensors:SensorConfiguration = null;
    var nextBim:BimDocument = null;
    try {
      nextSensors = new SensorConfiguration(null, nextDocument);
      nextBim = new BimDocument();
    } catch (error:Dynamic) {
      next.dispose();
      if (nextSensors != null) nextSensors.dispose();
      if (nextBim != null) nextBim.close();
      throw error;
    }
    replace(next, nextSensors, null, null, nextBim, nextDocument);
    if (reference != null) {
      projectReference = reference;
      projectBaseline = data;
    }
  }

  function openProjectDocument(absolute:String, text:String, project:ProjectSceneRecord):Void {
    var reference = FilePath.isAbsolute(project.reference) ? project.reference
      : FilePath.join([FilePath.directory(absolute), project.reference]);
    reference = FileSystem.fullPath(reference);
    var baseline = MateriaProjectRunner.load(reference);
    var data = materializeProject(baseline, project, SceneCodec.decode(text));
    var nextDocument = createDocument();
    var next:EditorScene = null, nextSensors:SensorConfiguration = null, nextBim:BimDocument = null;
    try {
      next = new EditorScene(data, nextDocument);
      nextSensors = new SensorConfiguration(SceneCodec.decodeSensors(text), nextDocument);
      nextBim = SceneCodec.decodeBim(text);
    } catch (error:Dynamic) {
      if (next != null) next.dispose();
      if (nextSensors != null) nextSensors.dispose();
      if (nextBim != null) nextBim.close();
      throw error;
    }
    replace(next, nextSensors, absolute, null, nextBim, nextDocument);
    projectReference = reference;
    projectBaseline = baseline;
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
    var project = projectReference == null ? null : projectSaveData(absolute);
    AtomicFile.write(absolute, SceneCodec.encode(scene, sensors,
      scriptOwnership==null?null:scriptOwnership.record(), bim,
      project == null ? null : project.record,
      project == null ? null : project.authored));
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
    edits = new ProjectEditCoordinator(nextDocument);
    scene = next;
    sensors = nextSensors;
    scriptOwnership=nextOwnership;
    projectReference = null;
    projectBaseline = null;
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
    var firstApplication = true;
    try {
      return edits.apply(label, function() {
        if (firstApplication) {
          change();
          firstApplication = false;
        } else if (!bim.redo()) {
          throw "BIM redo history is out of sync with project history";
        }
      }, function() {
        if (!bim.undo()) throw "BIM undo history is out of sync with project history";
      });
    } catch (error:Dynamic) {
      if (!firstApplication) bim.undo();
      throw error;
    }
  }

  static function checkedPath(value:String):String {
    if (StringTools.trim(value).length == 0 || SceneCodec.containsNul(value))
      throw "Choose a valid scene filename";
    // fullPath/realpath requires an existing destination; Save As does not.
    return FilePath.isAbsolute(value) ? value : Sys.getCwd() + "/" + value;
  }

  function projectSaveData(destination:String):{record:ProjectSceneRecord, authored:Array<SceneObjectData>} {
    var baseline = projectBaseline, reference = projectReference;
    if (baseline == null || reference == null) throw "Generated project has no source baseline";
    var sources = new Map<String, SceneObjectData>();
    for (item in baseline) sources.set(item.id, item);
    var present = new Map<String, Bool>();
    var overrides:Array<Dynamic> = [], instances:Array<ProjectSceneInstance> = [];
    var authored:Array<SceneObjectData> = [];
    for (item in scene.recordsForSave()) {
      var source = sources.get(item.id);
      if (source != null) {
        if (item.type != source.type || item.meshSnapshot != source.meshSnapshot)
          throw 'Generated geometry for "${item.id}" must come from its project source';
        present.set(item.id, true);
        if (!sameAppearance(item, source)) overrides.push(withoutMesh(item));
      } else if (item.type == "cad-preview") {
        var origin:Null<SceneObjectData> = null;
        for (candidate in baseline) if (candidate.meshSnapshot == item.meshSnapshot) {
          origin = candidate;
          break;
        }
        if (origin == null) throw 'CAD preview "${item.id}" has no project source';
        instances.push({sourceId: origin.id, object: withoutMesh(item)});
      } else {
        authored.push(item);
      }
    }
    var removed:Array<String> = [];
    for (item in baseline) if (!present.exists(item.id)) removed.push(item.id);
    return {record: {version: 1, reference: relativeReference(destination, reference),
      overrides: overrides, removed: removed, instances: instances}, authored: authored};
  }

  static function materializeProject(baseline:Array<SceneObjectData>, project:ProjectSceneRecord,
      authored:Array<SceneObjectData>):Array<SceneObjectData> {
    var sources = new Map<String, SceneObjectData>();
    for (item in baseline) sources.set(item.id, item);
    var removed = new Map<String, Bool>();
    for (id in project.removed) {
      if (!sources.exists(id)) throw 'Removed project part "$id" no longer exists';
      removed.set(id, true);
    }
    var overrides = new Map<String, Dynamic>();
    for (item in project.overrides) {
      var id:String = Reflect.field(item, "id");
      if (!sources.exists(id)) throw 'Project override "$id" no longer exists';
      overrides.set(id, item);
    }
    var data:Array<SceneObjectData> = [];
    var ids = new Map<String, Bool>();
    for (item in baseline) if (!removed.exists(item.id)) {
      var override = overrides.get(item.id);
      if (ids.exists(item.id)) throw 'Duplicate project part ID: ${item.id}';
      ids.set(item.id, true);
      data.push(override == null ? item : applyAppearance(item, override, item.id));
    }
    for (instance in project.instances) {
      var source = sources.get(instance.sourceId);
      if (source == null) throw 'Project instance source "${instance.sourceId}" no longer exists';
      var id:String = Reflect.field(instance.object, "id");
      if (ids.exists(id)) throw 'Duplicate project object ID: $id';
      ids.set(id, true);
      data.push(applyAppearance(source, instance.object, id));
    }
    for (item in authored) {
      if (item.type == "cad-preview") throw "Project-owned previews must reference a generated part";
      if (ids.exists(item.id)) throw 'Duplicate project object ID: ${item.id}';
      ids.set(item.id, true);
      data.push(item);
    }
    if (data.length > 10000) throw "Scene documents support at most 10000 objects";
    return data;
  }

  static function applyAppearance(source:SceneObjectData, edit:Dynamic, id:String):SceneObjectData {
    if (Reflect.hasField(edit, "type") && Reflect.field(edit, "type") != source.type)
      throw 'Project part "$id" changed type';
    var value:Dynamic = withoutMesh(source);
    Reflect.setField(value, "id", id);
    for (field in ["label", "x", "y", "z", "width", "height", "depth",
        "collisionEnabled", "dynamicBody", "mass", "red", "green", "blue", "visible"])
      if (Reflect.hasField(edit, field)) Reflect.setField(value, field, Reflect.field(edit, field));
    for (field in Reflect.fields(edit)) if (field != "id" && field != "type" &&
        ["label", "x", "y", "z", "width", "height", "depth", "collisionEnabled",
          "dynamicBody", "mass", "red", "green", "blue", "visible"].indexOf(field) < 0)
      throw 'Project part "$id" has an unsupported edit';
    // Validate authored fields without serializing generated mesh data.
    Reflect.setField(value, "meshSnapshot", "_");
    var result = SceneCodec.decode(Json.stringify({format: SceneCodec.FORMAT,
      version: SceneCodec.VERSION, objects: [value]}))[0];
    result.meshSnapshot = source.meshSnapshot;
    return result;
  }

  static function withoutMesh(item:SceneObjectData):Dynamic {
    return {id: item.id, type: item.type, label: item.label,
      x: item.x, y: item.y, z: item.z,
      width: item.width, height: item.height, depth: item.depth,
      collisionEnabled: item.collisionEnabled, dynamicBody: item.dynamicBody, mass: item.mass,
      red: item.red, green: item.green, blue: item.blue, visible: item.visible};
  }

  static function sameAppearance(left:SceneObjectData, right:SceneObjectData):Bool
    return left.label == right.label && left.x == right.x && left.y == right.y && left.z == right.z &&
      left.width == right.width && left.height == right.height && left.depth == right.depth &&
      left.collisionEnabled == right.collisionEnabled && left.dynamicBody == right.dynamicBody &&
      left.mass == right.mass && left.red == right.red && left.green == right.green &&
      left.blue == right.blue && left.visible == right.visible;

  static function relativeReference(destination:String, target:String):String {
    var from = FilePath.normalize(FileSystem.fullPath(FilePath.directory(destination))).split("/");
    var to = FilePath.normalize(target).split("/");
    if (from.length == 0 || to.length == 0 || from[0] != to[0]) return target;
    var index = 0;
    while (index < from.length && index < to.length && from[index] == to[index]) index++;
    var result:Array<String> = [];
    for (_ in index...from.length) result.push("..");
    for (part in to.slice(index)) result.push(part);
    return result.length == 0 ? "." : result.join("/");
  }

  public function dispose():Void { scene.dispose(); sensors.dispose();if(scriptOwnership!=null)scriptOwnership.dispose();bim.close(); }

  static function createDocument():EditorDocument
    return new EditorDocument("project", new EditHistory(MAX_HISTORY_OPERATIONS,
      MAX_HISTORY_ESTIMATED_BYTES));
}
