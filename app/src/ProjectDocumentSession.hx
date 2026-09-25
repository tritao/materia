package app;

import sys.io.File;
import sys.io.AtomicFile;
import sys.FileSystem;
import haxe.io.Path as FilePath;
import haxe.Json;
import app.ScriptOwnership.ScriptMaterialization;
import nativekit.ui.editing.EditorDocument;
import nativekit.ui.editing.EditHistory;
import nativekit.ui.editing.EditOperation;
import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.properties.PropertyDescriptorOptions;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;
import bimkit.BimDocument;
import app.ProjectSceneRecord.ProjectSceneInstance;
import materia.project.AssemblyRecord;
import materia.project.AssemblyDefinition;
import materia.project.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.project.AssemblyDefinition.AssemblyJointRole;
import materia.project.AssemblyDefinition.AssemblyJointLimits;
import materia.project.AssemblyDefinition.AssemblyJointType;
import materia.project.AssemblyDefinition.AssemblyStateRecord;
import materia.project.AssemblyDefinitionCodec;
import materia.project.AssemblyFrames;
import cadkit.modeling.AssemblyState;
import nativekit.scene.GeometryData;

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
  public var projectAssembly(default,null):Null<AssemblyRecord> = null;
  public var projectAssemblyDefinition(default,null):Null<AssemblyDefinition> = null;
  public var projectAssemblyState(default,null):Null<AssemblyStateRecord> = null;
  var assemblyRuntime:Null<AssemblyState> = null;
  var assemblyLocalCentersByDefinition:Null<Map<String, Array<Float>>> = null;
  var assemblyMetresPerUnit:Float = 1.0;
  final assemblyOccurrenceIds:Map<String, Bool> = new Map();
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
    openContent(absolute, text);
  }

  /** Transfer an unsaved document across a development module reload. */
  public function liveState():String {
    var destination = path != null ? path : projectReference != null
      ? projectReference + ".materia" : Sys.getCwd() + "/untitled.materia";
    var project = projectReference == null ? null : projectSaveData(destination);
    return Json.stringify({path: path, destination: destination,
      content: SceneCodec.encode(scene, sensors,
        scriptOwnership == null ? null : scriptOwnership.record(), bim,
        project == null ? null : project.record,
        project == null ? null : project.authored),
      dirty: isDirty()});
  }

  public function restoreLiveState(value:String):Void {
    var state:Dynamic = Json.parse(value);
    var destination:String = Reflect.field(state, "destination");
    var content:String = Reflect.field(state, "content");
    var oldPath:Null<String> = Reflect.field(state, "path");
    openContent(destination, content);
    path = oldPath;
    if (Reflect.field(state, "dirty") == true) document.markExternallyDirty();
  }

  function openContent(absolute:String, text:String):Void {
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
  public function openGeneratedScene(data:Array<SceneObjectData>, ?manifestPath:String,
      ?assembly:AssemblyRecord, ?geometryBySnapshot:Map<String, GeometryData>,
      ?assemblyDefinition:AssemblyDefinition, ?assemblyState:AssemblyStateRecord,
      ?localCentersByDefinition:Map<String, Array<Float>>, metresPerUnit:Float = 1.0):Void {
    if (data == null || data.length == 0)
      throw "Generated project preview contains no scene objects";
    var reference = manifestPath == null ? null : FileSystem.fullPath(manifestPath);
    var nextDocument = createDocument();
    var next = new EditorScene(data, nextDocument, null, geometryBySnapshot);
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
    var runtime:Null<AssemblyState> = null;
    try {
      runtime = assemblyDefinition == null ? null : new AssemblyState(assemblyDefinition, assemblyState);
      configureAssembly(next, assemblyDefinition);
    } catch (error:Dynamic) {
      next.dispose();
      if (nextSensors != null) nextSensors.dispose();
      if (nextBim != null) nextBim.close();
      throw error;
    }
    replace(next, nextSensors, null, null, nextBim, nextDocument);
    projectAssembly = assembly;
    installAssemblyRuntime(assemblyDefinition, runtime, localCentersByDefinition, metresPerUnit);
    if (reference != null) {
      projectReference = reference;
      projectBaseline = data;
    }
  }

  function openProjectDocument(absolute:String, text:String, project:ProjectSceneRecord):Void {
    var reference = FilePath.isAbsolute(project.reference) ? project.reference
      : FilePath.join([FilePath.directory(absolute), project.reference]);
    reference = FileSystem.fullPath(reference);
    var generated = MateriaProjectRunner.loadProject(reference);
    var stateRecord = generated.assemblyState;
    if (project.assemblyState != null) {
      if (generated.assemblyDefinition == null)
        throw "Generated project state has no kinematic assembly definition";
      stateRecord = AssemblyDefinitionCodec.decodeState(generated.assemblyDefinition, project.assemblyState);
    }
    if (stateRecord != null && generated.assemblyDefinition != null)
      generated = MateriaProjectRunner.evaluateAssemblyState(generated, stateRecord);
    var baseline = generated.objects;
    var data = materializeProject(baseline, project, SceneCodec.decode(text));
    var nextDocument = createDocument();
    var next:EditorScene = null, nextSensors:SensorConfiguration = null, nextBim:BimDocument = null;
    try {
      next = new EditorScene(data, nextDocument, null, generated.geometryBySnapshot);
      nextSensors = new SensorConfiguration(SceneCodec.decodeSensors(text), nextDocument);
      nextBim = SceneCodec.decodeBim(text);
    } catch (error:Dynamic) {
      if (next != null) next.dispose();
      if (nextSensors != null) nextSensors.dispose();
      if (nextBim != null) nextBim.close();
      throw error;
    }
    var runtime:Null<AssemblyState> = null;
    try {
      runtime = generated.assemblyDefinition == null ? null :
        new AssemblyState(generated.assemblyDefinition, stateRecord);
      configureAssembly(next, generated.assemblyDefinition);
    } catch (error:Dynamic) {
      if (next != null) next.dispose();
      if (nextSensors != null) nextSensors.dispose();
      if (nextBim != null) nextBim.close();
      throw error;
    }
    replace(next, nextSensors, absolute, null, nextBim, nextDocument);
    projectReference = reference;
    projectBaseline = baseline;
    projectAssembly = generated.assembly;
    installAssemblyRuntime(generated.assemblyDefinition, runtime,
      generated.localCentersByDefinition, generated.metresPerUnit);
  }

  function configureAssembly(target:EditorScene, definition:Null<AssemblyDefinition>):Void {
    if (definition == null) return;
    var ids:Array<String> = [];
    for (occurrence in definition.occurrences) ids.push(occurrence.id);
    target.configureAssemblyOccurrences(ids, function(id) return assemblyPropertiesForOccurrence(id));
  }

  function installAssemblyRuntime(definition:Null<AssemblyDefinition>, state:Null<AssemblyState>,
      centers:Null<Map<String, Array<Float>>>, metresPerUnit:Float):Void {
    assemblyOccurrenceIds.clear();
    projectAssemblyDefinition = definition;
    assemblyRuntime = state;
    assemblyLocalCentersByDefinition = centers;
    assemblyMetresPerUnit = metresPerUnit;
    if (definition == null || state == null) {
      projectAssemblyState = null;
      return;
    }
    for (occurrence in definition.occurrences) assemblyOccurrenceIds.set("project:" + occurrence.id, true);
    projectAssemblyState = state.record();
    projectAssembly = MateriaProjectRunner.legacySnapshot(definition, state);
  }

  function assemblyPropertiesForOccurrence(sceneId:String):Array<PropertyDescriptor> {
    var result:Array<PropertyDescriptor> = [];
    var definition = projectAssemblyDefinition, centers = assemblyLocalCentersByDefinition;
    if (definition == null || assemblyRuntime == null || centers == null ||
        !StringTools.startsWith(sceneId, "project:")) return result;
    var occurrenceId = sceneId.substr(8);
    var occurrence:Null<AssemblyComponentOccurrence> = null;
    for (item in definition.occurrences) if (item.id == occurrenceId) { occurrence = item; break; }
    if (occurrence == null) return result;
    for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree &&
        (joint.parent == occurrenceId || joint.child == occurrenceId) &&
        AssemblyDefinitionCodec.hasCoordinate(joint.type))
      result.push(assemblyJointProperty(joint.id, joint.type, joint.limits,
        joint.type == AssemblyJointType.Prismatic ? assemblyMetresPerUnit : 1.0));
    return result;
  }

  function assemblyJointProperty(jointId:String, type:AssemblyJointType,
      limits:AssemblyJointLimits, factor:Float):PropertyDescriptor {
    var options = new PropertyDescriptorOptions();
    options.category = "Assembly";
    options.unit = type == AssemblyJointType.Prismatic ? "m" : "rad";
    options.step = type == AssemblyJointType.Prismatic ? Math.max(0.0001, factor) : 0.01;
    options.minimum = limits.lower == null ? null : limits.lower * factor;
    options.maximum = limits.upper == null ? null : limits.upper * factor;
    options.recordHistory = false;
    options.validator = function(_, value) return switch (value) {
      case PropertyValue.Float(number) if (Math.isFinite(number)): null;
      case PropertyValue.Int(_): null;
      default: "Joint coordinate must be finite";
    };
    return new PropertyDescriptor("assembly-joint:" + jointId, jointId, PropertyType.Float,
      function(_) {
        var state = assemblyRuntime;
        if (state == null) throw "Assembly state is no longer available";
        return PropertyValue.Float(state.joint(jointId) * factor);
      }, function(_, value) {
        var number:Float = switch (value) {
          case PropertyValue.Float(next): next;
          case PropertyValue.Int(next): next;
          default: throw "Joint coordinate must be numeric";
        };
        setAssemblyJointCoordinate(jointId, number / factor);
      }, options);
  }

  /** Change a tree-joint coordinate as one undoable editor operation. */
  public function setAssemblyJointCoordinate(jointId:String, value:Float):Bool {
    var state = assemblyRuntime;
    if (state == null) throw "This project has no editable assembly state";
    var before = state.joint(jointId);
    if (!Math.isFinite(value)) throw "Joint coordinate must be finite";
    if (before == value) return false;
    return document.apply(new EditOperation("Set joint " + jointId,
      function() applyAssemblyJointCoordinate(jointId, value),
      function() applyAssemblyJointCoordinate(jointId, before)));
  }

  function applyAssemblyJointCoordinate(jointId:String, value:Float):Void {
    var definition = projectAssemblyDefinition, current = assemblyRuntime;
    var centers = assemblyLocalCentersByDefinition;
    if (definition == null || current == null || centers == null)
      throw "Assembly placement data is unavailable";
    var candidate = new AssemblyState(definition, current.record());
    candidate.setJoint(jointId, value);
    var transforms:Array<{id:String, x:Float, y:Float, z:Float, rotation:Array<Float>}> = [];
    for (occurrence in definition.occurrences) {
      var center = centers.get(occurrence.definition);
      if (center == null || center.length != 3)
        throw 'Assembly component "${occurrence.definition}" has no local preview center';
      var pose = candidate.worldPose(occurrence.id);
      var world = AssemblyFrames.transformPoint(pose, center[0], center[1], center[2]);
      transforms.push({id: "project:" + occurrence.id, x: world.x * assemblyMetresPerUnit,
        y: world.y * assemblyMetresPerUnit, z: world.z * assemblyMetresPerUnit,
        rotation: [pose.qx, pose.qy, pose.qz, pose.qw]});
    }
    var nextRecord = candidate.record();
    var nextCompatibility = MateriaProjectRunner.legacySnapshot(definition, candidate);
    scene.setAssemblyOccurrenceTransforms(transforms);
    assemblyRuntime = candidate;
    projectAssemblyState = nextRecord;
    projectAssembly = nextCompatibility;
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
    projectAssembly = null;
    projectAssemblyDefinition = null;
    projectAssemblyState = null;
    assemblyRuntime = null;
    assemblyLocalCentersByDefinition = null;
    assemblyMetresPerUnit = 1.0;
    assemblyOccurrenceIds.clear();
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
        var assemblyManaged = assemblyOccurrenceIds.exists(item.id);
        var unchanged = assemblyManaged ? sameAppearanceWithoutPose(item, source) : sameAppearance(item, source);
        if (!unchanged) overrides.push(withoutMesh(item, !assemblyManaged));
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
    var savedAssemblyState = projectAssemblyDefinition == null || assemblyRuntime == null ? null
      : AssemblyDefinitionCodec.encodeState(projectAssemblyDefinition, assemblyRuntime.record());
    return {record: {version: 1, reference: relativeReference(destination, reference),
      overrides: overrides, removed: removed, instances: instances,
      assemblyState: savedAssemblyState}, authored: authored};
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
        "collisionEnabled", "dynamicBody", "mass", "red", "green", "blue", "visible", "rotation"])
      if (Reflect.hasField(edit, field)) Reflect.setField(value, field, Reflect.field(edit, field));
    for (field in Reflect.fields(edit)) if (field != "id" && field != "type" &&
        ["label", "x", "y", "z", "width", "height", "depth", "collisionEnabled",
          "dynamicBody", "mass", "red", "green", "blue", "visible", "rotation"].indexOf(field) < 0)
      throw 'Project part "$id" has an unsupported edit';
    // Validate authored fields without serializing generated mesh data.
    Reflect.setField(value, "meshSnapshot", "_");
    var result = SceneCodec.decode(Json.stringify({format: SceneCodec.FORMAT,
      version: SceneCodec.VERSION, objects: [value]}))[0];
    result.meshSnapshot = source.meshSnapshot;
    return result;
  }

  static function withoutMesh(item:SceneObjectData, includePose:Bool = true):Dynamic {
    var result:Dynamic = {id: item.id, type: item.type, label: item.label,
      width: item.width, height: item.height, depth: item.depth,
      collisionEnabled: item.collisionEnabled, dynamicBody: item.dynamicBody, mass: item.mass,
      red: item.red, green: item.green, blue: item.blue, visible: item.visible};
    if (includePose) {
      Reflect.setField(result, "x", item.x);
      Reflect.setField(result, "y", item.y);
      Reflect.setField(result, "z", item.z);
      Reflect.setField(result, "rotation", item.rotation);
    }
    return result;
  }

  static function sameAppearance(left:SceneObjectData, right:SceneObjectData):Bool
    return left.label == right.label && left.x == right.x && left.y == right.y && left.z == right.z &&
      left.width == right.width && left.height == right.height && left.depth == right.depth &&
      left.collisionEnabled == right.collisionEnabled && left.dynamicBody == right.dynamicBody &&
      left.mass == right.mass && left.red == right.red && left.green == right.green &&
      left.blue == right.blue && left.visible == right.visible && sameRotation(left.rotation, right.rotation);

  static function sameAppearanceWithoutPose(left:SceneObjectData, right:SceneObjectData):Bool
    return left.label == right.label && left.width == right.width && left.height == right.height &&
      left.depth == right.depth && left.collisionEnabled == right.collisionEnabled &&
      left.dynamicBody == right.dynamicBody && left.mass == right.mass && left.red == right.red &&
      left.green == right.green && left.blue == right.blue && left.visible == right.visible;

  static function sameRotation(left:Null<Array<Float>>, right:Null<Array<Float>>):Bool {
    if (left == null || right == null) return left == right;
    if (left.length != 4 || right.length != 4) return false;
    for (index in 0...4) if (left[index] != right[index]) return false;
    return true;
  }

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
