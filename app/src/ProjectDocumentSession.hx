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
import app.ProjectSceneRecord.ProjectFieldOverride;
import materia.project.Appearance.Appearances;
import materia.units.LengthUnit;
import materia.project.MaterialDef;
import materia.project.MaterialLibrary;
import materia.project.AssemblyRecord;
import materia.project.AssemblyDefinition;
import materia.project.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.project.AssemblyDefinition.AssemblyJointRole;
import materia.project.AssemblyDefinition.AssemblyJointLimits;
import materia.project.AssemblyDefinition.AssemblyJointType;
import materia.project.AssemblyDefinition.AssemblyStateRecord;
import materia.project.AssemblyDefinition.KinematicJoint;
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
  public var customMaterials(default, null):Array<MaterialDef> = [];
  var assemblyRuntime:Null<AssemblyState> = null;
  var assemblyLocalCentersByDefinition:Null<Map<String, Array<Float>>> = null;
  var assemblyMetresPerUnit:Float = 1.0;
  final assemblyOccurrenceIds:Map<String, Bool> = new Map();
  final assemblyDependentJoints:Map<String, Bool> = new Map();
  var projectBaseline:Null<Array<SceneObjectData>> = null;
  final demoContent:Bool;
  var staleProjectEdits:Array<String> = [];
  var staleProjectRecord:Null<ProjectSceneRecord> = null;
  public function staleEdits():Array<String> return staleProjectEdits.copy();
  public function discardStaleEdits():Bool {
    if (staleProjectEdits.length == 0) return false;
    var old = staleProjectEdits.copy();
    var oldRecord = staleProjectRecord;
    return document.apply(new EditOperation("Discard stale project edits",
      function() { staleProjectEdits = []; staleProjectRecord = null; },
      function() { staleProjectEdits = old; staleProjectRecord = oldRecord; }));
  }
  /** Application-owned runtime cleanup invoked only after replacement data validates. */
  public var beforeReplace:Null<Void->Void> = null;

  public function new(?initialBim:BimDocument, demoContent:Bool = true) {
    this.demoContent = demoContent;
    document = createDocument();
    edits = new ProjectEditCoordinator(document);
    scene = new EditorScene(demoContent ? null : [], document);
    sensors = new SensorConfiguration(null, document);
    bim = initialBim == null ? new BimDocument() : initialBim;
    bim.cad.clearHistory();
  }

  public function newDocument():Void {
    var nextDocument = createDocument();
    replace(new EditorScene(demoContent ? null : [], nextDocument), new SensorConfiguration(null, nextDocument),
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
        project == null ? null : project.authored, customMaterials),
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
    var root = SceneCodec.parse(text);
    var loadedMaterials = SceneCodec.decodeCustomMaterialsRoot(root);
    var project = SceneCodec.decodeProjectRoot(root);
    if (project != null) {
      openProjectDocument(absolute, root, project, loadedMaterials);
      customMaterials = loadedMaterials;
      return;
    }
    var script=SceneCodec.decodeScriptRoot(root);
    if(script!=null){
      var nextBim = SceneCodec.decodeBimRoot(root);
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
      replace(materialized.scene,materialized.sensors,absolute,ownership,nextBim,nextDocument);
      customMaterials = loadedMaterials;
      return;
    }
    var data = SceneCodec.decodeRoot(root);
    var nextDocument = createDocument();
    var next = new EditorScene(data, nextDocument);
    var nextSensors:SensorConfiguration = null;
    var nextBim:BimDocument;
    try {
      nextSensors = new SensorConfiguration(SceneCodec.decodeSensorsRoot(root), nextDocument);
      nextBim = SceneCodec.decodeBimRoot(root);
    }
    catch (error:Dynamic) { next.dispose(); if (nextSensors != null) nextSensors.dispose(); throw error; }
    replace(next, nextSensors, absolute, null, nextBim, nextDocument);
    customMaterials = loadedMaterials;
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
    if (reference != null) next.configureComponentFinishes(data);
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

  function openProjectDocument(absolute:String, root:Dynamic, project:ProjectSceneRecord,
      materials:Array<MaterialDef>):Void {
    var reference = FilePath.isAbsolute(project.reference) ? project.reference
      : FilePath.join([FilePath.directory(absolute), project.reference]);
    reference = FileSystem.fullPath(reference);
    var generated = MateriaProjectRunner.loadProject(reference);
    var dependentJoints = project.assemblyDependentJoints == null ? [] : project.assemblyDependentJoints.copy();
    validateAssemblyDependentJoints(generated.assemblyDefinition, dependentJoints);
    var stateRecord = generated.assemblyState;
    if (project.assemblyState != null) {
      if (generated.assemblyDefinition == null)
        throw "Generated project state has no kinematic assembly definition";
      stateRecord = AssemblyDefinitionCodec.decodeState(generated.assemblyDefinition, project.assemblyState);
    }
    if (stateRecord != null && generated.assemblyDefinition != null)
      generated = MateriaProjectRunner.evaluateAssemblyState(generated, stateRecord);
    var baseline = generated.objects;
    var diagnostics:Array<String> = [];
    var data = materializeProject(baseline, project, SceneCodec.decodeRoot(root), diagnostics, materials);
    var nextDocument = createDocument();
    var next:EditorScene = null, nextSensors:SensorConfiguration = null, nextBim:BimDocument = null;
    try {
      next = new EditorScene(data, nextDocument, null, generated.geometryBySnapshot);
      next.configureComponentFinishes(baseline);
      nextSensors = new SensorConfiguration(SceneCodec.decodeSensorsRoot(root), nextDocument);
      nextBim = SceneCodec.decodeBimRoot(root);
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
    staleProjectEdits = diagnostics;
    staleProjectRecord = diagnostics.length == 0 ? null : project;
    projectAssembly = generated.assembly;
    installAssemblyRuntime(generated.assemblyDefinition, runtime,
      generated.localCentersByDefinition, generated.metresPerUnit, dependentJoints);
  }

  function configureAssembly(target:EditorScene, definition:Null<AssemblyDefinition>):Void {
    if (definition == null) return;
    var ids:Array<String> = [];
    for (occurrence in definition.occurrences) ids.push(occurrence.id);
    target.configureAssemblyOccurrences(ids, function(id) return assemblyPropertiesForOccurrence(id));
  }

  function installAssemblyRuntime(definition:Null<AssemblyDefinition>, state:Null<AssemblyState>,
      centers:Null<Map<String, Array<Float>>>, metresPerUnit:Float,
      ?dependentJointIds:Array<String>):Void {
    assemblyOccurrenceIds.clear();
    assemblyDependentJoints.clear();
    projectAssemblyDefinition = definition;
    assemblyRuntime = state;
    assemblyLocalCentersByDefinition = centers;
    assemblyMetresPerUnit = metresPerUnit;
    if (definition == null || state == null) {
      projectAssemblyState = null;
      return;
    }
    var dependencies = dependentJointIds == null ? [] : dependentJointIds;
    validateAssemblyDependentJoints(definition, dependencies);
    for (id in dependencies) assemblyDependentJoints.set(id, true);
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
    var hasClosure = false;
    for (joint in definition.joints) if (joint.role == AssemblyJointRole.Closure) hasClosure = true;
    for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree &&
        (joint.parent == occurrenceId || joint.child == occurrenceId) &&
        AssemblyDefinitionCodec.hasCoordinate(joint.type)) {
      if (hasClosure) result.push(assemblyDependentProperty(joint.id));
      result.push(assemblyJointProperty(joint.id, joint.type, joint.limits,
        joint.type == AssemblyJointType.Prismatic ? assemblyMetresPerUnit : 1.0));
    }
    return result;
  }

  function assemblyDependentProperty(jointId:String):PropertyDescriptor {
    var options = new PropertyDescriptorOptions();
    options.category = "Assembly";
    options.recordHistory = false;
    return new PropertyDescriptor("assembly-dependent:" + jointId, "Solved by closures",
      PropertyType.Bool, function(_) return PropertyValue.Bool(assemblyDependentJoints.exists(jointId)),
      function(_, value) switch (value) {
        case PropertyValue.Bool(dependent): setAssemblyJointDependent(jointId, dependent);
        default: throw "Assembly dependent setting must be boolean";
      }, options);
  }

  function assemblyJointProperty(jointId:String, type:AssemblyJointType,
      limits:AssemblyJointLimits, factor:Float):PropertyDescriptor {
    var options = new PropertyDescriptorOptions();
    options.category = "Assembly";
    options.unit = type == AssemblyJointType.Prismatic ? "m" : "rad";
    options.step = type == AssemblyJointType.Prismatic ? Math.max(0.0001, factor) : 0.01;
    options.minimum = limits.lower == null ? null : limits.lower * factor;
    options.maximum = limits.upper == null ? null : limits.upper * factor;
    options.readOnly = assemblyDependentJoints.exists(jointId);
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
    if (assemblyDependentJoints.exists(jointId))
      throw 'Joint "$jointId" is solved from assembly closures and cannot be driven directly';
    if (!Math.isFinite(value)) throw "Joint coordinate must be finite";
    var before = state.record();
    var after = evaluateAssemblyJointCoordinate(before, jointId, value);
    if (sameAssemblyState(before, after)) return false;
    return document.apply(new EditOperation("Set joint " + jointId,
      function() applyAssemblyStateRecord(after),
      function() applyAssemblyStateRecord(before)));
  }

  /** Select whether a tree coordinate is driven by the user or solved from loop closures. */
  public function setAssemblyJointDependent(jointId:String, dependent:Bool):Bool {
    if (projectAssemblyDefinition == null || assemblyRuntime == null)
      throw "This project has no editable assembly definition";
    var wasDependent = assemblyDependentJoints.exists(jointId);
    if (wasDependent == dependent) return false;
    var beforeIds = assemblyDependentJointIds();
    var afterIds = beforeIds.copy();
    if (dependent) afterIds.push(jointId);
    else afterIds.remove(jointId);
    validateAssemblyDependentJoints(projectAssemblyDefinition, afterIds);
    var label = dependent ? "Make joint " + jointId + " dependent" : "Make joint " + jointId + " driven";
    return document.apply(new EditOperation(label,
      function() applyAssemblyDependentJoints(afterIds),
      function() applyAssemblyDependentJoints(beforeIds)));
  }

  function evaluateAssemblyJointCoordinate(stateRecord:AssemblyStateRecord, jointId:String,
      value:Float):AssemblyStateRecord {
    var definition = projectAssemblyDefinition;
    if (definition == null) throw "Assembly definition is unavailable";
    var candidate = new AssemblyState(definition, stateRecord);
    candidate.setJoint(jointId, value);
    return evaluateAssemblyConfigurationState(candidate, assemblyDependentJointIds());
  }

  function evaluateAssemblyConfigurationState(candidate:AssemblyState,
      dependentJointIds:Array<String>):AssemblyStateRecord {
    var definition = projectAssemblyDefinition;
    if (definition == null) throw "Assembly definition is unavailable";
    if (dependentJointIds.length > 0) {
      var result = candidate.solveClosures(dependentJointIds);
      if (!result.converged)
        throw 'Assembly closure solve ${result.status}: ${result.message}; unresolved: ${result.closureIds.join(", ")}';
    } else if (!assemblyClosuresSatisfied(candidate)) {
      throw "Joint edit breaks an assembly closure; mark dependent joints in the inspector to solve the linkage";
    }
    return candidate.record();
  }

  function applyAssemblyDependentJoints(dependentJointIds:Array<String>):Void {
    var definition = projectAssemblyDefinition;
    if (definition == null) throw "Assembly definition is unavailable";
    validateAssemblyDependentJoints(definition, dependentJointIds);
    assemblyDependentJoints.clear();
    for (id in dependentJointIds) assemblyDependentJoints.set(id, true);
    scene.refreshAssemblyProperties();
  }

  function applyAssemblyStateRecord(stateRecord:AssemblyStateRecord):Void {
    var definition = projectAssemblyDefinition;
    var centers = assemblyLocalCentersByDefinition;
    if (definition == null || centers == null)
      throw "Assembly placement data is unavailable";
    var candidate = new AssemblyState(definition, stateRecord);
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
    var materialized=ownership.materialize();
    try scene.reconcileRecords(materialized.scene.records()) catch(error:Dynamic) {
      materialized.scene.dispose(); materialized.sensors.dispose(); throw error;
    }
    materialized.scene.dispose();
    var previousSensors = sensors;
    sensors = materialized.sensors;
    previousSensors.dispose();
    return {scene: scene, sensors: sensors, backend: materialized.backend,
      timestep: materialized.timestep};
  }

  public function save(?file:String):Void {
    var destination = file == null ? path : file;
    if (destination == null) throw "Choose a filename for this scene";
    var absolute = checkedPath(destination);
    var project = projectReference == null ? null : projectSaveData(absolute);
    AtomicFile.write(absolute, SceneCodec.encode(scene, sensors,
      scriptOwnership==null?null:scriptOwnership.record(), bim,
      project == null ? null : project.record,
      project == null ? null : project.authored, customMaterials));
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
    staleProjectEdits = [];
    staleProjectRecord = null;
    projectAssembly = null;
    projectAssemblyDefinition = null;
    projectAssemblyState = null;
    customMaterials = [];
    assemblyRuntime = null;
    assemblyLocalCentersByDefinition = null;
    assemblyMetresPerUnit = 1.0;
    assemblyOccurrenceIds.clear();
    assemblyDependentJoints.clear();
    bim=nextBim;
    path = file;
    generation++;
    previous.dispose();
    previousSensors.dispose();
    if(previousOwnership!=null&&previousOwnership!=nextOwnership)previousOwnership.dispose();
    if(previousBim!=nextBim)previousBim.close();
  }

  /** Applies a BIM mutation with an explicit inverse on the shared project history. */
  public function applyBimEdit(label:String, change:Void->Void, undo:Void->Void):Bool {
    if (label == null || label.length == 0 || change == null || undo == null)
      throw "BIM edits require a label, mutation, and inverse";
    var model = bim;
    var apply = function(action:Void->Void) {
      try action() catch (error:Dynamic) { model.cad.clearHistory(); throw error; }
      model.cad.clearHistory();
    };
    return edits.apply(label, function() apply(change), function() apply(undo));
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
    var overrides:Array<ProjectFieldOverride> = [], instances:Array<ProjectSceneInstance> = [];
    var authored:Array<SceneObjectData> = [];
    for (item in scene.recordsForSave()) {
      var source = sources.get(item.id);
      if (source != null) {
        if (item.type != source.type || item.meshSnapshot != source.meshSnapshot)
          throw 'Generated geometry for "${item.id}" must come from its project source';
        present.set(item.id, true);
        addDeltas(overrides, item.id, item, source, !assemblyOccurrenceIds.exists(item.id));
      } else if (item.type == "cad-preview") {
        var origin:Null<SceneObjectData> = null;
        for (candidate in baseline) if (candidate.meshSnapshot == item.meshSnapshot) {
          origin = candidate;
          break;
        }
        if (origin == null) throw 'CAD preview "${item.id}" has no project source';
        var deltas:Array<ProjectFieldOverride> = [];
        addDeltas(deltas, item.id, item, origin, true);
        instances.push({sourceId: origin.id, id: item.id, overrides: deltas});
      } else authored.push(item);
    }
    var removed:Array<String> = [];
    for (item in baseline) if (!present.exists(item.id)) removed.push(item.id);
    var stale = staleProjectRecord;
    if (stale != null) {
      for (id in stale.removed) if (!sources.exists(id)) removed.push(id);
      for (edit in stale.overrides) if (!sources.exists(edit.targetId)) overrides.push(edit);
      for (instance in stale.instances) if (!sources.exists(instance.sourceId)) instances.push(instance);
    }
    var savedAssemblyState = projectAssemblyDefinition == null || assemblyRuntime == null ? null
      : AssemblyDefinitionCodec.encodeState(projectAssemblyDefinition, assemblyRuntime.record());
    return {record: {version: 1, reference: relativeReference(destination, reference),
      overrides: overrides, removed: removed, instances: instances,
      assemblyState: savedAssemblyState,
      assemblyDependentJoints: projectAssemblyDefinition == null ? null : assemblyDependentJointIds()},
      authored: authored};
  }

  function assemblyDependentJointIds():Array<String> {
    var result:Array<String> = [];
    var definition = projectAssemblyDefinition;
    if (definition == null) return result;
    for (joint in definition.joints)
      if (assemblyDependentJoints.exists(joint.id)) result.push(joint.id);
    return result;
  }

  static function validateAssemblyDependentJoints(definition:Null<AssemblyDefinition>,
      dependentJointIds:Array<String>):Void {
    if (dependentJointIds == null) throw "Assembly dependent-joint list is required";
    if (dependentJointIds.length == 0) return;
    if (definition == null || dependentJointIds.length > definition.joints.length)
      throw "Assembly dependent joints need a matching assembly definition";
    var joints = new Map<String, KinematicJoint>();
    var hasClosure = false;
    for (joint in definition.joints) {
      joints.set(joint.id, joint);
      if (joint.role == AssemblyJointRole.Closure) hasClosure = true;
    }
    if (!hasClosure) throw "Assembly dependent joints require at least one closure";
    var seen = new Map<String, Bool>();
    for (id in dependentJointIds) {
      var joint = joints.get(id);
      if (joint == null || seen.exists(id) || joint.role != AssemblyJointRole.Tree ||
          !AssemblyDefinitionCodec.hasCoordinate(joint.type))
        throw 'Assembly dependent joint "$id" is missing, duplicated, or not a movable tree joint';
      seen.set(id, true);
    }
  }

  static function assemblyClosuresSatisfied(state:AssemblyState):Bool {
    var positionTolerance = 1e-6 / LengthUnit.metresPerUnit(state.definition.lengthUnit == null
      ? "mm" : state.definition.lengthUnit);
    var angularTolerance = 1e-5;
    var axisTolerance = 1 - Math.cos(angularTolerance);
    var rotationTolerance = 1 - Math.cos(angularTolerance * 0.5);
    for (residual in state.closureResiduals())
      if (residual.position > positionTolerance || residual.axis > axisTolerance ||
          residual.rotation > rotationTolerance) return false;
    return true;
  }

  static function sameAssemblyState(first:AssemblyStateRecord, second:AssemblyStateRecord):Bool
    return Json.stringify(first) == Json.stringify(second);

  static var EDITABLE_FIELDS:Array<String> = ["label", "x", "y", "z", "rotation",
    "collisionEnabled", "dynamicBody", "mass", "materialId", "visible"];
  static var VISUAL_FIELDS:Array<String> = ["visual.baseColor", "visual.finish",
    "visual.metallic", "visual.roughness"];

  static function sameValue(property:String, a:Dynamic, b:Dynamic):Bool {
    if (property == "rotation") return sameRotation(cast a, cast b);
    if (Std.isOfType(a, Float) || Std.isOfType(b, Float)) {
      var first:Float = a, second:Float = b;
      return Math.abs(first - second) < 1e-6;
    }
    return a == b;
  }

  static function addDeltas(result:Array<ProjectFieldOverride>, id:String, item:SceneObjectData,
      source:SceneObjectData, includePose:Bool):Void {
    for (property in EDITABLE_FIELDS) {
      if (!includePose && ["x", "y", "z", "rotation"].indexOf(property) >= 0) continue;
      var value = Reflect.field(item, property);
      if (property == "materialId" && value == null) continue;
      if (sameValue(property, value, Reflect.field(source, property))) continue;
      result.push({targetId: id, property: property,
        kind: property == "rotation" ? "vector" :
          Std.isOfType(value, Bool) ? "boolean" : Std.isOfType(value, String) ? "text" : "number",
        value: value});
    }
    if (Math.abs(item.red - source.red) >= 1e-6 || Math.abs(item.green - source.green) >= 1e-6 ||
        Math.abs(item.blue - source.blue) >= 1e-6)
      result.push({targetId: id, property: "visual.baseColor", kind: "vector",
        value: [item.red, item.green, item.blue]});
    var appearance = item.appearance == null ? Appearances.neutral() : item.appearance;
    var baselineAppearance = source.appearance == null ? Appearances.neutral() : source.appearance;
    if (appearance.finish != baselineAppearance.finish)
      result.push({targetId: id, property: "visual.finish", kind: "text", value: appearance.finish});
    if (Math.abs(appearance.metallic - baselineAppearance.metallic) >= 1e-6)
      result.push({targetId: id, property: "visual.metallic", kind: "number", value: appearance.metallic});
    if (Math.abs(appearance.roughness - baselineAppearance.roughness) >= 1e-6)
      result.push({targetId: id, property: "visual.roughness", kind: "number", value: appearance.roughness});
  }

  static function materializeProject(baseline:Array<SceneObjectData>, project:ProjectSceneRecord,
      authored:Array<SceneObjectData>, diagnostics:Array<String>, materials:Array<MaterialDef>):Array<SceneObjectData> {
    var sources = new Map<String, SceneObjectData>();
    for (item in baseline) sources.set(item.id, item);
    var removed = new Map<String, Bool>();
    for (id in project.removed) {
      if (!sources.exists(id)) diagnostics.push("removed:" + id);
      else removed.set(id, true);
    }
    var overrides = new Map<String, Array<ProjectFieldOverride>>();
    for (edit in project.overrides) {
      var id = edit.targetId;
      if (!sources.exists(id)) { diagnostics.push("override:" + id); continue; }
      var list = overrides.get(id);
      if (list == null) { list = []; overrides.set(id, list); }
      list.push(edit);
    }
    var data:Array<SceneObjectData> = [];
    var ids = new Map<String, Bool>();
    for (item in baseline) if (!removed.exists(item.id)) {
      if (ids.exists(item.id)) throw 'Duplicate project part ID: ${item.id}';
      ids.set(item.id, true);
      data.push(applyDeltas(item, item.id, overrides.get(item.id), diagnostics, materials));
    }
    for (instance in project.instances) {
      var source = sources.get(instance.sourceId);
      if (source == null) { diagnostics.push("instance:" + instance.id); continue; }
      if (ids.exists(instance.id)) throw 'Duplicate project object ID: ${instance.id}';
      ids.set(instance.id, true);
      data.push(applyDeltas(source, instance.id, instance.overrides, diagnostics, materials));
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

  static function applyDeltas(source:SceneObjectData, id:String, edits:Null<Array<ProjectFieldOverride>>,
      diagnostics:Array<String>, materials:Array<MaterialDef>):SceneObjectData {
    var value:Dynamic = withoutMesh(source);
    Reflect.setField(value, "id", id);
    if (edits != null) for (edit in edits) {
      if ((EDITABLE_FIELDS.indexOf(edit.property) < 0 && VISUAL_FIELDS.indexOf(edit.property) < 0)
          || edit.targetId != id) {
        diagnostics.push("override:" + id + ":" + edit.property);
        continue;
      }
      var previous:Dynamic = Json.parse(Json.stringify(value));
      switch (edit.property) {
        case "visual.baseColor":
          var color:Array<Dynamic> = cast edit.value;
          if (color != null && color.length == 3) {
            Reflect.setField(value, "red", color[0]);
            Reflect.setField(value, "green", color[1]);
            Reflect.setField(value, "blue", color[2]);
          } else Reflect.setField(value, "red", null);
        case "visual.finish", "visual.metallic", "visual.roughness":
          var appearance:Dynamic = Reflect.field(value, "appearance");
          Reflect.setField(appearance, edit.property.substr(7), edit.value);
        default: Reflect.setField(value, edit.property, edit.value);
      }
      try decodeProjectObject(value, materials) catch (_:Dynamic) {
        value = previous;
        diagnostics.push("override:" + id + ":" + edit.property);
      }
    }
    return decodeProjectObject(value, materials, source.meshSnapshot);
  }

  static function decodeProjectObject(value:Dynamic, materials:Array<MaterialDef>, ?snapshot:String):SceneObjectData {
    Reflect.setField(value, "meshSnapshot", "_");
    var result = SceneCodec.decode(Json.stringify({format: SceneCodec.FORMAT,
      version: SceneCodec.VERSION, materials: MaterialLibrary.all().concat(materials),
      objects: [SceneCodec.encodeObject(cast value, materials)]}))[0];
    result.meshSnapshot = snapshot;
    return result;
  }

  static function withoutMesh(item:SceneObjectData):Dynamic return {
    id: item.id, type: item.type, label: item.label, x: item.x, y: item.y, z: item.z,
    width: item.width, height: item.height, depth: item.depth,
    collisionEnabled: item.collisionEnabled, dynamicBody: item.dynamicBody, mass: item.mass,
    red: item.red, green: item.green, blue: item.blue, appearance: item.appearance == null ? Appearances.neutral() : Json.parse(Json.stringify(item.appearance)),
    materialId: item.materialId,
    visible: item.visible, rotation: item.rotation
  };

  static function sameRotation(left:Null<Array<Float>>, right:Null<Array<Float>>):Bool {
    if (left == null || right == null) return left == right;
    if (left.length != 4 || right.length != 4) return false;
    for (index in 0...4) if (Math.abs(left[index] - right[index]) >= 1e-6) return false;
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
