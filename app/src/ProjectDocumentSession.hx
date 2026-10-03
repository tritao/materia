package app;

import app.CncProgramPlayer.CncJob;
import app.MateriaProjectRunner.GeneratedAssemblyScene;
import materia.project.SceneArtifact.SceneArtifactMission;
import materia.project.SceneArtifact.SceneArtifactMobileBase;
import materia.project.SceneArtifact.SceneArtifactRobotSensor;
import materia.project.SceneArtifact.SceneArtifactRobotTool;

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
import materia.project.Appearance;
import materia.units.LengthUnit;
import materia.project.MaterialDef;
import materia.project.MaterialLibrary;
import materia.assembly.AssemblyRecord;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import cadkit.modeling.AssemblyDrag;
import cadkit.modeling.AssemblyMateDrag;
import cadkit.modeling.AssemblyMateJoints;
import cadkit.modeling.AssemblyMateJoints.AssemblyMateJoint;
import cadkit.modeling.AssemblyMateSolver.AssemblyMateSolveResult;
import materia.assembly.AssemblyDefinition.AssemblyMate;
import materia.assembly.AssemblyDefinition.AssemblyMateKind;
import cadkit.modeling.AssemblyState;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.GeometricConnectors.GeometricConnectorError;
import machinekit.document.MachineKitRecipes;
import nativekit.scene.GeometryData;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalData;

/** Owns the current document; unsuccessful I/O leaves it and its history intact. */
class ProjectDocumentSession {
  public static inline var MAX_HISTORY_OPERATIONS:Int = 1000;
  public static inline var MAX_HISTORY_ESTIMATED_BYTES:Int = 64 * 1024 * 1024;

  public var document(default, null):EditorDocument;
  public var edits(default, null):ProjectEditCoordinator;
  public var scene(default, null):EditorScene;
  public var sensors(default, null):SensorConfiguration;
  public var bim(default, null):BimDocument;
  public var recipeDocument(default, null):Null<cadkit.parametric.Document> = null;
  public var path(default, null):Null<String> = null;
  public var generation(default, null):Int = 0;
  public var scriptOwnership(default,null):Null<ScriptOwnership> = null;
  public var projectReference(default,null):Null<String> = null;
  public var projectAssembly(default,null):Null<AssemblyRecord> = null;
  public var projectAssemblyDefinition(default,null):Null<AssemblyDefinition> = null;
  public var projectAssemblyState(default,null):Null<AssemblyStateRecord> = null;
  public var projectPhysical(default,null):Null<AssemblyPhysicalData> = null;
  public function assemblyPreviewCenter(definitionId:String):Null<Array<Float>> {
    var centers = assemblyLocalCentersByDefinition;
    var value = centers == null ? null : centers.get(definitionId);
    return value == null ? null : value.copy();
  }
  public var customMaterials(default, null):Array<MaterialDef> = [];
  public var robotMotions(default, null):Array<RobotMotionTrack> = [];
  /** Vacuum commands the open project ships with; they are not part of the saved document. */
  /** The project's machining job, run on its machine in the simulation. */
  public var cncJob(default, null):Null<CncJob> = null;
  /** The project's assembly is a wheeled robot that drives on the floor in the simulation. */
  public var mobileBase(default, null):Null<SceneArtifactMobileBase> = null;
  /** Work the project's robot does on its own in the simulation. */
  public var mission(default, null):Null<SceneArtifactMission> = null;
  /** The tools the project's robot works with, simulated on their links. */
  public var robotTools(default, null):Array<SceneArtifactRobotTool> = [];
  /** The sensors on the project's robot, simulated where their parts mount them. */
  public var robotSensors(default, null):Array<SceneArtifactRobotSensor> = [];
  var assemblyRuntime:Null<AssemblyState> = null;
  /** Mates authored over the generated assembly, and the face connectors they name (see `ProjectAssemblyMates`). */
  public var assemblyMates(default, null):ProjectAssemblyMates = ProjectAssemblyMates.empty();
  /** The last placement by the mates: status and diagnosis; null when there are none. */
  public var assemblyMateResult(default, null):Null<AssemblyMateSolveResult> = null;
  /** Why the mates could not be laid over the rebuilt project (a face lost or ambiguous), or null. */
  public var assemblyMateProblem(default, null):Null<String> = null;
  var assemblyFaceDescriptors:Map<String, String> = new Map();
  /** The project's own definition, before the mates overlay; `projectAssemblyDefinition` is it with the overlay laid over. */
  var generatedAssemblyDefinition:Null<AssemblyDefinition> = null;
  var mateJointCache:Null<{key:String, joint:AssemblyMateJoint}> = null;
  var assemblyRevision:Int = 0;
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

  /** The label the project gave each generated object, so callers can tell which ones were renamed. */
  public function generatedLabels():Map<String, String> {
    var result = new Map<String, String>();
    if (projectBaseline != null) for (record in projectBaseline) result.set(record.id, record.label);
    return result;
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

  /** Open a bundled example as an untitled document so Save requests a new path. */
  public function openExample(file:String):Void {
    var absolute = checkedPath(file);
    openContent(absolute, File.getContent(absolute));
    path = null;
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
        project == null ? null : project.authored, customMaterials,
        recipeDocument == null ? null : DocumentCodec.encode(recipeDocument), robotMotions),
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
    var loadedMotions = RobotMotionTrack.decode(Reflect.field(root, "robotMotions"));
    var loadedMaterials = SceneCodec.decodeCustomMaterialsRoot(root);
    var project = SceneCodec.decodeProjectRoot(root);
    if (project != null) {
      openProjectDocument(absolute, root, project, loadedMaterials);
      customMaterials = loadedMaterials;
      robotMotions = loadedMotions;
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
      robotMotions = loadedMotions;
      return;
    }
    var data = SceneCodec.decodeRoot(root);
    rejectLegacyHumans(SceneCodec.decodeSensorsRoot(root));
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
    robotMotions = loadedMotions;
  }

  public function openScript(reference:String):ScriptMaterialization {
    var nextDocument = createDocument();
    var ownership=new ScriptOwnership(reference,null,nextDocument),materialized:ScriptMaterialization;
    try materialized=ownership.materialize() catch(error:Dynamic){ownership.dispose();throw error;}
    replace(materialized.scene,materialized.sensors,null,ownership,new BimDocument(),nextDocument);
    return materialized;
  }

  /** Open everything a project's generator made, retaining its source manifest. */
  public function openGeneratedProject(generated:GeneratedAssemblyScene, manifestPath:String):Void
    openGeneratedScene(generated.objects, manifestPath, generated.assembly,
      generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
      generated.localCentersByDefinition, generated.metresPerUnit,
      generated.physical, generated.recipeDocument, generated.robotMotions,
      generated.faceDescriptorsByDefinition, generated.cncJob, generated.mobileBase, generated.mission, generated.robotTools,
      generated.robotSensors);

  /** Open generated geometry while retaining its source manifest. */
  public function openGeneratedScene(data:Array<SceneObjectData>, ?manifestPath:String,
      ?assembly:AssemblyRecord, ?geometryBySnapshot:Map<String, GeometryData>,
      ?assemblyDefinition:AssemblyDefinition, ?assemblyState:AssemblyStateRecord,
      ?localCentersByDefinition:Map<String, Array<Float>>, metresPerUnit:Float = 1.0,
      ?physical:AssemblyPhysicalData, ?recipeText:String, ?motions:Array<RobotMotionTrack>,
      ?faceDescriptors:Map<String, String>, ?cnc:CncJob,
      ?mobile:SceneArtifactMobileBase, ?work:SceneArtifactMission, ?tools:Array<SceneArtifactRobotTool>,
      ?sensors:Array<SceneArtifactRobotSensor>):Void {
    if (data == null || data.length == 0)
      throw "Generated project preview contains no scene objects";
    var reference = manifestPath == null ? null : FileSystem.fullPath(manifestPath);
    var nextDocument = createDocument();
    var next = new EditorScene(data, nextDocument, null, geometryBySnapshot);
    if (reference != null) next.configureComponentFinishes(data);
    var nextSensors:SensorConfiguration = null;
    var nextBim:BimDocument = null;
    try {
      nextSensors = new SensorConfiguration(null, nextDocument, assemblyDefinition != null);
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
    var nextRecipe = decodeRecipe(recipeText);
    replace(next, nextSensors, null, null, nextBim, nextDocument);
    recipeDocument = nextRecipe;
    projectAssembly = assembly;
    installAssemblyRuntime(assemblyDefinition, runtime, localCentersByDefinition, metresPerUnit, null, faceDescriptors);
    projectPhysical = physical;
    robotMotions = motions == null ? [] : motions.copy();
    cncJob = cnc;
    mobileBase = mobile;
    mission = work;
    robotTools = tools == null ? [] : tools.copy();
    robotSensors = sensors == null ? [] : sensors.copy();
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
    var savedRecipe:Null<String> = Reflect.field(root, "recipeDocument");
    var diagnostics:Array<String> = [];
    var projectRequirement = MateriaProjectRunner.executionRequirement(reference);
    var generated = MateriaProjectRunner.loadProject(reference,
      projectRequirement.reconcilesSavedRecipe ? savedRecipe : null);
    if (projectRequirement.reconcilesSavedRecipe) {
      if (generated.recipeDiagnostics != null)
        for (diagnostic in generated.recipeDiagnostics) diagnostics.push(diagnostic);
    } else if (savedRecipe != null && generated.recipeDocument != null) {
      var reconciled = reconcileRecipe(generated.recipeDocument, savedRecipe, diagnostics);
      if (reconciled.changed)
        generated = MateriaProjectRunner.loadProject(reference, reconciled.text);
      else
        generated.recipeDocument = reconciled.text;
    }
    // The saved state belongs to the definition with the mates overlay laid over it (a mate may have become a joint).
    var generatedDefinition = generated.assemblyDefinition;
    var overlaid = overlayDefinition(generatedDefinition, ProjectAssemblyMates.decode(project.assemblyMates),
      generated.faceDescriptorsByDefinition);
    generated.assemblyDefinition = overlaid.definition;
    // Null means no saved choice (derive from the definition); an empty list is an explicit choice of none.
    var dependentJoints = project.assemblyDependentJoints == null ? null : project.assemblyDependentJoints.copy();
    if (dependentJoints != null) validateAssemblyDependentJoints(generated.assemblyDefinition, dependentJoints);
    var stateRecord = generated.assemblyState;
    if (project.assemblyState != null) {
      if (generated.assemblyDefinition == null)
        throw "Generated project state has no kinematic assembly definition";
      stateRecord = AssemblyDefinitionCodec.decodeState(generated.assemblyDefinition, project.assemblyState);
    }
    if (stateRecord != null && generated.assemblyDefinition != null)
      generated = MateriaProjectRunner.evaluateAssemblyState(generated, stateRecord);
    var baseline = generated.objects;
    var data = materializeProject(baseline, project, SceneCodec.decodeRoot(root), diagnostics, materials);
    rejectLegacyHumans(SceneCodec.decodeSensorsRoot(root));
    var nextDocument = createDocument();
    var next:EditorScene = null, nextSensors:SensorConfiguration = null, nextBim:BimDocument = null;
    try {
      next = new EditorScene(data, nextDocument, null, generated.geometryBySnapshot);
      next.configureComponentFinishes(baseline);
      nextSensors = new SensorConfiguration(SceneCodec.decodeSensorsRoot(root), nextDocument,
        generated.assemblyDefinition != null);
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
    var nextRecipe = decodeRecipe(generated.recipeDocument);
    replace(next, nextSensors, absolute, null, nextBim, nextDocument);
    recipeDocument = nextRecipe;
    projectReference = reference;
    projectBaseline = baseline;
    staleProjectEdits = diagnostics;
    staleProjectRecord = diagnostics.length == 0 ? null : project;
    projectAssembly = generated.assembly;
    installAssemblyRuntime(generated.assemblyDefinition, runtime,
      generated.localCentersByDefinition, generated.metresPerUnit, dependentJoints, generated.faceDescriptorsByDefinition,
      overlaid.overlay, generatedDefinition, overlaid.problem);
    projectPhysical = generated.physical;
    robotMotions = generated.robotMotions == null ? [] : generated.robotMotions.copy();
    cncJob = generated.cncJob;
    mobileBase = generated.mobileBase;
    mission = generated.mission;
    robotTools = generated.robotTools == null ? [] : generated.robotTools.copy();
    robotSensors = generated.robotSensors == null ? [] : generated.robotSensors.copy();
  }

  function configureAssembly(target:EditorScene, definition:Null<AssemblyDefinition>):Void {
    if (definition == null) return;
    var ids:Array<String> = [];
    for (occurrence in definition.occurrences) ids.push(occurrence.id);
    target.configureAssemblyOccurrences(ids, function(id) return assemblyPropertiesForOccurrence(id),
      function(id, point) return beginAssemblyDrag(id, point));
  }

  function installAssemblyRuntime(definition:Null<AssemblyDefinition>, state:Null<AssemblyState>,
      centers:Null<Map<String, Array<Float>>>, metresPerUnit:Float,
      ?dependentJointIds:Array<String>, ?faceDescriptors:Map<String, String>, ?mates:ProjectAssemblyMates,
      ?generated:AssemblyDefinition, ?mateProblem:String):Void {
    assemblyOccurrenceIds.clear();
    assemblyFaceDescriptors = faceDescriptors == null ? new Map() : faceDescriptors;
    assemblyMates = mates == null ? ProjectAssemblyMates.empty() : mates;
    generatedAssemblyDefinition = generated == null ? definition : generated;
    assemblyMateResult = null;
    assemblyMateProblem = mateProblem;
    assemblyRevision++;
    assemblyDependentJoints.clear();
    projectAssemblyDefinition = definition;
    assemblyRuntime = state;
    assemblyLocalCentersByDefinition = centers;
    assemblyMetresPerUnit = metresPerUnit;
    if (definition == null || state == null) {
      projectAssemblyState = null;
      return;
    }
    // Without a saved choice, the joints the definition's driven inputs leave dependent; a saved list overrides.
    var dependencies = dependentJointIds == null ? state.dependentJoints() : dependentJointIds;
    validateAssemblyDependentJoints(definition, dependencies);
    for (id in dependencies) assemblyDependentJoints.set(id, true);
    for (occurrence in definition.occurrences) assemblyOccurrenceIds.set("project:" + occurrence.id, true);
    projectAssemblyState = state.record();
    projectAssembly = MateriaProjectRunner.legacySnapshot(definition, state);
    if (assemblyMates.mates.length > 0 && assemblyMateProblem == null) layMates(state);
  }

  /**
   * The generated definition with `overlay` laid over it, its face connectors framed again from the rebuilt
   * project's descriptors. When the overlay no longer fits (a face lost, a component gone), `problem` says why and
   * the definition is the generated one (or the overlay with its last frames).
   */
  static function overlayDefinition(generated:Null<AssemblyDefinition>, overlay:ProjectAssemblyMates,
      descriptors:Null<Map<String, String>>):{definition:Null<AssemblyDefinition>, overlay:ProjectAssemblyMates, problem:Null<String>} {
    if (generated == null || overlay.isEmpty()) return {definition: generated, overlay: overlay, problem: null};
    var framed = overlay, problem:Null<String> = null;
    try framed = overlay.reframe(generated, descriptors == null ? new Map() : descriptors)
    catch (error:Dynamic) problem = describeMateError(error);
    try return {definition: framed.effective(generated), overlay: framed, problem: problem}
    catch (error:Dynamic) return {definition: generated, overlay: framed, problem: describeMateError(error)};
  }

  static function describeMateError(error:Dynamic):String
    return Std.isOfType(error, GeometricConnectorError) ? (cast error : GeometricConnectorError).toString() : Std.string(error);

  /** Places the parts by the mates after a (re)build (not an edit: it follows the source). */
  function layMates(state:AssemblyState):Void {
    var generated = generatedAssemblyDefinition;
    if (generated == null) return;
    try {
      var solved = assemblyMates.solve(generated, state.record());
      assemblyMateResult = solved.result;
      if (solved.result.converged) applyAssemblyStateRecord(solved.state);
    } catch (error:Dynamic) {
      assemblyMateProblem = describeMateError(error);
    }
  }

  /**
   * The joint the mates of part `sceneId` amount to (see `AssemblyMateJoints.infer`), or null when the project has
   * no assembly or the part is not a root occurrence. Cached until the assembly next changes.
   */
  public function assemblyMateJoint(sceneId:String):Null<AssemblyMateJoint> {
    var definition = projectAssemblyDefinition, state = assemblyRuntime;
    if (definition == null || state == null || assemblyMates.mates.length == 0) return null;
    var key = sceneId + "@" + assemblyRevision;
    var cached = mateJointCache;
    if (cached != null && cached.key == key) return cached.joint;
    var occurrence = try mateOccurrence(definition, sceneId) catch (_:Dynamic) null;
    var joint = occurrence == null ? null : try AssemblyMateJoints.infer(definition, state.record(), occurrence.id) catch (_:Dynamic) null;
    mateJointCache = joint == null ? null : {key: key, joint: joint};
    return joint;
  }

  /**
   * Replaces the mates of part `sceneId` by the joint they amount to (a turn makes a revolute joint, a slide a
   * prismatic one), as one undoable edit; returns the joint's id. The part keeps its place (the joint starts at 0)
   * and from then on moves, drags and simulates on the joint.
   */
  public function convertMatesToJoint(sceneId:String):String {
    var definition = requireMateDefinition(), state = assemblyRuntime, generated = generatedAssemblyDefinition;
    if (state == null || generated == null) throw "This project has no editable assembly state";
    var occurrence = mateOccurrence(definition, sceneId);
    var before = state.record();
    var inferred = AssemblyMateJoints.infer(definition, before, occurrence.id);
    if (inferred.type == null) throw 'The mates of "${occurrence.id}" make no joint: ${inferred.reason}';
    var jointId = freshJointId(definition, "joint-" + occurrence.id);
    var conversion = AssemblyMateJoints.convert(definition, before, inferred, jointId);
    var after = assemblyMates.withJoint(conversion), afterDefinition = after.effective(generated);
    var coordinates = [for (coordinate in before.jointCoordinates) {joint: coordinate.joint, value: coordinate.value}];
    coordinates.push({joint: jointId, value: 0.0});
    var afterState:AssemblyStateRecord = {schemaVersion: before.schemaVersion, definition: before.definition,
      jointCoordinates: coordinates, rootPoses: [for (root in before.rootPoses) if (root.occurrence != occurrence.id) root]};
    AssemblyDefinitionCodec.validateState(afterDefinition, afterState);
    var overlayBefore = assemblyMates, definitionBefore = definition;
    document.apply(new EditOperation("Make the mates of " + occurrence.id + " a joint",
      function() setAssemblyStructure(after, afterDefinition, afterState),
      function() setAssemblyStructure(overlayBefore, definitionBefore, before)));
    return jointId;
  }

  /** Installs an overlay whose joints changed the assembly's structure, with a state of the new definition. */
  function setAssemblyStructure(overlay:ProjectAssemblyMates, definition:AssemblyDefinition, state:AssemblyStateRecord):Void {
    assemblyMates = overlay;
    assemblyMateProblem = null;
    projectAssemblyDefinition = definition;
    applyAssemblyStateRecord(state);
    var generated = generatedAssemblyDefinition;
    assemblyMateResult = overlay.mates.length == 0 || generated == null ? null : overlay.solve(generated, state).result;
    scene.refreshAssemblyProperties();
    generation++;
  }

  static function freshJointId(definition:AssemblyDefinition, base:String):String {
    var taken = new Map<String, Bool>();
    for (joint in definition.joints) taken.set(joint.id, true);
    if (definition.mates != null) for (mate in definition.mates) taken.set(mate.id, true);
    if (!taken.exists(base)) return base;
    var index = 2;
    while (taken.exists(base + "-" + index)) index++;
    return base + "-" + index;
  }

  /**
   * Mates face `firstFace` of occurrence `first` (a scene id, `project:<occurrence>`) to face `secondFace` of
   * `second`, as one undoable edit; returns the mate's id. The faces become connectors of their components
   * (captured from the project's face descriptors), and the parts move to satisfy every mate. A mate that
   * cannot hold is kept, the placement unchanged, and `assemblyMateResult` says what conflicts.
   */
  public function addAssemblyFaceMate(kind:AssemblyMateKind, first:String, firstFace:Int, second:String, secondFace:Int,
      ?value:Float):String {
    var definition = requireMateDefinition();
    var firstOccurrence = mateOccurrence(definition, first), secondOccurrence = mateOccurrence(definition, second);
    var firstSide = assemblyMates.faceConnector(firstOccurrence.definition, firstFace,
      assemblyFaceDescriptors.get(firstOccurrence.definition));
    var secondSide = firstSide.overlay.faceConnector(secondOccurrence.definition, secondFace,
      assemblyFaceDescriptors.get(secondOccurrence.definition));
    return addMateTo(secondSide.overlay, kind, firstOccurrence.id, firstSide.name, secondOccurrence.id, secondSide.name, value);
  }

  /** Mates two existing connectors (named on the components of `first` and `second`), as one undoable edit. */
  public function addAssemblyConnectorMate(kind:AssemblyMateKind, first:String, firstConnector:String, second:String,
      secondConnector:String, ?value:Float):String {
    var definition = requireMateDefinition();
    return addMateTo(assemblyMates, kind, mateOccurrence(definition, first).id, firstConnector,
      mateOccurrence(definition, second).id, secondConnector, value);
  }

  /** Removes mate `id` (and the face connectors only it named), as one undoable edit; the parts stay where they are. */
  public function removeAssemblyMate(id:String):Bool {
    requireMateDefinition();
    var state = assemblyRuntime, generated = generatedAssemblyDefinition;
    var before = assemblyMates, after = assemblyMates.withoutMate(id);
    var beforeResult = assemblyMateResult;
    var afterResult = after.mates.length == 0 || state == null || generated == null ? null : after.solve(generated, state.record()).result;
    return document.apply(new EditOperation("Remove mate " + id,
      function() setMates(after, afterResult, null),
      function() setMates(before, beforeResult, null)));
  }

  /** The feature face `faceIndex` of part `sceneId` offers a mate (from the project's face descriptors), or null. */
  public function assemblyFaceFeature(sceneId:String, faceIndex:Int):Null<cadkit.parametric.GeometricConnectors.GeometricFeatureKind> {
    var definition = projectAssemblyDefinition;
    if (definition == null) return null;
    var occurrence = try mateOccurrence(definition, sceneId) catch (_:Dynamic) null;
    if (occurrence == null) return null;
    return ProjectAssemblyMates.describedFeature(assemblyFaceDescriptors.get(occurrence.definition), faceIndex);
  }

  /** Whether faces of this project's parts can be mated (it has an assembly and described faces). */
  public function canMateFaces():Bool {
    if (projectAssemblyDefinition == null || assemblyRuntime == null) return false;
    for (_ in assemblyFaceDescriptors.keys()) return true;
    return false;
  }

  /** Whether the mates still let part `sceneId` move (see `AssemblyMateSolveResult.movable`). */
  public function assemblyPartStillFree(sceneId:String):Bool {
    var result = assemblyMateResult;
    return result != null && StringTools.startsWith(sceneId, "project:") && result.movable.indexOf(sceneId.substr(8)) >= 0;
  }

  /** A one-line account of the mates for the status bar, or null when there are none. */
  public function assemblyMateStatus():Null<String> {
    var problem = assemblyMateProblem;
    if (problem != null) return "Mates: " + problem;
    var result = assemblyMateResult;
    if (result == null) return null;
    var report = result.report;
    if (!result.converged) {
      var conflicting = report.conflictingOwners();
      return conflicting.length > 0 ? "Mates conflict: " + conflicting.join(", ") : "Mates: " + result.message;
    }
    var redundant = result.implied;
    var free = report.degreesOfFreedom == 0 ? "fully placed"
      : report.degreesOfFreedom + " degrees of freedom free" + (result.movable.length > 0 ? " (" + result.movable.join(", ") + ")" : "");
    return "Mates: " + free + (redundant.length > 0 ? "; redundant: " + redundant.join(", ") : "") +
      (result.degenerate ? " (singular placement)" : "");
  }

  function addMateTo(overlay:ProjectAssemblyMates, kind:AssemblyMateKind, first:String, firstConnector:String,
      second:String, secondConnector:String, value:Null<Float>):String {
    var definition = requireMateDefinition(), state = assemblyRuntime;
    if (state == null) throw "This project has no editable assembly state";
    var mate:AssemblyMate = {id: overlay.freshId(kind + "-"), kind: kind, first: first, firstConnector: firstConnector,
      second: second, secondConnector: secondConnector, axis: {x: 0, y: 0, z: 1}};
    if (value != null) mate.value = value;
    var after = overlay.withMate(mate), generated = generatedAssemblyDefinition;
    if (generated == null) throw "This project has no assembly to mate";
    var solved = after.solve(generated, state.record());
    var before = assemblyMates, beforeResult = assemblyMateResult, beforeState = state.record();
    var afterState = solved.result.converged ? solved.state : beforeState;
    document.apply(new EditOperation("Add " + kind + " mate",
      function() setMates(after, solved.result, afterState),
      function() setMates(before, beforeResult, beforeState)));
    return mate.id;
  }

  function setMates(overlay:ProjectAssemblyMates, result:Null<AssemblyMateSolveResult>, state:Null<AssemblyStateRecord>):Void {
    assemblyMates = overlay;
    assemblyMateResult = result;
    assemblyMateProblem = null;
    var generated = generatedAssemblyDefinition, runtime = assemblyRuntime;
    if (generated != null) projectAssemblyDefinition = overlay.effective(generated);
    if (state != null) applyAssemblyStateRecord(state);
    else if (runtime != null) applyAssemblyStateRecord(runtime.record());
    scene.refreshAssemblyProperties();
  }

  function requireMateDefinition():AssemblyDefinition {
    var definition = projectAssemblyDefinition;
    if (definition == null || assemblyRuntime == null) throw "This project has no assembly to mate";
    return definition;
  }

  /** The root occurrence a scene id names; parts inside nested assemblies cannot be mated yet. */
  static function mateOccurrence(definition:AssemblyDefinition, sceneId:String):AssemblyComponentOccurrence {
    var id = StringTools.startsWith(sceneId, "project:") ? sceneId.substr(8) : sceneId;
    for (occurrence in definition.occurrences) if (occurrence.id == id) {
      if (occurrence.assembly != null) throw 'Mate a part inside "$id", not the sub-assembly itself';
      return occurrence;
    }
    throw 'There is no assembly part "$id" to mate';
  }

  function assemblyPropertiesForOccurrence(sceneId:String):Array<PropertyDescriptor> {
    var result:Array<PropertyDescriptor> = [];
    if (recipeDocument != null && StringTools.startsWith(sceneId, "project:")) {
      var occurrenceId = sceneId.substr(8);
      for (element in recipeDocument.allElements()) {
        var identity = element.property("machinekit.occurrence");
        if (element.kind != "instance" || identity == null || identity.value != occurrenceId) continue;
        result = result.concat(BimInspectorDescriptors.forInstanceInputs(cast element,
          function(label, change, undo) applyRecipeEdit(label, change)));
        break;
      }
    }
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
    for (mate in assemblyMates.mates) if (mate.first == occurrenceId || mate.second == occurrenceId)
      result.push(assemblyMateProperty(mate));
    return result;
  }

  /** A mate on the selected part, as a checked box: clearing it removes the mate (one undoable edit). */
  function assemblyMateProperty(mate:AssemblyMate):PropertyDescriptor {
    var options = new PropertyDescriptorOptions();
    options.category = "Mates";
    options.recordHistory = false;
    var label = mate.kind + ": " + mate.first + "." + mate.firstConnector + " ↔ " + mate.second + "." + mate.secondConnector;
    var id = mate.id;
    return new PropertyDescriptor("assembly-mate:" + id, label, PropertyType.Bool,
      function(_) {
        for (current in assemblyMates.mates) if (current.id == id) return PropertyValue.Bool(true);
        return PropertyValue.Bool(false);
      },
      function(_, value) switch (value) {
        case PropertyValue.Bool(keep): if (!keep) removeAssemblyMate(id);
        default: throw "A mate's setting must be boolean";
      }, options);
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

  /**
   * Starts an IK drag of an assembly occurrence (`project:<id>`) from a world point in metres: the
   * point follows the targets given to `update`, moving the joints above it and the dependent joints
   * (so closures stay closed). Returns null when nothing can move it.
   */
  public function beginAssemblyDrag(sceneId:String, worldPoint:Array<Float>):Null<SceneAssemblyDrag> {
    var state = assemblyRuntime;
    var definition = projectAssemblyDefinition;
    if (state == null || definition == null || !StringTools.startsWith(sceneId, "project:")) return null;
    if (worldPoint == null || worldPoint.length != 3) throw "Assembly drag needs a world point";
    var occurrence = sceneId.substr(8);
    var unit = assemblyMetresPerUnit;
    var pose = state.worldPose(occurrence);
    var local = AssemblyFrames.transformPoint(AssemblyFrames.inverse(pose),
      worldPoint[0] / unit, worldPoint[1] / unit, worldPoint[2] / unit);
    // A part with mates moves as they allow (and holds them); otherwise its joints carry it.
    var mated = false;
    for (mate in assemblyMates.mates) if (mate.first == occurrence || mate.second == occurrence) mated = true;
    if (mated) {
      // The runtime definition already has the mates overlay laid over it.
      var mateDrag = try new AssemblyMateDrag(definition, state.record(), occurrence,
        new kinematicskit.Vector3(local.x, local.y, local.z)) catch (_:Dynamic) null;
      if (mateDrag != null) return new ProjectMateDrag(this, mateDrag, state.record(), unit);
    }
    var drag = try new AssemblyDrag(state, occurrence, null, assemblyDependentJointIds(), false,
      new kinematicskit.Vector3(local.x, local.y, local.z)) catch (_:Dynamic) null;
    if (drag == null) return null;
    return new ProjectAssemblyDrag(this, drag, state.record(), unit);
  }

  @:allow(app.ProjectAssemblyDrag)
  function previewAssemblyDrag(drag:AssemblyDrag):Void
    previewAssemblyPoses(drag.previewPose);

  /** Shows each occurrence at `poseOf(occurrence)` without recording history (a drag's preview). */
  @:allow(app.ProjectMateDrag)
  function previewAssemblyPoses(poseOf:String->AssemblyFrame):Void {
    var definition = projectAssemblyDefinition;
    var centers = assemblyLocalCentersByDefinition;
    if (definition == null || centers == null) throw "Assembly placement data is unavailable";
    var transforms:Array<{id:String, x:Float, y:Float, z:Float, rotation:Array<Float>}> = [];
    for (occurrence in definition.occurrences) {
      var center = centers.get(occurrence.definition);
      if (center == null || center.length != 3)
        throw 'Assembly component "${occurrence.definition}" has no local preview center';
      var pose = poseOf(occurrence.id);
      var world = AssemblyFrames.transformPoint(pose, center[0], center[1], center[2]);
      transforms.push({id: "project:" + occurrence.id, x: world.x * assemblyMetresPerUnit,
        y: world.y * assemblyMetresPerUnit, z: world.z * assemblyMetresPerUnit,
        rotation: [pose.qx, pose.qy, pose.qz, pose.qw]});
    }
    scene.setAssemblyOccurrenceTransforms(transforms);
  }

  @:allow(app.ProjectAssemblyDrag)
  @:allow(app.ProjectMateDrag)
  function finishAssemblyDrag(label:String, before:AssemblyStateRecord, after:Null<AssemblyStateRecord>):Bool {
    if (after == null || sameAssemblyState(before, after)) {
      applyAssemblyStateRecord(before);
      return false;
    }
    applyAssemblyStateRecord(after);
    document.record(new EditOperation(label,
      function() applyAssemblyStateRecord(after),
      function() applyAssemblyStateRecord(before)));
    return true;
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
    assemblyRevision++;
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

  /**
   * Scenes from before workers were scene objects kept their people in the sensors section. That
   * section is no longer read, and dropping it silently would lose the workers, so say so.
   */
  static function rejectLegacyHumans(sensors:Dynamic):Void {
    if (sensors == null || !Reflect.hasField(sensors, "humans")) return;
    var raw:Dynamic = Reflect.field(sensors, "humans");
    if (Std.isOfType(raw, Array) && (cast raw:Array<Dynamic>).length == 0) return;
    throw 'This scene keeps its people in a "humans" section, which is no longer supported; workers are scene objects now';
  }

  public function save(?file:String):Void {
    var destination = file == null ? path : file;
    if (destination == null) throw "Choose a filename for this scene";
    var absolute = checkedPath(destination);
    var project = projectReference == null ? null : projectSaveData(absolute);
    AtomicFile.write(absolute, SceneCodec.encode(scene, sensors,
      scriptOwnership==null?null:scriptOwnership.record(), bim,
      project == null ? null : project.record,
      project == null ? null : project.authored, customMaterials,
      recipeDocument == null ? null : DocumentCodec.encode(recipeDocument), robotMotions));
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
    var previousRecipe=recipeDocument;
    document = nextDocument;
    edits = new ProjectEditCoordinator(nextDocument);
    scene = next;
    sensors = nextSensors;
    scriptOwnership=nextOwnership;
    projectReference = null;
    recipeDocument = null;
    projectBaseline = null;
    staleProjectEdits = [];
    staleProjectRecord = null;
    projectAssembly = null;
    projectAssemblyDefinition = null;
    projectAssemblyState = null;
    projectPhysical = null;
    customMaterials = [];
    robotMotions = [];
    cncJob = null;
    mobileBase = null;
    mission = null;
    robotTools = [];
    robotSensors = [];
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
    if(previousRecipe!=null){MachineKitRecipes.forget(previousRecipe);previousRecipe.close();}
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

  /** Edit a generated part and rebuild its geometry within the project history. */
  public function applyRecipeEdit(label:String, change:Void->Void):Bool {
    var recipe = recipeDocument;
    if (recipe == null || projectReference == null) throw "This project has no editable recipe document";
    var first = true;
    try {
      return edits.apply(label, function() {
      if (first) { change(); first = false; }
      else if (!recipe.redo()) throw "Recipe redo history is out of sync";
      refreshRecipeScene();
    }, function() {
      if (!recipe.undo()) throw "Recipe undo history is out of sync";
      refreshRecipeScene();
      });
    } catch (error:Dynamic) {
      if (!first) { recipe.undo(); try refreshRecipeScene() catch (_:Dynamic) {} }
      throw error;
    }
  }

  function refreshRecipeScene():Void {
    var recipe = recipeDocument, reference = projectReference;
    if (recipe == null || reference == null) throw "This project has no editable recipe document";
    var saved = projectSaveData(path == null ? reference + ".materia" : path);
    var generated = MateriaProjectRunner.loadProject(reference, DocumentCodec.encode(recipe));
    var generatedDefinition = generated.assemblyDefinition;
    var overlaid = overlayDefinition(generatedDefinition, assemblyMates, generated.faceDescriptorsByDefinition);
    generated.assemblyDefinition = overlaid.definition;
    var stateRecord = generated.assemblyState;
    if (saved.record.assemblyState != null && generated.assemblyDefinition != null)
      stateRecord = AssemblyDefinitionCodec.decodeState(generated.assemblyDefinition, saved.record.assemblyState);
    if (stateRecord != null && generated.assemblyDefinition != null)
      generated = MateriaProjectRunner.evaluateAssemblyState(generated, stateRecord);
    var diagnostics:Array<String> = [];
    var data = materializeProject(generated.objects, saved.record, saved.authored, diagnostics, customMaterials);
    scene.refreshGenerated(data, generated.geometryBySnapshot);
    projectBaseline = generated.objects;
    projectAssembly = generated.assembly;
    installAssemblyRuntime(generated.assemblyDefinition,
      generated.assemblyDefinition == null ? null : new AssemblyState(generated.assemblyDefinition, stateRecord),
      generated.localCentersByDefinition, generated.metresPerUnit, saved.record.assemblyDependentJoints,
      generated.faceDescriptorsByDefinition, overlaid.overlay, generatedDefinition, overlaid.problem);
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
    var record:ProjectSceneRecord = {version: 1, reference: relativeReference(destination, reference),
      overrides: overrides, removed: removed, instances: instances,
      assemblyState: savedAssemblyState,
      assemblyDependentJoints: savedDependentJoints()};
    if (!assemblyMates.isEmpty()) record.assemblyMates = assemblyMates.encode();
    return {record: record, authored: authored};
  }

  /** The dependent joints to save: null when they are what the definition derives, so later source changes apply. */
  function savedDependentJoints():Null<Array<String>> {
    if (projectAssemblyDefinition == null || assemblyRuntime == null) return null;
    var chosen = assemblyDependentJointIds();
    return chosen.join(",") == assemblyRuntime.dependentJoints().join(",") ? null : chosen;
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
    var rawAppearance:Dynamic = Reflect.field(value, "appearance");
    var appearance:Appearance = rawAppearance == null ? Appearances.neutral() : {
      finish: Reflect.field(rawAppearance, "finish"),
      metallic: Reflect.field(rawAppearance, "metallic"),
      roughness: Reflect.field(rawAppearance, "roughness")};
    var rawRotation:Dynamic = Reflect.field(value, "rotation");
    var rotation:Null<Array<Float>> = null;
    if (rawRotation != null) {
      var components:Array<Dynamic> = cast rawRotation;
      rotation = [];
      for (component in components) {
        var coordinate:Float = component;
        rotation.push(coordinate);
      }
    }
    var candidate:SceneObjectData = {
      id: Reflect.field(value, "id"), type: Reflect.field(value, "type"),
      label: Reflect.field(value, "label"), x: Reflect.field(value, "x"),
      y: Reflect.field(value, "y"), z: Reflect.field(value, "z"),
      width: Reflect.field(value, "width"), height: Reflect.field(value, "height"),
      depth: Reflect.field(value, "depth"),
      collisionEnabled: Reflect.field(value, "collisionEnabled"),
      dynamicBody: Reflect.field(value, "dynamicBody"), mass: Reflect.field(value, "mass"),
      red: Reflect.field(value, "red"), green: Reflect.field(value, "green"),
      blue: Reflect.field(value, "blue"), visible: Reflect.field(value, "visible"),
      materialId: Reflect.field(value, "materialId"), appearance: appearance,
      rotation: rotation, cadGraph: null, meshSnapshot: "_", sketchDraft: null
    };
    var result = SceneCodec.decode(Json.stringify({format: SceneCodec.FORMAT,
      version: SceneCodec.VERSION, materials: MaterialLibrary.all().concat(materials),
      objects: [SceneCodec.encodeObject(candidate, materials)]}))[0];
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

  public function dispose():Void { scene.dispose(); sensors.dispose();if(scriptOwnership!=null)scriptOwnership.dispose();bim.close();if(recipeDocument!=null){MachineKitRecipes.forget(recipeDocument);recipeDocument.close();} }

  static function decodeRecipe(text:Null<String>):Null<cadkit.parametric.Document> {
    if (text == null) return null;
    MachineKitRecipes.register();
    return DocumentCodec.decode(text);
  }

  static function reconcileRecipe(freshText:String, savedText:String,
      diagnostics:Array<String>):{text:String, changed:Bool}
    return MachineKitRecipes.reconcile(freshText, savedText, diagnostics);

  static function createDocument():EditorDocument
    return new EditorDocument("project", new EditHistory(MAX_HISTORY_OPERATIONS,
      MAX_HISTORY_ESTIMATED_BYTES));
}
