package app;

import haxe.Json;
import haxe.crypto.Sha256;
import haxe.io.Bytes;
import haxe.io.Path as ProjectPath;
import sys.io.AtomicFile;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.project.MaterialLibrary;
import materia.project.MeshMassProperties;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyRecord.AssemblyConnector;
import materia.assembly.AssemblyRecord.AssemblyInstance;
import materia.assembly.AssemblyRecord.AssemblyJoint;
import materia.assembly.AssemblyRecord;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyDefinitionCodec;
import nativekit.scene.GeometryData;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalData;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart;
import sys.FileSystem;
import sys.io.File;
#if !wasm
import sys.io.Process;
#end
import sys.thread.Mutex;
#if !wasm
import sys.thread.Thread;
#end

/** Resolves a Materia project entrypoint and materializes its generated viewport geometry. */
class MateriaProjectRunner {
  static final MAX_OUTPUT_BYTES:Int = 150000000;

  /** Inspect a project without compiling or executing its entrypoint. */
  public static function executionRequirement(projectPath:String):ProjectExecutionRequirement {
    var manifestPath = FileSystem.fullPath(projectPath);
    if (!FileSystem.exists(manifestPath) || FileSystem.isDirectory(manifestPath))
      throw 'Materia project file not found: $manifestPath';
    var root:Dynamic = Json.parse(File.getContent(manifestPath));
    if (fieldText(root, "format") != "materia.project" || fieldInt(root, "version") != 1)
      throw "Unsupported Materia project format";
    var build = field(root, "build");
    if (fieldText(build, "system") != "haxeon") throw "Materia project requires the Haxeon build system";
    var entries = field(root, "entrypoints");
    var entryId = fieldText(root, "defaultEntrypoint");
    var entry = Reflect.field(entries, entryId);
    if (entry == null) throw 'Materia project has no default entrypoint "$entryId"';
    if (fieldText(entry, "kind") != "cad-preview")
      throw "Unsupported Materia viewport entrypoint kind";
    var reconcilesSavedRecipe = Reflect.field(entry, "reconcilesSavedRecipe") == true;
    if (reconcilesSavedRecipe && Reflect.field(entry, "documentInput") != true)
      throw "Project recipe reconciliation requires a document input";
    return {kind: "requires-project-code", projectPath: manifestPath,
      entrypoint: entryId, module: fieldText(entry, "module"),
      reconcilesSavedRecipe: reconcilesSavedRecipe};
  }

  public static function load(projectPath:String):Array<SceneObjectData> return loadProject(projectPath).objects;

  public static function loadProject(projectPath:String, ?recipeDocument:String,
      ?control:ProjectLoadControl):GeneratedAssemblyScene {
    var scene = loadGeneratedProject(projectPath, recipeDocument, control);
    var manifestPath = FileSystem.fullPath(projectPath);
    var motion = motionDocument(manifestPath);
    scene.robotMotions = projectMotions(motion, scene);
    scene.robotGrips = RobotGripEvent.decode(motion == null ? null : Reflect.field(motion, "grips"));
    applyDynamicParts(manifestPath, scene);
    return scene;
  }

  /** The project's motion file, or null when its manifest names none. */
  static function motionDocument(manifestPath:String):Dynamic {
    var root:Dynamic = Json.parse(File.getContent(manifestPath));
    var reference:Dynamic = Reflect.field(root, "robotMotions");
    if (reference == null) return null;
    if (!Std.isOfType(reference, String) || StringTools.trim(reference).length == 0)
      throw 'Project field "robotMotions" must name a file';
    var file = resolveProjectPath(directory(manifestPath), reference);
    if (!FileSystem.exists(file) || FileSystem.isDirectory(file))
      throw 'Project robot motion file not found: $file';
    var document:Dynamic = Json.parse(File.getContent(file));
    if (Reflect.field(document, "version") != 1 || !Std.isOfType(Reflect.field(document, "tracks"), Array))
      throw "Unsupported project robot motion file";
    return document;
  }

  /**
   * The manifest's optional `dynamicParts` lists parts that the simulation moves freely instead of
   * bolting to their assembly, such as a workpiece; they need mass and must not be joined to anything.
   */
  static function applyDynamicParts(manifestPath:String, scene:GeneratedAssemblyScene):Void {
    var root:Dynamic = Json.parse(File.getContent(manifestPath));
    var listed:Dynamic = Reflect.field(root, "dynamicParts");
    if (listed == null) return;
    if (!Std.isOfType(listed, Array)) throw 'Project field "dynamicParts" must be a list of part ids';
    for (id in (cast listed:Array<Dynamic>)) {
      if (!Std.isOfType(id, String)) throw 'Project field "dynamicParts" must be a list of part ids';
      var found = false;
      for (record in scene.objects) if (record.id == "project:" + id) {
        record.dynamicBody = true;
        found = true;
      }
      if (!found) throw 'Project dynamic part "$id" is not a part of the generated scene';
    }
  }

  /**
   * Joint motion the project ships with. The manifest's optional `robotMotions` names a JSON file
   * `{"version": 1, "tracks": [{"joint": "<assembly joint id>", "loop": true, "keys": [...]}],
   * "grips": [{"time": 1.4, "link": "<assembly link id>", "action": "grip"}]}` beside it. Positions
   * are joint coordinates relative to the generated initial pose, like every motion track, and the
   * tracks drive the project's own assembly in a simulation.
   */
  static function projectMotions(document:Dynamic, scene:GeneratedAssemblyScene):Array<RobotMotionTrack> {
    if (document == null) return [];
    var definition = scene.assemblyDefinition;
    if (definition == null) throw "Project robot motions need a kinematic assembly";
    var raw:Array<Dynamic> = [for (track in (cast Reflect.field(document, "tracks"):Array<Dynamic>)) {
      version: 1, robotId: "assembly:" + definition.id, jointId: Reflect.field(track, "joint"),
      loop: Reflect.field(track, "loop"), keys: Reflect.field(track, "keys")
    }];
    return RobotMotionTrack.decode(raw);
  }

  static function loadGeneratedProject(projectPath:String, ?recipeDocument:String,
      ?control:ProjectLoadControl):GeneratedAssemblyScene {
    var manifestPath = FileSystem.fullPath(projectPath);
    if (!FileSystem.exists(manifestPath) || FileSystem.isDirectory(manifestPath))
      throw 'Materia project file not found: $manifestPath';
    var projectRoot = directory(manifestPath), root:Dynamic;
    try root = Json.parse(File.getContent(manifestPath))
    catch (error:Dynamic) throw "Could not parse Materia project " + manifestPath + ": " + Std.string(error);
    if (fieldText(root, "format") != "materia.project" || fieldInt(root, "version") != 1)
      throw "Unsupported Materia project format";
    var build = field(root, "build");
    if (fieldText(build, "system") != "haxeon") throw "Materia project requires the Haxeon build system";
    var haxeonManifest = resolveProjectPath(projectRoot, fieldText(build, "manifest"));
    if (!FileSystem.exists(haxeonManifest) || FileSystem.isDirectory(haxeonManifest))
      throw 'Haxeon project manifest not found: $haxeonManifest';
    var entrypoints = field(root, "entrypoints"),
      entryId = fieldText(root, "defaultEntrypoint"),
      entry:Dynamic = Reflect.field(entrypoints, entryId);
    if (entry == null) throw 'Materia project has no default entrypoint "$entryId"';
    var kind = fieldText(entry, "kind");
    if (kind != "cad-preview") throw 'Unsupported Materia viewport entrypoint kind: $kind';

    var installation = installationRoot();
    var home = ProjectPath.join([installation, "haxeon"]);
    var haxe = ProjectPath.join([home, ".tools", "haxe", "haxe"]);
    if (!FileSystem.exists(haxe)) throw 'Pinned Haxe compiler was not found at $haxe';
    var tools = projectToolsDirectory();
    if (control != null) control.phase("Checking for a cached build");
    var cache = recipeDocument == null ? cachePath(entry, projectRoot, manifestPath, haxeonManifest,
      fieldText(entry, "module"), fieldText(entry, "function"), haxe, home, tools, control) : null;
    if (control != null) control.throwIfCancelled();
    if (cache != null && FileSystem.exists(cache)) {
      if (control != null) control.phase("Reading the cached build");
      try {
        var metadata = FileSystem.metadata(cache);
        if (metadata == null || metadata.size > MAX_OUTPUT_BYTES) throw "Cached artifact is too large";
        var cached = previewRecords(File.getBytes(cache));
        Sys.println('Materia project: loaded ${cached.objects.length} components from cache');
        return cached;
      } catch (_:Dynamic) {
        try FileSystem.deleteFile(cache) catch (_:Dynamic) {}
      }
    }
    var tempRootValue = Sys.getEnv("TMPDIR");
    var temporaryRoot = ProjectPath.join([tempRootValue == null ? "/tmp" : tempRootValue,
      "materia-project-" + Sys.getPid() + "-" + Std.string(Sys.time())
        + "-" + Std.random(1000000000)]);
    FileSystem.createDirectory(temporaryRoot);
    var outputPrefix = ProjectPath.join([temporaryRoot, "preview"]);
    var records:GeneratedAssemblyScene;
    try {
      Sys.println("Materia project: compiling its entrypoint");
      if (control != null) control.phase("Compiling the project");
      var acceptsDocument = Reflect.field(entry, "documentInput") == true;
      if (recipeDocument != null && !acceptsDocument)
        throw "Project entrypoint does not accept an editable document";
      buildModule(haxe, home, tools, haxeonManifest, fieldText(entry, "module"),
        fieldText(entry, "function"), outputPrefix, acceptsDocument, control);
      if (control != null) control.throwIfCancelled();
      Sys.println("Materia project: executing its entrypoint");
      if (control != null) control.phase("Running the project");
      var hashlink = ProjectPath.join([home, ".tools", "hashlink", "hl"]);
      if (!FileSystem.exists(hashlink)) throw 'Pinned HashLink executable was not found at $hashlink';
      if (recipeDocument != null) File.saveContent(outputPrefix + ".document.json", recipeDocument);
      runCommand(hashlink, recipeDocument == null ? [outputPrefix + ".hl", outputPrefix + ".mtrg"]
        : [outputPrefix + ".hl", outputPrefix + ".mtrg", outputPrefix + ".document.json"],
        "Could not generate Materia project artifact", control);
      if (control != null) control.throwIfCancelled();
      if (!FileSystem.exists(outputPrefix + ".mtrg"))
        throw "Project entrypoint did not write a geometry artifact";
      var metadata = FileSystem.metadata(outputPrefix + ".mtrg");
      if (metadata == null || metadata.size > MAX_OUTPUT_BYTES)
        throw "Project geometry preview exceeds the 150 MB limit";
      var result = File.getBytes(outputPrefix + ".mtrg");
      Sys.println('Materia project: received binary geometry artifact (${result.length} bytes)');
      if (result.length > MAX_OUTPUT_BYTES) throw "Project geometry preview exceeds the 150 MB limit";
      if (control != null) control.phase("Reading the geometry");
      records = previewRecords(result);
      Sys.println('Materia project: decoded ${records.objects.length} component records');
      if (cache != null) try AtomicFile.writeBytes(cache, result)
        catch (error:Dynamic) Sys.println("Materia project: could not save cache: " + Std.string(error));
    } catch (error:Dynamic) {
      cleanupArtifacts(outputPrefix, temporaryRoot);
      throw error;
    }
    cleanupArtifacts(outputPrefix, temporaryRoot);
    return records;
  }

  static function cachePath(entry:Dynamic, projectRoot:String, manifestPath:String,
      haxeonManifest:String, module:String, functionName:String, haxe:String,
      home:String, tools:String, ?control:ProjectLoadControl):Null<String> {
    if (entry == null || !Reflect.hasField(entry, "cache")) return null;
    var declaration = field(entry, "cache");
    var rawInputs = field(declaration, "inputs");
    if (!Std.isOfType(rawInputs, Array)) throw "Project cache inputs must be an array of file paths";
    var inputs:Array<String> = [manifestPath];
    var declaredInputs:Array<Dynamic> = rawInputs;
    for (input in declaredInputs) {
      if (!Std.isOfType(input, String))
        throw "Project cache input must be a file path";
      var path:String = input;
      if (StringTools.trim(path).length == 0) throw "Project cache input must be a file path";
      inputs.push(resolveProjectPath(projectRoot, path));
    }
    var cacheRoot = Sys.getEnv("XDG_CACHE_HOME");
    if (cacheRoot == null || cacheRoot.length == 0) {
      var userHome = Sys.getEnv("HOME");
      if (userHome == null || userHome.length == 0) return null;
      cacheRoot = ProjectPath.join([userHome, ".cache"]);
    }
    var cacheDirectory = ProjectPath.join([cacheRoot, "materia", "generated-artifacts"]);
    if (!ensureCacheDirectory(cacheDirectory)) return null;
    var arguments = ["--cwd", home, "-cp", ProjectPath.join([home, "src"]), "-cp", tools,
      "--run", "MateriaProjectFingerprint", haxeonManifest, module, functionName, tools, home].concat(inputs);
    var fingerprint = StringTools.trim(runCommand(haxe, arguments,
      "Could not fingerprint Materia project entrypoint", control));
    if (!Sha256Digest.isHex(fingerprint)) throw "Project fingerprint has an invalid result";
    return ProjectPath.join([cacheDirectory, fingerprint + ".mtrg"]);
  }

  static function ensureCacheDirectory(path:String):Bool {
    if (FileSystem.exists(path)) return FileSystem.isDirectory(path);
    var parent = ProjectPath.directory(path);
    if (parent == path || parent.length == 0 || !ensureCacheDirectory(parent)) return false;
    try FileSystem.createDirectory(path) catch (_:Dynamic) return false;
    return FileSystem.exists(path) && FileSystem.isDirectory(path);
  }

  static function cleanupArtifacts(outputPrefix:String, temporaryRoot:String):Void {
    for (suffix in [".hl", ".mtrg", ".document.json"]) {
      var artifact = outputPrefix + suffix;
      if (FileSystem.exists(artifact)) FileSystem.deleteFile(artifact);
    }
    if (FileSystem.exists(temporaryRoot)) FileSystem.deleteDirectory(temporaryRoot);
  }

  static function buildModule(haxe:String, home:String, tools:String, manifest:String,
      module:String, functionName:String, outputPrefix:String, documentInput:Bool,
      ?control:ProjectLoadControl):Void {
    var arguments = ["--cwd", home, "-cp", ProjectPath.join([home, "src"]), "-cp", tools,
      "--run", "MateriaProjectModuleBuild", manifest, module, functionName, outputPrefix, home,
      documentInput ? "true" : "false"];
    runCommand(haxe, arguments, "Could not compile Materia project entrypoint", control);
  }

  public static function runCommand(command:String, arguments:Array<String>, description:String,
      ?control:ProjectLoadControl):String {
    #if wasm
    // Projects are compiled by child processes, which the browser cannot start. The guard keeps callers'
    // following statements reachable for the compiler's no-return analysis.
    if (command.length >= 0) throw description + ": external commands are not available in the browser build";
    return "";
    #else
    var process:Process;
    try process = Process.run(command, arguments)
    catch (error:Dynamic) throw description + ": " + Std.string(error);
    if (control != null) control.attach(process);
    var mutex = new Mutex();
    var stdout = "", stderr = "", stdoutDone = false, stderrDone = false, processError:Dynamic = null;
    Thread.create(function() {
      var error:Dynamic = null;
      try stdout = process.readStdout() catch (caught:Dynamic) error = caught;
      mutex.acquire();
      if (error != null) processError = error;
      stdoutDone = true;
      mutex.release();
    });
    Thread.create(function() {
      var error:Dynamic = null;
      try stderr = process.readStderr() catch (caught:Dynamic) error = caught;
      mutex.acquire();
      if (error != null && processError == null) processError = error;
      stderrDone = true;
      mutex.release();
    });
    while (true) {
      mutex.acquire();
      var done = stdoutDone && stderrDone;
      mutex.release();
      if (done) break;
      Sys.sleep(0.001);
    }
    var status = process.exitCode();
    process.close();
    if (control != null) {
      control.detach();
      control.throwIfCancelled();
    }
    if (processError != null)
      throw description + ": " + Std.string(processError);
    if (status != 0) {
      var details = StringTools.trim(stderr + "\n" + stdout);
      if (details.length > 6000) details = details.substr(details.length - 6000);
      throw '$description (exit $status):\n$details';
    }
    return stdout;
    #end
  }

  static function previewRecords(snapshot:Bytes):GeneratedAssemblyScene {
    var artifact = SceneArtifact.decode(snapshot);
    var artifactHash = digestHex(Sha256.make(snapshot));
    var records:Array<SceneObjectData> = [];
    var geometryBySnapshot:Map<String, GeometryData> = new Map();
    var scale = artifact.metresPerUnit;
    var poses:Map<String, AssemblyFrame> = new Map();
    var componentUseCount = new Map<String, Int>();
    var resolvedAssembly = artifact.assembly;
    var runtimeState:Null<AssemblyState> = null;
    if (artifact.assemblyDefinition != null) {
      runtimeState = new AssemblyState(artifact.assemblyDefinition, artifact.assemblyState);
      for (occurrence in artifact.assemblyDefinition.occurrences) {
        poses.set(occurrence.id, runtimeState.worldPose(occurrence.id));
        var count = componentUseCount.get(occurrence.definition);
        componentUseCount.set(occurrence.definition, count == null ? 1 : count + 1);
      }
      resolvedAssembly = legacySnapshot(artifact.assemblyDefinition, runtimeState);
    } else if (artifact.assembly != null) {
      for (instance in artifact.assembly.instances) poses.set(instance.id, instance.pose);
    }

    var boundsByDefinition:Map<String, {minimum:Array<Float>, maximum:Array<Float>}> = new Map();
    var geometryKeyByDefinition:Map<String, String> = new Map();
    var localCentersByDefinition:Map<String, Array<Float>> = new Map();
    var physicalParts:Array<AssemblyPhysicalPart> = [];
    for (component in artifact.parts) {
      var label = component.name;
      var minimum = [1e300, 1e300, 1e300], maximum = [-1e300, -1e300, -1e300];
      for (vertex in 0...component.vertexCount) for (axis in 0...3) {
        var coordinate = component.vertices.getDouble(vertex * 24 + axis * 8);
        if (!Math.isFinite(coordinate)) throw 'Project component "$label" has a non-finite vertex';
        minimum[axis] = Math.min(minimum[axis], coordinate);
        maximum[axis] = Math.max(maximum[axis], coordinate);
      }
      var geometry = CadPreviewGeometry.fromArtifact(component, minimum, maximum, scale);
      var geometryKey = "materia.artifact-part/1:" + artifactHash + ":" + component.id;
      geometryBySnapshot.set(geometryKey, geometry);
      boundsByDefinition.set(component.id, {minimum: minimum, maximum: maximum});
      geometryKeyByDefinition.set(component.id, geometryKey);
      localCentersByDefinition.set(component.id, [
        (minimum[0] + maximum[0]) * 0.5,
        (minimum[1] + maximum[1]) * 0.5,
        (minimum[2] + maximum[2]) * 0.5]);
      var properties = MeshMassProperties.compute(component.vertices, component.indices);
      var materialId = component.materialId == null ? "neutral" : component.materialId;
      var collision = cadkit.ConvexHullVertices.safeFromMesh(component.vertices,
        component.vertexCount, 0.0005 / scale);
      physicalParts.push({id: component.id, materialId: materialId,
        collisionHull: collision.vertices, collisionWarning: collision.warning,
        collisionErrorRatio: collision.errorRatio,
        volume: component.volume == null ? properties.volume : component.volume,
        centerOfMass: component.centerOfMass == null ? properties.centerOfMass : component.centerOfMass.copy(),
        inertia: component.inertia == null ? properties.inertia : component.inertia.copy(),
        density: component.materialDensity == null
          ? MaterialLibrary.require(materialId).physical.density : component.materialDensity});
    }

    if (artifact.assemblyDefinition != null) {
      var parts = new Map<String, SceneArtifactPart>();
      for (part in artifact.parts) parts.set(part.id, part);
      for (occurrence in artifact.assemblyDefinition.occurrences) {
        var component = parts.get(occurrence.definition);
        if (component == null) throw 'Missing geometry definition "${occurrence.definition}"';
        addOccurrenceRecord(records, component, occurrence.id, occurrence.definition,
          poses.get(occurrence.id), componentUseCount.get(occurrence.definition),
          boundsByDefinition, geometryKeyByDefinition, scale);
      }
    } else if (artifact.assembly != null) {
      // A distinct name from the map above: the compiler keeps a local map's facts only when it is declared once.
      var partsByInstance = new Map<String, SceneArtifactPart>();
      for (part in artifact.parts) partsByInstance.set(part.id, part);
      for (instance in artifact.assembly.instances) {
        var component = partsByInstance.get(instance.id);
        if (component == null) continue;
        addOccurrenceRecord(records, component, instance.id, instance.id, poses.get(instance.id), 1,
          boundsByDefinition, geometryKeyByDefinition, scale);
      }
    } else {
      for (component in artifact.parts)
        addOccurrenceRecord(records, component, component.id, component.id, null, 1,
          boundsByDefinition, geometryKeyByDefinition, scale);
    }

    return {objects: records, assembly: resolvedAssembly,
      geometryBySnapshot: geometryBySnapshot,
      assemblyDefinition: artifact.assemblyDefinition,
      assemblyState: runtimeState == null ? null : runtimeState.record(),
      localCentersByDefinition: localCentersByDefinition,
      metresPerUnit: scale, physical: {metresPerUnit: scale, parts: physicalParts},
      recipeDocument: artifact.recipeDocument, recipeDiagnostics: artifact.recipeDiagnostics};
  }

  /** Re-evaluate generated occurrence placements for a project-owned configuration. */
  public static function evaluateAssemblyState(generated:GeneratedAssemblyScene,
      stateRecord:AssemblyStateRecord):GeneratedAssemblyScene {
    var definition = generated.assemblyDefinition;
    if (definition == null) throw "Generated project has no kinematic assembly definition";
    var state = new AssemblyState(definition, stateRecord);
    var objects:Array<SceneObjectData> = [];
    var byId:Map<String, SceneObjectData> = new Map();
    for (source in generated.objects) {
      var copy:SceneObjectData = {id: source.id, label: source.label, type: source.type,
        x: source.x, y: source.y, z: source.z, width: source.width, height: source.height,
        depth: source.depth, collisionEnabled: source.collisionEnabled, dynamicBody: source.dynamicBody,
        mass: source.mass, red: source.red, green: source.green, blue: source.blue,
        appearance: source.appearance, materialId: source.materialId,
        visible: source.visible, rotation: source.rotation == null ? null : source.rotation.copy(),
        cadGraph: source.cadGraph, meshSnapshot: source.meshSnapshot, sketchDraft: source.sketchDraft};
      objects.push(copy);
      byId.set(copy.id, copy);
    }
    for (occurrence in definition.occurrences) {
      var item = byId.get("project:" + occurrence.id);
      var center = generated.localCentersByDefinition.get(occurrence.definition);
      if (item == null || center == null || center.length != 3)
        throw 'Generated project is missing preview placement data for "${occurrence.id}"';
      var pose = state.worldPose(occurrence.id);
      var worldCenter = AssemblyFrames.transformPoint(pose, center[0], center[1], center[2]);
      item.x = worldCenter.x * generated.metresPerUnit;
      item.y = worldCenter.y * generated.metresPerUnit;
      item.z = worldCenter.z * generated.metresPerUnit;
      item.rotation = [pose.qx, pose.qy, pose.qz, pose.qw];
    }
    return {objects: objects, assembly: legacySnapshot(definition, state),
      geometryBySnapshot: generated.geometryBySnapshot, assemblyDefinition: definition,
      assemblyState: state.record(), localCentersByDefinition: generated.localCentersByDefinition,
      metresPerUnit: generated.metresPerUnit, physical: generated.physical,
      recipeDocument: generated.recipeDocument, recipeDiagnostics: generated.recipeDiagnostics,
      robotMotions: generated.robotMotions, robotGrips: generated.robotGrips};
  }

  static function addOccurrenceRecord(records:Array<SceneObjectData>, component:SceneArtifactPart,
      occurrenceId:String, definitionId:String, pose:Null<AssemblyFrame>, useCount:Null<Int>,
      boundsByDefinition:Map<String, {minimum:Array<Float>, maximum:Array<Float>}>,
      geometryKeyByDefinition:Map<String, String>, scale:Float):Void {
    var bounds = boundsByDefinition.get(definitionId);
    if (bounds == null) throw 'Missing bounds for geometry definition "$definitionId"';
    var minimum = bounds.minimum, maximum = bounds.maximum;
    var centerX = (minimum[0] + maximum[0]) * 0.5;
    var centerY = (minimum[1] + maximum[1]) * 0.5;
    var centerZ = (minimum[2] + maximum[2]) * 0.5;
    var center = pose == null ? {x: centerX, y: centerY, z: centerZ}
      : AssemblyFrames.transformPoint(pose, centerX, centerY, centerZ);
    var label = useCount != null && useCount > 1 ? component.name + " · " + occurrenceId : component.name;
    var volume = component.volume == null
      ? MeshMassProperties.compute(component.vertices, component.indices).volume : component.volume;
    var materialId = component.materialId == null ? "neutral" : component.materialId;
    var density = component.materialDensity == null
      ? MaterialLibrary.require(materialId).physical.density : component.materialDensity;
    var mass = volume * scale * scale * scale * density;
    if (!Math.isFinite(mass) || mass <= 0) throw 'Invalid mass for generated part "$occurrenceId"';
    records.push({id: "project:" + occurrenceId, label: label, type: "cad-preview",
      x: center.x * scale, y: center.y * scale, z: center.z * scale,
      width: Math.max(0.000001, (maximum[0] - minimum[0]) * scale),
      height: Math.max(0.000001, (maximum[1] - minimum[1]) * scale),
      depth: Math.max(0.000001, (maximum[2] - minimum[2]) * scale),
      collisionEnabled: true, dynamicBody: false, mass: mass,
      red: component.red, green: component.green, blue: component.blue,
      appearance: component.appearance, materialId: component.materialId, visible: true,
      meshSnapshot: geometryKeyByDefinition.get(definitionId),
      rotation: pose == null ? null : [pose.qx, pose.qy, pose.qz, pose.qw]});
  }

  public static function legacySnapshot(definition:AssemblyDefinition, state:AssemblyState):AssemblyRecord {
    var definitions = new Map<String, AssemblyComponentDefinition>();
    var instances:Array<AssemblyInstance> = [];
    for (component in definition.definitions) definitions.set(component.id, component);
    for (occurrence in definition.occurrences) {
      var component = definitions.get(occurrence.definition);
      if (component == null) throw 'Assembly occurrence "${occurrence.id}" has no component definition';
      var connectors:Array<AssemblyConnector> = [];
      for (connector in component.connectors) connectors.push({name: connector.name, frame: connector.frame});
      instances.push({id: occurrence.id, pose: state.worldPose(occurrence.id), connectors: connectors});
    }
    var joints:Array<AssemblyJoint> = [];
    // Put tree edges first so the legacy tree view cannot mistake a closure for a parent edge.
    for (role in [AssemblyJointRole.Tree, AssemblyJointRole.Closure])
      for (joint in definition.joints) if (joint.role == role) {
        var value = AssemblyDefinitionCodec.hasCoordinate(joint.type) && role == AssemblyJointRole.Tree
          ? state.joint(joint.id) : 0.0;
        joints.push({id: joint.id, kind: joint.type, parent: joint.parent,
          parentConnector: joint.parentConnector, child: joint.child,
          childConnector: joint.childConnector, value: value});
      }
    return {instances: instances, joints: joints};
  }

  static function projectToolsDirectory():String {
    return ProjectPath.join([installationRoot(), "app", "tools"]);
  }

  public static function installationRoot():String {
    var configured = Sys.getEnv("MATERIA_INSTALL_ROOT");
    var current = configured == null || configured.length == 0
      ? ProjectPath.directory(FileSystem.fullPath(Sys.executablePath()))
      : FileSystem.fullPath(configured);
    while (true) {
      var tools = ProjectPath.join([current, "app", "tools", "MateriaProjectModuleBuild.hx"]);
      var compiler = ProjectPath.join([current, "haxeon", ".tools", "haxe", "haxe"]);
      if (FileSystem.exists(tools) && FileSystem.exists(compiler)) return current;
      var parent = ProjectPath.directory(current);
      if (parent == current || parent.length == 0) break;
      current = parent;
    }
    throw "Materia installation is missing app/tools or the pinned Haxeon toolchain";
  }

  static function digestHex(bytes:Bytes):String {
    var digits = "0123456789abcdef", result = new StringBuf();
    for (index in 0...bytes.length) {
      var value = bytes.get(index);
      result.add(digits.charAt(value >>> 4));
      result.add(digits.charAt(value & 15));
    }
    return result.toString();
  }

  static function resolveProjectPath(root:String, relative:String):String
    return ProjectPath.isAbsolute(relative) ? relative : ProjectPath.join([root, relative]);

  static function directory(path:String):String {
    var separator = path.lastIndexOf("/");
    if (separator < 0) return "";
    return separator == 0 ? "/" : path.substr(0, separator);
  }

  static function field(value:Dynamic, name:String):Dynamic {
    if (value == null || !Reflect.hasField(value, name)) throw 'Project entry is missing "$name"';
    return Reflect.field(value, name);
  }

  static function fieldText(value:Dynamic, name:String):String {
    var result:Dynamic = field(value, name);
    if (!Std.isOfType(result, String))
      throw 'Project field "$name" must be non-empty text';
    var text:String = result;
    if (StringTools.trim(text).length == 0)
      throw 'Project field "$name" must be non-empty text';
    return text;
  }

  static function fieldInt(value:Dynamic, name:String):Int {
    var result:Dynamic = field(value, name);
    if (!Std.isOfType(result, Int)) throw 'Project field "$name" must be an integer';
    var number:Int = result;
    return number;
  }

}

typedef ProjectExecutionRequirement = {
  var kind:String;
  var projectPath:String;
  var entrypoint:String;
  var module:String;
  var reconcilesSavedRecipe:Bool;
}

typedef GeneratedAssemblyScene = {
  var objects:Array<SceneObjectData>;
  var assembly:Null<AssemblyRecord>;
  var geometryBySnapshot:Map<String, GeometryData>;
  var assemblyDefinition:Null<AssemblyDefinition>;
  var assemblyState:Null<AssemblyStateRecord>;
  var localCentersByDefinition:Map<String, Array<Float>>;
  var metresPerUnit:Float;
  var physical:AssemblyPhysicalData;
  var recipeDocument:Null<String>;
  @:optional var recipeDiagnostics:Array<String>;
  /** Joint motion the project ships with, applied to its own assembly in simulation. */
  @:optional var robotMotions:Array<RobotMotionTrack>;
  /** Vacuum commands that go with the motion: which tool grips or lets go, and when. */
  @:optional var robotGrips:Array<RobotGripEvent>;
}
