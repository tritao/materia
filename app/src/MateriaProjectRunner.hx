package app;

import toolpathkit.tool.Tool;


import haxe.Json;
import haxe.crypto.Sha256;
import haxe.io.Bytes;
import haxe.io.Path as ProjectPath;
import sys.io.AtomicFile;
import materia.project.SceneArtifact;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import cadkit.modeling.AssemblyState;
import sys.FileSystem;
import sys.io.File;
#if !wasm
import sys.io.Process;
#end
import sys.thread.Mutex;
#if !wasm
import sys.thread.Thread;
#end

/** A closed triangle mesh whose triangles share vertices, in a part's frame and length unit. */


/** Resolves a Materia project entrypoint and materializes its generated viewport geometry. */
class MateriaProjectRunner {
  static final MAX_OUTPUT_BYTES:Int = 150000000;
  static final projectToolMutex:Mutex = new Mutex();

  /** Inspect a project without compiling or executing its entrypoint. */
  public static function executionRequirement(projectPath:String, ?jobId:String):ProjectExecutionRequirement {
    var manifestPath = FileSystem.fullPath(projectPath);
    if (!FileSystem.exists(manifestPath) || FileSystem.isDirectory(manifestPath))
      throw 'Materia project file not found: $manifestPath';
    var root:Dynamic = Json.parse(File.getContent(manifestPath));
    if (fieldText(root, "format") != "materia.project" || fieldInt(root, "version") != 1)
      throw "Unsupported Materia project format";
    var build = field(root, "build");
    if (fieldText(build, "system") != "haxeon") throw "Materia project requires the Haxeon build system";
    var entries = field(root, "entrypoints");
    var entryId = ProjectJobs.entrypoint(root, jobId);
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
      ?control:ProjectLoadControl, ?jobId:String):GeneratedAssemblyScene {
    var scene = loadGeneratedProject(projectPath, recipeDocument, control, jobId);
    var manifestPath = FileSystem.fullPath(projectPath);
    var selected = ProjectJobs.selected(Json.parse(File.getContent(manifestPath)), jobId);
    scene.projectJob = selected == null ? null : selected.id;
    var motion = motionDocument(manifestPath);
    scene.robotMotions = projectMotions(motion, scene);
    applyDynamicParts(manifestPath, scene);
    if (control != null) control.throwIfCancelled();
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
   * `{"version": 1, "tracks": [{"joint": "<assembly joint id>", "loop": true, "keys": [...]}]}`
   * beside it. Positions are joint coordinates relative to the generated initial pose, like every
   * motion track, and the tracks drive the project's own assembly in a simulation. Work with a
   * tool (picking, placing) is a mission of skills, not timed commands beside the tracks.
   */
  static function projectMotions(document:Dynamic, scene:GeneratedAssemblyScene):Array<RobotMotionTrack> {
    if (document == null) return [];
    if (Reflect.hasField(document, "grips"))
      throw "Project motion grips are no longer supported: pick and place with the scene's mission";
    var definition = scene.assemblyDefinition;
    if (definition == null) throw "Project robot motions need a kinematic assembly";
    var raw:Array<Dynamic> = [for (track in (cast Reflect.field(document, "tracks"):Array<Dynamic>)) {
      version: 1, robotId: "assembly:" + definition.id, jointId: Reflect.field(track, "joint"),
      loop: Reflect.field(track, "loop"), keys: Reflect.field(track, "keys")
    }];
    return RobotMotionTrack.decode(raw);
  }

  static function loadGeneratedProject(projectPath:String, ?recipeDocument:String,
      ?control:ProjectLoadControl, ?jobId:String):GeneratedAssemblyScene {
    return previewRecords(generateArtifact(projectPath, recipeDocument, control, jobId), control);
  }

  /** Produces immutable portable bytes; publishing needs no native scene handles. */
  public static function generateArtifact(projectPath:String, ?recipeDocument:String,
      ?control:ProjectLoadControl, ?jobId:String):Bytes {
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
      entryId = ProjectJobs.entrypoint(root, jobId),
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
    var buildCache = buildCachePrefix(entry, projectRoot, manifestPath, haxeonManifest,
      fieldText(entry, "module"), fieldText(entry, "function"), haxe, home, tools, control);
    // Compiled code is reusable even when the entrypoint's output is not deterministic.
    var cache = recipeDocument == null && Reflect.hasField(entry, "cache") && buildCache != null
      ? buildCache + ".mtrg" : null;
    if (control != null) control.throwIfCancelled();
    if (cache != null && FileSystem.exists(cache)) {
      if (control != null) control.phase("Reading the cached build");
      try {
        var metadata = FileSystem.metadata(cache);
        if (metadata == null || metadata.size > MAX_OUTPUT_BYTES) throw "Cached artifact is too large";
        var cached = File.getBytes(cache);
        SceneArtifact.decodeView(cached);
        Sys.println("Materia project: reused its generated artifact");
        return cached;
      } catch (_:Dynamic) {
        if (control != null) control.throwIfCancelled();
        try FileSystem.deleteFile(cache) catch (_:Dynamic) {}
      }
    }
    var tempRootValue = Sys.getEnv("TMPDIR");
    var temporaryRoot = ProjectPath.join([tempRootValue == null ? "/tmp" : tempRootValue,
      "materia-project-" + Sys.getPid() + "-" + Std.string(Sys.time())
        + "-" + Std.random(1000000000)]);
    FileSystem.createDirectory(temporaryRoot);
    var outputPrefix = ProjectPath.join([temporaryRoot, "preview"]);
    var records:Bytes;
    try {
      var acceptsDocument = Reflect.field(entry, "documentInput") == true;
      if (recipeDocument != null && !acceptsDocument)
        throw "Project entrypoint does not accept an editable document";
      var compiledCache = buildCache == null ? null : buildCache + ".hl";
      if (compiledCache != null && FileSystem.exists(compiledCache)) {
        if (control != null) control.phase("Reading the compiled generator");
        File.saveBytes(outputPrefix + ".hl", File.getBytes(compiledCache));
        Sys.println("Materia project: reused its compiled entrypoint");
      } else {
        Sys.println("Materia project: compiling its entrypoint");
        if (control != null) control.phase("Compiling the project");
        buildModule(haxe, home, tools, haxeonManifest, fieldText(entry, "module"),
          fieldText(entry, "function"), outputPrefix, acceptsDocument, control);
        if (control != null) control.throwIfCancelled();
        if (compiledCache != null) try AtomicFile.writeBytes(compiledCache, File.getBytes(outputPrefix + ".hl"))
          catch (error:Dynamic) Sys.println("Materia project: could not save compiled generator: " + Std.string(error));
      }
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
      SceneArtifact.decodeView(result);
      records = result;
      if (cache != null) try AtomicFile.writeBytes(cache, result)
        catch (error:Dynamic) Sys.println("Materia project: could not save cache: " + Std.string(error));
    } catch (error:Dynamic) {
      cleanupArtifacts(outputPrefix, temporaryRoot);
      throw error;
    }
    cleanupArtifacts(outputPrefix, temporaryRoot);
    return records;
  }

  /** Resolves source-only runtime settings into a self-contained downloadable artifact. */
  public static function exportArtifact(projectPath:String, ?jobId:String):Bytes {
    var path = FileSystem.fullPath(projectPath);
    var artifact = SceneArtifact.decodeView(generateArtifact(path, null, null, jobId));
    var manifest:Dynamic = Json.parse(File.getContent(path));
    var selected = ProjectJobs.selected(manifest, jobId);
    var motion:Dynamic = motionDocument(path);
    var dynamicParts:Dynamic = Reflect.field(manifest, "dynamicParts");
    var parts:Array<Dynamic> = dynamicParts == null ? [] : cast dynamicParts;
    var tracks:Array<Dynamic> = motion == null ? [] : cast Reflect.field(motion, "tracks");
    artifact.project = SceneArtifact.decodeProject({job: selected == null ? null : selected.id,
      dynamicParts: parts, motions: tracks});
    return SceneArtifact.encode(artifact);
  }

  static function buildCachePrefix(entry:Dynamic, projectRoot:String, manifestPath:String,
      haxeonManifest:String, module:String, functionName:String, haxe:String,
      home:String, tools:String, ?control:ProjectLoadControl):Null<String> {
    var declaration = Reflect.hasField(entry, "cache") ? field(entry, "cache") : null;
    var emptyInputs:Array<String> = [];
    var rawInputs:Dynamic = declaration == null ? emptyInputs : field(declaration, "inputs");
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
    var arguments = [haxeonManifest, module, functionName, tools, home].concat(inputs);
    var fingerprint = StringTools.trim(runProjectTool("MateriaProjectFingerprint", arguments,
      haxe, home, tools, "Could not fingerprint Materia project entrypoint", control));
    if (!Sha256Digest.isHex(fingerprint)) throw "Project fingerprint has an invalid result";
    return ProjectPath.join([cacheDirectory, fingerprint]);
  }

  static function ensureCacheDirectory(path:String):Bool {
    if (FileSystem.exists(path)) return FileSystem.isDirectory(path);
    var parent = ProjectPath.directory(path);
    if (parent == path || parent.length == 0 || !ensureCacheDirectory(parent)) return false;
    try FileSystem.createDirectory(path) catch (_:Dynamic) return false;
    return FileSystem.exists(path) && FileSystem.isDirectory(path);
  }

  static function cleanupArtifacts(outputPrefix:String, temporaryRoot:String):Void {
    for (suffix in [".hl", ".mtrg", ".document.json", ".hl.functions", ".hl.hli", ".hl.live.json", ".hl.hlp"]) {
      var artifact = outputPrefix + suffix;
      if (FileSystem.exists(artifact)) FileSystem.deleteFile(artifact);
    }
    var wrapper = ProjectPath.join([temporaryRoot, "MateriaGeneratedEntrypoint.hx"]);
    if (FileSystem.exists(wrapper)) FileSystem.deleteFile(wrapper);
    if (FileSystem.exists(temporaryRoot)) FileSystem.deleteDirectory(temporaryRoot);
  }

  static function buildModule(haxe:String, home:String, tools:String, manifest:String,
      module:String, functionName:String, outputPrefix:String, documentInput:Bool,
      ?control:ProjectLoadControl):Void {
    runProjectTool("MateriaProjectModuleBuild",
      [manifest, module, functionName, outputPrefix, home, documentInput ? "true" : "false"],
      haxe, home, tools, "Could not compile Materia project entrypoint", control);
  }

  /** Bootstrap tools once per source version, then execute them on HashLink instead of Haxe's interpreter. */
  static function runProjectTool(mainClass:String, arguments:Array<String>, haxe:String,
      home:String, tools:String, description:String, ?control:ProjectLoadControl):String {
    var cacheRoot = Sys.getEnv("XDG_CACHE_HOME");
    if (cacheRoot == null || cacheRoot.length == 0) {
      var userHome = Sys.getEnv("HOME");
      if (userHome != null && userHome.length > 0) cacheRoot = ProjectPath.join([userHome, ".cache"]);
    }
    var toolDirectory = cacheRoot == null || cacheRoot.length == 0 ? null
      : ProjectPath.join([cacheRoot, "materia", "project-tools"]);
    if (toolDirectory == null || !ensureCacheDirectory(toolDirectory))
      return runCommand(haxe, ["--cwd", home, "-cp", ProjectPath.join([home, "src"]), "-cp", tools,
        "--run", mainClass].concat(arguments), description, control);
    var identificationStarted = Sys.time();
    var sources:Array<String> = [];
    collectToolSources(ProjectPath.join([home, "src"]), sources, control);
    collectToolSources(tools, sources, control);
    sources.sort(Reflect.compare);
    var identity = new StringBuf();
    identity.add("materia-project-tool-v1:" + mainClass + "\n");
    var compilerStamp = FileSystem.metadata(haxe);
    if (compilerStamp == null) throw "Could not inspect the pinned Haxe compiler";
    identity.add(haxe + ":" + compilerStamp.size + ":" + compilerStamp.modified + "\n");
    for (source in sources) {
      if (control != null) control.throwIfCancelled();
      identity.add(source + ":" + digestHex(Sha256.make(File.getBytes(source))) + "\n");
    }
    var artifact = ProjectPath.join([toolDirectory, Sha256.encode(identity.toString()) + ".hl"]);
    if (control != null) control.measure("Tool source identification", Sys.time() - identificationStarted);
    // Atomic publication also protects independent editor processes; the mutex avoids duplicate builds in this one.
    projectToolMutex.acquire();
    try {
      if (control != null) control.throwIfCancelled();
      if (!FileSystem.exists(artifact)) {
        var temporary = artifact + "." + Sys.getPid() + "." + Std.random(1000000000) + ".tmp";
        try {
          runCommand(haxe, ["--cwd", home, "-cp", ProjectPath.join([home, "src"]), "-cp", tools,
            "-main", mainClass, "-hl", temporary], "Could not build " + mainClass, control);
          FileSystem.rename(temporary, artifact);
        } catch (error:Dynamic) {
          if (FileSystem.exists(temporary)) FileSystem.deleteFile(temporary);
          throw error;
        }
      }
    } catch (error:Dynamic) {
      projectToolMutex.release();
      throw error;
    }
    projectToolMutex.release();
    return runCommand(ProjectPath.join([home, ".tools", "hashlink", "hl"]),
      [artifact].concat(arguments), description, control);
  }

  static function collectToolSources(root:String, paths:Array<String>, control:Null<ProjectLoadControl>):Void {
    if (control != null) control.throwIfCancelled();
    for (name in FileSystem.readDirectory(root)) {
      var path = ProjectPath.join([root, name]);
      if (FileSystem.isDirectory(path)) collectToolSources(path, paths, control);
      else if (StringTools.endsWith(name, ".hx")) paths.push(path);
    }
  }

  public static function runCommand(command:String, arguments:Array<String>, description:String,
      ?control:ProjectLoadControl):String {
    #if wasm
    // Projects are compiled by child processes, which the browser cannot start. The guard keeps callers'
    // following statements reachable for the compiler's no-return analysis.
    if (command.length >= 0) throw description + ": external commands are not available in the browser build";
    return "";
    #else
    if (Sys.systemName() == "Linux" || Sys.systemName() == "Mac")
      return runStreamingCommand(command, arguments, description, control);
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
      var message = description + " (" + processExitDescription(status) + ")";
      throw details.length == 0 ? message : message + ":\n" + details;
    }
    return stdout;
    #end
  }

  #if !wasm
  /** CLOEXEC pipes prevent resident compiler workers from retaining the helper's output streams. */
  static function runStreamingCommand(command:String, arguments:Array<String>, description:String,
      ?control:ProjectLoadControl):String {
    var process = Process.spawn(command, arguments);
    if (control != null) control.attachChild(process);
    var stdout = new haxe.io.BytesBuffer(), stderr = new haxe.io.BytesBuffer();
    var buffer = Bytes.alloc(16384), outDone = false, errDone = false;
    var status = -1;
    try {
      process.closeStdin();
      while (!outDone || !errDone || status < 0) {
        if (control != null) control.throwIfCancelled();
        if (!outDone) {
          var count = process.readStdout(buffer, 0, buffer.length);
          if (count > 0) stdout.addBytes(buffer, 0, count);
          else if (count == -1) outDone = true;
        }
        if (!errDone) {
          var count = process.readStderr(buffer, 0, buffer.length);
          if (count > 0) stderr.addBytes(buffer, 0, count);
          else if (count == -1) errDone = true;
        }
        status = process.pollExit();
        if (!outDone || !errDone || status < 0) Sys.sleep(0.001);
      }
    } catch (error:Dynamic) {
      if (control != null) control.detach();
      process.close();
      throw error;
    }
    if (control != null) control.detach();
    process.close();
    if (control != null) control.throwIfCancelled();
    var output = stdout.getBytes().toString();
    if (status != 0) {
      var details = StringTools.trim(stderr.getBytes().toString() + "\n" + output);
      if (details.length > 6000) details = details.substr(details.length - 6000);
      throw description + " (exit " + status + "):\n" + details;
    }
    return output;
  }
  #end

  /** HashLink encodes POSIX signal termination separately from normal exit codes. */
  public static function processExitDescription(status:Int):String {
    if (Sys.systemName() != "Windows" && status >= 0 && (status & 0x40000000) != 0) {
      var signal = status & 0x3fffffff;
      var name = switch signal {
        case 2: "SIGINT";
        case 6: "SIGABRT";
        case 9: "SIGKILL";
        case 11: "SIGSEGV";
        case 15: "SIGTERM";
        case _: null;
      };
      return name == null ? 'terminated by signal $signal' : 'terminated by $name (signal $signal)';
    }
    return 'exit $status';
  }

  static function previewRecords(snapshot:Bytes, ?control:ProjectLoadControl):GeneratedAssemblyScene
    return new ProjectArtifactLoader(PreparedSceneCaches.current()).load(snapshot, control);

  /** Re-evaluate generated occurrence placements for a project-owned configuration. */
  public static function evaluateAssemblyState(generated:GeneratedAssemblyScene,
      stateRecord:AssemblyStateRecord):GeneratedAssemblyScene {
    var definition = generated.assemblyDefinition;
    if (definition == null) throw "Generated project has no kinematic assembly definition";
    var state = generated.assemblyModel != null && generated.assemblyModel.definition == definition
      ? AssemblyState.fromModel(generated.assemblyModel, stateRecord) : new AssemblyState(definition, stateRecord);
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
    return {objects: objects, projectJob: generated.projectJob,
      geometryBySnapshot: generated.geometryBySnapshot, assemblyDefinition: state.definition,
      assemblyState: state.record(), assemblyModel: state.model, localCentersByDefinition: generated.localCentersByDefinition,
      faceDescriptorsByDefinition: generated.faceDescriptorsByDefinition, metresPerUnit: generated.metresPerUnit, physical: generated.physical,
      recipeDocument: generated.recipeDocument, recipeDiagnostics: generated.recipeDiagnostics,
      robotMotions: generated.robotMotions, cncJob: generated.cncJob,
      mobileBase: generated.mobileBase, mission: generated.mission, robotTools: generated.robotTools,
      robotSensors: generated.robotSensors, machineMotion: generated.machineMotion};
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
