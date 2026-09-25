package app;

import haxe.Json;
import haxe.crypto.Sha256;
import haxe.io.Bytes;
import haxe.io.Path as ProjectPath;
import sys.io.AtomicFile;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.project.AssemblyFrames;
import materia.project.AssemblyRecord.AssemblyFrame;
import materia.project.AssemblyRecord.AssemblyConnector;
import materia.project.AssemblyRecord.AssemblyInstance;
import materia.project.AssemblyRecord.AssemblyJoint;
import materia.project.AssemblyRecord;
import materia.project.AssemblyDefinition;
import materia.project.AssemblyDefinition.AssemblyComponentDefinition;
import materia.project.AssemblyDefinition.AssemblyJointRole;
import cadkit.modeling.AssemblyState;
import materia.project.AssemblyDefinitionCodec;
import nativekit.scene.GeometryData;
import sys.FileSystem;
import sys.io.File;
import sys.io.Process;
import sys.thread.Mutex;
import sys.thread.Thread;

/** Resolves a Materia project entrypoint and materializes its generated viewport geometry. */
class MateriaProjectRunner {
  static final MAX_OUTPUT_BYTES:Int = 150000000;
  static var temporarySequence:Int = 0;

  public static function load(projectPath:String):Array<SceneObjectData> return loadProject(projectPath).objects;

  public static function loadProject(projectPath:String):GeneratedAssemblyScene {
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

    var home = Sys.getEnv("HAXEON_HOME");
    if (home == null || home.length == 0) throw "HAXEON_HOME is not set; launch Materia through Haxeon";
    home = FileSystem.fullPath(home);
    var haxe = ProjectPath.join([home, ".tools", "haxe", "haxe"]);
    if (!FileSystem.exists(haxe)) throw 'Pinned Haxe compiler was not found at $haxe';
    var tools = projectToolsDirectory();
    var cache = cachePath(entry, projectRoot, manifestPath, haxeonManifest,
      fieldText(entry, "module"), fieldText(entry, "function"), haxe, home, tools);
    if (cache != null && FileSystem.exists(cache)) {
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
    temporarySequence++;
    var temporaryRoot = ProjectPath.join([tempRootValue == null ? "/tmp" : tempRootValue,
      "materia-project-" + Sys.getPid() + "-" + temporarySequence]);
    FileSystem.createDirectory(temporaryRoot);
    var outputPrefix = ProjectPath.join([temporaryRoot, "preview"]);
    var records:GeneratedAssemblyScene;
    try {
      Sys.println("Materia project: compiling its entrypoint");
      buildModule(haxe, home, tools, haxeonManifest, fieldText(entry, "module"),
        fieldText(entry, "function"), outputPrefix);
      Sys.println("Materia project: executing its entrypoint");
      var hashlink = ProjectPath.join([home, ".tools", "hashlink", "hl"]);
      if (!FileSystem.exists(hashlink)) throw 'Pinned HashLink executable was not found at $hashlink';
      runCommand(hashlink, [outputPrefix + ".hl", outputPrefix + ".mtrg"],
        "Could not generate Materia project artifact");
      if (!FileSystem.exists(outputPrefix + ".mtrg"))
        throw "Project entrypoint did not write a geometry artifact";
      var metadata = FileSystem.metadata(outputPrefix + ".mtrg");
      if (metadata == null || metadata.size > MAX_OUTPUT_BYTES)
        throw "Project geometry preview exceeds the 150 MB limit";
      var result = File.getBytes(outputPrefix + ".mtrg");
      Sys.println('Materia project: received binary geometry artifact (${result.length} bytes)');
      if (result.length > MAX_OUTPUT_BYTES) throw "Project geometry preview exceeds the 150 MB limit";
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
      home:String, tools:String):Null<String> {
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
      "--run", "MateriaProjectFingerprint", haxeonManifest, module, functionName, tools].concat(inputs);
    var fingerprint = StringTools.trim(runCommand(haxe, arguments,
      "Could not fingerprint Materia project entrypoint"));
    if (!~/^[0-9a-f]{64}$/.match(fingerprint)) throw "Project fingerprint has an invalid result";
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
    for (suffix in [".hl", ".mtrg"]) {
      var artifact = outputPrefix + suffix;
      if (FileSystem.exists(artifact)) FileSystem.deleteFile(artifact);
    }
    if (FileSystem.exists(temporaryRoot)) FileSystem.deleteDirectory(temporaryRoot);
  }

  static function buildModule(haxe:String, home:String, tools:String, manifest:String,
      module:String, functionName:String, outputPrefix:String):Void {
    var arguments = ["--cwd", home, "-cp", ProjectPath.join([home, "src"]), "-cp", tools,
      "--run", "MateriaProjectModuleBuild", manifest, module, functionName, outputPrefix];
    runCommand(haxe, arguments, "Could not compile Materia project entrypoint");
  }

  static function runCommand(command:String, arguments:Array<String>, description:String):String {
    var process:Process;
    try process = Process.run(command, arguments)
    catch (error:Dynamic) throw description + ": " + Std.string(error);
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
    if (processError != null)
      throw description + ": " + Std.string(processError);
    if (status != 0) {
      var details = StringTools.trim(stderr + "\n" + stdout);
      if (details.length > 6000) details = details.substr(details.length - 6000);
      throw '$description (exit $status):\n$details';
    }
    return stdout;
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
    if (artifact.assemblyDefinition != null) {
      var state = new AssemblyState(artifact.assemblyDefinition, artifact.assemblyState);
      for (occurrence in artifact.assemblyDefinition.occurrences) {
        poses.set(occurrence.id, state.worldPose(occurrence.id));
        var count = componentUseCount.get(occurrence.definition);
        componentUseCount.set(occurrence.definition, count == null ? 1 : count + 1);
      }
      resolvedAssembly = legacySnapshot(artifact.assemblyDefinition, state);
    } else if (artifact.assembly != null) {
      for (instance in artifact.assembly.instances) poses.set(instance.id, instance.pose);
    }

    var boundsByDefinition:Map<String, {minimum:Array<Float>, maximum:Array<Float>}> = new Map();
    var geometryKeyByDefinition:Map<String, String> = new Map();
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
      var parts = new Map<String, SceneArtifactPart>();
      for (part in artifact.parts) parts.set(part.id, part);
      for (instance in artifact.assembly.instances) {
        var component = parts.get(instance.id);
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
      geometryBySnapshot: geometryBySnapshot};
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
    records.push({id: "project:" + occurrenceId, label: label, type: "cad-preview",
      x: center.x * scale, y: center.y * scale, z: center.z * scale,
      width: Math.max(0.000001, (maximum[0] - minimum[0]) * scale),
      height: Math.max(0.000001, (maximum[1] - minimum[1]) * scale),
      depth: Math.max(0.000001, (maximum[2] - minimum[2]) * scale),
      collisionEnabled: false, dynamicBody: false, mass: 1.0,
      red: component.red, green: component.green, blue: component.blue, visible: true,
      meshSnapshot: geometryKeyByDefinition.get(definitionId),
      rotation: pose == null ? null : [pose.qx, pose.qy, pose.qz, pose.qw]});
  }

  static function legacySnapshot(definition:AssemblyDefinition, state:AssemblyState):AssemblyRecord {
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
    var current = Sys.getCwd();
    var candidate = ProjectPath.join([current, "tools", "MateriaProjectModuleBuild.hx"]);
    if (FileSystem.exists(candidate)) return ProjectPath.join([current, "tools"]);
    candidate = ProjectPath.join([current, "app", "tools", "MateriaProjectModuleBuild.hx"]);
    if (FileSystem.exists(candidate)) return ProjectPath.join([current, "app", "tools"]);
    throw "Could not locate MateriaProjectModuleBuild.hx next to the app source tree";
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

typedef GeneratedAssemblyScene = {
  var objects:Array<SceneObjectData>;
  var assembly:Null<AssemblyRecord>;
  var geometryBySnapshot:Map<String, GeometryData>;
}
