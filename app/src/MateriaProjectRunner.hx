package app;

import haxe.Json;
import haxe.io.Bytes;
import haxe.io.Path as ProjectPath;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import sys.FileSystem;
import sys.io.File;
import sys.io.Process;
import sys.thread.Mutex;
import sys.thread.Thread;

/** Resolves a Materia project entrypoint and materializes its generated viewport geometry. */
class MateriaProjectRunner {
  static final MAX_OUTPUT_BYTES:Int = 150000000;
  static var temporarySequence:Int = 0;

  public static function load(projectPath:String):Array<SceneObjectData> {
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
    var tempRootValue = Sys.getEnv("TMPDIR");
    temporarySequence++;
    var temporaryRoot = ProjectPath.join([tempRootValue == null ? "/tmp" : tempRootValue,
      "materia-project-" + Sys.getPid() + "-" + temporarySequence]);
    FileSystem.createDirectory(temporaryRoot);
    var outputPrefix = ProjectPath.join([temporaryRoot, "preview"]);
    var records:Array<SceneObjectData>;
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
      Sys.println('Materia project: decoded ${records.length} component records');
    } catch (error:Dynamic) {
      cleanupArtifacts(outputPrefix, temporaryRoot);
      throw error;
    }
    cleanupArtifacts(outputPrefix, temporaryRoot);
    return records;
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

  static function runCommand(command:String, arguments:Array<String>, description:String):Void {
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
  }

  static function previewRecords(snapshot:Bytes):Array<SceneObjectData> {
    var artifact = SceneArtifact.decode(snapshot);
    var records:Array<SceneObjectData> = [];
    var scale = artifact.metresPerUnit;
    for (component in artifact.parts) {
      var label = component.name;
      var minimum = [1e300, 1e300, 1e300], maximum = [-1e300, -1e300, -1e300];
      for (vertex in 0...component.vertexCount) for (axis in 0...3) {
        var coordinate = component.vertices.getDouble(vertex * 24 + axis * 8);
        if (!Math.isFinite(coordinate)) throw 'Project component "$label" has a non-finite vertex';
        minimum[axis] = Math.min(minimum[axis], coordinate);
        maximum[axis] = Math.max(maximum[axis], coordinate);
      }
      var componentSnapshot = encodeSnapshot(component, minimum, maximum, scale);
      if (componentSnapshot.length > 50000000) throw 'Project component "$label" exceeds the 50 MB mesh limit';
      records.push({id: "project:" + component.id, label: label, type: "cad-preview",
        x: (minimum[0] + maximum[0]) * scale * 0.5,
        y: (minimum[1] + maximum[1]) * scale * 0.5,
        z: (minimum[2] + maximum[2]) * scale * 0.5,
        width: Math.max(0.000001, (maximum[0] - minimum[0]) * scale),
        height: Math.max(0.000001, (maximum[1] - minimum[1]) * scale),
        depth: Math.max(0.000001, (maximum[2] - minimum[2]) * scale),
        collisionEnabled: false, dynamicBody: false, mass: 1.0,
        red: component.red, green: component.green, blue: component.blue,
        visible: true, meshSnapshot: componentSnapshot});
    }
    return records;
  }

  static function encodeSnapshot(component:SceneArtifactPart, minimum:Array<Float>,
      maximum:Array<Float>, scale:Float):String {
    var ranges:Array<String> = [];
    for (range in component.faceRanges)
      ranges.push(range.faceIndex + "," + range.firstIndex + "," + range.indexCount);
    return ["materia.geometry-preview/3", Std.string(scale), Std.string(component.vertexCount),
      Std.string(component.indexCount), triple(minimum), triple(maximum), ranges.join(";"),
      MateriaBase64.encode(component.vertices), MateriaBase64.encode(component.normals),
      MateriaBase64.encode(component.indices)].join("|");
  }

  static function triple(values:Array<Float>):String
    return Std.string(values[0]) + "," + Std.string(values[1]) + "," + Std.string(values[2]);

  static function projectToolsDirectory():String {
    var current = Sys.getCwd();
    var candidate = ProjectPath.join([current, "tools", "MateriaProjectModuleBuild.hx"]);
    if (FileSystem.exists(candidate)) return ProjectPath.join([current, "tools"]);
    candidate = ProjectPath.join([current, "app", "tools", "MateriaProjectModuleBuild.hx"]);
    if (FileSystem.exists(candidate)) return ProjectPath.join([current, "app", "tools"]);
    throw "Could not locate MateriaProjectModuleBuild.hx next to the app source tree";
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
    if (!Std.isOfType(result, String) || StringTools.trim(cast result).length == 0)
      throw 'Project field "$name" must be non-empty text';
    return cast result;
  }

  static function fieldInt(value:Dynamic, name:String):Int {
    var result:Dynamic = field(value, name);
    if (!Std.isOfType(result, Int)) throw 'Project field "$name" must be an integer';
    return cast result;
  }

}
