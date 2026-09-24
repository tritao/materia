package app;

import haxe.Json;
import haxe.io.Bytes;
import haxe.io.Path as ProjectPath;
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
      var moduleBytes = File.getBytes(outputPrefix + ".hl");
      var identity = File.getBytes(outputPrefix + ".hli");
      var functionId = Std.parseInt(File.getContent(outputPrefix + ".entry"));
      if (functionId == null) throw "Project module has no stable entrypoint ID";
      var result = MateriaRuntimeModule.evaluateBytes(moduleBytes, identity, functionId);
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
    for (suffix in [".hl", ".hli", ".entry"]) {
      var artifact = outputPrefix + suffix;
      if (FileSystem.exists(artifact)) FileSystem.deleteFile(artifact);
    }
    if (FileSystem.exists(temporaryRoot)) FileSystem.deleteDirectory(temporaryRoot);
  }

  static function buildModule(haxe:String, home:String, tools:String, manifest:String,
      module:String, functionName:String, outputPrefix:String):Void {
    var arguments = ["--cwd", home, "-cp", ProjectPath.join([home, "src"]), "-cp", tools,
      "--run", "MateriaProjectModuleBuild", manifest, module, functionName, outputPrefix];
    var process:Process;
    try process = Process.run(haxe, arguments)
    catch (error:Dynamic) throw "Could not start the Haxeon project compiler: " + Std.string(error);
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
      throw "Could not read project compiler output: " + Std.string(processError);
    if (status != 0) {
      var details = StringTools.trim(stderr + "\n" + stdout);
      if (details.length > 6000) details = details.substr(details.length - 6000);
      throw 'Could not compile Materia project entrypoint (exit $status):\n$details';
    }
  }

  static function previewRecords(snapshot:Bytes):Array<SceneObjectData> {
    var components = new MateriaProjectArtifactReader(snapshot).read();
    var records:Array<SceneObjectData> = [];
    var colors = [[0.77, 0.53, 0.26], [0.72, 0.31, 0.19], [0.35, 0.48, 0.63],
      [0.72, 0.66, 0.48], [0.28, 0.34, 0.40], [0.84, 0.76, 0.57]];
    for (index in 0...components.length) {
      var component = components[index];
      var label = component.name;
      var minimum = component.minimum, maximum = component.maximum;
      var componentSnapshot = MateriaProjectArtifactReader.encodeSnapshot(component);
      if (componentSnapshot.length > 50000000) throw 'Project component "$label" exceeds the 50 MB mesh limit';
      var color = colors[index % colors.length];
      records.push({id: "project-part-" + (index + 1), label: label, type: "cad-preview",
        x: (minimum[0] + maximum[0]) * 0.0005,
        y: (minimum[1] + maximum[1]) * 0.0005,
        z: (minimum[2] + maximum[2]) * 0.0005,
        width: Math.max(0.000001, (maximum[0] - minimum[0]) * 0.001),
        height: Math.max(0.000001, (maximum[1] - minimum[1]) * 0.001),
        depth: Math.max(0.000001, (maximum[2] - minimum[2]) * 0.001),
        collisionEnabled: false, dynamicBody: false, mass: 1.0,
        red: color[0], green: color[1], blue: color[2], visible: true, meshSnapshot: componentSnapshot});
    }
    return records;
  }

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

private typedef ProjectArtifactFaceRange = {
  var faceIndex:Int;
  var firstIndex:Int;
  var indexCount:Int;
}

private typedef ProjectArtifactComponent = {
  var name:String;
  var vertexCount:Int;
  var indexCount:Int;
  var minimum:Array<Float>;
  var maximum:Array<Float>;
  var vertices:Bytes;
  var normals:Bytes;
  var indices:Bytes;
  var faceRanges:Array<ProjectArtifactFaceRange>;
}

/** Reads the compact MTRG binary payload returned by a project entrypoint. */
private class MateriaProjectArtifactReader {
  static final MAX_VERTICES:Int = 2000000;
  static final MAX_TRIANGLES:Int = 4000000;
  static final MAX_FACE_RANGES:Int = 100000;

  final source:Bytes;
  var offset:Int = 0;

  public function new(source:Bytes) this.source = source;

  public function read():Array<ProjectArtifactComponent> {
    for (expected in [77, 84, 82, 71]) if (readByte() != expected)
      throw "Project returned an invalid geometry artifact signature";
    if (readInt32() != 1) throw "Project returned an unsupported geometry artifact version";
    var componentCount = readInt32();
    if (componentCount <= 0 || componentCount > 1000)
      throw "Project geometry artifact must contain between 1 and 1000 components";
    var components:Array<ProjectArtifactComponent> = [];
    for (_ in 0...componentCount) {
      var nameLength = readInt32();
      if (nameLength <= 0 || nameLength > 4096) throw "Project geometry artifact has an invalid component name";
      var name = readBytes(nameLength).getString(0, nameLength);
      if (StringTools.trim(name).length == 0) throw "Project geometry artifact has an empty component name";
      var vertexCount = readInt32(), indexCount = readInt32(), faceRangeCount = readInt32();
      if (vertexCount <= 0 || vertexCount > MAX_VERTICES || indexCount <= 0 || indexCount % 3 != 0 ||
          indexCount / 3 > MAX_TRIANGLES || faceRangeCount < 0 || faceRangeCount > MAX_FACE_RANGES)
        throw 'Project component "$name" has invalid mesh counts';
      var vertices = readBytes(vertexCount * 24), normals = readBytes(vertexCount * 24),
        indices = readBytes(indexCount * 4);
      var minimum = [1e300, 1e300, 1e300], maximum = [-1e300, -1e300, -1e300];
      for (vertex in 0...vertexCount) for (axis in 0...3) {
        var coordinate = vertices.getDouble(vertex * 24 + axis * 8);
        if (!Math.isFinite(coordinate)) throw 'Project component "$name" has a non-finite vertex';
        minimum[axis] = Math.min(minimum[axis], coordinate);
        maximum[axis] = Math.max(maximum[axis], coordinate);
      }
      var faceRanges:Array<ProjectArtifactFaceRange> = [];
      for (_ in 0...faceRangeCount) {
        var faceIndex = readInt32(), firstIndex = readInt32(), rangeIndexCount = readInt32();
        if (faceIndex < 0 || firstIndex < 0 || rangeIndexCount < 0 || firstIndex % 3 != 0 ||
            rangeIndexCount % 3 != 0 || firstIndex + rangeIndexCount > indexCount)
          throw 'Project component "$name" has an invalid face range';
        faceRanges.push({faceIndex: faceIndex, firstIndex: firstIndex, indexCount: rangeIndexCount});
      }
      components.push({name: name, vertexCount: vertexCount, indexCount: indexCount, minimum: minimum,
        maximum: maximum, vertices: vertices, normals: normals, indices: indices, faceRanges: faceRanges});
    }
    if (offset != source.length) throw "Project geometry artifact contains trailing data";
    return components;
  }

  public static function encodeSnapshot(component:ProjectArtifactComponent):String {
    var ranges:Array<String> = [];
    for (range in component.faceRanges)
      ranges.push(range.faceIndex + "," + range.firstIndex + "," + range.indexCount);
    return ["materia.geometry-preview/2", Std.string(component.vertexCount), Std.string(component.indexCount),
      triple(component.minimum), triple(component.maximum), ranges.join(";"),
      MateriaBase64.encode(component.vertices), MateriaBase64.encode(component.normals),
      MateriaBase64.encode(component.indices)].join("|");
  }

  static function triple(values:Array<Float>):String
    return Std.string(values[0]) + "," + Std.string(values[1]) + "," + Std.string(values[2]);

  function readByte():Int {
    if (offset >= source.length) throw "Project geometry artifact ended unexpectedly";
    return source.get(offset++);
  }

  function readInt32():Int {
    var first = readByte(), second = readByte(), third = readByte(), fourth = readByte();
    return first | (second << 8) | (third << 16) | (fourth << 24);
  }

  function readBytes(length:Int):Bytes {
    if (length < 0 || length > source.length - offset) throw "Project geometry artifact has an invalid buffer length";
    var bytes = Bytes.view(source, offset, length);
    offset += length;
    return bytes;
  }
}
