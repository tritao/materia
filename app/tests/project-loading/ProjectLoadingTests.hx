import app.MateriaProjectRunner;
import app.ProjectLoadControl;
import app.PreparedProjectCache;
import app.CadPreviewGeometry;
import materia.project.SceneArtifact;
import haxe.crypto.Sha256;
import haxe.io.Bytes;
import haxe.Json;
import sys.FileSystem;
import sys.io.File;

/** Exercise real child processes and both caches, including editable and uncached generators. */
class ProjectLoadingTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;

  static function sameBytes(a:Bytes, b:Bytes):Bool {
    if (a.length != b.length) return false;
    for (i in 0...a.length) if (a.get(i) != b.get(i)) return false;
    return true;
  }

  static function hasPhase(control:ProjectLoadControl, name:String):Bool {
    var phases:Array<Dynamic> = control.profile().phases;
    for (phase in phases) if (phase.phase == name) return true;
    return false;
  }

  static function files(root:String, extension:String):Array<String>
    return [for (name in FileSystem.readDirectory(root)) if (StringTools.endsWith(name, extension)) root + "/" + name];

  static function remove(root:String):Void {
    if (!FileSystem.exists(root)) return;
    if (FileSystem.isDirectory(root)) {
      for (name in FileSystem.readDirectory(root)) remove(root + "/" + name);
      FileSystem.deleteDirectory(root);
    } else FileSystem.deleteFile(root);
  }

  static function modelOwnership():Void {
    var frame = materia.assembly.AssemblyFrames.identity();
    var definition:materia.assembly.AssemblyDefinition = {schemaVersion: materia.assembly.AssemblyDefinitionCodec.VERSION, id: "shared-fixture", lengthUnit: "mm",
      definitions: [{id: "body", connectors: [{name: "pin", frame: frame}]}],
      occurrences: [{id: "base", definition: "body", initialPose: frame},
        {id: "arm", definition: "body", initialPose: frame}],
      joints: [{id: "slide", type: materia.assembly.AssemblyDefinition.AssemblyJointType.Prismatic,
        role: materia.assembly.AssemblyDefinition.AssemblyJointRole.Tree,
        parent: "base", parentConnector: "pin", child: "arm", childConnector: "pin",
        axis: {x: 1.0, y: 0.0, z: 0.0}, limits: {lower: -10.0, upper: 10.0, velocity: null, effort: null}, defaultValue: 0.0}]};
    var independent = new cadkit.modeling.AssemblyState(definition);
    definition.joints[0].defaultValue = 2.0;
    check(independent.joint("slide") == 0.0 && independent.definition.joints[0].defaultValue == 0.0,
      "editable definitions retain snapshot isolation");
    var model = cadkit.modeling.CompiledAssembly.takeOwnership(definition);
    check(model.definition == definition, "owned flat topology is transferred without copying");
    var root = materia.assembly.AssemblyFrames.identity(); root.x = 3.0;
    var saved:materia.assembly.AssemblyDefinition.AssemblyStateRecord = {schemaVersion: materia.assembly.AssemblyDefinitionCodec.VERSION, definition: definition.id,
      jointCoordinates: [{joint: "slide", value: 2.0}], rootPoses: [{occurrence: "base", pose: root}]};
    var first = cadkit.modeling.AssemblyState.fromModel(model, saved);
    var second = cadkit.modeling.AssemblyState.fromModel(model, saved);
    check(first.model == second.model && first.model.kinematics == second.model.kinematics,
      "states share topology and compiled kinematics");
    first.setJoint("slide", 5.0);
    root.x = 90.0;
    check(first.worldPose("arm").x == 8.0 && second.worldPose("arm").x == 5.0,
      "joint values, root inputs and evaluated poses remain independent");
    var placement = materia.assembly.AssemblyFrames.identity(); placement.x = 4.0;
    first.setRootPose("base", placement); placement.x = 99.0;
    var record = first.record(); record.rootPoses[0].pose.x = 100.0;
    check(first.worldPose("arm").x == 9.0 && second.worldPose("arm").x == 5.0,
      "setters and exported records cannot alias mutable root poses");
    var replacement = cadkit.modeling.CompiledAssembly.snapshot(definition);
    check(replacement != model && replacement.kinematics != model.kinematics, "topology snapshots compile a fresh model");
    Sys.println("Assembly model ownership tests passed");
  }

  public static function main():Int {
    modelOwnership();
    if (Sys.args().indexOf("--ownership") >= 0) return 0;
    var installation = MateriaProjectRunner.installationRoot();
    if (Sys.args().indexOf("--gantry") >= 0) {
      for (relative in ["gantry-picker/materia.project.json", "gantry-picker/materia.yaw.project.json",
          "gantry-welder/materia.project.json"]) {
        var manifest = installation + "/machinekit/examples/" + relative;
        var started = Sys.time();
        var first = MateriaProjectRunner.loadProject(manifest);
        var cold = Sys.time() - started;
        var session = new app.ProjectDocumentSession(null, false);
        session.openGeneratedProject(first, manifest);
        var installed:cadkit.modeling.AssemblyState = @:privateAccess session.assemblyRuntime;
        check(first.assemblyModel != null && installed != null && installed.model == first.assemblyModel,
          "scene installation reuses the loader's compiled assembly model");
        started = Sys.time();
        var repeatControl = new ProjectLoadControl();
        var second = MateriaProjectRunner.loadProject(manifest, null, repeatControl);
        check(!hasPhase(repeatControl, "Preparing geometry and physics"), "Gantry repeat skips preparation");
        check(Json.stringify(first.physical) == Json.stringify(second.physical), "cached Gantry physics");
        var warm = Sys.time() - started;
        check(first.objects.length > 0 && second.objects.length == first.objects.length, "cached Gantry components");
        check(Json.stringify(first.assemblyDefinition) == Json.stringify(second.assemblyDefinition), "cached Gantry assembly");
        check(Json.stringify(first.mission) == Json.stringify(second.mission), "cached Gantry mission");
        Sys.println(relative + ": first=" + cold + " s, repeat=" + warm + " s, components=" + first.objects.length);
      }
      return 0;
    }
    var temporary = Sys.getEnv("TMPDIR");
    if (temporary == null || temporary.length == 0) temporary = "/tmp";
    var root = temporary + "/materia-loading-test-" + Sys.getPid() + "-" + Std.random(1000000000);
    FileSystem.createDirectory(root);
    var previousCache = Sys.getEnv("XDG_CACHE_HOME");
    Sys.putEnv("XDG_CACHE_HOME", root + "/cache");
    try {
      var config:Dynamic = {version: 1, sourceRoots: ["."], target: "host",
        dependencies: {projectkit: {path: installation + "/projectkit"}}};
      Reflect.setField(config, "package", {name: "loading-fixture"});
      File.saveContent(root + "/haxeon.json", Json.stringify(config));
      var data = root + "/input.txt", counter = root + "/runs.txt";
      File.saveContent(data, "first");
      File.saveContent(counter, "");
      var source = 'import haxe.io.Bytes;\nimport sys.io.File;\nimport materia.project.SceneArtifact;\n'
        + 'class LoadingPreview { public static function preview(?document:String):Bytes {\n'
        + 'File.saveContent(' + Json.stringify(counter) + ', File.getContent(' + Json.stringify(counter) + ') + "x");\n'
        + 'var vertices = Bytes.alloc(96); var coordinates = [0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0];\n'
        + 'for (i in 0...12) vertices.setDouble(i * 8, coordinates[i]);\n'
        + 'var indices = Bytes.alloc(48); var triangles = [0,2,1,0,1,3,0,3,2,1,2,3];\n'
        + 'for (i in 0...12) indices.setInt32(i * 4, triangles[i]);\n'
        + 'var part:materia.project.SceneArtifact.SceneArtifactPart = {id: "tetra", name: "Tetra", red: 0.5, green: 0.5, blue: 0.5, vertexCount: 4, indexCount: 12, '
        + 'vertices: vertices, normals: Bytes.alloc(96), indices: indices, faceRanges: []};\n'
        + 'return SceneArtifact.encode({metresPerUnit: 1.0, parts: [part], recipeDocument: '
        + '(document == null ? File.getContent(' + Json.stringify(data) + ') : document) + "-v1"}); }}\n';
      File.saveContent(root + "/LoadingPreview.hx", source);
      var entry:Dynamic = {kind: "cad-preview", module: "LoadingPreview",
        documentInput: true, cache: {inputs: ["input.txt"]}};
      Reflect.setField(entry, "function", "preview");
      var manifest = root + "/materia.project.json";
      var project = {format: "materia.project", version: 1, defaultEntrypoint: "preview",
        build: {system: "haxeon", manifest: "haxeon.json"}, entrypoints: {preview: entry}};
      File.saveContent(manifest, Json.stringify(project));
      var artifacts = root + "/cache/materia/generated-artifacts";
      var coldControl = new ProjectLoadControl();
      var coldScene = MateriaProjectRunner.loadProject(manifest, null, coldControl);
      check(coldScene.recipeDocument == "first-v1", "cold output");
      check(hasPhase(coldControl, "Preparing geometry and physics"), "cold load prepares data");
      var preparedDirectory = root + "/cache/materia/prepared-scenes";
      check(files(preparedDirectory, ".mtrp").length == 1, "cold load caches portable preparation");
      check(files(artifacts, ".hl").length == 1, "cold load caches compiled code");
      check(files(artifacts, ".mtrg").length == 1, "declared deterministic output is cached");
      check(files(root + "/cache/materia/project-tools", ".hl").length == 2, "both tools run as compiled executables");
      var warmControl = new ProjectLoadControl();
      var warmScene = MateriaProjectRunner.loadProject(manifest, null, warmControl);
      check(warmScene.recipeDocument == "first-v1", "warm output");
      check(!hasPhase(warmControl, "Preparing geometry and physics"), "warm load skips preparation");
      check(Json.stringify(coldScene.physical) == Json.stringify(warmScene.physical), "prepared physical properties match");
      check(Json.stringify(coldScene.objects) == Json.stringify(warmScene.objects), "prepared occurrences match");
      check(Json.stringify(coldScene.localCentersByDefinition) == Json.stringify(warmScene.localCentersByDefinition), "prepared centers match");
      for (key in coldScene.geometryBySnapshot.keys()) {
        var fresh = coldScene.geometryBySnapshot.get(key), cached = warmScene.geometryBySnapshot.get(key);
        check(cached != null && fresh != cached && fresh.vertexCount() == cached.vertexCount() &&
          fresh.triangleCount() == cached.triangleCount(), "fresh native geometry ownership with matching mesh counts");
      }
      var artifactBytes = File.getBytes(files(artifacts, ".mtrg")[0]);
      var digest = Sha256.make(artifactBytes), artifactHash = new StringBuf();
      var digits = "0123456789abcdef";
      for (i in 0...digest.length) { var value = digest.get(i); artifactHash.add(digits.charAt(value >>> 4)); artifactHash.add(digits.charAt(value & 15)); }
      var artifact = SceneArtifact.decode(artifactBytes);
      var portable = new PreparedProjectCache(artifactHash.toString(), app.PreparedSceneCaches.current()).read(artifact.parts);
      check(portable != null && portable.length == artifact.parts.length, "portable prepared data decodes");
      for (i in 0...artifact.parts.length) {
        var item = portable[i];
        var expected = CadPreviewGeometry.prepare(artifact.parts[i], item.geometry.minimum,
          item.geometry.maximum, artifact.metresPerUnit);
        check(sameBytes(expected.positions, item.geometry.positions) && sameBytes(expected.normals, item.geometry.normals),
          "cached packed streams match fresh conversion byte for byte");
      }
      var preparedPath = files(preparedDirectory, ".mtrp")[0];
      var memory = new MemoryPreparedStore();
      var context:app.PreparedSceneCaches.PreparedSceneCacheContext = {store: memory, buildIdentity: "fixture-build-a"};
      var loader = new app.ProjectArtifactLoader(context);
      check(loader.tryLoadCached(artifactBytes) == null && memory.writes == 0,
        "cache-only lookup never computes or publishes missing preparation");
      loader.load(artifactBytes);
      var cachedControl = new ProjectLoadControl();
      loader.load(artifactBytes, cachedControl);
      check(memory.writes == 1 && !hasPhase(cachedControl, "Preparing geometry and physics"), "injected store reuses preparation");
      var fast = loader.tryLoadCached(artifactBytes);
      check(fast != null && fast.objects.length == coldScene.objects.length && memory.writes == 1,
        "cache-only lookup materializes a valid cached scene without rewriting it");
      context.buildIdentity = "fixture-build-b";
      loader.load(artifactBytes);
      check(memory.writes == 2, "build identity invalidates preparation");
      memory.unavailable = true;
      check(loader.load(artifactBytes).objects.length == coldScene.objects.length, "unavailable cache remains optional");
      var workerStore = new MemoryPreparedStore();
      var workerLoader = new app.ProjectArtifactLoader({store: workerStore, buildIdentity: "worker-fixture"});
      var packed = app.PreparedSceneCodec.encode(portable);
      var workerControl = new ProjectLoadControl();
      var workerScene = workerLoader.load(artifactBytes, workerControl, packed, true);
      check(workerStore.writes == 1 && !hasPhase(workerControl, "Preparing geometry and physics") &&
        Json.stringify(workerScene.physical) == Json.stringify(coldScene.physical),
        "worker data materializes without preparation and publishes exact portable physics");
      packed.set(packed.length - 1, packed.get(packed.length - 1) ^ 1);
      var workerRejected = false;
      try workerLoader.load(artifactBytes, null, packed, true) catch (_:Dynamic) workerRejected = true;
      check(workerRejected && workerStore.writes == 1, "invalid worker data fails without cache publication");
      var exported = root + "/Standalone.mtrg";
      File.saveBytes(exported, artifactBytes);
      check(app.ProjectSourceLoader.executionRequirement(exported).kind == "prebuilt-artifact", "artifact requires no source execution");
      check(app.editor.ProjectUiExtension.open(exported) == null, "binary artifacts are never parsed as project UI manifests");
      check(app.ProjectSourceLoader.load(exported).objects.length == coldScene.objects.length, "standalone artifact opens without generator");
      check(File.getContent(counter) == "x", "artifact opens never execute source");
      var standalone = SceneArtifact.decode(artifactBytes);
      standalone.recipeDocument = null;
      File.saveBytes(exported, SceneArtifact.encode(standalone));
      var session = new app.ProjectDocumentSession(null, false);
      session.open(exported);
      session.save(root + "/Saved.materia");
      var reopened = new app.ProjectDocumentSession(null, false);
      reopened.open(root + "/Saved.materia");
      check(reopened.projectReference == exported && reopened.scene.records().length == coldScene.objects.length,
        "saved document retains artifact reference and reopens without source");
      File.saveContent(root + "/Broken.mtrg", "bad");
      var rejected = false;
      try reopened.open(root + "/Broken.mtrg") catch (_:Dynamic) rejected = true;
      check(rejected && reopened.projectReference == exported && reopened.scene.records().length == coldScene.objects.length,
        "failed artifact replacement preserves current document");

      var bytes = File.getBytes(preparedPath);
      bytes.set(bytes.length - 1, bytes.get(bytes.length - 1) ^ 1);
      File.saveBytes(preparedPath, bytes);
      var repairControl = new ProjectLoadControl();
      var repaired = MateriaProjectRunner.loadProject(manifest, null, repairControl);
      check(hasPhase(repairControl, "Preparing geometry and physics"), "corrupt preparation is rebuilt");
      check(Json.stringify(repaired.physical) == Json.stringify(coldScene.physical), "repair preserves physics");
      var repairedControl = new ProjectLoadControl();
      MateriaProjectRunner.loadProject(manifest, null, repairedControl);
      check(!hasPhase(repairedControl, "Preparing geometry and physics"), "repaired cache is reusable");
      check(File.getContent(counter) == "x", "warm open skips execution");
      var preparedBeforeRecipe = files(preparedDirectory, ".mtrp").length;
      check(MateriaProjectRunner.loadProject(manifest, "recipe-a").recipeDocument == "recipe-a-v1", "first editable input");
      check(files(preparedDirectory, ".mtrp").length > preparedBeforeRecipe, "changed artifact selects new preparation");
      check(MateriaProjectRunner.loadProject(manifest, "recipe-b").recipeDocument == "recipe-b-v1", "changed editable input");
      check(files(artifacts, ".hl").length == 1, "recipe changes reuse compiled code");
      check(File.getContent(counter) == "xxx", "editable input always executes");
      File.saveContent(files(artifacts, ".mtrg")[0], "corrupt");
      check(MateriaProjectRunner.loadProject(manifest).recipeDocument == "first-v1", "corrupt output regenerates");
      check(File.getContent(counter) == "xxxx", "corrupt output executes again");
      File.saveContent(data, "other");
      check(MateriaProjectRunner.loadProject(manifest).recipeDocument == "other-v1", "same-size declared input edit invalidates output");
      File.saveContent(root + "/LoadingPreview.hx", StringTools.replace(source, "-v1", "-v2"));
      check(MateriaProjectRunner.loadProject(manifest).recipeDocument == "other-v2", "same-size source edit invalidates code and output");
      Reflect.deleteField(entry, "cache");
      File.saveContent(manifest, Json.stringify(project));
      var before = files(artifacts, ".mtrg").length;
      MateriaProjectRunner.loadProject(manifest);
      var compiled = files(artifacts, ".hl").length;
      MateriaProjectRunner.loadProject(manifest);
      check(files(artifacts, ".hl").length == compiled, "uncached output still reuses compiled code");
      check(files(artifacts, ".mtrg").length == before, "uncached generators never cache output");
      check(File.getContent(counter).length == 8, "uncached generator executes on each open");
      var control = new ProjectLoadControl();
      control.cancel();
      var cancelled = false;
      try MateriaProjectRunner.loadProject(manifest, null, control)
      catch (error:Dynamic) cancelled = Std.string(error) == ProjectLoadControl.CANCELLED;
      check(cancelled && File.getContent(counter).length == 8, "cancelled open executes no generator");
    } catch (error:Dynamic) {
      Sys.putEnv("XDG_CACHE_HOME", previousCache);
      remove(root);
      throw error;
    }
    Sys.putEnv("XDG_CACHE_HOME", previousCache);
    remove(root);
    Sys.println("Project loading tests passed");
    return 0;
  }
}

private class MemoryPreparedStore implements app.PreparedSceneStore {
  var entries:Map<String, Bytes> = new Map();
  public var writes:Int = 0;
  public var unavailable:Bool = false;
  public function new() {}
  public function read(key:String):Null<Bytes> {
    if (unavailable) throw "storage unavailable";
    return entries.get(key);
  }
  public function write(key:String, bytes:Bytes):Void {
    if (unavailable) throw "storage unavailable";
    entries.set(key, bytes); writes++;
  }
}
