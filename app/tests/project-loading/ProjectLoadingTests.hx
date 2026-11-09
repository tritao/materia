import app.MateriaProjectRunner;
import app.ProjectLoadControl;
import app.PreparedProjectCache;
import app.CadPreviewGeometry;
import materia.project.SceneArtifact;
import cadkit.cache.DiskStore;
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

  static function files(root:String,
    extension:String):Array<String> return [for (name in FileSystem.readDirectory(root)) if (StringTools.endsWith(
      name,
      extension
    )) root + "/" + name];

  /** The store's entries are `<sha256>.memo` files: an 8-byte frame, then the payload. Which kind one is shows in its first bytes. */
  static function kindOf(path:String):String {
    var bytes = File.getBytes(path);
    if (bytes.length < 11) return "other";
    if (bytes.get(8) == "{".code) return "record";
    if (bytes.get(8) == "[".code) return "reads";
    if (bytes.get(8) == "H".code && bytes.get(9) == "L".code && bytes.get(10) == "B".code) return "generator";
    try SceneArtifact.decodeView(bytes.sub(8, bytes.length - 8)) catch (_:Dynamic) return "kernel";
    return "output";
  }

  static function stored(store:String, kind:String):Array<String> return [for (path in files(store, ".memo")) if (kindOf(path) == kind) path];

  static function payload(path:String):Bytes {
    var bytes = File.getBytes(path);
    return bytes.sub(8, bytes.length - 8);
  }

  /** `text` as a well-formed store entry, so the damage a test plants is in the payload and not in the frame. */
  static function framed(text:String):Bytes {
    var body = Bytes.ofString(text), bytes = Bytes.alloc(8 + body.length);
    bytes.setInt32(0, 0x4D454D31);
    bytes.setInt32(4, body.length);
    bytes.blit(8, body, 0, body.length);
    return bytes;
  }

  /**
   * A generator the runner starts runs from a temporary directory with no native libraries beside it. Its kernel cache must
   * still be on (the native identity comes from the library the process loaded), and say so loudly if it is not: the editor
   * once ran with the cache silently off. The generator fails when it has no native identity, and the store must hold
   * kernel-operation entries after the open.
   */
  /** A CadKit generator project in `root` (see kernelCacheInGeneration); it also writes its native identity to MATERIA_IDENTITY_OUT. */
  static function kernelFixture(root:String, installation:String):String {
    var config:Dynamic = {
      version: 1,
      sourceRoots: ["."],
      target: "host",
      dependencies: {projectkit: {path: installation + "/projectkit"}, cadkit: {path: installation + "/cadkit"}}
    };
    Reflect.setField(config, "package", {name: "kernel-cache-fixture"});
    File.saveContent(root + "/haxeon.json", Json.stringify(config));
    File.saveContent(root + "/KernelPreview.hx", 'import haxe.io.Bytes;\nimport materia.project.SceneArtifact;\n'
      + 'class KernelPreview { public static function preview():Bytes {\n'
      + 'var drilled = cadkit.Shape.box(10, 10, 10).cut(cadkit.Shape.box(4, 4, 20));\n'
      + 'if (drilled.volume() < 1) throw "no geometry";\n'
      + 'var identity = cadkit.cache.NativeIdentity.get();\n'
      + 'if (identity == null) throw "kernel cache off: no native identity";\n'
      + 'var out = Sys.getEnv("MATERIA_IDENTITY_OUT"); if (out != null) sys.io.File.saveContent(out, identity);\n'
      + 'var vertices = Bytes.alloc(96); var coordinates = [0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0];\n'
      + 'for (i in 0...12) vertices.setDouble(i * 8, coordinates[i]);\n'
      + 'var indices = Bytes.alloc(48); var triangles = [0,2,1,0,1,3,0,3,2,1,2,3];\n'
      + 'for (i in 0...12) indices.setInt32(i * 4, triangles[i]);\n'
      + 'var part:materia.project.SceneArtifact.SceneArtifactPart = {id: "tetra", name: "Tetra", red: 0.5, green: 0.5, blue: 0.5, vertexCount: 4, indexCount: 12, '
      + 'vertices: vertices, normals: Bytes.alloc(96), indices: indices, faceRanges: []};\n'
      + 'return SceneArtifact.encode({metresPerUnit: 1.0, parts: [part]}); }}\n');
    var entry:Dynamic = {kind: "cad-preview", module: "KernelPreview", cache: {}};
    Reflect.setField(entry, "function", "preview");
    var manifest = root + "/materia.project.json";
    File.saveContent(manifest, Json.stringify({
      format: "materia.project",
      version: 1,
      defaultEntrypoint: "preview",
      build: {system: "haxeon", manifest: "haxeon.json"},
      entrypoints: {preview: entry}
    }));
    return manifest;
  }

  /** Opens the fixture with a fresh store and returns the native identity its generator saw. */
  static function nativeIdentityOf(root:String, installation:String, name:String):String {
    var directory = root + "/" + name;
    FileSystem.createDirectory(directory);
    var manifest = kernelFixture(directory, installation);
    Sys.putEnv("XDG_CACHE_HOME", directory + "/cache");
    Sys.putEnv("MATERIA_IDENTITY_OUT", directory + "/identity.txt");
    DiskStore.useProcess(new DiskStore(directory + "/cache/materia/memo", 1e9));
    MateriaProjectRunner.loadProject(manifest);
    DiskStore.useProcess(null);
    return File.getContent(directory + "/identity.txt");
  }

  static function copyFile(from:String, to:String):Void File.saveBytes(to, File.getBytes(from));

  /**
   * The native identity is the content of the loaded CadKit library and the OCCT install's `occt.identity`: a library with other
   * content (same size too) changes it, an identical copy elsewhere keeps it, another OCCT stamp changes it, and an OCCT install
   * without a stamp turns the kernel cache off with a message.
   */
  static function nativeIdentityTracksBuild(installation:String):Void {
    var temporary = Sys.getEnv("TMPDIR");
    if (temporary == null || temporary.length == 0) temporary = "/tmp";
    var root = temporary + "/materia-native-identity-test-" + Sys.getPid() + "-" + Std.random(1000000000);
    FileSystem.createDirectory(root);
    var previousCache = Sys.getEnv("XDG_CACHE_HOME"), previousLibraries = Sys.getEnv("LD_LIBRARY_PATH");
    try {
      var cadkit:Null<String> = null, occt:Null<String> = null;
      for (directory in (previousLibraries == null ? "" : previousLibraries).split(":")) {
        if (directory == "" || !FileSystem.exists(directory) || !FileSystem.isDirectory(directory)) continue;
        if (cadkit == null && FileSystem.exists(directory + "/libcadkit-core.so")) cadkit = directory + "/libcadkit-core.so";
        if (occt == null && FileSystem.exists(directory + "/libTKernel.so")) occt = directory;
      }
      var occtDirectory = Sys.getEnv("CADKIT_OCCT_DIR");
      if (occt == null && occtDirectory != null) occt = occtDirectory + "/lib";
      check(cadkit != null && occt != null, "the library path names the CadKit library and the OCCT libraries");
      function libraries(directory:String):Void Sys.putEnv("LD_LIBRARY_PATH", directory + (previousLibraries == null ? "" : ":" + previousLibraries));
      // The same bytes in two places.
      FileSystem.createDirectory(root + "/one");
      FileSystem.createDirectory(root + "/two");
      var cadkitLibrary:String = cadkit;
      copyFile(cadkitLibrary, root + "/one/libcadkit-core.so");
      copyFile(cadkitLibrary, root + "/two/libcadkit-core.so");
      libraries(root + "/one");
      var identity = nativeIdentityOf(root, installation, "a");
      libraries(root + "/two");
      check(nativeIdentityOf(root, installation, "b") == identity, "an identical copy of the CadKit library keeps the native identity");
      // Same size, other content (an unused byte of the file changes: the first byte after the ELF identification).
      var bytes = File.getBytes(cadkitLibrary);
      bytes.set(bytes.length - 1, bytes.get(bytes.length - 1) ^ 1);
      FileSystem.createDirectory(root + "/three");
      File.saveBytes(root + "/three/libcadkit-core.so", bytes);
      libraries(root + "/three");
      check(nativeIdentityOf(root, installation, "c") != identity, "a CadKit library with other content changes the native identity");
      // Another OCCT stamp, and none, on a copy of the install (the shared one is in use by other processes).
      var occtLibraries:String = occt, prefix = haxe.io.Path.directory(occtLibraries);
      check(FileSystem.exists(prefix + "/occt.identity"), "the OCCT install has an identity stamp (" + prefix + "/occt.identity)");
      FileSystem.createDirectory(root + "/occt");
      check(Sys.command("cp", ["-a", occtLibraries, root + "/occt/lib"]) == 0, "copied the OCCT libraries");
      File.saveContent(root + "/occt/occt.identity", File.getContent(prefix + "/occt.identity"));
      Sys.putEnv("LD_LIBRARY_PATH", root + "/one:" + root + "/occt/lib:" + previousLibraries);
      check(nativeIdentityOf(root, installation, "d") == identity, "a copy of the OCCT install with the same stamp keeps the native identity");
      File.saveContent(root + "/occt/occt.identity", "another build\n");
      check(nativeIdentityOf(root, installation, "e") != identity, "another OCCT stamp changes the native identity");
      FileSystem.deleteFile(root + "/occt/occt.identity");
      var message:Null<String> = null;
      try nativeIdentityOf(root, installation, "f") catch (error:Dynamic) message = Std.string(error);
      check(message != null && message.indexOf("no native identity") >= 0, "an OCCT install without a stamp turns the kernel cache off: " + message);
    } catch (error:Dynamic) {
      Sys.putEnv("XDG_CACHE_HOME", previousCache);
      Sys.putEnv("LD_LIBRARY_PATH", previousLibraries);
      DiskStore.useProcess(null);
      remove(root);
      throw error;
    }
    Sys.putEnv("XDG_CACHE_HOME", previousCache);
    Sys.putEnv("LD_LIBRARY_PATH", previousLibraries);
    DiskStore.useProcess(null);
    remove(root);
  }

  static function kernelCacheInGeneration(installation:String):Void {
    var temporary = Sys.getEnv("TMPDIR");
    if (temporary == null || temporary.length == 0) temporary = "/tmp";
    var root = temporary + "/materia-kernel-cache-test-" + Sys.getPid() + "-" + Std.random(1000000000);
    FileSystem.createDirectory(root);
    var previousCache = Sys.getEnv("XDG_CACHE_HOME");
    Sys.putEnv("XDG_CACHE_HOME", root + "/cache");
    try {
      var manifest = kernelFixture(root, installation);
      var store = root + "/cache/materia/memo";
      DiskStore.useProcess(new DiskStore(store, 1e9));
      MateriaProjectRunner.loadProject(manifest);
      check(stored(store, "kernel").length > 0, "the generator's kernel operations are in the store after the open");
      check(stored(store, "generator").length == 1 && stored(store, "record").length == 1, "and so are its generator and record");
      // The generator created the store's directories while it ran; asking whether they existed must not make its output stale.
      var again = new ProjectLoadControl();
      MateriaProjectRunner.loadProject(manifest, null, again);
      check(!hasPhase(again, "Running the project"), "a generator that used the kernel cache is not regenerated on the next open");
    } catch (error:Dynamic) {
      Sys.putEnv("XDG_CACHE_HOME", previousCache);
      DiskStore.useProcess(null);
      remove(root);
      throw error;
    }
    Sys.putEnv("XDG_CACHE_HOME", previousCache);
    DiskStore.useProcess(null);
    remove(root);
  }

  /**
   * The files a generator reads are noticed as it runs (nothing is declared) and kept with its output: editing one, creating a
   * file it looked for and found missing, and changing a directory it listed each regenerate the output without a compile; a file
   * the generator writes itself is not an input, and an unchanged reopen hits.
   */
  static function generatorInputs(installation:String):Void {
    var temporary = Sys.getEnv("TMPDIR");
    if (temporary == null || temporary.length == 0) temporary = "/tmp";
    var root = temporary + "/materia-reads-test-" + Sys.getPid() + "-" + Std.random(1000000000);
    FileSystem.createDirectory(root);
    FileSystem.createDirectory(root + "/listed");
    var previousCache = Sys.getEnv("XDG_CACHE_HOME");
    Sys.putEnv("XDG_CACHE_HOME", root + "/cache");
    try {
      var config:Dynamic = {version: 1, sourceRoots: ["."], target: "host", dependencies: {projectkit: {path: installation + "/projectkit"}}};
      Reflect.setField(config, "package", {name: "reads-fixture"});
      File.saveContent(root + "/haxeon.json", Json.stringify(config));
      var counter = root + "/runs.txt", data = root + "/data.txt", optional = root + "/optional.txt";
      File.saveContent(counter, "");
      File.saveContent(data, "one");
      File.saveContent(root + "/listed/a.txt", "a");
      File.saveContent(root + "/ReadsPreview.hx", 'import haxe.io.Bytes;\nimport sys.FileSystem;\nimport sys.io.File;\nimport materia.project.SceneArtifact;\n'
        + 'class ReadsPreview { public static function preview():Bytes {\n'
        + 'File.saveContent(' + Json.stringify(counter) + ', File.getContent(' + Json.stringify(counter) + ') + "x");\n'
        + 'var text = File.getContent(' + Json.stringify(data) + ') + (FileSystem.exists(' + Json.stringify(optional) + ') ? "+optional" : "")'
        + ' + FileSystem.readDirectory(' + Json.stringify(root + "/listed") + ').join(",");\n'
        + 'var vertices = Bytes.alloc(96); var coordinates = [0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0];\n'
        + 'for (i in 0...12) vertices.setDouble(i * 8, coordinates[i]);\n'
        + 'var indices = Bytes.alloc(48); var triangles = [0,2,1,0,1,3,0,3,2,1,2,3];\n'
        + 'for (i in 0...12) indices.setInt32(i * 4, triangles[i]);\n'
        + 'var part:materia.project.SceneArtifact.SceneArtifactPart = {id: "tetra", name: "Tetra", red: 0.5, green: 0.5, blue: 0.5, vertexCount: 4, indexCount: 12, '
        + 'vertices: vertices, normals: Bytes.alloc(96), indices: indices, faceRanges: []};\n'
        + 'return SceneArtifact.encode({metresPerUnit: 1.0, parts: [part], recipeDocument: text}); }}\n');
      var entry:Dynamic = {kind: "cad-preview", module: "ReadsPreview", cache: {}};
      Reflect.setField(entry, "function", "preview");
      var manifest = root + "/materia.project.json";
      File.saveContent(manifest, Json.stringify({
        format: "materia.project",
        version: 1,
        defaultEntrypoint: "preview",
        build: {system: "haxeon", manifest: "haxeon.json"},
        entrypoints: {preview: entry}
      }));
      var store = root + "/cache/materia/memo";
      DiskStore.useProcess(new DiskStore(store, 1e9));
      function open():String return MateriaProjectRunner.loadProject(manifest).recipeDocument;
      function runs():Int return File.getContent(counter).length;
      check(open() == "onea.txt" && runs() == 1, "first open generates");
      check(open() == "onea.txt" && runs() == 1, "an unchanged reopen hits, although the generator wrote a file it had read");
      File.saveContent(data, "two");
      check(open() == "twoa.txt" && runs() == 2, "editing a file the generator read regenerates it");
      check(open() == "twoa.txt" && runs() == 2, "and the new output hits");
      File.saveContent(optional, "x");
      check(open() == "two+optionala.txt" && runs() == 3, "creating a file the generator looked for and did not find regenerates it");
      File.saveContent(root + "/listed/b.txt", "b");
      check(open() == "two+optionala.txt,b.txt" && runs() == 4, "changing a directory the generator listed regenerates it");
      File.saveContent(root + "/listed/b.txt", "bb");
      check(open() == "two+optionala.txt,b.txt" && runs() == 4, "editing a file in a listed directory does not");
      check(stored(store, "generator").length == 1, "none of this compiled again");
    } catch (error:Dynamic) {
      Sys.putEnv("XDG_CACHE_HOME", previousCache);
      DiskStore.useProcess(null);
      remove(root);
      throw error;
    }
    Sys.putEnv("XDG_CACHE_HOME", previousCache);
    DiskStore.useProcess(null);
    remove(root);
  }

  static function remove(root:String):Void {
    if (!FileSystem.exists(root)) return;
    if (FileSystem.isDirectory(root)) {
      for (name in FileSystem.readDirectory(root)) remove(root + "/" + name);
      FileSystem.deleteDirectory(root);
    } else FileSystem.deleteFile(root);
  }

  static function modelOwnership():Void {
    var frame = materia.assembly.AssemblyFrames.identity();
    var definition:materia.assembly.AssemblyDefinition = {
      schemaVersion: materia.assembly.AssemblyDefinitionCodec.VERSION,
      id: "shared-fixture",
      lengthUnit: "mm",
      definitions: [{id: "body", connectors: [{name: "pin", frame: frame}]}],
      occurrences: [
        {id: "base", definition: "body", initialPose: frame},
        {id: "arm", definition: "body", initialPose: frame}
      ],
      joints: [{
        id: "slide",
        type: materia.assembly.AssemblyDefinition.AssemblyJointType.Prismatic,
        role: materia.assembly.AssemblyDefinition.AssemblyJointRole.Tree,
        parent: "base",
        parentConnector: "pin",
        child: "arm",
        childConnector: "pin",
        axis: {x: 1.0, y: 0.0, z: 0.0},
        limits: {lower: -10.0, upper: 10.0, velocity: null, effort: null},
        defaultValue: 0.0
      }]
    };
    var independent = new cadkit.modeling.AssemblyState(definition);
    definition.joints[0].defaultValue = 2.0;
    check(
      independent.joint("slide") == 0.0 && independent.definition.joints[0].defaultValue == 0.0,
      "editable definitions retain snapshot isolation"
    );
    var model = cadkit.modeling.CompiledAssembly.takeOwnership(definition);
    check(model.definition == definition, "owned flat topology is transferred without copying");
    var root = materia.assembly.AssemblyFrames.identity();
    root.x = 3.0;
    var saved:materia.assembly.AssemblyDefinition.AssemblyStateRecord = {
      schemaVersion: materia.assembly.AssemblyDefinitionCodec.VERSION,
      definition: definition.id,
      jointCoordinates: [{joint: "slide", value: 2.0}],
      rootPoses: [{occurrence: "base", pose: root}]
    };
    var first = cadkit.modeling.AssemblyState.fromModel(model, saved);
    var second = cadkit.modeling.AssemblyState.fromModel(model, saved);
    check(
      first.model == second.model && first.model.kinematics == second.model.kinematics,
      "states share topology and compiled kinematics"
    );
    first.setJoint("slide", 5.0);
    root.x = 90.0;
    check(
      first.worldPose("arm").x == 8.0 && second.worldPose("arm").x == 5.0,
      "joint values, root inputs and evaluated poses remain independent"
    );
    var placement = materia.assembly.AssemblyFrames.identity();
    placement.x = 4.0;
    first.setRootPose("base", placement);
    placement.x = 99.0;
    var record = first.record();
    record.rootPoses[0].pose.x = 100.0;
    check(
      first.worldPose("arm").x == 9.0 && second.worldPose("arm").x == 5.0,
      "setters and exported records cannot alias mutable root poses"
    );
    var replacement = cadkit.modeling.CompiledAssembly.snapshot(definition);
    check(
      replacement != model && replacement.kinematics != model.kinematics,
      "topology snapshots compile a fresh model"
    );
    Sys.println("Assembly model ownership tests passed");
  }

  public static function main():Int {
    modelOwnership();
    if (Sys.args().indexOf("--ownership") >= 0) return 0;
    var installation = MateriaProjectRunner.installationRoot();
    if (Sys.args().indexOf("--gantry") >= 0) {
      for (relative in [
        "gantry-picker/materia.project.json",
        "gantry-picker/materia.yaw.project.json",
        "gantry-welder/materia.project.json"
      ]) {
        var manifest = installation + "/machinekit/examples/" + relative;
        var started = Sys.time();
        var first = MateriaProjectRunner.loadProject(manifest);
        var cold = Sys.time() - started;
        var session = new app.ProjectDocumentSession(null, false);
        session.openGeneratedProject(first, manifest);
        var installed:cadkit.modeling.AssemblyState = @:privateAccess session.assemblyRuntime;
        check(
          first.assemblyModel != null && installed != null && installed.model == first.assemblyModel,
          "scene installation reuses the loader's compiled assembly model"
        );
        started = Sys.time();
        var repeatControl = new ProjectLoadControl();
        var second = MateriaProjectRunner.loadProject(manifest, null, repeatControl);
        check(!hasPhase(repeatControl, "Preparing geometry and physics"), "Gantry repeat skips preparation");
        check(Json.stringify(first.physical) == Json.stringify(second.physical), "cached Gantry physics");
        var warm = Sys.time() - started;
        check(first.objects.length > 0 && second.objects.length == first.objects.length, "cached Gantry components");
        check(
          Json.stringify(first.assemblyDefinition) == Json.stringify(second.assemblyDefinition),
          "cached Gantry assembly"
        );
        check(Json.stringify(first.missions[0]) == Json.stringify(second.missions[0]), "cached Gantry mission");
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
      var config:Dynamic = {
        version: 1,
        sourceRoots: ["."],
        target: "host",
        dependencies: {projectkit: {path: installation + "/projectkit"}}
      };
      Reflect.setField(config, "package", {name: "loading-fixture"});
      File.saveContent(root + "/haxeon.json", Json.stringify(config));
      var data = root + "/input.txt", counter = root + "/runs.txt";
      File.saveContent(data, "first");
      File.saveContent(counter, "");
      var source = 'import haxe.io.Bytes;\nimport sys.io.File;\nimport materia.project.SceneArtifact;\n'
        + 'class LoadingPreview { public static function alternate(?document:String):Bytes return preview("alternate");\n'
        + 'public static function preview(?document:String):Bytes {\n'
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
      var entry:Dynamic = {
        kind: "cad-preview",
        module: "LoadingPreview",
        documentInput: true,
        cache: {}
      };
      Reflect.setField(entry, "function", "preview");
      var manifest = root + "/materia.project.json";
      var project = {
        format: "materia.project",
        version: 1,
        defaultEntrypoint: "preview",
        build: {system: "haxeon", manifest: "haxeon.json"},
        entrypoints: {preview: entry}
      };
      File.saveContent(manifest, Json.stringify(project));
      var artifacts = root + "/cache/materia/memo";
      DiskStore.useProcess(new DiskStore(artifacts, 1e9));
      var coldControl = new ProjectLoadControl();
      var coldScene = MateriaProjectRunner.loadProject(manifest, null, coldControl);
      check(coldScene.recipeDocument == "first-v1", "cold output");
      check(hasPhase(coldControl, "Preparing geometry and physics"), "cold load prepares data");
      var preparedDirectory = root + "/cache/materia/prepared-scenes";
      check(files(preparedDirectory, ".mtrp").length == 1, "cold load caches portable preparation");
      check(stored(artifacts, "generator").length == 1, "cold load caches compiled code");
      check(stored(artifacts, "output").length == 1, "declared deterministic output is cached");
      check(files(root + "/cache/materia/project-tools", ".hl").length == 1, "the build tool runs as a compiled executable");
      check(hasPhase(coldControl, "Compiling the project"), "cold load compiles, and the compile reports its inputs");
      check(stored(artifacts, "record").length == 1, "cold load records the fingerprint's inputs");
      var warmControl = new ProjectLoadControl();
      var warmScene = MateriaProjectRunner.loadProject(manifest, null, warmControl);
      check(warmScene.recipeDocument == "first-v1", "warm output");
      check(!hasPhase(warmControl, "Compiling the project"), "warm load confirms the recorded fingerprint without compiling");
      // A new module could change resolution: its directory listing changes, so the program is compiled again (same fingerprint here, so the cached output is reused).
      File.saveContent(root + "/Unrelated.hx", "class Unrelated {}\n");
      var listingControl = new ProjectLoadControl();
      check(MateriaProjectRunner.loadProject(manifest, null, listingControl).recipeDocument == "first-v1", "new module output");
      check(hasPhase(listingControl, "Compiling the project"), "a new module compiles again");
      File.saveBytes(stored(artifacts, "record")[0], framed("{not json"));
      var recordControl = new ProjectLoadControl();
      check(MateriaProjectRunner.loadProject(manifest, null, recordControl).recipeDocument == "first-v1", "corrupt record output");
      check(hasPhase(recordControl, "Compiling the project"), "a corrupt fingerprint record is ignored");
      check(File.getContent(counter) == "x", "fingerprint records never change cached output");
      check(!hasPhase(warmControl, "Preparing geometry and physics"), "warm load skips preparation");
      check(
        Json.stringify(coldScene.physical) == Json.stringify(warmScene.physical),
        "prepared physical properties match"
      );
      check(Json.stringify(coldScene.objects) == Json.stringify(warmScene.objects), "prepared occurrences match");
      check(
        Json.stringify(coldScene.localCentersByDefinition) == Json.stringify(warmScene.localCentersByDefinition),
        "prepared centers match"
      );
      for (key in coldScene.geometryBySnapshot.keys()) {
        var fresh = coldScene.geometryBySnapshot.get(key), cached = warmScene.geometryBySnapshot.get(key);
        check(
          cached != null && fresh != cached
          && fresh.vertexCount() == cached.vertexCount() && fresh.triangleCount() == cached.triangleCount(),
          "fresh native geometry ownership with matching mesh counts"
        );
      }
      var artifactBytes = payload(stored(artifacts, "output")[0]);
      var digest = Sha256.make(artifactBytes), artifactHash = new StringBuf();
      var digits = "0123456789abcdef";
      for (i in 0...digest.length) {
        var value = digest.get(i);
        artifactHash.add(digits.charAt(value >>> 4));
        artifactHash.add(digits.charAt(value & 15));
      }
      var artifact = SceneArtifact.decode(artifactBytes);
      var portable = new PreparedProjectCache(artifactHash.toString(),
        app.PreparedSceneCaches.current()).read(artifact.parts);
      check(portable != null && portable.length == artifact.parts.length, "portable prepared data decodes");
      for (i in 0...artifact.parts.length) {
        var item = portable[i];
        var expected = CadPreviewGeometry.prepare(
          artifact.parts[i],
          item.geometry.minimum,
          item.geometry.maximum,
          artifact.metresPerUnit
        );
        check(
          sameBytes(expected.positions, item.geometry.positions) && sameBytes(expected.normals, item.geometry.normals),
          "cached packed streams match fresh conversion byte for byte"
        );
      }
      var preparedPath = files(preparedDirectory, ".mtrp")[0];
      var memory = new MemoryPreparedStore();
      var context:app.PreparedSceneCaches.PreparedSceneCacheContext = {store: memory, buildIdentity: "fixture-build-a"};
      var loader = new app.ProjectArtifactLoader(context);
      check(
        loader.tryLoadCached(artifactBytes) == null && memory.writes == 0,
        "cache-only lookup never computes or publishes missing preparation"
      );
      loader.load(artifactBytes);
      var cachedControl = new ProjectLoadControl();
      loader.load(artifactBytes, cachedControl);
      check(
        memory.writes == 1 && !hasPhase(cachedControl, "Preparing geometry and physics"),
        "injected store reuses preparation"
      );
      var fast = loader.tryLoadCached(artifactBytes);
      check(
        fast != null && fast.objects.length == coldScene.objects.length && memory.writes == 1,
        "cache-only lookup materializes a valid cached scene without rewriting it"
      );
      context.buildIdentity = "fixture-build-b";
      loader.load(artifactBytes);
      check(memory.writes == 2, "build identity invalidates preparation");
      memory.unavailable = true;
      check(loader.load(artifactBytes).objects.length == coldScene.objects.length,
        "unavailable cache remains optional");
      var workerStore = new MemoryPreparedStore();
      var workerLoader = new app.ProjectArtifactLoader({store: workerStore, buildIdentity: "worker-fixture"});
      var packed = app.PreparedSceneCodec.encode(portable);
      var workerControl = new ProjectLoadControl();
      var workerScene = workerLoader.load(artifactBytes, workerControl, packed, true);
      check(
        workerStore.writes == 1 && !hasPhase(workerControl,
        "Preparing geometry and physics") && Json.stringify(workerScene.physical) == Json.stringify(coldScene.physical),
        "worker data materializes without preparation and publishes exact portable physics"
      );
      packed.set(packed.length - 1, packed.get(packed.length - 1) ^ 1);
      var workerRejected = false;
      try workerLoader.load(artifactBytes, null, packed, true) catch (_:Dynamic) workerRejected = true;
      check(workerRejected && workerStore.writes == 1, "invalid worker data fails without cache publication");
      var exported = root + "/Standalone.mtrg";
      File.saveBytes(exported, artifactBytes);
      check(
        app.ProjectSourceLoader.executionRequirement(exported).kind == "prebuilt-artifact",
        "artifact requires no source execution"
      );
      check(
        app.editor.ProjectUiExtension.open(exported) == null,
        "binary artifacts are never parsed as project UI manifests"
      );
      check(
        app.ProjectSourceLoader.load(exported).objects.length == coldScene.objects.length,
        "standalone artifact opens without generator"
      );
      check(File.getContent(counter) == "x", "artifact opens never execute source");
      var standalone = SceneArtifact.decode(artifactBytes);
      standalone.recipeDocument = null;
      var exportedBytes = SceneArtifact.encode(standalone);
      File.saveBytes(exported, exportedBytes);
      var hasNul = false;
      for (index in 0...exportedBytes.length) if (exportedBytes.get(index) == 0) hasNul = true;
      // Reading this file as text throws "HashLink String cannot contain NUL"; a document open must read bytes.
      check(hasNul, "the .mtrg fixture is binary");
      var session = new app.ProjectDocumentSession(null, false);
      session.open(exported);
      session.save(root + "/Saved.materia");
      var reopened = new app.ProjectDocumentSession(null, false);
      reopened.open(root + "/Saved.materia");
      check(
        reopened.projectReference == exported && reopened.scene.records().length == coldScene.objects.length,
        "saved document retains artifact reference and reopens without source"
      );
      File.saveContent(root + "/Broken.mtrg", "bad");
      var rejected = false;
      try reopened.open(root + "/Broken.mtrg") catch (_:Dynamic) rejected = true;
      check(
        rejected
        && reopened.projectReference == exported && reopened.scene.records().length == coldScene.objects.length,
        "failed artifact replacement preserves current document"
      );

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
      check(
        MateriaProjectRunner.loadProject(manifest, "recipe-a").recipeDocument == "recipe-a-v1",
        "first editable input"
      );
      check(files(preparedDirectory,
        ".mtrp").length > preparedBeforeRecipe, "changed artifact selects new preparation");
      check(
        MateriaProjectRunner.loadProject(manifest, "recipe-b").recipeDocument == "recipe-b-v1",
        "changed editable input"
      );
      check(stored(artifacts, "generator").length == 1, "recipe changes reuse compiled code");
      check(File.getContent(counter) == "xxx", "each new editable input executes once");
      check(
        MateriaProjectRunner.loadProject(manifest, "recipe-a").recipeDocument == "recipe-a-v1",
        "unchanged editable input reopens"
      );
      check(File.getContent(counter) == "xxx", "unchanged editable input hits the output cache");
      for (path in stored(artifacts, "output")) File.saveBytes(path, framed("corrupt"));
      check(MateriaProjectRunner.loadProject(manifest).recipeDocument == "first-v1", "corrupt output regenerates");
      check(File.getContent(counter) == "xxxx", "corrupt output executes again");
      File.saveContent(data, "other");
      check(
        MateriaProjectRunner.loadProject(manifest).recipeDocument == "other-v1",
        "same-size declared input edit invalidates output"
      );
      File.saveContent(root + "/LoadingPreview.hx", StringTools.replace(source, "-v1", "-v2"));
      var editControl = new ProjectLoadControl();
      check(
        MateriaProjectRunner.loadProject(manifest, null, editControl).recipeDocument == "other-v2",
        "same-size source edit invalidates code and output"
      );
      check(hasPhase(editControl, "Compiling the project"), "a source edit fails fingerprint confirmation");
      Reflect.deleteField(entry, "cache");
      File.saveContent(manifest, Json.stringify(project));
      var before = stored(artifacts, "output").length;
      MateriaProjectRunner.loadProject(manifest);
      var compiled = stored(artifacts, "generator").length;
      MateriaProjectRunner.loadProject(manifest);
      check(stored(artifacts, "generator").length == compiled, "uncached output still reuses compiled code");
      check(stored(artifacts, "output").length == before, "uncached generators never cache output");
      check(File.getContent(counter).length == 8, "uncached generator executes on each open");
      var control = new ProjectLoadControl();
      control.cancel();
      var cancelled = false;
      try MateriaProjectRunner.loadProject(manifest,
        null, control) catch (error:Dynamic) cancelled = Std.string(error) == ProjectLoadControl.CANCELLED;
      check(cancelled && File.getContent(counter).length == 8, "cancelled open executes no generator");
      // Variants select entrypoints; each keeps its own cached output, and switching never serves the other's.
      File.saveContent(manifest, '{"format":"materia.project","version":1,"defaultEntrypoint":"preview",'
        + '"build":{"system":"haxeon","manifest":"haxeon.json"},"entrypoints":{'
        + '"preview":{"kind":"cad-preview","module":"LoadingPreview","function":"preview","cache":{}},'
        + '"alternate":{"kind":"cad-preview","module":"LoadingPreview","function":"alternate","cache":{}}},'
        + '"defaultVariant":"main","variants":['
        + '{"id":"main","label":"Main","entrypoint":"preview","kind":"configuration","summary":"Main."},'
        + '{"id":"other","label":"Other","entrypoint":"alternate","kind":"study","summary":"Other."}]}');
      var runsBeforeVariants = File.getContent(counter).length;
      check(MateriaProjectRunner.loadProject(manifest, null, null, "main").recipeDocument == "other-v2", "default variant output");
      check(
        MateriaProjectRunner.loadProject(manifest, null, null, "other").recipeDocument == "alternate-v2",
        "variant change misses the other variant's output"
      );
      check(MateriaProjectRunner.loadProject(manifest, null, null, "main").recipeDocument == "other-v2", "variant switch back hits");
      check(MateriaProjectRunner.loadProject(manifest, null, null, "other").recipeDocument == "alternate-v2", "variant hits");
      check(File.getContent(counter).length == runsBeforeVariants + 2, "each variant executes once, then reuses its output");
      // Artifacts are store entries like any other: a store over its cap evicts them, and the next open regenerates the same output.
      var runsBeforeEviction = File.getContent(counter).length;
      var kept = MateriaProjectRunner.loadProject(manifest, null, null, "main").recipeDocument;
      check(stored(artifacts, "output").length > 0 && stored(artifacts, "generator").length > 0 && stored(artifacts, "record").length > 0,
        "the store holds an output, a generator and a record");
      var tiny = new DiskStore(artifacts, 1.0);
      tiny.evict();
      check(files(artifacts, ".memo").length == 0, "a store over its cap evicts artifacts, generators and records");
      var regenerated = MateriaProjectRunner.loadProject(manifest, null, null, "main").recipeDocument;
      check(regenerated == kept, "after eviction the next open regenerates the same output");
      check(File.getContent(counter).length == runsBeforeEviction + 1, "the evicted output was generated again");
    } catch (error:Dynamic) {
      Sys.putEnv("XDG_CACHE_HOME", previousCache);
      DiskStore.useProcess(null);
      remove(root);
      throw error;
    }
    Sys.putEnv("XDG_CACHE_HOME", previousCache);
    DiskStore.useProcess(null);
    remove(root);
    kernelCacheInGeneration(installation);
    nativeIdentityTracksBuild(installation);
    generatorInputs(installation);
    Sys.println("Project loading tests passed");
    return 0;
  }
}

private class MemoryPreparedStore implements app.PreparedSceneStore {
  var entries:Map<String, Bytes> = new Map();
  public var writes:Int = 0;
  public var unavailable:Bool = false;
  public function new() {
  }
  public function read(key:String):Null<Bytes> {
    if (unavailable) throw "storage unavailable";
    return entries.get(key);
  }
  public function write(key:String, bytes:Bytes):Void {
    if (unavailable) throw "storage unavailable";
    entries.set(key, bytes);
    writes++;
  }
}
