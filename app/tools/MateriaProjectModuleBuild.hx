import compiler.tools.CompilerDriver;
import build.execution.CompilerClient;
import haxe.crypto.Sha256;
import compiler.tools.CompilerArguments;
import haxe.io.Path;
import project.PackageResolver;
import project.PathSourceAcquirer;
import build.Target;
import sys.FileSystem;
import sys.io.File;
import build.execution.ContentDigest;

/** Build a project entrypoint as a separate HashLink artifact generator. */
class MateriaProjectModuleBuild {
  public static function request(args:Array<String>): {
    arguments: Array<String>,
    home: String,
    wrapperRoot: String,
    reusable: Bool,
    manifestPath: String,
    declared: Array<String>
  } {
    if (args.length
      < 7) throw "usage: MateriaProjectModuleBuild <haxeon.json> <module> <function> <output-prefix> <haxeon-root> <document-input> <tools-dir> [input-file ...]";
    var manifestPath = Path.normalize(FileSystem.fullPath(args[0]));
    var moduleName = args[1];
    var functionName = args[2];
    var outputPrefix = args[3];
    var identifier = ~/^[A-Za-z_][A-Za-z0-9_]*$/;
    for (part in moduleName
      .split(".")) if (!identifier.match(part)) throw "Project entrypoint has an invalid module name";
    if (!identifier.match(functionName)) throw "Project entrypoint has an invalid function name";
    var project = new PackageResolver(new PathSourceAcquirer()).resolve(manifestPath,
      null, false, Target.parse("host"));
    var home = FileSystem.fullPath(args[4]);
    // Use the same request preparation, batch mode and artifact writer as Haxeon builds.
    Sys.setCwd(home);
    var cache = Sys.getEnv("XDG_CACHE_HOME");
    if (cache == null || cache.length == 0) cache = Path.join([Sys.getEnv("HOME"), ".cache"]);
    var wrapperRoot = Path.join([
      cache,
      "materia",
      "project-sources",
      Sha256.encode(manifestPath + "\n" + moduleName + "\n" + functionName + "\n" + args[5])
    ]);
    var reusable = true;
    try FileSystem.createDirectory(wrapperRoot) catch (_:Dynamic) {
      wrapperRoot = Path.directory(outputPrefix);
      reusable = false;
    }
    var declared = new Map<String, Bool>();
    declare(declared, manifestPath);
    for (item in project.packages.packages) {
      declare(declared, Path.join([item.root, PackageResolver.MANIFEST_NAME]));
      for (path in item.ffiInterfaces) declare(declared, path);
      for (path in item.ffiProjections) declare(declared, path);
      for (imported in item.ffiImports) {
        declare(declared, imported.manifestPath);
        declare(declared, imported.header);
      }
    }
    var arguments = [
      "--target=hl",
      "--output=" + outputPrefix + ".hl",
      "--inputs-output=" + outputPrefix + ".inputs.json",
      "--entry=MateriaGeneratedEntrypoint",
      "--root=" + wrapperRoot
    ];
    for (packageValue in project.packages.packages) {
      for (root in packageValue.sourceRoots) arguments.push("--root=" + root);
      for (path in packageValue.ffiInterfaces) arguments.push("--ffi-interface=" + path);
      for (path in packageValue.ffiProjections) arguments.push("--ffi-projection=" + path);
    }
    for (define in project.manifest.defines) arguments.push("--define=" + define);
    if (project.manifest.inlineEnabled
      != null) arguments.push("--define=haxeon-inline=" + (project.manifest.inlineEnabled ? "1" : "0"));

    var modulePath = moduleName.split(".").join("/") + ".hx";
    var entryPath:Null<String> = null;
    for (root in project.rootPackage.sourceRoots) {
      var candidate = Path.join([root, modulePath]);
      if (FileSystem.exists(candidate)) {
        entryPath = candidate;
        break;
      }
    }
    if (entryPath == null) throw 'Project entrypoint module "$moduleName" is not under a package source root';
    arguments.push(entryPath);
    var documentInput = args[5] == "true";
    var invocation = documentInput
      ? "(args.length == 2 ? " + moduleName + "." + functionName + "(sys.io.File.getContent(args[1])) : "
      + moduleName + "." + functionName + "())" : moduleName + "." + functionName + "()";
    // The run notes the files it reads (`--record-inputs=FILE`, given by the runner) and writes the list when it is done.
    // The document is already part of the cache key, so it is not noted as a file.
    var wrapper = "class MateriaGeneratedEntrypoint {\n" + "  static function main():Void {\n"
      + "    var args = haxeon.build.InputFiles.arguments();\n" + "    if (args.length != 1" + (documentInput
      ? " && args.length != 2" : "") + ") throw \"Project generator requires an output path\";\n"
      + (documentInput ? "    if (args.length == 2) haxeon.build.InputFiles.ignoreFile(args[1]);\n" : "")
      + "    sys.io.File.saveBytes(args[0], " + invocation + ");\n" + "    haxeon.build.InputFiles.save();\n" + "  }\n" + "}\n";
    var wrapperPath = Path.join([wrapperRoot, "MateriaGeneratedEntrypoint.hx"]);
    if (!FileSystem.exists(wrapperPath) || File.getContent(wrapperPath) != wrapper) File.saveContent(
      wrapperPath,
      wrapper
    );
    arguments.push(wrapperPath);
    return {
      arguments: arguments,
      home: home,
      wrapperRoot: wrapperRoot,
      reusable: reusable,
      manifestPath: manifestPath,
      declared: [for (path in declared.keys()) path]
    };
  }

  static var generatedSources:String;
  /** Caches are outputs (Materia's and the compiler's); a source root that contains XDG_CACHE_HOME must not list them. */
  static var cacheRoot:String;
  /** Must match MateriaProjectRunner.FINGERPRINT_RECORD_VERSION. */
  static inline var RECORD_VERSION:Int = 4;
  static final OPTIONS = ["HAXEON_INLINE", "HAXEON_LOADSTORE", "HAXEON_STRENGTH"];

  /**
   * Compiles the entrypoint, then writes `<output-prefix>.record.json`: the fingerprint of everything this build read, and
   * the record that lets the editor confirm that fingerprint later by rehashing those files (see MateriaProjectRunner). The
   * modules come from the compile itself (`--inputs-output`), so no second analysis of the module graph runs.
   */
  static function main():Void {
    var args = Sys.args();
    var prepared = request(args);
    var arguments = prepared.arguments, home = prepared.home,
      wrapperRoot = prepared.wrapperRoot, manifestPath = prepared.manifestPath, reusable = prepared.reusable;
    var oneShot = function() {
      CompilerDriver.compile(CompilerArguments.parse(arguments));
      return 0;
    };
    var status = reusable ? CompilerClient.run(
      Path.join([home, ".tools", "haxe", "haxe"]),
      Path.join([home, "src"]),
      arguments,
      home,
      wrapperRoot,
      Path.directory(manifestPath),
      oneShot
    ) : oneShot();
    if (status != 0) Sys.exit(status);
    writeRecord(args, prepared.arguments, prepared.declared);
  }

  static function writeRecord(args:Array<String>, compilerArguments:Array<String>, declared:Array<String>):Void {
    var outputPrefix = args[3], home = FileSystem.fullPath(args[4]), tools = args[6];
    var cache = Sys.getEnv("XDG_CACHE_HOME");
    if (cache == null || cache.length == 0) cache = Path.join([Sys.getEnv("HOME"), ".cache"]);
    generatedSources = Path.normalize(Path.join([FileSystem.fullPath(cache), "materia", "project-sources"]));
    cacheRoot = Path.normalize(FileSystem.fullPath(cache));
    var parsed = CompilerArguments.parse(compilerArguments);
    var paths = new Map<String, Bool>();
    for (path in declared) add(paths, path);
    for (index in 7...args.length) add(paths, args[index]);
    // Compiler implementation changes still invalidate generated code.
    addHaxeSources(paths, Path.join([home, "src"]));
    add(paths, Path.join([tools, "MateriaProjectModuleBuild.hx"]));
    var rows:Array<Array<String>> = haxe.Json.parse(File.getContent(outputPrefix + ".inputs.json"));
    rows.sort((a, b) -> Reflect.compare(a[0], b[0]));
    var ordered = [for (path in paths.keys()) path];
    ordered.sort(Reflect.compare);
    var fields = new StringBuf();
    field(fields, "materia-generated-artifact-v5");
    // These options affect generated code even when source bytes are unchanged.
    for (name in OPTIONS) field(fields, name + ":" + optionValue(name));
    field(fields, args[1]);
    field(fields, args[2]);
    for (row in rows) {
      field(fields, row[0]);
      field(fields, row[1]);
      field(fields, row[2]);
    }
    // The record lets the editor confirm this fingerprint without compiling: the same file contents and the same
    // module-resolution directories (and binaries and options below) give the same compile and fingerprint.
    var recordFiles:Array<Array<String>> = [];
    var hashedSources = new Map<String, Bool>();
    // Explicit sources (the entry module, the wrapper) are named relative to the request; library modules are absolute.
    var explicitSources = [for (path in parsed.paths) FileSystem.exists(path) ? FileSystem.fullPath(path) : null];
    // FFI projection modules (CadKit.hx from CadKit.hxi/.hxmap) are generated from files already hashed above.
    var projections = new Map<String, Bool>();
    for (path in parsed.ffiInterfaces.concat(parsed.ffiProjections)) projections.set(Path.withoutExtension(Path.withoutDirectory(path)) + ".hx", true);
    var recordable = true;
    for (row in rows) {
      var path = row[1];
      if (!Path.isAbsolute(path) && projections.exists(path) && sourceFile(path, explicitSources) == null) continue;
      var absolute = sourceFile(path, explicitSources);
      // Every compiled module must be confirmable from its file; otherwise the editor always compiles.
      if (absolute == null || ContentDigest.makeHex(File.getBytes(absolute)) != row[2]) {
        recordable = false;
        continue;
      }
      if (paths.exists(absolute) || hashedSources.exists(absolute)) continue;
      hashedSources.set(absolute, true);
      recordFiles.push([absolute, row[2]]);
    }
    for (path in ordered) {
      var digest = ContentDigest.makeHex(File.getBytes(path));
      field(fields, path);
      field(fields, digest);
      recordFiles.push([path, digest]);
    }
    var recordDirectories:Array<Array<String>> = [];
    var visited = new Map<String, Bool>();
    for (root in parsed.roots) listSourceDirectories(root, recordDirectories, visited);
    for (root in parsed.packageRoots) listSourceDirectories(root.path, recordDirectories, visited);
    // Native code and tool binaries are not Haxe sources; their content invalidates generated geometry.
    var binaries = [
      Path.join([home, ".tools", "haxe", "haxe"]),
      build.HashLinkPath.executable(home),
      Path.join([home, "out", "haxeon_runtime.hdll"])
    ];
    // The generator loads every package's native libraries (CadKit's, StockKit's, ...), each built in its own directory.
    var nativeRoot = Path.join([Path.directory(tools), "build", "host", "native"]);
    if (FileSystem.exists(nativeRoot)) for (kit in FileSystem.readDirectory(nativeRoot)) {
      var nativeDirectory = Path.join([nativeRoot, kit]);
      if (FileSystem
        .isDirectory(nativeDirectory)) for (name in FileSystem.readDirectory(nativeDirectory)) if (StringTools.endsWith(
          name,
          ".hdll"
        ) || name.indexOf(".so") >= 0) binaries.push(Path.join([
          nativeDirectory,
          name
        ]));
    }
    binaries.sort(Reflect.compare);
    var recordBinaries:Array<Array<String>> = [];
    for (path in binaries) {
      if (!FileSystem.exists(path)) {
        recordBinaries.push([path, "missing", "", "", ""]);
        continue;
      }
      var digest = ContentDigest.makeHex(File.getBytes(path));
      // Whole Unix seconds, as Haxeon's FileSystem.metadata reports them to the runner.
      var stat = FileSystem.stat(path);
      recordBinaries.push([
        path,
        digest,
        Std.string(stat.size),
        Std.string(Std.int(stat.mtime.getTime() / 1000.0)),
        Std.string(Std.int(stat.ctime.getTime() / 1000.0))
      ]);
      field(fields, path);
      field(fields, digest);
    }
    var fingerprint = ContentDigest.makeHex(haxe.io.Bytes.ofString(fields.toString()));
    File.saveContent(outputPrefix + ".record.json", haxe.Json.stringify({
      version: RECORD_VERSION,
      fingerprint: fingerprint,
      recordable: recordable,
      options: [for (name in OPTIONS) [name, optionValue(name)]],
      files: recordFiles,
      directories: recordDirectories,
      nativeRoot: nativeRoot,
      binaries: recordBinaries
    }));
  }

  /** The file a module was read from: its own path when that resolves, else the one explicit source it names. */
  static function sourceFile(path:String, explicitSources:Array<Null<String>>):Null<String> {
    if (Path.isAbsolute(path)) return FileSystem.exists(path) && !FileSystem.isDirectory(path) ? FileSystem.fullPath(path) : null;
    var found:Null<String> = null;
    for (candidate in explicitSources) if (candidate != null && (candidate == path || StringTools.endsWith(candidate, "/" + path))) {
      if (found != null && found != candidate) return null;
      found = candidate;
    }
    return found;
  }

  static function optionValue(name:String):String return Sys.getEnv(name) == "0" ? "off" : "on";

  /**
   * Module resolution reads only directory names and `.hx` names, so a directory's sorted list of those (hidden entries
   * excluded: they are never packages) changes whenever a module could appear or disappear. MateriaProjectRunner computes
   * the same digest.
   */
  static function listSourceDirectories(root:String, out:Array<Array<String>>, visited:Map<String, Bool>):Void {
    if (!FileSystem.exists(root) || !FileSystem.isDirectory(root)) {
      out.push([Path.normalize(root), "missing"]);
      return;
    }
    var absolute = FileSystem.fullPath(root);
    if (visited.exists(absolute)) return;
    visited.set(absolute, true);
    var names:Array<String> = [];
    var directories:Array<String> = [];
    for (name in FileSystem.readDirectory(absolute)) {
      if (StringTools.startsWith(name, ".")) continue;
      var path = Path.join([absolute, name]);
      if (FileSystem.isDirectory(path)) {
        names.push(name + "/");
        directories.push(path);
      } else if (StringTools.endsWith(name, ".hx")) names.push(name);
    }
    names.sort(Reflect.compare);
    out.push([absolute, ContentDigest.makeHex(haxe.io.Bytes.ofString(names.join("\n")))]);
    for (path in directories) if (Path.normalize(path) != cacheRoot) listSourceDirectories(path, out, visited);
  }

  /** A declared input of the request: it must exist, and joins the fingerprint by content. */
  static function declare(paths:Map<String, Bool>, path:String):Void {
    if (path == null || !FileSystem.exists(path) || FileSystem.isDirectory(path)) throw 'CAD cache input is not a file: $path';
    paths.set(FileSystem.fullPath(path), true);
  }

  static function add(paths:Map<String, Bool>, path:String):Void declare(paths, path);

  static function addHaxeSources(paths:Map<String, Bool>, root:String):Void {
    // A caller may put XDG_CACHE_HOME inside a broad source root. Generated wrappers are outputs.
    if (Path.normalize(FileSystem.fullPath(root)) == generatedSources) return;
    if (!FileSystem.exists(root) || !FileSystem.isDirectory(root)) throw 'Haxeon source directory is missing: $root';
    for (name in FileSystem.readDirectory(root)) {
      var path = Path.join([root, name]);
      if (FileSystem.isDirectory(path)) addHaxeSources(paths, path);
      else if (StringTools.endsWith(name, ".hx")) add(paths, path);
    }
  }

  static function field(buffer:StringBuf, value:String):Void {
    buffer.add(value.length);
    buffer.add(":");
    buffer.add(value);
    buffer.add("\n");
  }
}
