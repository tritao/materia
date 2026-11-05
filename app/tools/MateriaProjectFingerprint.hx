import build.Target;
import build.execution.ContentDigest;
import haxe.io.Path;
import project.PackageResolver;
import project.PathSourceAcquirer;
import sys.FileSystem;
import sys.io.File;

/** Fingerprint the declared inputs of a deterministic CAD preview entrypoint. */
class MateriaProjectFingerprint {
  static var generatedSources:String;
  /** Caches are outputs (Materia's and the compiler's); a source root that contains XDG_CACHE_HOME must not list them. */
  static var cacheRoot:String;
  static function main():Void {
    var args = Sys.args();
    if (args.length
      < 6) throw "usage: MateriaProjectFingerprint <manifest> <module> <function> <tools-dir> <haxeon-root> <document-input> [input-file ...]";
    var manifest = FileSystem.fullPath(args[0]);
    var project = new PackageResolver(new PathSourceAcquirer()).resolve(manifest, null, false, Target.parse("host"));
    var home = FileSystem.fullPath(args[4]);
    var cache = Sys.getEnv("XDG_CACHE_HOME");
    if (cache == null || cache.length == 0) cache = Path.join([Sys.getEnv("HOME"), ".cache"]);
    generatedSources = Path.normalize(Path.join([FileSystem.fullPath(cache), "materia", "project-sources"]));
    cacheRoot = Path.normalize(FileSystem.fullPath(cache));
    var paths = new Map<String, Bool>();
    add(paths, manifest);
    for (item in project.packages.packages) {
      add(paths, Path.join([item.root, PackageResolver.MANIFEST_NAME]));
      for (path in item.ffiInterfaces) add(paths, path);
      for (path in item.ffiProjections) add(paths, path);
      for (imported in item.ffiImports) {
        add(paths, imported.manifestPath);
        add(paths, imported.header);
      }
    }
    var request = MateriaProjectModuleBuild.request([
      manifest,
      args[1],
      args[2],
      Path.join([cache, "materia", "fingerprint-analysis"]),
      home,
      args[5]
    ]);
    var parsed = compiler.tools.CompilerArguments.parse(request.arguments);
    var compiler = new compiler.tools.CompilerSession().prepare(parsed, _ -> {});
    var analysis = compiler.analyze(parsed.entry);
    var moduleSources:Array<{name:String, path:String, text:String}> = [];
    for (name in analysis.moduleNames) {
      var state = compiler.modules.get(name);
      if (state == null) throw "Missing analyzed module: " + name;
      // Include the deterministic wrapper: it encodes the function and document-input contract.
      moduleSources.push({name: name, path: state.source.path, text: state.source.text});
    }
    // Compiler implementation changes still invalidate generated code.
    addHaxeSources(paths, Path.join([home, "src"]));
    for (name in ["MateriaProjectModuleBuild.hx", "MateriaProjectFingerprint.hx"]) add(
      paths,
      Path.join([args[3], name])
    );
    for (index in 6...args.length) add(paths, args[index]);
    var ordered = [for (path in paths.keys()) path];
    ordered.sort(Reflect.compare);
    var fields = new StringBuf();
    field(fields, "materia-generated-artifact-v3");
    // These options affect generated code even when source bytes are unchanged.
    for (name in OPTIONS) field(fields, name + ":" + optionValue(name));
    field(fields, args[1]);
    field(fields, args[2]);
    moduleSources.sort((a, b) -> Reflect.compare(a.name, b.name));
    for (source in moduleSources) {
      field(fields, source.name);
      field(fields, source.path);
      field(fields, source.text);
    }
    // The record lets the editor confirm this fingerprint without re-running the module analysis: the same file contents
    // and the same module-resolution directories (and binaries and options below) give the same analysis and fingerprint.
    var recordFiles:Array<Array<String>> = [];
    var hashedSources = new Map<String, Bool>();
    // Explicit sources (the entry module, the wrapper) are named relative to the request; library modules are absolute.
    var explicitSources = [for (path in parsed.paths) FileSystem.exists(path) ? FileSystem.fullPath(path) : null];
    // FFI projection modules (CadKit.hx from CadKit.hxi/.hxmap) are generated from files already hashed above.
    var projections = new Map<String, Bool>();
    for (path in parsed.ffiInterfaces.concat(parsed.ffiProjections)) projections.set(Path.withoutExtension(Path.withoutDirectory(path)) + ".hx", true);
    var recordable = true;
    for (source in moduleSources) {
      if (!Path.isAbsolute(source.path) && projections.exists(source.path) && sourceFile(source.path, explicitSources) == null) continue;
      var absolute = sourceFile(source.path, explicitSources);
      // Every analyzed module must be confirmable from its file; otherwise the editor always runs this analysis.
      if (absolute == null || ContentDigest.make(File.getBytes(absolute)).toHex() != ContentDigest.make(haxe.io.Bytes.ofString(source.text)).toHex()) {
        recordable = false;
        continue;
      }
      if (paths.exists(absolute) || hashedSources.exists(absolute)) continue;
      hashedSources.set(absolute, true);
      recordFiles.push([absolute, ContentDigest.make(haxe.io.Bytes.ofString(source.text)).toHex()]);
    }
    for (path in ordered) {
      var digest = ContentDigest.make(File.getBytes(path)).toHex();
      field(fields, path);
      field(fields, digest);
      recordFiles.push([path, digest]);
    }
    var recordDirectories:Array<Array<String>> = [];
    var visited = new Map<String, Bool>();
    for (root in parsed.roots) listSourceDirectories(root, recordDirectories, visited);
    for (root in parsed.packageRoots) listSourceDirectories(root.path, recordDirectories, visited);
    // Native code and tool binaries are not Haxe sources. Their build stamps
    // invalidate generated geometry without reading large shared libraries.
    var binaries = [
      Path.join([home, ".tools", "haxe", "haxe"]),
      build.HashLinkPath.executable(home),
      Path.join([home, "out", "haxeon_runtime.hdll"])
    ];
    // The generator loads every package's native libraries (CadKit's, StockKit's, ...), each built in its own directory.
    var nativeRoot = Path.join([Path.directory(args[3]), "build", "host", "native"]);
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
        recordBinaries.push([path, "missing", ""]);
        continue;
      }
      var stat = FileSystem.stat(path);
      // Whole Unix seconds, as Haxeon's FileSystem.metadata reports them to the editor.
      recordBinaries.push([path, Std.string(stat.size), Std.string(Std.int(stat.mtime.getTime() / 1000.0))]);
      // Empty files contain no executable native code.
      if (stat.size == 0) continue;
      field(fields, path);
      field(fields, Std.string(stat.size));
      field(fields, Std.string(stat.mtime.getTime()));
    }
    var fingerprint = ContentDigest.make(haxe.io.Bytes.ofString(fields.toString())).toHex();
    Sys.println(fingerprint);
    // Second line: what the fingerprint depends on, for MateriaProjectRunner's fast confirmation.
    if (recordable) Sys.println(haxe.Json.stringify({
      version: RECORD_VERSION,
      fingerprint: fingerprint,
      options: [for (name in OPTIONS) [name, optionValue(name)]],
      files: recordFiles,
      directories: recordDirectories,
      nativeRoot: nativeRoot,
      binaries: recordBinaries
    }));
  }

  /** Must match MateriaProjectRunner.FINGERPRINT_RECORD_VERSION. */
  static inline var RECORD_VERSION:Int = 1;
  static final OPTIONS = ["HAXEON_INLINE", "HAXEON_LOADSTORE", "HAXEON_STRENGTH"];

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
    out.push([absolute, ContentDigest.make(haxe.io.Bytes.ofString(names.join("\n"))).toHex()]);
    for (path in directories) if (Path.normalize(path) != cacheRoot) listSourceDirectories(path, out, visited);
  }

  static function add(paths:Map<String, Bool>, path:String):Void {
    // HashLink fullPath can return null for a missing file. Validate before resolving it.
    if (path == null
      || !FileSystem.exists(path) || FileSystem.isDirectory(path)) throw 'CAD cache input is not a file: $path';
    var absolute = FileSystem.fullPath(path);
    paths.set(absolute, true);
  }

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
