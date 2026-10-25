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
	static function main():Void {
		var args = Sys.args();
		if (args.length < 6)
			throw "usage: MateriaProjectFingerprint <manifest> <module> <function> <tools-dir> <haxeon-root> <document-input> [input-file ...]";
		var manifest = FileSystem.fullPath(args[0]);
		var project = new PackageResolver(new PathSourceAcquirer()).resolve(manifest, null, false, Target.parse("host"));
		var home = FileSystem.fullPath(args[4]);
		var cache = Sys.getEnv("XDG_CACHE_HOME");
		if (cache == null || cache.length == 0) cache = Path.join([Sys.getEnv("HOME"), ".cache"]);
		generatedSources = Path.normalize(Path.join([FileSystem.fullPath(cache), "materia", "project-sources"]));
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
		var request = MateriaProjectModuleBuild.request([manifest,args[1],args[2],
			Path.join([cache,"materia","fingerprint-analysis"]),home,args[5]]);
		var parsed = compiler.tools.CompilerArguments.parse(request.arguments);
		var compiler = new compiler.tools.CompilerSession().prepare(parsed, _ -> {});
		var analysis = compiler.analyze(parsed.entry);
		var moduleSources:Array<{name:String,path:String,text:String}> = [];
		for (name in analysis.moduleNames) {
			var state = compiler.modules.get(name);
			if (state == null) throw "Missing analyzed module: " + name;
			// Include the deterministic wrapper: it encodes the function and document-input contract.
			moduleSources.push({name:name,path:state.source.path,text:state.source.text});
		}
		// Compiler implementation changes still invalidate generated code.
		addHaxeSources(paths, Path.join([home, "src"]));
		for (name in ["MateriaProjectModuleBuild.hx", "MateriaProjectFingerprint.hx"])
			add(paths, Path.join([args[3], name]));
		for (index in 6...args.length) add(paths, args[index]);
		var ordered = [for (path in paths.keys()) path];
		ordered.sort(Reflect.compare);
		var fields = new StringBuf();
		field(fields, "materia-generated-artifact-v3");
		// These options affect generated code even when source bytes are unchanged.
		for (name in ["HAXEON_INLINE", "HAXEON_LOADSTORE", "HAXEON_STRENGTH"])
			field(fields, name + ":" + (Sys.getEnv(name) == "0" ? "off" : "on"));
		field(fields, args[1]);
		field(fields, args[2]);
		moduleSources.sort((a,b) -> Reflect.compare(a.name,b.name));
		for (source in moduleSources) {
			field(fields,source.name);field(fields,source.path);field(fields,source.text);
		}
		for (path in ordered) {
			field(fields, path);
			field(fields, ContentDigest.make(File.getBytes(path)).toHex());
		}
		// Native code and tool binaries are not Haxe sources. Their build stamps
		// invalidate generated geometry without reading large shared libraries.
		var binaries = [Path.join([home, ".tools", "haxe", "haxe"]),
				Path.join([home, ".tools", "hashlink", "hl"]),
				Path.join([home, "out", "haxeon_runtime.hdll"])];
		// The generator loads every package's native libraries (CadKit's, StockKit's, ...), each built in its own directory.
		var nativeRoot = Path.join([Path.directory(args[3]), "build", "host", "native"]);
		if (FileSystem.exists(nativeRoot)) for (kit in FileSystem.readDirectory(nativeRoot)) {
			var nativeDirectory = Path.join([nativeRoot, kit]);
			if (FileSystem.isDirectory(nativeDirectory)) for (name in FileSystem.readDirectory(nativeDirectory))
				if (StringTools.endsWith(name, ".hdll") || name.indexOf(".so") >= 0)
					binaries.push(Path.join([nativeDirectory, name]));
		}
		binaries.sort(Reflect.compare);
		for (path in binaries) {
			if (!FileSystem.exists(path)) continue;
			var stat = FileSystem.stat(path);
			// Empty files contain no executable native code.
			if (stat.size == 0) continue;
			field(fields, path);
			field(fields, Std.string(stat.size));
			field(fields, Std.string(stat.mtime.getTime()));
		}
		Sys.println(ContentDigest.make(haxe.io.Bytes.ofString(fields.toString())).toHex());
	}

	static function add(paths:Map<String, Bool>, path:String):Void {
		var absolute = FileSystem.fullPath(path);
		if (!FileSystem.exists(absolute) || FileSystem.isDirectory(absolute))
			throw 'CAD cache input is not a file: $absolute';
		paths.set(absolute, true);
	}

	static function addHaxeSources(paths:Map<String, Bool>, root:String):Void {
		// A caller may put XDG_CACHE_HOME inside a broad source root. Generated wrappers are outputs.
		if (Path.normalize(FileSystem.fullPath(root)) == generatedSources) return;
		if (!FileSystem.exists(root) || !FileSystem.isDirectory(root))
			throw 'Haxeon source directory is missing: $root';
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
