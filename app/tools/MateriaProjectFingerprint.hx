import build.Target;
import haxe.crypto.Sha256;
import haxe.io.Path;
import project.PackageResolver;
import project.PathSourceAcquirer;
import sys.FileSystem;
import sys.io.File;

/** Fingerprint the declared inputs of a deterministic CAD preview entrypoint. */
class MateriaProjectFingerprint {
	static function main():Void {
		var args = Sys.args();
		if (args.length < 4)
			throw "usage: MateriaProjectFingerprint <manifest> <module> <function> <tools-dir> [input-file ...]";
		var manifest = FileSystem.fullPath(args[0]);
		var project = new PackageResolver(new PathSourceAcquirer()).resolve(manifest, null, false, Target.parse("host"));
		var home = Sys.getEnv("HAXEON_HOME");
		if (home == null || home.length == 0) throw "HAXEON_HOME is not set";
		var paths = new Map<String, Bool>();
		add(paths, manifest);
		for (item in project.packages.packages) {
			add(paths, Path.join([item.root, PackageResolver.MANIFEST_NAME]));
			for (root in item.sourceRoots) addHaxeSources(paths, root);
			for (path in item.ffiInterfaces) add(paths, path);
			for (path in item.ffiProjections) add(paths, path);
			for (imported in item.ffiImports) {
				add(paths, imported.manifestPath);
				add(paths, imported.header);
			}
		}
		for (root in [Path.join([home, "src"]), Path.join([home, "stdlib"])])
			addHaxeSources(paths, root);
		for (name in ["MateriaProjectModuleBuild.hx", "MateriaProjectFingerprint.hx"])
			add(paths, Path.join([args[3], name]));
		for (index in 4...args.length) add(paths, args[index]);
		var ordered = [for (path in paths.keys()) path];
		ordered.sort(Reflect.compare);
		var fields = new StringBuf();
		field(fields, "materia-generated-artifact-v1");
		field(fields, args[1]);
		field(fields, args[2]);
		for (path in ordered) {
			field(fields, path);
			field(fields, Sha256.make(File.getBytes(path)).toHex());
		}
		// Native code and tool binaries are not Haxe sources. Their build stamps
		// invalidate generated geometry without reading large shared libraries.
		var binaries = [Path.join([home, ".tools", "haxe", "haxe"]),
				Path.join([home, ".tools", "hashlink", "hl"]),
				Path.join([home, "out", "haxeon_runtime.hdll"])];
		var nativeDirectory = Path.join([Path.directory(args[3]), "build", "host", "native", "app"]);
		if (FileSystem.exists(nativeDirectory)) for (name in FileSystem.readDirectory(nativeDirectory))
			if (StringTools.endsWith(name, ".hdll") || name.indexOf(".so") >= 0)
				binaries.push(Path.join([nativeDirectory, name]));
		binaries.sort(Reflect.compare);
		for (path in binaries) {
			if (!FileSystem.exists(path)) continue;
			var stat = FileSystem.stat(path);
			field(fields, path);
			field(fields, Std.string(stat.size));
			field(fields, Std.string(stat.mtime.getTime()));
		}
		Sys.println(Sha256.encode(fields.toString()));
	}

	static function add(paths:Map<String, Bool>, path:String):Void {
		var absolute = FileSystem.fullPath(path);
		if (!FileSystem.exists(absolute) || FileSystem.isDirectory(absolute))
			throw 'CAD cache input is not a file: $absolute';
		paths.set(absolute, true);
	}

	static function addHaxeSources(paths:Map<String, Bool>, root:String):Void {
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
