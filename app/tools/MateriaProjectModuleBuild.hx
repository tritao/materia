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

/** Build a project entrypoint as a separate HashLink artifact generator. */
class MateriaProjectModuleBuild {
	static function main():Void {
		var args = Sys.args();
		if (args.length != 6)
			throw "usage: MateriaProjectModuleBuild <haxeon.json> <module> <function> <output-prefix> <haxeon-root> <document-input>";
		var manifestPath = Path.normalize(FileSystem.fullPath(args[0]));
		var moduleName = args[1];
		var functionName = args[2];
		var outputPrefix = args[3];
		var identifier = ~/^[A-Za-z_][A-Za-z0-9_]*$/;
		for (part in moduleName.split("."))
			if (!identifier.match(part)) throw "Project entrypoint has an invalid module name";
		if (!identifier.match(functionName)) throw "Project entrypoint has an invalid function name";
		var project = new PackageResolver(new PathSourceAcquirer()).resolve(manifestPath, null, false, Target.parse("host"));
		var home = FileSystem.fullPath(args[4]);
		// Use the same request preparation, batch mode and artifact writer as Haxeon builds.
		Sys.setCwd(home);
		var cache = Sys.getEnv("XDG_CACHE_HOME");
		if (cache == null || cache.length == 0) cache = Path.join([Sys.getEnv("HOME"), ".cache"]);
		var wrapperRoot = Path.join([cache, "materia", "project-sources", Sha256.encode(manifestPath + "\n" + moduleName + "\n" + functionName + "\n" + args[5])]);
		var reusable = true;
		try FileSystem.createDirectory(wrapperRoot) catch (_:Dynamic) {
			wrapperRoot = Path.directory(outputPrefix);
			reusable = false;
		}
		var arguments = ["--target=hl", "--output=" + outputPrefix + ".hl",
			"--entry=MateriaGeneratedEntrypoint", "--root=" + wrapperRoot];
		for (packageValue in project.packages.packages) {
			for (root in packageValue.sourceRoots) arguments.push("--root=" + root);
			for (path in packageValue.ffiInterfaces) arguments.push("--ffi-interface=" + path);
			for (path in packageValue.ffiProjections) arguments.push("--ffi-projection=" + path);
		}
		for (define in project.manifest.defines) arguments.push("--define=" + define);
		if (project.manifest.inlineEnabled != null)
			arguments.push("--define=haxeon-inline=" + (project.manifest.inlineEnabled ? "1" : "0"));

		var modulePath = moduleName.split(".").join("/") + ".hx";
		var entryPath:Null<String> = null;
		for (root in project.rootPackage.sourceRoots) {
			var candidate = Path.join([root, modulePath]);
			if (FileSystem.exists(candidate)) {
				entryPath = candidate;
				break;
			}
		}
		if (entryPath == null)
			throw 'Project entrypoint module "$moduleName" is not under a package source root';
		arguments.push(entryPath);
		var documentInput = args[5] == "true";
		var invocation = documentInput
			? "(args.length == 2 ? " + moduleName + "." + functionName + "(sys.io.File.getContent(args[1])) : " + moduleName + "." + functionName + "())"
			: moduleName + "." + functionName + "()";
		var wrapper = "class MateriaGeneratedEntrypoint {\n"
			+ "  static function main():Void {\n"
			+ "    var args = Sys.args();\n"
			+ "    if (args.length != 1" + (documentInput ? " && args.length != 2" : "")
			+ ") throw \"Project generator requires an output path\";\n"
			+ "    sys.io.File.saveBytes(args[0], " + invocation + ");\n"
			+ "  }\n"
			+ "}\n";
		var wrapperPath = Path.join([wrapperRoot, "MateriaGeneratedEntrypoint.hx"]);
		if (!FileSystem.exists(wrapperPath) || File.getContent(wrapperPath) != wrapper) File.saveContent(wrapperPath, wrapper);
		arguments.push(wrapperPath);
		var oneShot = function() {
			CompilerDriver.compile(CompilerArguments.parse(arguments));
			return 0;
		};
		var status = reusable ? CompilerClient.run(Path.join([home, ".tools", "haxe", "haxe"]), Path.join([home, "src"]), arguments, home,
			wrapperRoot, Path.directory(manifestPath), oneShot) : oneShot();
		if (status != 0) Sys.exit(status);
	}
}
