import compiler.Compiler;
import compiler.Compiler.FfiConfiguration;
import compiler.Compiler.FfiInterfaceSource;
import compiler.Compiler.FfiProjectionSource;
import compiler.hl.HlWriter;
import compiler.runtime.CompilerIntrinsics;
import compiler.tools.CompilerDriver;
import haxe.io.Path;
import project.PackageResolver;
import project.PathSourceAcquirer;
import build.Target;
import sys.FileSystem;
import sys.io.File;

/** Build one Materia project entrypoint as an isolated Haxeon runtime module. */
class MateriaProjectModuleBuild {
	static function main():Void {
		var args = Sys.args();
		if (args.length != 4)
			throw "usage: MateriaProjectModuleBuild <haxeon.json> <module> <function> <output-prefix>";
		var manifestPath = Path.normalize(FileSystem.fullPath(args[0]));
		var moduleName = args[1];
		var functionName = args[2];
		var outputPrefix = args[3];
		var project = new PackageResolver(new PathSourceAcquirer()).resolve(manifestPath, null, false, Target.parse("host"));
		var interfaces:Array<FfiInterfaceSource> = [];
		var projections:Array<FfiProjectionSource> = [];
		var roots:Array<String> = [];
		for (packageValue in project.packages.packages) {
			for (root in packageValue.sourceRoots)
				if (roots.indexOf(root) < 0) roots.push(root);
			for (path in packageValue.ffiInterfaces)
				interfaces.push({path: path, text: File.getContent(path)});
			for (path in packageValue.ffiProjections)
				projections.push({path: path, text: File.getContent(path)});
		}
		var home = Sys.getEnv("HAXEON_HOME");
		if (home == null || home.length == 0)
			throw "HAXEON_HOME is not set";
		var compiler = new Compiler(null, null, new FfiConfiguration(interfaces, projections));
		CompilerIntrinsics.register(compiler);
		var defines = CompilerDriver.targetDefines("hl");
		compiler.configure("cli:host:" + defines.join("|"), "cli:host", defines);
		compiler.addSourceRoot(Path.join([home, "stdlib"]));
		for (root in roots) compiler.addSourceRoot(root);

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
		compiler.update(modulePath, File.getContent(entryPath));
		var result = compiler.compile(moduleName);
		var functionId = result.functionIds.get(moduleName + "." + functionName);
		if (functionId == null) {
			var moduleLeaf = moduleName.substr(moduleName.lastIndexOf(".") + 1);
			functionId = result.functionIds.get(moduleName + "." + moduleLeaf + "." + functionName);
		}
		if (functionId == null)
			throw 'Project entrypoint function "$moduleName.$functionName" was not exported by the compiled module';
		File.saveBytes(outputPrefix + ".hl", HlWriter.encode(result.module));
		File.saveBytes(outputPrefix + ".hli", result.runtimeIdentity);
		File.saveContent(outputPrefix + ".entry", Std.string(functionId));
	}
}
