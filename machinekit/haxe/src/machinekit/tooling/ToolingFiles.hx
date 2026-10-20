package machinekit.tooling;

/** Generated output trees also work in fresh checkouts with no prior tooling build. */
class ToolingFiles {
	public static function directory(path:String):Void {
		if (sys.FileSystem.exists(path)) {
			if (!sys.FileSystem.isDirectory(path))
				throw "Tooling output path is not a directory: " + path;
			return;
		}
		var parent = haxe.io.Path.directory(path);
		if (parent != "" && parent != path)
			directory(parent);
		sys.FileSystem.createDirectory(path);
		if (!sys.FileSystem.exists(path) || !sys.FileSystem.isDirectory(path))
			throw "Could not create tooling output directory: " + path;
	}
}
