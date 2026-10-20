import haxe.Json;
import sys.FileSystem;
import sys.io.File;

/**
 * Computes the data behind the docs' interactive figures with the real kits, so a figure can only show what
 * Materia actually does. Runs as a workspace test from this project's directory: by default it recomputes every
 * figure and fails if one differs from its committed `data/<name>.json`; with `DOCS_FIGURES_WRITE=1` it rewrites
 * them instead.
 */
class DocsFigures {
	static inline var DATA = "data";

	public static function main():Void {
		var write = Sys.getEnv("DOCS_FIGURES_WRITE") == "1";
		var stale = 0;
		for (figure in [{name: "paths-vs-trajectories", data: PathsVsTrajectories.compute()}]) {
			var path = '$DATA/${figure.name}.json';
			var text = Json.stringify(figure.data) + "\n";
			if (write) {
				if (!FileSystem.exists(DATA)) FileSystem.createDirectory(DATA);
				File.saveContent(path, text);
				Sys.println('wrote $path');
			} else if (!FileSystem.exists(path) || File.getContent(path) != text) {
				Sys.println('$path is stale: rerun with DOCS_FIGURES_WRITE=1 and review the change');
				stale++;
			}
		}
		if (stale > 0) Sys.exit(1);
		Sys.println(write ? "figures written" : "figures up to date");
	}

	/** Rounded for stable, compact output: micrometres and microseconds are below anything a figure shows. */
	public static function round(value:Float):Float
		return Math.round(value * 1e6) / 1e6;
}
