// Generated content written in place: a file is rewritten only when its bytes change, and files a run no longer
// produces are removed afterwards. A running `astro dev` keeps watching the same directories, so it picks up
// edits instead of losing a collection that was deleted and recreated under it.
import { existsSync, mkdirSync, readFileSync, readdirSync, rmSync, rmdirSync, statSync, writeFileSync } from "node:fs";
import { dirname, join, resolve } from "node:path";

const written = new Set();

/** Writes `contents` (a string or bytes) to `path` unless the file already holds exactly that. */
export function writeOutput(path, contents) {
	const target = resolve(path);
	written.add(target);
	const bytes = typeof contents === "string" ? Buffer.from(contents) : Buffer.from(contents);
	if (existsSync(target) && readFileSync(target).equals(bytes)) return;
	mkdirSync(dirname(target), { recursive: true });
	writeFileSync(target, bytes);
}

/** Removes every file below `root` that this run did not write, and the directories left empty. */
export function pruneOutput(root) {
	const prune = (dir) => {
		if (!existsSync(dir)) return;
		for (const name of readdirSync(dir)) {
			const path = join(dir, name);
			if (statSync(path).isDirectory()) {
				prune(path);
				if (readdirSync(path).length === 0) rmdirSync(path);
			} else if (!written.has(resolve(path))) rmSync(path);
		}
	};
	prune(resolve(root));
}
