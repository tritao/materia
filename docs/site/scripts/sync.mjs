// Copies the Markdown under docs/ into the site's content collection.
//
// docs/ stays the source of truth and reads well on GitHub; this step adapts it for Starlight:
// README.md becomes the section index, the leading H1 is dropped (Starlight renders the title),
// relative links to other docs pages become site routes, links to the rest of the repository
// become GitHub links, and stub/draft pages get a banner.
import { readFileSync, writeFileSync, mkdirSync, rmSync, readdirSync, statSync } from "node:fs";
import { dirname, join, relative, resolve, posix } from "node:path";
import { fileURLToPath } from "node:url";
import { slugOf, routeOf } from "./routes.mjs";

const site = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const docs = resolve(site, "..");
const repo = resolve(docs, "..");
const out = join(site, "src/content/docs");
const GITHUB = "https://github.com/tritao/materia";

function markdownFiles(dir) {
	const files = [];
	for (const name of readdirSync(dir)) {
		const path = join(dir, name);
		if (path === site) continue;
		if (statSync(path).isDirectory()) files.push(...markdownFiles(path));
		else if (name.endsWith(".md")) files.push(path);
	}
	return files;
}

const BANNERS = {
	stub: "Stub: this page lists what it will cover; the text is not written yet.",
	draft: "Draft: written but not yet reviewed.",
};

function splitFrontMatter(text) {
	const match = /^---\n([\s\S]*?)\n---\n/.exec(text);
	return match ? { front: match[1], body: text.slice(match[0].length) } : { front: null, body: text };
}

function rewriteLinks(body, source) {
	const fromDir = dirname(source);
	let fenced = false;
	return body
		.split("\n")
		.map((line) => {
			if (/^\s*```/.test(line)) fenced = !fenced;
			if (fenced) return line;
			return line.replace(/\]\(([^)\s]+)\)/g, (whole, target) => {
				if (/^[a-z]+:|^#|^\//i.test(target)) return whole;
				const [path, hash] = target.split("#");
				const absolute = resolve(fromDir, path);
				const anchor = hash ? `#${hash}` : "";
				const inDocs = !relative(docs, absolute).startsWith("..") && !absolute.startsWith(site);
				if (inDocs && path.endsWith(".md")) return `](${routeOf(slugOf(relative(docs, absolute)))}${anchor})`;
				const kind = statSync(absolute, { throwIfNoEntry: false })?.isDirectory() ? "tree" : "blob";
				return `](${GITHUB}/${kind}/main/${relative(repo, absolute)}${anchor})`;
			});
		})
		.join("\n");
}

rmSync(out, { recursive: true, force: true });
let count = 0;
for (const file of markdownFiles(docs)) {
	const path = relative(docs, file);
	let { front, body } = splitFrontMatter(readFileSync(file, "utf8"));
	const heading = /^\s*# (.+)\n/.exec(body);
	if (heading) body = body.slice(heading.index + heading[0].length);
	const lines = front ? front.split("\n") : [];
	if (!lines.some((line) => line.startsWith("title:"))) {
		if (!heading) throw new Error(`${path}: needs a title in front matter or a leading H1`);
		lines.unshift(`title: ${JSON.stringify(heading[1])}`);
	}
	const status = /^status:\s*(\w+)/m.exec(lines.join("\n"))?.[1];
	if (BANNERS[status] && !lines.some((line) => line.startsWith("banner:")))
		lines.push(`banner:\n  content: ${JSON.stringify(BANNERS[status])}`);
	lines.push(`editUrl: ${GITHUB}/edit/main/docs/${path}`);
	const target = join(out, (slugOf(path) || "index") + (path.endsWith("README.md") && slugOf(path) ? "/index.md" : ".md"));
	mkdirSync(dirname(target), { recursive: true });
	writeFileSync(target, `---\n${lines.join("\n")}\n---\n${rewriteLinks(body, file)}`);
	count++;
}
// Figure data computed by docs/figures (see its DocsFigures.hx) is served next to the pages.
const figures = join(docs, "figures/data");
const publicFigures = join(site, "public/figures");
rmSync(publicFigures, { recursive: true, force: true });
mkdirSync(publicFigures, { recursive: true });
for (const name of readdirSync(figures).filter((name) => name.endsWith(".json")))
	writeFileSync(join(publicFigures, name), readFileSync(join(figures, name)));

console.log(`synced ${count} pages into ${posix.join(...relative(site, out).split("/"))}`);
