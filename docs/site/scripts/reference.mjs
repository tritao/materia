// Renders the API reference from docs/api/data/*.json (written by docs/api/generate.py) into pages:
// /reference/ lists the kits, /reference/KIT/ its packages and types, and /reference/KIT/PACKAGE/
// every public type in that package with its public fields and doc comments.
//
// Also builds the index concept pages use to link `TypeName` and `TypeName.member` code spans to the
// type's entry (see linkTypes).
import { readFileSync, readdirSync, writeFileSync, mkdirSync } from "node:fs";
import { join } from "node:path";

const GITHUB = "https://github.com/tritao/materia";

/** Kit order and one-line descriptions for the reference index; kits not listed follow alphabetically. */
const KITS = {
	"haxeon-platform": "Windows, input, files and the native runtime services",
	"haxeon-gpu": "GPU resources and frames",
	"haxeon-ui": "Layout, text, widgets and docking",
	"haxeon-editor": "Editor building blocks",
	"haxeon-animation": "Skeletal animation and rigs",
	scenekit: "Retained scene, rendering and picking",
	projectkit: "Shared data: scene artifacts, assembly records, units",
	cadkit: "CAD documents, sketches, shapes and assemblies",
	machinekit: "Machine components, catalogs, transmissions and assemblies",
	collisionkit: "Collision worlds and distance queries",
	gearkit: "Gear geometry and metrology",
	bimkit: "Buildings as CAD documents",
	animkit: "Character import",
	humankit: "Human workers",
	kinematicskit: "Kinematic models and solvers",
	motionkit: "Paths, planners, machine-axis views and plans",
	trajectorykit: "Joint trajectories",
	processkit: "Process paths and skills",
	toolpathkit: "The toolpath program format and its lowering to motion",
	camkit: "Toolpaths from geometry",
	cnckit: "G-code in and out",
	stockkit: "Material removal",
	manufacturingkit: "Sheet stock and tooling records",
	simkit: "Simulation sessions",
	robotkit: "Robot model, runtime, world, remote robots and autonomy",
	visionkit: "Camera geometry",
	automationkit: "Facilities, missions, fleets and traffic",
};

const packageSlug = (name) => name.replace(/[^A-Za-z0-9]+/g, "-").toLowerCase();
const anchorOf = (name) => name.toLowerCase();

/** Escapes text for Markdown prose: angle brackets would open HTML, except inside code spans. */
function prose(text) {
	return text
		.split(/(`[^`]*`)/)
		.map((part, i) => (i % 2 ? part : part.replace(/</g, "&lt;").replace(/>/g, "&gt;")))
		.join("")
		.replace(/^(\s*)#/gm, "$1\\#");
}

/** Doc comments are written for source readers; strip the leading `*` some of them keep per line. */
function doc(text) {
	return prose(text.split("\n").map((line) => line.replace(/^\s*\*\s?/, "")).join("\n").trim());
}

export function loadReference(dataDir) {
	const kits = readdirSync(dataDir)
		.filter((name) => name.endsWith(".json"))
		.map((name) => JSON.parse(readFileSync(join(dataDir, name), "utf8")));
	const order = Object.keys(KITS);
	kits.sort((a, b) => {
		const ia = order.indexOf(a.kit), ib = order.indexOf(b.kit);
		return (ia < 0 ? order.length : ia) - (ib < 0 ? order.length : ib) || a.kit.localeCompare(b.kit);
	});
	// Type name → routes; a name declared more than once across the kits is ambiguous and never auto-linked.
	const byName = new Map();
	for (const kit of kits)
		for (const pkg of kit.packages)
			for (const type of pkg.types) {
				const route = `/reference/${kit.kit}/${packageSlug(pkg.name)}/#${anchorOf(type.name)}`;
				const qualified = pkg.name === "(top level)" ? type.name : `${pkg.name}.${type.name}`;
				byName.set(qualified, [route]);
				byName.set(type.name, [...(byName.get(type.name) ?? []), route]);
			}
	return { kits, byName };
}

function typeHeading(type) {
	const params = type.params?.length ? `<${type.params.join(", ")}>` : "";
	let line = `${type.kind} \`${type.name}${params}\``;
	if (type.underlying) line = `abstract \`${type.name}${params}(${type.underlying})\``;
	if (type.alias) line += ` = \`${type.alias}\``;
	if (type.extends) line += ` extends ${type.extends.map((t) => `\`${t}\``).join(", ")}`;
	if (type.implements) line += ` implements ${type.implements.map((t) => `\`${t}\``).join(", ")}`;
	return line;
}

function fieldLine(field) {
	const modifiers = [field.static ? "static" : "", field.kind === "var" ? (field.final ? "final" : "var") : "",
		field.kind === "method" ? "function" : ""].filter(Boolean).join(" ");
	const name = field.optional ? `?${field.name}` : field.name;
	const code = `${modifiers ? modifiers + " " : ""}${name}${field.signature ?? ""}`;
	const text = field.doc ? ` — ${doc(field.doc).replace(/\n+/g, " ")}` : "";
	return `- \`${code}\`${text}`;
}

function packagePage(kit, pkg) {
	const lines = [
		"---",
		`title: ${JSON.stringify(pkg.name)}`,
		`description: ${JSON.stringify(`Public types in ${pkg.name} (${kit.kit}).`)}`,
		"tableOfContents:",
		"  maxHeadingLevel: 2",
		"---",
		"",
		`[${kit.kit}](/reference/${kit.kit}/) · ${pkg.types.length} public type${pkg.types.length === 1 ? "" : "s"}`,
		"",
	];
	for (const type of pkg.types) {
		lines.push(`## ${type.name}`, "", `${typeHeading(type)} · [source](${GITHUB}/blob/main/${type.file})`, "");
		if (type.doc) lines.push(doc(type.doc), "");
		if (type.fields?.length) {
			const statics = type.fields.filter((f) => f.static), members = type.fields.filter((f) => !f.static);
			for (const field of [...members, ...statics]) lines.push(fieldLine(field));
			lines.push("");
		}
	}
	return lines.join("\n");
}

function kitPage(kit) {
	const types = kit.packages.reduce((n, p) => n + p.types.length, 0);
	const lines = [
		"---",
		`title: ${JSON.stringify(kit.kit)}`,
		`description: ${JSON.stringify(KITS[kit.kit] ?? `Public API of ${kit.kit}.`)}`,
		"---",
		"",
		`${KITS[kit.kit] ? KITS[kit.kit] + ". " : ""}${kit.packages.length} packages, ${types} public types.`,
		"",
	];
	for (const pkg of kit.packages) {
		const route = `/reference/${kit.kit}/${packageSlug(pkg.name)}/`;
		lines.push(`## [${pkg.name}](${route})`, "");
		lines.push(pkg.types.map((t) => `[${t.name}](${route}#${anchorOf(t.name)})`).join(" · "), "");
	}
	return lines.join("\n");
}

function indexPage(kits) {
	const lines = [
		"---",
		"title: API reference",
		"description: Every public type of every kit, generated from the source.",
		"---",
		"",
		"Generated from the source by `docs/api/generate.py`: each kit's public types, their public",
		"fields and methods, and their doc comments. Types are shown as written in the source, so a",
		"parameter type is the name the source uses. Concept pages link here; the reference does not",
		"explain *why*, the [concepts](/concepts/) do.",
		"",
		"| Kit | What it owns | Types |",
		"|---|---|---|",
	];
	for (const kit of kits) {
		const types = kit.packages.reduce((n, p) => n + p.types.length, 0);
		lines.push(`| [${kit.kit}](/reference/${kit.kit}/) | ${KITS[kit.kit] ?? ""} | ${types} |`);
	}
	return lines.join("\n") + "\n";
}

/** Writes the reference pages below `out` and returns how many were written. */
export function writeReference(reference, out) {
	const root = join(out, "reference");
	mkdirSync(root, { recursive: true });
	writeFileSync(join(root, "index.md"), indexPage(reference.kits));
	let pages = 1;
	for (const kit of reference.kits) {
		mkdirSync(join(root, kit.kit), { recursive: true });
		writeFileSync(join(root, kit.kit, "index.md"), kitPage(kit));
		pages++;
		for (const pkg of kit.packages) {
			writeFileSync(join(root, kit.kit, packageSlug(pkg.name) + ".md"), packagePage(kit, pkg));
			pages++;
		}
	}
	return pages;
}

/**
 * Links `Type`, `Type.member`, `Type.member()` and `package.Type` code spans in a page's prose to the
 * type's reference entry, when the name is declared exactly once. Code already inside a link and
 * fenced blocks are left alone.
 */
export function linkTypes(body, reference) {
	let fenced = false;
	return body
		.split("\n")
		.map((line) => {
			if (/^\s*```/.test(line)) fenced = !fenced;
			if (fenced) return line;
			return line.replace(/(\[[^\]]*\]\([^)]*\))|`([A-Za-z_][\w.]*)(\(\))?`/g, (whole, link, name) => {
				if (link) return whole;
				const parts = name.split(".");
				for (let i = parts.length; i > 0; i--) {
					const routes = reference.byName.get(parts.slice(0, i).join("."));
					if (!routes) continue;
					if (routes.length !== 1 || !/^[A-Z]/.test(parts[i - 1])) return whole;
					return `[${whole}](${routes[0]})`;
				}
				return whole;
			});
		})
		.join("\n");
}
