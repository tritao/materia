// Renders the component catalogue from docs/components/data/components.json (written by the docs-components
// workspace program from MachineKit's registry): /components/ lists every recipe with a thumbnail, grouped by
// family and category (machinekit ComponentClassification), with search, family and kind filters;
// /components/FAMILY/ has a section per category and shows each part with an orbitable model, its parameters,
// catalogue rows and their provenance, connectors, mass and bill-of-materials line.
import { existsSync, readFileSync } from "node:fs";
import { join } from "node:path";
import GithubSlugger from "github-slugger";
import { writeOutput } from "./output.mjs";

// The viewer settings the thumbnails share (docs/screens/components.py), so a thumbnail is the viewer's first frame.
const VIEW = JSON.parse(readFileSync(new URL("../src/model-view.json", import.meta.url), "utf8"));

/** A part's <model-viewer> attributes: the shared settings, the export's lighting and the camera angle it chose. */
export function viewerAttributes(component, display) {
	const attributes = {
		...VIEW.attributes,
		"environment-image": component.view.environment,
		exposure: String(display.exposure),
		"tone-mapping": display.toneMapping,
		"camera-orbit": `${component.view.theta}deg ${component.view.phi}deg auto`,
	};
	return Object.entries(attributes).map(([name, value]) => `${name}="${escape(value)}"`).join(" ");
}

const FAMILIES = {
	standard: "Standard parts",
	motion: "Motion",
	transmission: "Transmissions",
	structural: "Structure",
	robotics: "Robotics",
	pneumatic: "Pneumatics",
	control: "Control and electrical",
	power: "Power",
	milling: "Milling",
	routing: "Routing",
	printing: "Printing",
	pressbrake: "Press brake",
	hobbing: "Hobbing",
	enclosure: "Enclosures",
	tooling: "Tooling",
	assembly: "Assembly",
};

/** Kinds of recipe (machinekit ComponentKind), in filter order; generic recipes carry no badge. */
const KINDS = {
	generic: { label: "Generic recipes" },
	supplier: { label: "Supplier parts", badge: "supplier" },
	reference: { label: "References and studies", badge: "reference" },
	"machine-part": { label: "Machine parts", badge: "machine part" },
};

const familyLabel = (id) => FAMILIES[id] ?? id.charAt(0).toUpperCase() + id.slice(1);
const thumb = (component) => `/components/thumbs/${component.model.replace(/\.glb$/, ".webp")}`;
const escape = (text) =>
	String(text ?? "").replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");
// Markdown table cells: escape HTML and the pipe.
const cell = (text) => escape(text).replace(/\|/g, "\\|");
// Table cells holding code spans: Markdown shows entities literally there, so only the pipe is escaped.
const pipes = (text) => String(text ?? "").replace(/\|/g, "\\|");

/** Assumed parts say so in their designation or description (see machinekit AssumedCatalogComponent). */
const assumed = (component) =>
	/-ASSUMED-/.test(component.designation ?? "") || /^Assumed\b/.test(component.description ?? "");

export function loadComponents(path) {
	if (!existsSync(path)) return null;
	const exported = JSON.parse(readFileSync(path, "utf8"));
	const components = exported.components;
	const categoryOrder = exported.categories ?? [];
	const categoryRank = (category) => {
		const index = categoryOrder.indexOf(category);
		return index < 0 ? categoryOrder.length : index;
	};
	const families = new Map();
	for (const component of components) {
		if (!families.has(component.family)) families.set(component.family, []);
		families.get(component.family).push(component);
	}
	const order = Object.keys(FAMILIES);
	const sorted = [...families.keys()].sort((a, b) => {
		const ia = order.indexOf(a), ib = order.indexOf(b);
		return (ia < 0 ? order.length : ia) - (ib < 0 ? order.length : ib) || a.localeCompare(b);
	});
	// A family page has one section per category, in the registry's order, and its parts by label within it.
	// Each part's anchor is its heading's slug, numbered as Starlight numbers repeated headings, so the
	// slugger sees the category and part headings in page order.
	const result = [];
	for (const id of sorted) {
		const list = families.get(id).sort((a, b) =>
			categoryRank(a.category) - categoryRank(b.category) || a.label.localeCompare(b.label));
		const categories = [];
		for (const component of list) {
			const name = component.category ?? "Other";
			if (!categories.length || categories.at(-1).name !== name) categories.push({ name, components: [] });
			categories.at(-1).components.push(component);
		}
		const slugger = new GithubSlugger();
		for (const category of categories) {
			category.anchor = slugger.slug(category.name);
			for (const component of category.components) component.anchor = slugger.slug(component.label);
		}
		result.push({ id, label: familyLabel(id), components: list, categories });
	}
	return { display: exported.display, components, families: result };
}

/** machinekit MassSource as text: Computed(Preview), Declared, EffectiveDensity(900). */
function massSource(source) {
	const computed = /^Computed\((\w+)\)$/.exec(source);
	if (computed) return `computed from the ${computed[1].toLowerCase()} geometry`;
	const effective = /^EffectiveDensity\(([\d.]+)\)$/.exec(source);
	if (effective) return `assumed from its envelope at ${effective[1]} kg/m³`;
	return source.toLowerCase();
}

function badges(component) {
	const list = [];
	const kind = KINDS[component.kind]?.badge;
	if (kind) list.push(`<span class="part-badge part-badge-kind">${kind}</span>`);
	// machinekit GeometryFidelity: an envelope preview is a placeholder shape; nominal needs no badge.
	if (component.fidelity === "envelope") list.push(`<span class="part-badge part-badge-warn">envelope shape</span>`);
	else if (component.fidelity === "datasheet" || component.fidelity === "supplier")
		list.push(`<span class="part-badge">${component.fidelity} geometry</span>`);
	if (assumed(component)) list.push(`<span class="part-badge part-badge-warn">assumed</span>`);
	if (component.catalogs?.length) list.push(`<span class="part-badge">catalogue</span>`);
	if (component.error) list.push(`<span class="part-badge part-badge-warn">no default part</span>`);
	return list.join(" ");
}

function indexPage(catalogue) {
	const total = catalogue.components.length;
	const lines = [
		"---",
		"title: Components",
		`description: ${JSON.stringify(`Every parametric component in MachineKit: ${total} recipes, with models, parameters and catalogue provenance.`)}`,
		"tableOfContents: false",
		"---",
		"",
		`Every parametric component MachineKit ships: ${total} recipes, generated from the code, filed by family and`,
		"category. Each model is the part at its default parameters; open one to orbit it, see what you can change,",
		"and where its catalogue data comes from. Parts marked **assumed** use ratings or envelopes that are not yet",
		"from a datasheet; **supplier** parts carry one supplier's data, **reference** models are drawings and",
		"studies to check against, and **machine parts** were designed for one example machine. An **envelope shape**",
		"is still a placeholder of the part's outer size; the plan to replace them is in",
		"[the component geometry plan](https://github.com/tritao/materia/blob/main/machinekit/COMPONENT_GEOMETRY_PLAN.md).",
		"",
		'<div class="part-filters">',
		'<input type="search" placeholder="Search components" aria-label="Search components" data-part-search>',
		'<select aria-label="Family" data-part-family><option value="">All families</option>' +
			catalogue.families.map((family) => `<option value="${family.id}">${escape(family.label)} (${family.components.length})</option>`).join("") +
			"</select>",
		'<select aria-label="Kind" data-part-kind><option value="">All kinds</option>' +
			Object.entries(KINDS).map(([id, kind]) => {
				const count = catalogue.components.filter((component) => component.kind === id).length;
				return `<option value="${id}">${kind.label} (${count})</option>`;
			}).join("") +
			"</select>",
		"</div>",
		"",
	];
	for (const family of catalogue.families) {
		lines.push(`<section class="part-family" data-family="${family.id}">`,
			`<h2 class="part-family-title"><a href="/components/${family.id}/">${escape(family.label)}</a></h2>`);
		for (const category of family.categories) {
			lines.push('<div class="part-group">', `<h3 class="part-category">${escape(category.name)}</h3>`, '<div class="part-grid">');
			for (const component of category.components) {
				const image = component.model
					? `<img src="${thumb(component)}" alt="" width="320" height="240" loading="lazy">`
					: `<span class="part-noimage">No model</span>`;
				const search = [component.label, component.designation, component.description, family.label, category.name, component.id]
					.join(" ").toLowerCase();
				lines.push(
					`<a class="part-card" href="/components/${family.id}/#${component.anchor}" data-kind="${component.kind ?? ""}" data-search="${escape(search)}">` +
						`${image}<span class="part-name">${escape(component.label)}</span>` +
						`<span class="part-meta">${badges(component)}</span></a>`,
				);
			}
			lines.push("</div>", "</div>");
		}
		lines.push("</section>");
	}
	lines.push("", '<p class="part-empty" hidden>No component matches.</p>', "");
	return lines.join("\n");
}

function parameterTable(component) {
	if (!component.parameters?.length) return [];
	const rows = ["| Parameter | Kind | Default | Range or choices |", "|---|---|---|---|"];
	for (const parameter of component.parameters) {
		let range = "";
		const unit = parameter.unit ? " " + parameter.unit : "";
		if (parameter.minimum != null && parameter.maximum != null) range = `${parameter.minimum} to ${parameter.maximum}${unit}`;
		else if (parameter.minimum != null) range = `at least ${parameter.minimum}${unit}`;
		else if (parameter.maximum != null) range = `at most ${parameter.maximum}${unit}`;
		if (parameter.options)
			range = parameter.options.length > 8
				? `${parameter.options.slice(0, 6).map((o) => `\`${o}\``).join(", ")} and ${parameter.options.length - 6} more`
				: parameter.options.map((o) => `\`${o}\``).join(", ");
		const value = parameter.value == null ? "unset" : parameter.value === "" ? "empty"
			: `\`${parameter.value}${parameter.unit && typeof parameter.value === "number" ? " " + parameter.unit : ""}\``;
		rows.push(`| \`${pipes(parameter.name)}\` | ${pipes(parameter.type)} | ${pipes(value)} | ${pipes(range)} |`);
	}
	return [...rows, ""];
}

function catalogueRows(component) {
	const lines = [];
	for (const catalog of component.catalogs ?? []) {
		lines.push(`<details><summary>Catalogue: ${catalog.rows.length} ${escape(catalog.parameter)} rows and their sources</summary>`, "");
		lines.push("| Designation | Data | Conformance | Source |", "|---|---|---|---|");
		for (const row of catalog.rows) {
			const data = row.dimensions === "Unverified"
				? '<span class="part-badge part-badge-warn">unverified</span>'
				: row.dimensions.toLowerCase() + (row.verified ? ` (checked: ${row.verified.join(", ")})` : "");
			const conformance = { ExactInterface: "exact interface", NominalEnvelope: "nominal envelope", GenericApproximation: "approximation" }[row.conformance] ?? row.conformance;
			const source = /^https?:/.test(row.source) ? `[${new URL(row.source).hostname}](${row.source})` : cell(row.source);
			lines.push(`| \`${pipes(row.designation)}\` | ${data} | ${conformance}${row.standard ? ", " + cell(row.standard) : ""} | ${source} |`);
		}
		lines.push("", "</details>", "");
	}
	return lines;
}

function familyPage(family, display) {
	const lines = [
		"---",
		`title: ${JSON.stringify(family.label)}`,
		`description: ${JSON.stringify(`${family.components.length} ${family.label.toLowerCase()} components in MachineKit.`)}`,
		"---",
		"",
		`[All components](/components/) · ${family.components.length} components in ${family.categories.length} ` +
			(family.categories.length === 1 ? "category" : "categories"),
		"",
	];
	for (const category of family.categories) {
		lines.push(`## ${category.name}`, "");
		for (const component of category.components) {
			lines.push(`### ${component.label}`, "");
			if (component.model)
				lines.push(
					`<model-viewer class="part-model" src="/components/${component.model}" poster="${thumb(component)}" alt="${escape(component.label)} at its default parameters" camera-controls reveal="auto" loading="eager" ${viewerAttributes(component, display)}></model-viewer>`,
					'<p class="part-hint">Drag to orbit; scroll or pinch to zoom.</p>',
					"",
				);
			const facts = [`\`${component.id}\``];
			if (component.designation) facts.push(`designation \`${component.designation}\``);
			if (component.mass) facts.push(`${component.mass.kg} kg, ${massSource(component.mass.source)}`);
			lines.push(`${facts.join(" · ")} ${badges(component)}`, "");
			if (component.description) lines.push(escape(component.description), "");
			if (component.error) lines.push(`The recipe's defaults do not build a part: ${escape(component.error)}`, "");
			lines.push(...parameterTable(component));
			lines.push(...catalogueRows(component));
			if (component.connectors?.length)
				lines.push(`Connectors: ${component.connectors.map((c) => `\`${c.name}\` (${c.role.toLowerCase()})`).join(", ")}.`, "");
			if (component.bom)
				lines.push(`Bill of materials: ${escape(component.bom.description)}${component.bom.material ? ", " + escape(component.bom.material) : ""}; part number \`${component.bom.partNumber.replace(/`/g, "")}\`.`, "");
		}
	}
	return lines.join("\n");
}

/** Writes the catalogue pages below `out`; returns how many were written. */
export function writeComponents(catalogue, out) {
	const root = join(out, "components");
	writeOutput(join(root, "index.md"), indexPage(catalogue));
	for (const family of catalogue.families) writeOutput(join(root, `${family.id}.md`), familyPage(family, catalogue.display));
	return 1 + catalogue.families.length;
}
