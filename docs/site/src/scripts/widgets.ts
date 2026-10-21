// Live widgets: `<figure class="materia-figure materia-widget" data-widget="NAME">` elements run real Materia code
// in the reader's browser, through the docs runtime (docs/runtime, Wasm GC). The runtime downloads only when a
// widget scrolls near the viewport; until then, and in browsers without Wasm GC, the figure keeps its Markdown.
import * as Plot from "@observablehq/plot";
import { loadRuntime, type DocsRuntime } from "./runtime";

type Widget = (figure: HTMLElement, runtime: DocsRuntime) => void;

const widgets: Record<string, Widget> = {
	gcode: gcodeWidget,
};

let runtime: Promise<DocsRuntime> | undefined;
const loadOnce = () => (runtime ??= loadRuntime(fetch("/runtime/docs-runtime.wasm")));

export function renderWidgets(): void {
	const figures = [...document.querySelectorAll<HTMLElement>("figure.materia-widget[data-widget]")];
	if (!figures.length) return;
	const observer = new IntersectionObserver(
		(entries) => {
			for (const entry of entries) {
				if (!entry.isIntersecting) continue;
				observer.unobserve(entry.target);
				const figure = entry.target as HTMLElement, name = figure.dataset.widget!;
				const widget = widgets[name];
				if (!widget) continue;
				loadOnce()
					.then((loaded) => widget(figure, loaded))
					.catch((error) => {
						console.error(`widget ${name}:`, error);
						const note = element("p", { class: "widget-note" },
							"This interactive example needs a browser with WebAssembly GC (Chrome 119, Firefox 120, Safari 18.2 or later).");
						figure.append(note);
					});
			}
		},
		{ rootMargin: "400px" },
	);
	for (const figure of figures) observer.observe(figure);
}

function element<K extends keyof HTMLElementTagNameMap>(tag: K, attributes: Record<string, string> = {}, text?: string) {
	const node = document.createElement(tag);
	for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, value);
	if (text !== undefined) node.textContent = text;
	return node;
}

function token(element: HTMLElement, name: string): string {
	return getComputedStyle(element).getPropertyValue(name).trim();
}

/**
 * Edit G-code; CncKit compiles it as the editor does. The plot shows the program's moves seen from above (rapids
 * dashed), and every diagnostic names its line and column.
 */
function gcodeWidget(figure: HTMLElement, runtime: DocsRuntime): void {
	// Starlight's code blocks render one element per line, so the block's own text has no line breaks.
	const lines = [...figure.querySelectorAll(".ec-line")].map((line) => line.textContent ?? "");
	const source = lines.length ? lines.join("\n") : (figure.querySelector("pre")?.textContent?.replace(/\n$/, "") ?? "");
	const caption = figure.querySelector("figcaption");
	figure.replaceChildren();

	const editor = element("textarea", {
		class: "widget-code", spellcheck: "false", rows: String(Math.min(16, source.split("\n").length + 1)),
		"aria-label": "G-code program",
	}) as HTMLTextAreaElement;
	editor.value = source;
	const grid = element("div", { class: "figure-grid widget-gcode" });
	const plot = element("div");
	const report = element("div", { class: "widget-report", "aria-live": "polite" });
	grid.append(editor, plot);
	figure.append(grid, report);
	if (caption) figure.append(caption);

	let pending = 0;
	const draw = () => {
		const result = runtime.gcode(editor.value);
		const rows = result.moves.flatMap((move, index) =>
			move.points.map(([x, y]) => ({ x, y, index, kind: move.kind, line: move.line })),
		);
		const cut = token(figure, "--series-1"), muted = token(figure, "--figure-muted"), ink = token(figure, "--figure-ink");
		plot.replaceChildren(
			Plot.plot({
				width: 360,
				height: 300,
				aspectRatio: 1,
				style: { color: ink, background: "transparent", fontSize: "12px" },
				x: { label: "X (mm)", grid: true },
				y: { label: "Y (mm)", grid: true },
				marks: [
					Plot.line(rows.filter((row) => row.kind === "rapid"), {
						x: "x", y: "y", z: "index", stroke: muted, strokeWidth: 1.5, strokeDasharray: "4,3",
					}),
					Plot.line(rows.filter((row) => row.kind !== "rapid"), { x: "x", y: "y", z: "index", stroke: cut, strokeWidth: 2 }),
					Plot.tip(rows, Plot.pointer({ x: "x", y: "y", title: (row) => `line ${row.line}: ${row.kind}` })),
				],
			}),
		);
		report.replaceChildren();
		const summary = `${result.moves.length} move${result.moves.length === 1 ? "" : "s"}`;
		if (!result.diagnostics.length) {
			report.append(element("p", {}, `${summary}, no diagnostics.`));
			return;
		}
		const list = element("ul");
		for (const diagnostic of result.diagnostics) {
			const item = element("li", { class: `widget-${diagnostic.severity}` });
			item.append(
				element("strong", {}, `Line ${diagnostic.line}, column ${diagnostic.column}`),
				document.createTextNode(` (${diagnostic.severity}, ${diagnostic.code}): ${diagnostic.message}`),
			);
			list.append(item);
		}
		report.append(element("p", {}, `${summary}.`), list);
	};
	editor.addEventListener("input", () => {
		clearTimeout(pending);
		pending = window.setTimeout(draw, 150);
	});
	new MutationObserver(draw).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
	draw();
}
