// Renders `<figure class="materia-figure" data-figure="NAME">` elements from `/figures/NAME.json`, the data that
// docs/figures computes with the real kits. Each figure has one renderer below; the figure element keeps its
// Markdown fallback until the data loads.
import * as Plot from "@observablehq/plot";

type Renderer = (figure: HTMLElement, data: any) => void;

const renderers: Record<string, Renderer> = {
	"paths-vs-trajectories": pathsVsTrajectories,
};

export function renderFigures(): void {
	for (const figure of document.querySelectorAll<HTMLElement>("figure.materia-figure[data-figure]")) {
		const name = figure.dataset.figure!;
		const render = renderers[name];
		if (!render) continue;
		fetch(`/figures/${name}.json`)
			.then((response) => {
				if (!response.ok) throw new Error(`${response.status}`);
				return response.json();
			})
			.then((data) => render(figure, data))
			.catch((error) => console.error(`figure ${name}:`, error));
	}
}

/** CSS custom property values, read when drawing: SVG presentation attributes do not resolve `var()`. */
function token(element: HTMLElement, name: string): string {
	return getComputedStyle(element).getPropertyValue(name).trim();
}

/** Re-runs `draw` when the reader switches between light and dark themes. */
function onThemeChange(draw: () => void): void {
	new MutationObserver(draw).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
}

function element<K extends keyof HTMLElementTagNameMap>(tag: K, attributes: Record<string, string> = {}, text?: string) {
	const node = document.createElement(tag);
	for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, value);
	if (text !== undefined) node.textContent = text;
	return node;
}

const format = (value: number, digits = 2) => value.toFixed(digits);

interface Method {
	id: string;
	label: string;
	summary: string;
	duration: number;
	rows: Record<string, number>[];
}

function pathsVsTrajectories(figure: HTMLElement, data: any): void {
	const methods: Method[] = data.methods.map((method: any) => ({
		...method,
		rows: method.samples.map((sample: number[]) =>
			Object.fromEntries(method.columns.map((column: string, index: number) => [column, sample[index]])),
		),
	}));
	for (const method of methods)
		for (const row of method.rows) {
			row.speed = Math.hypot(row.vx, row.vy);
			row.offArc = Math.abs(Math.hypot(row.x, row.y) - data.start[0]);
		}
	const end = Math.max(...methods.map((method) => method.duration));
	const sampleStep = methods[0].rows[1].t - methods[0].rows[0].t;
	const at = (method: Method, t: number) =>
		method.rows[Math.min(method.rows.length - 1, Math.max(0, Math.round(t / sampleStep)))];

	const quantities = [
		{ key: "ax", label: "X acceleration (m/s²)" },
		{ key: "jx", label: "X jerk (m/s³)" },
	];
	let quantity = quantities[0];
	let time = 0;

	const caption = figure.querySelector("figcaption");
	figure.replaceChildren();

	const controls = element("div", { class: "figure-controls" });
	const play = element("button", { type: "button" }, "Play");
	const slider = element("input", {
		type: "range", min: "0", max: String(end), step: String(sampleStep), value: "0",
		"aria-label": "Time since the start of the move",
	}) as HTMLInputElement;
	const clock = element("span", { class: "figure-time", "aria-live": "off" });
	const toggle = element("button", { type: "button", "aria-label": "Switch the lower chart's quantity" });
	controls.append(play, slider, clock, toggle);

	const legend = element("p", { class: "figure-legend" });
	const grid = element("div", { class: "figure-grid" });
	const plane = element("div");
	const stack = element("div", { class: "figure-stack" });
	grid.append(plane, stack);

	const details = element("details");
	details.append(element("summary", {}, "Table: what each method did"));
	const table = element("table");
	table.innerHTML =
		"<thead><tr><th>Method</th><th>Duration</th><th>Peak speed</th><th>Peak axis acceleration</th>" +
		"<th>Peak axis jerk</th><th>Furthest from the arc</th></tr></thead>";
	const body = element("tbody");
	for (const method of methods) {
		const peak = (keys: string[]) => Math.max(...method.rows.flatMap((row) => keys.map((key) => Math.abs(row[key]))));
		const row = element("tr");
		for (const cell of [
			method.label,
			`${format(method.duration)} s`,
			`${format(peak(["speed"]))} m/s`,
			`${format(peak(["ax", "ay"]), 1)} m/s²`,
			`${format(peak(["jx", "jy"]), 0)} m/s³`,
			`${format(peak(["offArc"]) * 1000, 0)} mm`,
		])
			row.append(element("td", {}, cell));
		body.append(row);
	}
	table.append(body);
	details.append(table);

	figure.append(controls, legend, grid);
	if (caption) figure.append(caption);
	figure.append(details);

	function draw(): void {
		const colors = [token(figure, "--series-1"), token(figure, "--series-2")];
		const surface = token(figure, "--sl-color-bg") || "white";
		const color = (index: number) => colors[index];

		legend.replaceChildren(
			...methods.map((method, index) => {
				const item = element("span", {}, `${method.label}: ${format(method.duration)} s`);
				item.style.setProperty("--swatch", color(index));
				return item;
			}),
		);
		clock.textContent = `t = ${format(time)} s`;
		toggle.textContent = quantity === quantities[0] ? "Show jerk" : "Show acceleration";

		const positions = methods.map((method, index) => ({ ...at(method, Math.min(time, method.duration)), index }));
		plane.replaceChildren(
			Plot.plot({
				width: 280,
				height: 280,
				marginLeft: 44,
				marginBottom: 36,
				ariaLabel: "Paths of both methods in the XY plane",
				x: { domain: [-0.02, 0.22], label: "x (m)", grid: true, ticks: 5 },
				y: { domain: [-0.02, 0.22], label: "y (m)", grid: true, ticks: 5 },
				marks: [
					...methods.map((method, index) =>
						Plot.line(method.rows, { x: "x", y: "y", stroke: color(index), strokeWidth: 2 }),
					),
					Plot.dot(positions, {
						x: "x", y: "y", r: 5, fill: (d: any) => color(d.index), stroke: surface, strokeWidth: 2,
					}),
					// Direct labels clear of both lines: outside the arc, and on the origin side of the chord.
					Plot.text(["arc", "straight"], {
						x: [0.155, 0.07], y: [0.155, 0.07], text: (d: string) => d,
						fill: "currentColor", textAnchor: "middle",
					}),
				],
			}),
		);

		const timeChart = (key: string, label: string, height: number) => {
			const chart = Plot.plot({
				width: 420,
				height,
				marginLeft: 48,
				marginBottom: 34,
				ariaLabel: `${label} over time for both methods`,
				x: { domain: [0, end], label: "time (s)" },
				y: { label, grid: true, nice: true },
				marks: [
					Plot.ruleY([0], { stroke: "currentColor", strokeOpacity: 0.3 }),
					...methods.map((method, index) =>
						Plot.line(method.rows, { x: "t", y: key, stroke: color(index), strokeWidth: 2 }),
					),
					Plot.ruleX([time], { stroke: "currentColor", strokeOpacity: 0.6 }),
				],
			});
			// Hovering a time chart scrubs the whole figure: the rule acts as a crosshair.
			chart.addEventListener("pointermove", (event: PointerEvent) => {
				const box = chart.getBoundingClientRect();
				const scale = (chart as any).scale("x");
				const x = (event.clientX - box.left) * (420 / box.width);
				const t = scale.invert(x);
				if (t >= 0 && t <= end) setTime(t);
			});
			return chart;
		};
		stack.replaceChildren(
			timeChart("speed", "speed (m/s)", 150),
			timeChart(quantity.key, quantity.label, 160),
		);
	}

	let frame = 0;
	function setTime(t: number): void {
		time = Math.round(t / sampleStep) * sampleStep;
		slider.value = String(time);
		cancelAnimationFrame(frame);
		frame = requestAnimationFrame(draw);
	}

	slider.addEventListener("input", () => setTime(Number(slider.value)));
	toggle.addEventListener("click", () => {
		quantity = quantity === quantities[0] ? quantities[1] : quantities[0];
		draw();
	});
	let playing = 0;
	play.addEventListener("click", () => {
		cancelAnimationFrame(playing);
		const slowMotion = 4;
		const started = performance.now();
		const step = (now: number) => {
			const t = ((now - started) / 1000) / slowMotion;
			setTime(Math.min(t, end));
			if (t < end) playing = requestAnimationFrame(step);
		};
		playing = requestAnimationFrame(step);
	});
	onThemeChange(draw);
	draw();
}
