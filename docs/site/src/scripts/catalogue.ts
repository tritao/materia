/** Search, family and kind filters for the component catalogue's index (/components/). */
export function filterComponents(): void {
	const search = document.querySelector<HTMLInputElement>("[data-part-search]");
	const family = document.querySelector<HTMLSelectElement>("[data-part-family]");
	const kind = document.querySelector<HTMLSelectElement>("[data-part-kind]");
	if (!search || !family || !kind) return;
	const sections = [...document.querySelectorAll<HTMLElement>(".part-family")];
	const empty = document.querySelector<HTMLElement>(".part-empty");
	const apply = () => {
		const words = search.value.toLowerCase().split(/\s+/).filter(Boolean);
		let shown = 0;
		for (const section of sections) {
			const inFamily = !family.value || section.dataset.family === family.value;
			let sectionShown = 0;
			for (const group of section.querySelectorAll<HTMLElement>(".part-group")) {
				let groupShown = 0;
				for (const card of group.querySelectorAll<HTMLElement>(".part-card")) {
					const visible = inFamily && (!kind.value || card.dataset.kind === kind.value) &&
						words.every((word) => (card.dataset.search ?? "").includes(word));
					card.hidden = !visible;
					if (visible) groupShown++;
				}
				// A category, and a family, with nothing left to show is hidden with its heading.
				group.hidden = groupShown === 0;
				sectionShown += groupShown;
			}
			section.hidden = sectionShown === 0;
			shown += sectionShown;
		}
		if (empty) empty.hidden = shown > 0;
	};
	search.addEventListener("input", apply);
	family.addEventListener("change", apply);
	kind.addEventListener("change", apply);
}
