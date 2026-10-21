/** Search and family filters for the component catalogue's index (/components/). */
export function filterComponents(): void {
	const search = document.querySelector<HTMLInputElement>("[data-part-search]");
	const family = document.querySelector<HTMLSelectElement>("[data-part-family]");
	if (!search || !family) return;
	const cards = [...document.querySelectorAll<HTMLElement>(".part-card")];
	const empty = document.querySelector<HTMLElement>(".part-empty");
	const apply = () => {
		const words = search.value.toLowerCase().split(/\s+/).filter(Boolean);
		let shown = 0;
		for (const card of cards) {
			const visible = (!family.value || card.dataset.family === family.value) &&
				words.every((word) => (card.dataset.search ?? "").includes(word));
			card.hidden = !visible;
			if (visible) shown++;
		}
		if (empty) empty.hidden = shown > 0;
	};
	search.addEventListener("input", apply);
	family.addEventListener("change", apply);
}
