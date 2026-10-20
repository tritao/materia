/** Site slug for a docs-relative Markdown path: `concepts/README.md` → `concepts`, `OPEN_CORE.md` → `open-core`. */
export function slugOf(path) {
	return path
		.replace(/\.md$/, "")
		.replace(/(^|\/)README$/, "")
		.toLowerCase()
		.replace(/_/g, "-");
}

/** Site route for a slug; the empty slug is the home page. */
export function routeOf(slug) {
	return slug ? `/${slug}/` : "/";
}
