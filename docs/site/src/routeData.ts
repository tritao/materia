// The site has two halves, each with its own sidebar: the product pages (product/, the component catalogue, open
// core) and the documentation (everything else). astro.config.mjs lists both; this keeps the half a page is in,
// and pages through that half only.
import { defineRouteMiddleware } from "@astrojs/starlight/route-data";
import { PRODUCT_GROUP, inProduct } from "./sections";

type Entry = { type: "link"; href: string; label: string; isCurrent: boolean; attrs: Record<string, unknown> } | { type: "group"; label: string; entries: Entry[] };

const links = (entries: Entry[]): Extract<Entry, { type: "link" }>[] =>
	entries.flatMap((entry) => (entry.type === "link" ? [entry] : links(entry.entries)));

export const onRequest = defineRouteMiddleware((context) => {
	const route = context.locals.starlightRoute;
	const product = inProduct(context.url.pathname);
	const sidebar = (route.sidebar as Entry[]).filter((entry) => (entry.type === "group" && entry.label === PRODUCT_GROUP) === product);
	route.sidebar = sidebar as typeof route.sidebar;
	// Previous and next follow the sidebar the page shows, so they never cross into the other half.
	const flat = links(sidebar);
	const current = flat.findIndex((link) => link.isCurrent);
	if (current < 0) return;
	const { prev, next } = route.pagination;
	route.pagination = {
		prev: prev && (flat[current - 1] as typeof prev | undefined),
		next: next && (flat[current + 1] as typeof next | undefined),
	};
});
