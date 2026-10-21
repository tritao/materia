// The menu bar's sections, shared by the header and the sidebar middleware.

/** The sidebar group that holds the product pages; astro.config.mjs uses the same label. */
export const PRODUCT_GROUP = "Product";

/** Whether a path belongs to the product half of the site rather than the documentation. */
export const inProduct = (path: string) => /^\/(product|components|open-core)(\/|$)/.test(path);

/** The menu: each item is current on the paths its pattern matches. */
export const MENU = [
	{ label: "Product", href: "/product/", current: /^\/product\/(features\/)?$/ },
	{ label: "Gallery", href: "/product/gallery/", current: /^\/product\/gallery\// },
	{ label: "Components", href: "/components/", current: /^\/components\// },
	{ label: "Docs", href: "/documentation/", current: /^\/(documentation|tutorials|concepts|manual|reference|docs-plan|inventory)\// },
	{ label: "Roadmap", href: "/product/roadmap/", current: /^\/product\/roadmap\// },
	{ label: "Open source", href: "/product/licensing/", current: /^\/(product\/licensing|open-core)\// },
];
