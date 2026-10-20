import { readFileSync } from "node:fs";
import { defineConfig } from "astro/config";
import starlight from "@astrojs/starlight";
import starlightLinksValidator from "starlight-links-validator";
import { slugOf } from "./scripts/routes.mjs";

/**
 * Sidebar groups for the concept pages, read from docs/concepts/README.md so the index stays the one
 * place that orders them: each `## Heading` is a group, each `- [Label](page.md)` an entry, up to the
 * first `---` rule (the writing guide below it is not navigation). Groups after the first start collapsed;
 * Starlight opens the one holding the current page.
 */
function conceptGroups() {
	const index = readFileSync(new URL("../concepts/README.md", import.meta.url), "utf8");
	const groups = [];
	for (const line of index.split("---\n")[0].split("\n")) {
		const heading = /^## (.+)/.exec(line);
		if (heading) groups.push({ label: heading[1].replace(/:.*/, ""), collapsed: groups.length > 0, items: [] });
		const entry = /^- \[([^\]]+)\]\(([^)]+\.md)\)/.exec(line);
		if (entry && groups.length) groups.at(-1).items.push({ label: entry[1], slug: slugOf(`concepts/${entry[2]}`) });
	}
	return groups;
}

export default defineConfig({
	integrations: [
		starlight({
			title: "Materia",
			description: "Design, simulate, plan and run machines, from CAD to the controller.",
			social: [{ icon: "github", label: "GitHub", href: "https://github.com/tritao/materia" }],
			plugins: [starlightLinksValidator()],
			components: { MarkdownContent: "./src/components/MarkdownContent.astro" },
			sidebar: [
				{ label: "Concepts", items: [{ label: "Overview", slug: "concepts" }, ...conceptGroups()] },
				{
					label: "Project",
					items: [
						{ label: "Open core", slug: "open-core" },
						{ label: "Documentation plan", slug: "docs-plan" },
					],
				},
			],
		}),
	],
});
