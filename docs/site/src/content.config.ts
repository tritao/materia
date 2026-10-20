import { defineCollection } from "astro:content";
import { z } from "astro/zod";
import { docsLoader } from "@astrojs/starlight/loaders";
import { docsSchema } from "@astrojs/starlight/schema";

export const collections = {
	docs: defineCollection({
		loader: docsLoader(),
		// `status` tracks writing progress: stub, draft, reviewed (see docs/concepts/README.md).
		schema: docsSchema({ extend: z.object({ status: z.string().optional() }) }),
	}),
};
