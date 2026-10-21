# Documentation site

The site is built with [Astro](https://astro.build) and
[Starlight](https://starlight.astro.build) from the Markdown under `docs/`.
`docs/` stays the source of truth; nothing in this directory is content.

```sh
cd docs/site
npm ci
npm run dev     # http://localhost:4321, rebuilds on change (rerun `npm run sync` after editing docs/)
npm run build   # static site in dist/; fails on broken internal links
```

`scripts/sync.mjs` copies `docs/**/*.md` into `src/content/docs/` (generated,
ignored by Git) and adapts it:

- `README.md` becomes the section index; `OPEN_CORE.md` becomes `/open-core/`.
- The leading `# Title` is dropped, since Starlight renders the front-matter
  title.
- Relative links to other docs pages become site routes; links to the rest of
  the repository become GitHub links.
- `status: stub` and `status: draft` pages get a banner.
- Figure data in `docs/figures/data/` is copied to `public/figures/`.

The Concepts sidebar is read from `docs/concepts/README.md`: each `##` heading
is a group and each list link an entry, so reorder pages there.

Front matter must be valid YAML: quote a `title` or `description` that
contains `: `.

## Hosting

The site is served by Cloudflare Workers (static assets) and deployed by
Workers Builds from the GitHub repository on every push to `main`:

| Setting | Value |
|---|---|
| Path (root directory) | `docs/site` |
| Build command | `npm run build` |
| Deploy command | `npx wrangler deploy` |
| Build watch paths | include `docs/*` |

`wrangler.jsonc` names the Worker and points it at `dist/`; the Node version
comes from `.node-version`.

## Figures

A Markdown page embeds an interactive figure as

```html
<figure class="materia-figure" data-figure="NAME">
<figcaption>What the reader should see.</figcaption>
</figure>
```

followed by a Markdown line linking the data, for GitHub readers.
`src/scripts/figures.ts` renders it from `/figures/NAME.json`. The data is
computed by `docs/figures` with the real kits, never by hand:

```sh
# from the repository root: check the committed data is current
./haxeon/scripts/haxeon workspace test --workspace materia.workspace.json --only docs-figures
# recompute it after a kit change, then review the diff
DOCS_FIGURES_WRITE=1 ./haxeon/scripts/haxeon workspace test --workspace materia.workspace.json --only docs-figures --no-test-cache
```

## API reference

`/reference/` is generated from the kits' source. `docs/api/generate.py` walks the editor's package
graph, parses each kit with Haxeon's own parser (`docs/api/src/ApiXml.hx`, run on the pinned Haxe;
declarations and doc comments only, no compile) and writes one JSON file per kit to `docs/api/data/`.
`scripts/reference.mjs` renders those files into a page per package, and links `Type` and
`Type.member` code spans on concept and tutorial pages to their entries when the name is unique.

The data is committed, so the site builds without Haxe. Regenerate it after changing a kit's public
API; CI runs `docs/api/generate.py --check` and fails when it is stale:

```sh
docs/api/generate.py
```

## Live widgets

`<figure class="materia-figure materia-widget" data-widget="NAME">` runs real kit code in the reader's
browser through the docs runtime, `docs/runtime` (CncKit and ToolpathKit compiled to Wasm GC by
`docs/runtime/build.sh`). The figure's Markdown stays the fallback until the runtime loads, which
happens only when the widget scrolls near the viewport. The runtime is committed with a hash of its
inputs; CI fails when they change and runs `docs/runtime/check.mjs` against it.
