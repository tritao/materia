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

The site is deployed by Cloudflare Pages from the GitHub repository:

| Setting | Value |
|---|---|
| Production branch | `main` |
| Root directory | `docs/site` |
| Build command | `npm run build` |
| Build output directory | `dist` |
| Build watch paths | include `docs/*` |

The Node version comes from `.node-version` in this directory.

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
