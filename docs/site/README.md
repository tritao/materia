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

The Concepts sidebar is read from `docs/concepts/README.md`: each `##` heading
is a group and each list link an entry, so reorder pages there.

Front matter must be valid YAML: quote a `title` or `description` that
contains `: `.
