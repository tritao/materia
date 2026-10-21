---
title: Documentation plan
description: How Materia's user manual, developer manual, concept docs and product docs are structured, built and published.
status: draft
---

# Documentation plan

## Goal

One documentation site for Materia, released alongside the code. It serves three
readers:

| Reader | Needs | Main sections |
|---|---|---|
| **Machine builders who write code**, the primary reader today | Understand the stack, define a machine, simulate it, run it | Concepts, tutorials, developer guides, API reference |
| **Editor users** | Open, inspect, modify and simulate machines without writing code | User manual |
| **Contributors** | Build from source, run tests, change kits | Contributor guide |

Product docs (landing page, feature pages, example gallery, roadmap, release
notes, licensing) sit on top. Manuals for commercial products come later,
following [OPEN_CORE.md](OPEN_CORE.md).

## Principles

1. **Diátaxis.** Tutorials (learn by doing), how-to guides (do a task),
   reference (look it up), explanation (understand). Each page is exactly one
   of these.
2. **Generate what changes.** The API reference, snippets, figures and
   screenshots come from the code. Prose covers what is stable: concepts,
   intent and boundaries.
3. **Every figure comes from real Materia code.** It is either computed by the
   kits when the docs are built, or run live in wasm. Never a JavaScript
   re-implementation of a concept.
4. **Docs are tests.** Snippets compile, figures rebuild, links resolve, and
   CI fails if any of them break.
5. **Plans are not docs.** `PLAN.md`, `TODO.md` and handoff files stay where
   they are. Their decisions are distilled into concept pages; readers are
   never sent into them.
6. **Honest about maturity.** The project is 0.x and every kit has a stability
   label. Docs are versioned by release tag, plus a "main" view.

## Site structure

```
docs/
  concepts/        explanation: what Materia is and how it fits together
  tutorials/       end-to-end: first machine, gantry, router, arm
  guides/          how-to: import STEP, add a transmission, export a BOM, ...
  manual/          editor user manual: commands, panels, project files
  reference/       generated API reference per kit, CLI, file formats
  contributing/    build, test, worktrees, haxeon notes
  machines/        open machine build guides (BOM, fabrication, assembly, commissioning)
  products/        commercial product manuals (later)
  OPEN_CORE.md
  DOCS_PLAN.md
```

Kit READMEs stay short (what the kit is, how to build and test it) and link to
the site for concepts.

## Technology

| Need | Choice | Notes |
|---|---|---|
| Site generator | **Astro + Starlight**, pages in MDX | Search, versioning, sidebar; islands load interactive components only when they scroll into view |
| 3D figures | **three.js** over baked glTF | Small and universal; switch to a slim SceneKit viewer if one is built (it would also be a product feature) |
| 2D plots | **Observable Plot** (SVG) | Light and accessible |
| Live widgets | **Haxeon wasm-gc docs runtime** | Pure-Haxe kits first; native kits once their wasm size is measured |
| Full editor embeds | The existing web build, deep-linked, loaded on click | Cached by a service worker, so one download serves the whole site |
| API reference | Generated from Haxeon's typed output | Check whether Haxeon can emit Haxe's `-xml` dump (then `dox` works); otherwise write a small generator over the typed AST |
| Hosting | **Cloudflare Pages** or **Netlify** | Threads in the editor need COOP/COEP headers, which GitHub Pages cannot set; check per-file size limits against the release wasm |

## Interactive figures

Three weights, chosen per figure:

1. **Baked** (most figures). A Haxeon `docs-figures` program runs the real kits
   natively at docs build time, as `app/web/examples/WebExampleExport.hx`
   already does for examples, and writes JSON/glTF. The page lets the reader
   scrub time, toggle layers and choose between precomputed variants. A static
   image fallback serves print, PDF and search.
2. **Live** (where the reader's input is the point). A small wasm-gc runtime of
   pure-Haxe kits, exposing a few functions. Candidates: MachineKit
   (transmissions and derived limits), CncKit (G-code and diagnostics),
   ToolpathKit and StockKit.
3. **Open in Materia** (tutorials and *The life of a machine*). The full web
   editor, deep-linked to an example and a starting state.

Figure snapshots are compared in CI; a changed output is reviewed like a
changed test.

## Phases

| Phase | Work | Done when |
|---|---|---|
| **D0** | Inventory: tag every existing `.md` as keep, merge, internal or delete; final site map; stability label per kit | [INVENTORY.md](INVENTORY.md) committed with the tags, site map and stability labels |
| **D1** | Site skeleton: Starlight serving `docs/`, link check, CI build and preview deploys | Every page renders; broken links fail CI |
| **D2** | Concepts, first six: Life of a machine, Conventions, Execution plans, Fidelity levels, Assemblies and mates, Controller boundary | Six pages at `status: reviewed`; their open questions resolved or filed |
| **D3** | Figure pipeline and first figure (paths vs. trajectories, baked from MotionKit) | Figure recomputed as a workspace test; snapshot check |
| **D4** | Getting started and the gantry tutorial, with "Open in Materia" | A new reader goes from clone to a running mission by following it |
| **D5** | API reference generator for every kit | Reference pages built from code, linked from concept pages |
| **D6** | Docs runtime spike: MachineKit + CncKit in wasm-gc, size measured; transmissions and G-code widgets | Widgets live on their concept pages |
| **D7** | The remaining concept pages; the user manual for the editor | No stubs left in `concepts/` |
| **D8** | First open machine build guide (sheet router) | Deferred until the router is built and commissioned (maintainer, 2026-10-21) |
| **D9** | Product pages: landing, features, gallery, roadmap, release notes, licensing | Site ready to announce |
| **Later** | Commercial product manuals, structured for the Machinery Regulation | Per product |

**Release gate.** Before the site is announced, the decisions in
[OPEN_CORE.md](OPEN_CORE.md) are made and the LICENSE and notices files are in
place.

## Status

- D2 started. The concept skeleton, *The life of a machine* (draft) and the
  glossary are in `docs/concepts/` (71f9d100e). All six D2 pages are drafted:
  Life of a machine, Conventions, Execution plans, Fidelity levels, Assemblies
  and mates, Controller boundary. Each records its open questions; review moves
  them to `status: reviewed`.
- Open-core licensing accepted: Apache-2.0, CC BY 4.0 for docs, DCO
  ([OPEN_CORE.md](OPEN_CORE.md)).
- D1 done; the site is live at <https://materia-docs.joao-9f7.workers.dev>. The Starlight site in `docs/site/` builds every page
  under `docs/`, fails on invalid front matter or broken internal links, and
  runs in CI (`.github/workflows/docs.yml`, uploads the built site). Choosing
  hosting is Cloudflare Workers static assets deployed by Workers Builds
  (`docs/site/wrangler.jsonc`, settings in `docs/site/README.md`).
- D3 done: `docs/figures` computes figure data with the real kits as a
  workspace member (`docs-figures`), shares the workspace's native builds, and
  fails as a test when a committed figure is stale. The first figure (paths vs.
  trajectories) is live on its concept page.
- D7 concepts done: no stubs left in `concepts/`; every page is a draft
  checked against the code, with its inconsistencies under Open questions.
  Every diagram is laid out to read at the column's width. The editor user
  manual is drafted in [manual/](manual/README.md) from the app code, with the
  README's disagreements listed per page.
- D5 done: `docs/api/generate.py` parses every kit the editor uses with
  Haxeon's parser into `docs/api/data/` (27 kits, about 3,700 public types);
  the site renders a page per package under `/reference/` and links type names
  on concept pages to it. The API reference comes from source declarations,
  not the typed program, so a type is shown as the source writes it. CI fails
  when the data is stale.
- D0 done: [INVENTORY.md](INVENTORY.md) tags all 324 tracked Markdown files
  (114 keep, 43 merge, 159 internal, 8 delete) and labels each kit's stability
  (none stable yet). The labels live there rather than in
  `materia.workspace.json`, whose entries are test projects, not kits. The
  eight delete candidates were removed with the maintainer's approval.
- D9 drafted: [product/](product/README.md) has features, the example gallery,
  licensing and a draft roadmap built from the plans for the maintainer to
  edit. Release notes start with the first release.
- D6 done: `docs/runtime` compiles CncKit, ToolpathKit and RobotKit's robot
  model to Wasm GC (1.4 MB, 320 KB gzipped). Toolpaths has a live G-code
  widget (moves and positioned diagnostics); Transmissions has a belt-axis
  widget whose speed and acceleration come from `RobotModel.coupledLimits`.
  The runtime is committed with a hash of its inputs, which CI checks, and runs
  under Node in CI. MachineKit itself stays out: it depends on CadKit's native
  core, so the widget builds the robot model directly.
- D4 done: the gantry-picker tutorial (snippets checked against the
  code) and the web editor's `?example=ID` deep link behind its "Open in
  Materia" link (tested by `app/web/test.sh --launch`). The link stays hidden
  until the editor is hosted and `PUBLIC_MATERIA_EDITOR_URL` is set. The
  maintainer chose to skip the fresh-clone walk through the tutorial for now.
- D8 deferred by the maintainer until the sheet router is built and
  commissioned; it is outside this plan's finished scope.
- Plan finished on 2026-10-21. Every page written so far is a draft; the
  maintainer reviews them and moves each to `status: reviewed`.
