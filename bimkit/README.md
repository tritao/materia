# BimKit

BimKit is a typed façade over a normal CadKit `Document`. BIM classification and a wall's uncut authored feature live in typed element properties. Hosting lives in a core `bim.host` relationship with typed along, sill, output, and unhost-state properties. `DocumentCodec` persists BIM data together with geometry, definitions, and other document state; `BimCodec.encode()` delegates to it, while `BimCodec.decode()` also imports legacy BimKit format v2 files.

Window and Door Types are authored CadKit subgraph definitions with typed length inputs and named body/opening outputs. Their instances use CadKit's existing instance machinery and the same hosted-opening validation path, so type edits update shared instances and `makeUnique()` detaches one instance while copying the type's BIM properties.

The first supported host adapter is deliberately narrow: straight vertical walls whose local baseline follows +X, with rectangular definition outputs placed by distance along the baseline and sill height. The hosted instance placement is derived from those coordinates and parented to the wall. Opening resolution always uses the wall's uncut feature.

`RepeatedHostedWindows` shows four instances of one window definition on two walls between levels. BIM documents leave the document-level output unset; each element publishes its own geometry. An opening may be unhosted or removed transactionally. A wall must have no hosted openings before removal. Host placement and wall-local cuts follow level changes after successful recompute.

`BimBuildingFixture` is the integration model: one Project, Site, Building, two Storeys with Level datums, exterior and interior walls, shared Window and Door Types, slabs, Spaces, hosted openings, and Storey containment. Its smoke path exercises save/reopen, shared type edits, `makeUnique()`, Level edits, rehosting, deletion, and undo/redo.

The spatial façade creates geometry-free Project, Site, Building, Storey, and Space objects. Project → Site → Building → Storey is stored with `bim.aggregates`; Storey contents use `bim.contains`. BIM classes remain typed properties on generic CadKit elements. A Storey can reference separate CadKit Level datums for its base and top, and hierarchy/containment edits participate in document undo and redo.

Use `resizeWall` for thickness changes so hosted opening tools follow the new wall depth. Direct edits that leave a tool too shallow, move an opening outside its wall, or overlap two openings fail validation before recompute commits geometry. Failed host edits discard their new graph nodes; committed earlier nodes remain inactive for undo and redo. The core relationship preserves authored host coordinates and the placement to restore on unhosting.
