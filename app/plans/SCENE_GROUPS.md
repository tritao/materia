# Scene groups and sets (2026-11-06)

Goal: in the editor, hide or isolate everything belonging to one machine function (for example the hobber's X axis, with its
fixed and moving parts) and cross-cutting categories (all fasteners, everything that moves with X), without CAD-style layers.

## Decisions
- **Groups (primary):** every assembly member has exactly one functional group path (e.g. `X axis`, `B spindle/Belt drive`).
  It is a lightweight authored folder, not a nested subassembly: joints, mates and kinematics are unchanged. It is saved in the
  assembly description, validated (every member in exactly one group; nested paths allowed) and shown as the editor tree.
  Members of included subassemblies keep their subassembly path under the group the include is placed in.
- **Sets (secondary):** many-to-many, mostly derived, never hand-maintained:
  - *Moves with <joint>*: every member carried by that joint (from the joint tree).
  - *By kind*: fasteners (screws, nuts, washers), dowels, covers and wipers, castings and inserts, purchased versus made (from
    the BOM and procurement metadata), electrical.
  - Optionally authored sets for things no rule derives.
- **Editor:** the hierarchy panel shows groups as the tree, with an eye toggle (hide/show) and isolate (show only this) on groups
  and sets, and a "Sets" section listing the derived and authored sets. Visibility is per document, saved with the workspace
  and document view state, and composes with presentation views (a hidden group stays hidden in every view unless the view says
  otherwise).
- No layers concept; sets with visibility give the same behaviour without upkeep.

- Group scopes apply to local members and includes; an include supplies the functional ownership of descendants with no
  more specific group, retaining the include path beneath its folder. Authored child folders remain beneath that path.
- The portable mechanical occurrences carry functional group paths and derived category tags, so generated artifacts,
  recipe reloads and the editor all receive the same membership. MachineAssembly schema is 27; older descriptions are rejected.
- Sets are derived only in G1–G4. Optional authored sets are deferred until a concrete use needs a rule-independent selection.

- Set rows are selections, not second owners of their members. They list counts and expose visibility/isolate while each part
  retains one row in the primary functional tree. Existing document visibility masks remain the single persisted representation.
- Fasteners use the standard hardware recipe namespace and typed screw interface; motion nuts and nut mounts are excluded.
  Procurement categories use explicit Make/Buy intent; electrical uses typed interfaces and ports. Sets sort IDs and members
  for stable results across save/load and changes in construction order.

- Category metadata is computed when exporting an editor definition or assembly description, not on temporary solver
  flattenings. Iterative mechanical sizing does not evaluate BOM/procurement just to build its joint tree.

- Isolate reveals the selected folder and its ancestors across their full memberships, preserving independent part and
  descendant masks. This keeps group masks consistent when saved visibility is rebound on document reload.

- Purchased includes physical subparts explicitly marked IncludedInPurchase when they are not separately supplied.
  Explicit Make/Buy intent governs separately specified parts.

## Steps
- **G1. Library:** group path on members (API on `MachineAssembly`: a group scope while building and/or a `group` argument on
  `fixAt`/`addMember`/`include`), saved in the assembly description (one schema version, no compat), validation, derived sets
  (`AssemblySets`: moves-with per joint, by kind), unit tests.
- **G2. Editor:** the tree from groups (`app/src/editor/AssemblyHierarchy.hx`, `HierarchyPanel`, `HierarchyTreeView`,
  `SceneTreeRow`); group and set visibility and isolate; Sets section; persistence; app tests; capture check.
- **G3. Hobber:** declare groups in `FlexsplineHobber` (bed and column, X axis, Z axis, A swivel, Y axis, B spindle, C table and
  work, chip protection and enclosure, electrical). The per-axis drive, scale and cover helpers set their axis group. Check that
  every member is grouped.
- **G4. Bench mill** (both the legacy `BenchMill` and the framed concept), then the other examples that build assemblies by
  hand. Members left without a group fall into an `Ungrouped` group, flagged by a warning rather than an error, until they are
  migrated.

## Status
- **G1 complete:** scoped functional ownership, schema 27, nested ownership roundtrips and export-time derived sets.
  Focused library suite passed 22 checks. Older MachineAssembly descriptions must be regenerated.
- **G2 complete:** functional hierarchy, Sets section, independent eye/isolate controls and composed document masks.
  Focused editor visibility suite passed (118 assertions), including rendered isolate keyboard activation, overlapping
  masks, document independence and hidden-ancestor isolation across save/load. Full app suite passed with exit 0.
- **G3 complete:** all 269 hobber members are grouped, with 16 derived sets. Hobber baseline passed 2345 assertions
  (original baseline 2333). Hobber construction changes are group declarations/scopes only.
- **G4 complete:** legacy/framed mills and production hand-built examples, including inherited gantries and tracks.
  The legacy mill has 169 grouped members; the framed concept has 474 grouped members and 33 derived sets.
  Mill frame suite passed 951 assertions (baseline 917); hobbing suite passed 1159 assertions (baseline 1159).
- Real-machine editor-tree integration passed 2858 checks across the hobber and both mills: artifact ownership,
  fixed/moving visibility, group/set isolation and persisted masks. Generated evidence is
  `app/tests/build/scene-groups-ui-tree.txt`, from `app/tests/haxeon.machine-groups.json` with an absolute output-path argument.
  The hobber tree shows X axis (31), Fasteners (108) and Moves with X (124). UI-tree evidence and rendered control tests
  cover the editor without using the known broken PNG capture/layout serialization path.
- The initial app baseline reproduced the known CAD plate output-file failure. It did not recur in final validation;
  the final full app run passed, including CAD workflow tests. No unresolved design decisions remain; optional authored
  sets stay deferred as recorded above. G1–G4 are committed separately.
