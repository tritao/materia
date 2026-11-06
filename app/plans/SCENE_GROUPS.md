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
- Not started.
