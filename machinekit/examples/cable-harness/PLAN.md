# Cable harness plan

The LCSC Cable capture and the reference JST XH harness (README.md) are the cable half of a
three-part connection model: board connector, cable, machine. Decided 2026-10-25, with the pcbkit
plan (`pcbkit/PLAN.md`, "Interfaces, connector blocks and cables", I1–I4).

## Boundary

materia depends on pcbkit (the board engine, a pinned submodule once the sheet-router carrier
work starts), never the reverse.

| Owned by pcbkit, read here as data | Owned here |
|---|---|
| Interface schema: pins and roles, per-pin voltage and current, connector family and mating pair, keying and polarity, mating face, cable exit, plug clearance | LCSC Cable service capture and catalogue (`tools/suppliers/lcsccable.py`): wire styles, gauges, order options |
| Connector part data in `parts/<LCSC>.json` (housings, contacts, headers, terminal plugs: pitch, compatible contacts, gauge and insulation ranges, ratings, envelope) | Cable recipes: wire types, colours, cut/strip/tin lengths, end treatments, cavity maps, cable BOM |
| Mating rules (header ↔ housing ↔ contact, keying, polarity) | Cable-side validation: every cavity assigned, length budgets, end processing |
| Interface matching function (board connector vs cable end) | Harness geometry and routing through the machine (`machinekit.electrical.CableHarness`, drag chains) |
| Board outputs: connector positions and orientation, board STEP | Product-level cross-checks and traceability (below) |

## Steps

| # | Step |
|---|---|
| H1 | When pcbkit has I1a/I1b: `tools/suppliers/cable_assembly.py` reads connector facts and mating rules from pcbkit's part data instead of `lcsccable-reference.json`; the JST XH facts move to pcbkit (same sources: JST catalogue pages); delete the local copy (no compatibility layer) |
| H2 | A "pluggable terminal plug + stripped wire" end type (strip length, ferrule or bare, clamp range), for the trial board's KF2EDGR / WJ15EDGRC terminals and their plugs |
| H3 | Product-level cross-checks: the cable's cavity map vs the board connector's pins through pcbkit's interface matching (pinout, polarity, mating, ratings); traceability SR17 channel → cable cavity → board pin → net → block port; negative fixtures (reversed polarity, wrong housing family, gauge outside the contact's range, unassigned channel) |
| H4 | Endpoints from board outputs: a harness end takes its position and exit direction from the board's connector placement (pcbkit output), not from hand-entered coordinates |
| H5 | Move the cable model into MachineKit (Haxe) over time: recipes, validation and geometry as typed MachineKit electrical objects next to the machine's routing; keep the LCSC Cable capture as Python supplier tooling, like the other supplier importers |
| H6 | Beyond the reference: curved routes and bend-radius checks, branch junctions, a second connector family a real machine cable needs (chosen in the carrier's mechanical gate, pcbkit I4) |

## Notes

- LCSC Cable's API answered 403 until the capture sent browser-like `Referer`/`Origin` headers.
  Raw responses stay in `build/` (not committed); keep refreshes rare and manual, and check
  LCSC's terms before automating them.
- Assumptions in the reference harness (wire outer diameter, termination allowances, connector
  datums) stay marked as assumptions until sourced; nothing here is production-qualified yet.
