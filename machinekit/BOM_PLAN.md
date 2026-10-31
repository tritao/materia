# BOM plan

Goal: every machine produces a bill of materials that someone could buy and
build from. Each line is a real purchasable or manufacturable unit with a
stable, readable part number. Each line says whether it is made, bought or
still undecided. The BOM leaves the code as a versioned JSON file that a
separate sourcing layer (vendors, RFQs, quotes) reads without knowing
anything about haxeon.

Sourcing itself (vendors, quotes, price breaks, landed cost) is out of scope
here. This plan ends at the export contract (B8).

## What exists (B0 survey, 2026-10-19)

- `MachineComponent.bom` derives one `BomItem` per component: `partNumber`,
  `description`, `quantity`, `material`, and optional `typeId`, `valuesKey`,
  `familyId`, `manufacturingVariant`, `manufacturingProcess`.
- `MachineAssembly.billOfMaterials()` flattens the assembly and its
  subassemblies (`AssemblyInventory.billOfMaterials`), then `Bom` groups the
  lines by part number. It rejects two lines that share a part number but
  have different metadata. Lines without geometry (hoses, wire) come from
  `addBomItem`.
- The BOM is never written anywhere. The only consumers are a line-count
  assertion in `BenchMillPreview` and a few `CncRouterPreview` traces.
- `SupplierOffer` and `SourcingPlan` attach supplier offers to part numbers.
  Only the `supplier-fasteners` examples use them.
- `ManufacturingVariant` already records process, material, status and the
  hardware a variant needs (dh-gripper uses it).
- `SupplierCadComponent` pins vendor STEP files by hash.

Added on main after the first survey (checked 2026-10-31):

- `IncludedInPurchase` (component facet): marks a component as physically
  part of another, separately listed purchase. `Bom.addComponent` skips it and
  `lines()` throws if the owner part number is missing. Only
  `PurchasedRotaryPart` uses it so far.
- `AssumedCatalogComponent`: base class for unverified catalogue-class
  placeholders (25 subclasses). Its part numbers are
  `<ID>-ASSUMED-<12 hex of a values hash>`, stable but unreadable.
- `EngineeringMetadata` (facet): `ratingsBasis`/`errorsBasis` of
  Assumed/Catalog/Stated/Derived plus a `source`; Catalog requires a source.
- `BomItem.enclosurePanel`: panel construction, size, area and mass for
  laminated enclosure panels (sheet router).
- `tools/suppliers/` (Python): `lcsccable.py` captures a supplier catalogue
  with raw evidence and offline replay; `cable_assembly.py` emits a harness
  BOM. This is the precedent for the sourcing layer's language and layout.

Bench mill (`new BenchMill()`) gives 107 members and 76 lines. Problems:

1. **Part numbers come in three kinds:**
   - Recipe keys: `machinekit.motion.linear-rail|profile:5:HGR15|length:3:420`.
     Deterministic, but internal and unreadable.
   - Hand-written designations: `SHAFT-14x30`, `GENERIC-HG-HGR15-BLOCK`.
     These are fine.
   - Designations built from raw values:
     `MILL-MOTOR-PLATE-110-8-32.687988618379066`,
     `MILL-BRACKET-217-80-10-null-8`. They print floats and `null`, and they
     change whenever the geometry moves. There are 104 constructors in
     machinekit that build designations by interpolating values. Most use
     `Dimension.format`, but some print raw values.
2. **Modelling pieces show up as purchases.** The assumed shaft brake
   (`ShaftBrakePart`) is split into sectors so its collision hulls stay
   convex, which gives 26 BOM lines for one purchased brake.
3. **No make/buy/undecided classification.** Castings and brackets need
   drawing quotes, while rails, blocks and drives need SKU quotes. All of
   them sit in the same list.
4. **Placeholders look like parts.** `GENERIC-HV-SERVO`,
   `GENERIC-400V-SERVO-AMP`, and the spindle cartridge and ER20 holder marked
   "assumed envelope" are design requirements, not chosen parts.
5. **Missing content:**
   - No fasteners at all.
   - No wiring, cable chain, lubrication or enclosure parts.
   - Bought parts carry the material used for mass estimates
     (servo = "aluminium 6061").
6. **Wrong content.** The workpiece, end mill and toe clamps are in the
   machine BOM because `BenchMill` includes the workpiece by default.
7. **No structure in the output.** Subassemblies exist, but the BOM is flat
   with no record of where each part is used.

## Design

## Design amendment (2026-10-31, after reviewing the uncommitted B1–B4 work)

These replace the matching parts of "Design" and the phases. Where the plan and this section disagree, this section wins.

### A1. Procurement kind and selection maturity are separate fields
- `Procurement` is `Make(process)`, `Buy(manufacturer, mpn, standard)` or `NotSupplied`. Remove `Undecided`.
- Maturity is a separate field on every line. It is taken from `EngineeringMetadata.ratingsBasis` (`Assumed` / `Catalog` / `Stated` / `Derived`) and is `Assumed` for every `AssumedCatalogComponent`.
- An assumed servo is `Buy` + `Assumed` + a requirement. Choosing a vendor part changes maturity (and fills in the MPN). The kind stays `Buy`.
- Delete the rule "Assumed component needs catalogue evidence before purchase". Replace it with: every `Assumed` line must carry a requirement.
- **Requirements are structured, not prose:** a list of `{name, min?, max?, nominal?, unit}`, e.g. `peakTorque ≥ 3.8 N·m`, `busVoltage 400 V`, `shaftDiameter 19 mm`. Format values with `Dimension.format`, never `Std.string(Float)`. Each recipe that can be assumed declares which of its inputs or ratings form the requirement.

### A2. Procurement is declared where the part is defined
- Each recipe (`ComponentType`) declares its kind, and its requirement terms where relevant, next to its numbering. Non-recipe components declare it with the `ProcurementIntent` facet.
- `ProcurementMapping`'s guesses (the `Std.isOfType` chains, the hardcoded `LinearRail`/`LinearRailBlock`/`ScrewSupportUnit` list, the `"GENERIC-"` prefix test, the free-text requirement) may be used once, as a migration aid that writes the declarations. Delete them before B7. The B7 gate requires every line's kind to come from a declaration.

### A3. Part numbers are a pure function of recipe inputs
- `ComponentType.numbered` takes `ComponentValues -> String`, not `MachineComponent -> String`. Use no `cast`: the number is computed from the same values the recipe builds from, so it is stable by construction and needs no geometry.
- Non-recipe components keep a declared designation (B3).

### A4. Fingerprint and released revision are separate
- `BomRevision` stays as the **fingerprint**: a hash of the recipe key plus a per-recipe generator version, which the author bumps when a recipe's geometry code changes. It is not a hash of exported B-rep, which can change when OCCT changes.
- A **release** is an explicit, committed record: part number, revision letter (`A`, `B`, …), the fingerprint it pins, and the date. Store it in one JSON file per machine (e.g. `machinekit/examples/<machine>/releases.json`, schema version 1).
- Each export line carries `fingerprint`, plus `releasedRevision` and `releaseCurrent:Bool` when a release exists. Quotes (in the later sourcing layer) refer to released revisions.
- Creating the first releases is not part of this plan. Only the mechanism, its schema and a test are.

### A5. Quantity has a unit
- A BOM line's quantity is `{value, unit}`, with unit one of `piece`, `m`, `m2`, `kg`, `l`.
- Profiles, cable, hose and chain use length, plus a cut list (`cuts: [{length, count}]`) where pieces are cut from stock.
- `enclosurePanel` becomes an ordinary line in `m2` with its construction as properties. Remove the special field.
- Put this in the version 1 export now (B1). The format must not change shape later.

### A6. The engineering BOM stays true to the model; procurement is a view
- `BomDocument` keeps every modelled line, including `IncludedInPurchase` pieces (marked with their owner) and `NotSupplied` lines, in the subassembly tree.
- The procurement view (whole purchase units, no not-supplied lines, quantities rolled up) is derived from it by a function and exported alongside it in the same file. It is never produced by dropping lines on the way in.

### A7. Smaller items
- `BomUse.kind` is an enum, not a string.
- Keep Python to the gate comparison (`bom-gate-compare`). BOM behaviour is tested in haxeon tests; drop the other Python test and report scripts, or fold them into haxeon tests.


**Identity and revision are separate.** A part number names *which part*
it is, not its current dimensions:

- **Made parts** get a stable, author-declared drawing number, for example
  `MILL-MOTOR-PLATE-Z`. They also get a revision fingerprint: a hash of
  the recipe key or the exported geometry.
- **Bought parts** get a catalogue-style number built from their
  standardised parameters, for example `HGR15-R420` and `HGH15CA`, plus
  manufacturer and MPN once one is chosen.

Moving a hole on a bracket keeps its part number and changes its revision.
A quote then stays attached to the part but is marked "geometry changed
since quoted" rather than disappearing silently. `valuesKey` stays as the
exact configuration key for caching and grouping. It is no longer
overloaded as a part number.

**Procurement kind is required, with no default:**

- `Make(process)`: drawings, process (CNC, casting, sheet, print),
  material. This links to `ManufacturingVariant` where a family exists.
- `Buy`: catalogue item. Optional manufacturer/MPN, and a standard
  reference (ISO 4762, DIN 625, …).
- `Undecided(requirement)`: placeholder with the spec it must meet, for
  example "servo, ≥3.8 N·m peak, 400 V bus, 19 mm shaft". This is what an
  RFQ asks vendors to propose against.
- `NotSupplied`: workpieces, cutting tools and fixtures, which are listed
  but excluded from the machine's procurement BOM.

**Purchase units use `IncludedInPurchase`.** One component of a bought unit
carries the purchase line; every other piece (modelling splits such as the
brake sectors, or real subparts that ship inside the unit) carries
`IncludedInPurchase(owner)`. The BOM keeps only the owner line, and an
included piece whose owner is absent is an error. No second mechanism. Kits sold as a set (an SFU screw with nut and BK/BF supports)
are a sourcing concern: one quote covers several lines. They stay separate
in the BOM.

**One export, one schema version.** The `machinekit.bom` JSON has version 1,
and readers reject anything else. It contains:

- machine id, source commit, and generation time;
- the subassembly tree;
- the rolled-up procurement lines (kind, part number, revision,
  description, quantity, material and process for Make, requirement for
  Undecided);
- where each line is used (member paths).

## Phases

### B1 — Export and baseline
- Done in this commit: the exporter and baseline cover 22 constructed examples.
  Their procurement identities match the baseline. The full gate passed 11 of
  24 shards, with no new failing shard or exception message against the baseline.
  The machine-tending constructor currently rejects its stale reach certificate;
  that separately owned certificate must not be regenerated here. The `lathe`
  example currently contains only a plan, so there is no lathe assembly to
  export. See `BOM_BASELINE.md` for the measurements and outstanding gate.
- Add `MachineAssembly.bomDocument()` → the JSON above, at first with today's
  fields plus member paths and the subassembly tree.
- Add a single entrypoint `machinekit/scripts/bom-export` that builds a named
  example machine and writes `build/bom/<machine>.json`.
- Dump every example machine's BOM, then record line counts and the
  problems above for each one as the baseline (`BOM_BASELINE.md`).
- Acceptance: bench mill, CNC router, lathe and robot welder export; the
  baseline lists each problem class with counts.

### B2 — Part-number rules and lint
- The initial survey found three built-in defaults that cannot be constructed.
  `ComponentType.keyFor` also rounds numeric inputs, so its key is not exact
  as required by this plan. Keep both findings visible in the rollout.
- Rules: `[A-Z0-9][A-Z0-9.\-x/]*`, ≤ 40 chars, no `null`, no unformatted
  floats, no `|`. Dimensions in names only through `Dimension.format`.
- Add a lint check over every registered recipe's default part and every
  exported example BOM. It starts as a report and becomes a hard failure in
  B7.
- `ComponentType` gets a readable `partNumber` per recipe (rails, screws,
  nuts, supports, couplings, belts, pulleys, motors, drives).
  `valuesKey` stays the exact key.
- `AssumedCatalogComponent` part numbers become readable: id plus the
  rating-defining inputs (e.g. `SERVO-ASSUMED-3.8NM-400V-19`), with the hash
  only as the revision.
- Acceptance: all recipe-derived parts pass the lint.

### B3 — Made parts: drawing number + revision
- Decision: use author-declared drawing numbers. A geometry-derived identity
  would change when a drawing changes and would detach existing quotes; the
  separate revision identifies that change instead.
- `MachineComponent` gets an explicit drawing number for made parts, and
  constructors stop deriving identity from dimensions.
- Revision = short hash of the recipe key, or of the exported B-rep for
  parts without a recipe.
- Fix the bench mill's brackets, plates and nut adapters first, then the
  rest of the 104 interpolating constructors as part of B7.
- Acceptance: changing a bench-mill bracket dimension keeps its part number
  and changes its revision (a test).

### B4 — Procurement kind
- Add `procurement` to `BomItem`. It is required. Components without it fail
  the export.
- Map existing data: `ManufacturingVariant` → `Make(process)`;
  standard parts (`standard/`, rails, bearings) → `Buy`;
  `AssumedCatalogComponent` subclasses, `GENERIC-*` parts and components
  whose `EngineeringMetadata.ratingsBasis` is `Assumed` → `Undecided`, with a
  requirement string built from the ratings the sizing code already checks
  against. A `Catalog` basis with a source is evidence for `Buy`.
- Workpiece, tooling and clamps → `NotSupplied`. `BenchMill`'s default
  workpiece flag stays for the preview; the procurement view excludes these
  lines.
- The material field is exported only for Make lines.

### B5 — Purchase units
- Apply `IncludedInPurchase` to `ShaftBrakePart` (hub carries the brake line,
  disc and sectors are included) and any other split parts B1 finds.
- Extend the check if B1 shows a need: every included piece's owner present
  in the same assembly, not just somewhere in the BOM.
- Acceptance: bench mill's 26 brake lines become 1.

### B6 — Fasteners and other missing content
- Survey: rail and motor classes expose some hole patterns, and
  `SupportedSfu20Axis` explicitly installs screws. `Fastening` describes a
  connection derived from an already present screw; generic assembly mates do
  not declare fastener size, length, engagement or the corresponding hole
  pattern. Deriving purchases for every bolted connection therefore needs a
  broader assembly-interface redesign. Declare the bench-mill hardware from
  its actual patterns in this phase; retain derivation as the long-term goal.
- Bench-mill survey: the three axis motor plates currently place their four
  holes at `bodyDiameter` pitch, while the assumed 400 W and 750 W servo
  flanges use 50 mm and 70 mm pitch for 60 mm and 80 mm bodies respectively.
  Their hole patterns therefore do not match. Correct the plates to use the
  stated flange pattern before counting their fasteners. The spindle motor
  has no declared flange pattern, so its shelf holes alone do not establish
  a verified mounting fastener.
- Survey how screws are represented today (`standard/SocketHeadCapScrew`,
  `HexBolt`, screwdriving `Fastening`, the supplier-fasteners examples).
  Then decide whether bolted connections derive their fasteners or
  assemblies declare them. Derived is the preferred end state if mounting
  ports already know their bolt patterns.
- Add declared lines for cable, chain, lubrication and enclosure on the
  reference machines through `addBomItem`.
- Acceptance: bench mill fasteners in the BOM, matched to the motor-plate
  and rail bolt patterns.

### B7 — Rollout and hard gate
- Bring every example machine through B2–B6.
- Lint and procurement checks become hard failures in the machinekit
  tests. Example BOM exports are added to the test suite.

### B8 — Sourcing contract
- Decision: move `SupplierOffer` and `SourcingPlan` fully into the sourcing
  layer. Their supplier selection, prices, availability and review status are
  sourcing data; a supplier SKU does not define the engineering part. Keep
  catalogue manufacturer/MPN evidence in the engineering BOM when known.
- Freeze the version 1 JSON and document it next to the exporter.
- Decide whether `SupplierOffer`/`SourcingPlan` stay in machinekit as a
  thin "this part is this catalogue SKU" link, or move fully into the
  sourcing layer.
- The sourcing layer (separate plan, Python under `tools/suppliers/` next to
  `lcsccable.py`) reads `build/bom/*.json` and reports lines with no quote and
  quotes that are stale or orphaned.

## Open decisions

1. Made-part numbers: author-declared drawing numbers (proposed) versus
   descriptive names built from dimensions.
2. Fasteners: derived from bolted connections versus declared per assembly
   (B6 survey decides).
3. `SourcingPlan`: keep a thin SKU link in machinekit, or move it out (B8).
