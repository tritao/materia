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

**Purchase units.** A component can declare that it is piece `k` of `n` of
a purchase unit (`purchaseUnit: {partNumber, pieces}`). The BOM rolls the
pieces up into whole units and rejects incomplete sets. Modelling splits,
such as the brake sectors and any future multi-hull part, then never reach
procurement. Kits sold as a set (an SFU screw with nut and BK/BF supports)
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
- Add `MachineAssembly.bomDocument()` → the JSON above, at first with today's
  fields plus member paths and the subassembly tree.
- Add a single entrypoint `machinekit/scripts/bom-export` that builds a named
  example machine and writes `build/bom/<machine>.json`.
- Dump every example machine's BOM, then record line counts and the
  problems above for each one as the baseline (`BOM_BASELINE.md`).
- Acceptance: bench mill, CNC router, lathe and robot welder export; the
  baseline lists each problem class with counts.

### B2 — Part-number rules and lint
- Rules: `[A-Z0-9][A-Z0-9.\-x/]*`, ≤ 40 chars, no `null`, no unformatted
  floats, no `|`. Dimensions in names only through `Dimension.format`.
- Add a lint check over every registered recipe's default part and every
  exported example BOM. It starts as a report and becomes a hard failure in
  B7.
- `ComponentType` gets a readable `partNumber` per recipe (rails, screws,
  nuts, supports, couplings, belts, pulleys, motors, drives).
  `valuesKey` stays the exact key.
- Acceptance: all recipe-derived parts pass the lint.

### B3 — Made parts: drawing number + revision
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
  `GENERIC-*` and "assumed" → `Undecided`, with a requirement string built
  from the ratings the sizing code already checks against.
- Workpiece, tooling and clamps → `NotSupplied`. `BenchMill`'s default
  workpiece flag stays for the preview; the procurement view excludes these
  lines.
- The material field is exported only for Make lines.

### B5 — Purchase units
- Add `purchaseUnit` to components, with rollup and completeness checks in
  `Bom`.
- Apply it to `ShaftBrakePart` and any other split parts B1 finds.
- Acceptance: bench mill's 26 brake lines become 1.

### B6 — Fasteners and other missing content
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
- Freeze the version 1 JSON and document it next to the exporter.
- Decide whether `SupplierOffer`/`SourcingPlan` stay in machinekit as a
  thin "this part is this catalogue SKU" link, or move fully into the
  sourcing layer.
- The sourcing layer (separate plan) reads `build/bom/*.json` and reports
  lines with no quote and quotes that are stale or orphaned.

## Open decisions

1. Made-part numbers: author-declared drawing numbers (proposed) versus
   descriptive names built from dimensions.
2. Fasteners: derived from bolted connections versus declared per assembly
   (B6 survey decides).
3. `SourcingPlan`: keep a thin SKU link in machinekit, or move it out (B8).
