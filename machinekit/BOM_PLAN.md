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
  the recipe key and its generator version.
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
- `NotSupplied`: workpieces, cutting tools and fixtures, which are listed
  but excluded from the machine's procurement BOM.

Each line also has an engineering maturity. Assumed lines carry structured
requirements for selection, such as peak torque, bus voltage and shaft size.

**Purchase units use `IncludedInPurchase`.** One component of a bought unit
carries the purchase line; every other piece (modelling splits such as the
brake sectors, or real subparts that ship inside the unit) carries
`IncludedInPurchase(owner)`. The engineering tree retains each piece; the
procurement view keeps only the owner line. An included piece whose owner is
absent is an error. No second mechanism. Kits sold as a set (an SFU screw with nut and BK/BF supports)
are a sourcing concern: one quote covers several lines. They stay separate
in the BOM.

**One export, one schema version.** The `machinekit.bom` JSON has version 1,
and readers reject anything else. It contains:

- machine id, source commit, and generation time;
- the subassembly tree;
- the rolled-up procurement lines (kind, maturity, structured requirements,
  part number, fingerprint and release, description, quantity, and material
  and process for Make);
- where each line is used (member paths).

## Phases

### B1 — Export and baseline
- Done in `d8e2de07f`: the exporter and baseline cover 22 constructed examples.
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
- Done in `4f87b4bb0`: 212 constructible recipe defaults and all recipe-derived
  numbers in 22 machine exports pass lint; three defaults require source inputs.
  The full gate passed 11/24 shards with no new shard or exception message.
  The measurements and remaining non-recipe invalid counts are in
  `BOM_BASELINE.md`.
- Decision: the current enclosure recipes identify panel construction, blank
  dimensions and any opening in the number. This keeps different cut panels
  distinct while the recipes lack an author-declared drawing identifier; B3
  must supply that identifier for released made parts.
- The generic robot housing, pedestal, end-effector plate and wrist fork
  recipes likewise still number their dimensions. They need author-declared
  drawing identifiers in B3 before their numbers can survive design edits.
- Generic motor plates and carriages also need author-declared drawing IDs in
  B3. Their B2 numbers describe interfaces and dimensions, and the full recipe
  key detects differing inputs within a bill.
- Tooling panel and volume numbers use their existing plan IDs in B2. Those
  IDs are scoped to a plan, so B3 must establish globally distinct drawing
  identities before release records can safely refer to them.
- The required-input casting, insert and cassette recipes retain their
  required source data. Their representative instances are checked without
  inventing defaults; their current source labels also need drawing-identity
  review in B3.
- Cast inserts accept a recipe-level part ID so a foundation can carry its
  existing member IDs into numbering. The hobber qualifies those IDs by bed or
  column because both foundations use names such as `railLeft`.
- The drawing-section member's B2 number states only its bounds and cut
  length; different contours can share those bounds. Its exact recipe key
  prevents combining them in one bill, but B3 must add an author-declared
  section identity before bills from different machines can be compared.
- Home-switch brackets now carry their axis ID in their label, and linear
  mount receiver numbers include stock dimensions. Without those distinctions,
  different parts in the same machine collide during procurement roll-up.
- Fabricated rack tubes and their host pads carry their source member IDs in
  the recipe because equal-sized parts can have different drilling patterns.
  Other cut tubes use stock dimensions; B3 must establish globally distinct
  drawing identities for fabricated members.
- Stepped-shaft numbers describe section stock and feature counts until B3
  supplies author-declared drawing IDs. Different features can share those
  numbers; their exact recipe keys prevent merging them in a bill. The
  sheet-router service table has its own provisional drawing name.
- Power-supply numbers omit the trailing regenerative-power unit to keep the
  sheet-router rating within 40 characters; recipe values retain the units.
- Decision: ZPP engineering member numbers identify the supplier model and
  modeled role, without embedding a CAD file hash. `AR155_I` is written as
  `AR155-I` in the internal number to meet the character rule; the exact
  supplier model belongs in the later manufacturer part-number field.
- Decision: retain the generic lead-screw nut's selected material in its
  identity when it differs from bronze. The sheet router uses a steel ball
  nut, and the material is a purchase distinction, not a drawing-only input.
- For recipes whose number does not already declare material, an edited
  material adds the material library ID to the number. This preserves distinct
  purchase identities for the material variants supported by the recipe API;
  recipes with a material-specific number declare that fact themselves.
- Decision: gearbox numbers also distinguish a selected material from the
  recipe's aluminium default. Cobot joints use a steel mass allocation for
  some gearheads; the number must not silently identify it as aluminium.
- Decision: where a mill recipe already declares a fixed design name, use
  that name for its recipe number. The exact recipe key still distinguishes
  changed geometry; B3 adds the corresponding fingerprint and release state.
- Decision: the end mill model gives stickout but no overall tool length.
  Keep stickout in its assumed requirement identity until a specified tool
  length can establish that different installed reaches use the same purchase.
- Multiple block profile recipes identify the same rail stock but retain
  different full recipe keys. B2 includes the block profile in rail
  their internal numbers to satisfy the existing recipe identity contract.
  B3 must separate the purchased rail specification from the block profile
  so equal rail stock can roll up without losing exact recipe inputs.
- Decision: a routed belt's purchase key contains its profile, derived tooth
  count, width and stated belt properties, while its geometry key retains the
  pulley layout. Different layouts can therefore order the same belt without
  losing the exact inputs needed to cache its model.
- Replication layers and nut cassettes exclude source-only hole and connector
  labels from their purchase keys, while preserving exact physical dimensions
  and station coordinates; their complete recipe keys retain the source data.
  Rack receivers currently retain separate section IDs because their derived
  local coordinates contain floating-point differences. B3 must establish
  stable drawing identities and resolve repeated receiver definitions without
  rounding away real geometry changes.
  A cleat's transverse height uses the first rack section frame: subsequent
  sections translate along the receiver axis and cannot change that height.
  This avoids introducing floating-point noise from unrelated translation.
- Decision: decimal lint checks dimension-like fields rather than every
  dotted numeric segment. Catalogue identifiers such as `10.08.02.00203`
  are not decimal dimensions; authors still format dimension inputs with
  `Dimension.format` when defining each number.
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
- Done in `ad91da408`: bench-mill drawing identities and fingerprint/release
  records are tested; 22 example exports contain 1,143 procurement lines,
  including 399 code-only lines with unavailable fingerprints. The full gate
  passed 11/24 shards with no new shard or exception message.
- A made part declares its drawing number separately from its scene
  designation. Scene artifacts use the designation for geometry and display;
  BOM identity uses the drawing number. This keeps procurement identity
  stable without changing the existing tending-study scene inputs.
- The X- and Y-axis motor plates have separate drawing numbers even though
  their current dimensions match: their axis-specific designs may change
  independently. Their fingerprints record the actual current dimensions.
- Code-only components without declared revision inputs report a null
  fingerprint rather than a hash that falsely claims to cover their geometry.
  The bench-mill brackets, plates and nut adapters declare their exact inputs;
  B7 must finish declarations for the remaining code-only parts.
- Release records are ordered per part in each machine's version-one file;
  the last record is the latest issue. Dates must be chronological and a
  part/revision pair may appear only once. Missing release files mean no
  released revision; stale fingerprints remain visible as `releaseCurrent: false`.
- A recipe fingerprint hashes its exact purchase key and generator version.
  The full geometry cache key still retains placement and routing inputs. This
  keeps the fingerprint of a purchased belt or rail independent of how it is
  placed, while preserving every input that defines the purchased part.
- Decision: use author-declared drawing numbers. A geometry-derived identity
  would change when a drawing changes and would detach existing quotes; the
  separate revision identifies that change instead.
- `MachineComponent` gets an explicit drawing number for made parts, and
  constructors stop deriving identity from dimensions.
- Fingerprint the declared purchase inputs and generator version; released
  revision letters are explicit records, as specified in A4.
- Fix the bench mill's brackets, plates and nut adapters first, then the
  rest of the 104 interpolating constructors as part of B7.
- Acceptance: changing a bench-mill bracket dimension keeps its part number
  and changes its revision (a test).

### B4 — Procurement kind
- Done in `dfdbe0edf`. The 22 exportable examples contain 1,076 procurement lines: 516 Make and
  560 Buy. Their engineering trees retain NotSupplied lines. The export
  requires intent and maturity on every line, and structured selection terms
  on every Assumed line. The gate passed 11/24 shards, with no new failing
  shard or exception message against the untouched baseline.
- Procurement intent and structured numeric requirements are declared on
  recipes, or on a `ProcurementIntent` facet for code-only components.
  Selection maturity remains separate and comes from existing engineering
  metadata; any temporary migration mapping must be removed in B7.
- The bench mill's ER20 holder, cutter, workpiece and workholding clamps
  are user tooling, so their engineering occurrences are `NotSupplied`.
  Router plates are used for both machine structure and stock: the constructor
  marks stock explicitly, while other plates declare a fabrication process.
- Cabinet I/O counts describe connected equipment, not the cabinet purchase;
  its selection requirements cover its dimensions and DIN rows. Named stepper
  motors with sourced ratings use their catalogue model; the pancake motor's
  assumed ratings and the OpenBuilds geometry reference remain `Assumed`.
  A geometry reference alone does not identify a manufacturer part. Generic
  motors carry their frame and body length as selection requirements.
- Digital sensors declare the function currently modeled. In particular, the
  scanner model does not state a safety rating, so that information still has
  to be established before selecting a safety scanner.
- The router's 52 mm ER11 spindle model states neither power nor speed; its
  Buy requirements retain the known interface and leave those ratings open.
- The mobile base's modeled drive wheel and caster state their dimensions but
  not a load rating; wheel selection still needs that engineering input.
- The generic inline vacuum sensor states its connection and signal type but
  not its measurement range; selection still needs that range.
- The press-brake light curtain has no safety rating, and its hydraulic
  cylinder, valve, manifold and hose models do not state pressure ratings.
  Those values are still needed for selection. B5 includes the modeled cylinder
  rods in their cylinder purchases.
- The picking indicator lacks an electrical interface specification, and the
  adjustable foot lacks a load rating. Both remain assumed purchases requiring
  those inputs before supplier selection.
- `CobotJoint` describes an integrated motor and gearhead, while `CobotArm`
  also models each gearhead as a member. B5 includes each gearhead with its
  joint purchase.
- Pneumatic cylinder rods are separate model members included in the cylinder
  purchase in B5. The modeled valve
  has no coil-voltage rating, which remains needed for selecting a real valve.
- The welding torch, neck and nozzle are modeled separately for collision;
  B5 includes the neck and initial nozzle with the torch purchase. The
  magnetic return clamp follows the plan's NotSupplied treatment of clamps.
  The torch model does not state a current rating, so selection still needs
  that rating from the welding design.
- The screwdriver's powered body, feed slide and rotating nose are separate
  model members. B5 treats them as one purchased tool unit;
  its replaceable bit and workholding fixture are user-supplied.
- A manufacturing family's selected variant supplies its Make process. Cast
  inserts use their declared fabricated or catalogue origin to choose Make or
  Buy, retaining the catalogue manufacturer and reference when supplied. A
  nominal-study exterior remains Assumed even for a named catalogue product.
  Deposited weld metal and the modeled welding stock belong to the workpiece,
  so they are not purchases for the machine; the work table remains a purchased
  cell item.
- The purchased rotary table's maturity follows its torque, speed and load
  rating bases; its provisional mounting envelope is not promoted to catalogue
  evidence. Fine bearing runout is expressed in micrometres and ring-encoder
  angular accuracy in arcseconds to retain meaningful formatted values.
- Existing welding service lines explicitly declare Buy intent; B6 will express
  stock sold by length with cut lists and keep complete hose packs as units.
  Stock hose keeps its catalogue maker and reference, while custom hose states
  its tube and service requirements.
- Add `procurement` to `BomItem`. It is required. Components without it fail
  the export.
- Declare `Make`, `Buy` or `NotSupplied` at each recipe or code-only component.
  Assumed maturity is independent of that choice and requires structured
  selection requirements. A catalogue basis needs a source; a geometry
  reference alone does not establish a purchasable manufacturer part.
- Workpiece, tooling and clamps → `NotSupplied`. `BenchMill`'s default
  workpiece flag stays for the preview; the procurement view excludes these
  lines.
- The material field is exported only for Make lines.

### B5 — Purchase units
- Done in this commit. The 22 exportable examples have 1,004 procurement
  lines; the bench mill has 46, including one brake purchase. The full gate
  passed 11/24 shards with no new failing shard or exception message.
- A shaft brake's hub owns the purchase; its disc and convex collision sectors
  remain visible as included engineering pieces. The purchase number names the
  brake and bore rather than a sector.
- A cylinder includes its modeled rod, a cobot joint includes its modeled
  gearhead, and the initial torch neck and nozzle ship with the torch. The
  screwdriver's powered body includes its feed slide and rotating nose; its
  purchase requirements also state the feed stroke and spring rate. The bit
  remains user-supplied.
- A purchase owner must occur in the same assembly as each included piece.
  Assembly snapshots retain explicitly assigned purchase ownership so recipe
  reconstruction cannot turn included geometry back into a purchase.
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
