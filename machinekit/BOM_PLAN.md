# BOM plan

Goal: every machine produces a bill of materials that someone could buy and
build from. Each line is a real purchasable or manufacturable unit with a
stable, readable part number. Each line says whether it is made, bought or
not supplied, and records selection maturity separately. The BOM leaves the code as a versioned JSON file that a
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
- The B2 drawing-section number used only bounds and cut length. The declared
  extrusion-section identity follow-up below replaces it with an author ID;
  canonical geometry remains in the fingerprint.
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
- Done in `f92e86ef4`. The 22 exportable examples have 1,004 procurement
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
- Done in `1163f9c94`. The 22 exportable machines have 1,022 procurement
  lines, including 55 for the bench mill. The gate passed 11/24 shards with no
  new failing shard; the existing tending-certificate failures in shards 16
  and 23 differ only in the disclosed current input fingerprint.
- The reference mill declares one ISO 4762 M4x16 screw for each HGR15 rail
  mounting position. Its two 400 W motor flanges use four M5x12 screws each;
  the 750 W flange uses four M6x12 screws. Six millimetres of each screw engage
  the eight-millimetre motor plate. The plate's hole centres come from the
  stated flange, and its centre bore clears the flange pilot. Tapped holes are
  represented by their nominal major-diameter envelopes; the fabrication
  drawing still needs thread and tolerance callouts.
- The corrected motor plates change the tending reach study's current input
  fingerprint from `4b793a015e84eb25ab7624a49f30b28fd06322ddfb97905fd50d40a37c18d1f1`
  to `23389d61f8286d00696fcd4eb01ee0b1c29942346ab2e0392a6ced2fb62a27c2`.
  The existing certificate was already stale; it has not been regenerated.
- The bare mill has no enclosure; the enclosed mill already lists its panels.
  The bare mill declares three assumed cable chains, flexible power and encoder
  cable, a manual pump/distribution set and an initial litre of ISO VG 68 way
  oil. Cable cuts use Manhattan distance between modeled endpoints, the sum
  of axis travels and 500 mm allowance, rounded up to 100 mm. These are
  conservative planning lengths, not a verified route. Cable conductor size,
  encoder pinout, chain mounting, lubrication tubing and fitting sizes still
  require design before ordering.
- Fastener survey: rail hole positions and the catalogue screw size and length
  can establish rail screw count and candidate screws. Motor flange hole
  coordinates, clearances and flange thickness establish the motor pattern and
  grip. Bolted connectors can locate a potential joint. `SupportedSfu20Axis`
  already installs screws, while screwdriving `Fastening` describes a
  connection from a screw that already exists. Generic mates do not identify
  which connectors form a bolted pattern, the receiving thread, grip stack,
  required engagement, head clearance, washers or selected screw standard and
  property class. They therefore cannot determine a complete purchase safely.
  B6 declares rail and motor screws from their modeled patterns, rather than
  inferring a fastener from every fixed mate.
- The rail catalogue now stores mounting screw size and length separately;
  its HIWIN source generator emits the same structured values. The generator's
  existing byte-for-byte `--check` was already failing before this change
  because its unformatted output differs from the formatted checked-in file.
- The previous axis motor plates placed their four holes at `bodyDiameter`
  pitch, while the assumed 400 W and 750 W flanges use 50 mm and 70 mm pitch.
  The plates now use the stated flange patterns. The spindle motor
  has no declared flange pattern, so its shelf holes alone do not establish
  a verified mounting fastener.
- Follow-up phase F1, after this BOM plan: extend bolted joints with explicit
  pattern correspondence, receiving threads, grip and engagement, head and
  washer clearances, and fastener standard/property class; validate those
  against the connected components and derive purchase quantities and lengths
  from the validated joints. Replace the B6 declarations only when that
  derivation covers the mill's rail and motor connections, with tests for
  missing data, incompatible patterns and duplicate counting.
- Survey how screws are represented today (`standard/SocketHeadCapScrew`,
  `HexBolt`, screwdriving `Fastening`, the supplier-fasteners examples).
  Then decide whether bolted connections derive their fasteners or
  assemblies declare them. Derived is the preferred end state if mounting
  ports already know their bolt patterns.
- Add declared lines for cable, chain, lubrication and enclosure on the
  reference machines through `addBomItem`.
- Acceptance: bench mill fasteners in the BOM, matched to the motor-plate
  and rail bolt patterns.

### F1 — Joint-derived fasteners (2026-11-04)

Status: first slice done (local main 15c70f491): the bench mill's rail and motor screws are derived and the B6
declarations are deleted. Second slice below (items 1-3 of the old open list, and the nut flange adopters).

Vocabulary (one path, no hand-declared fallback):

- `component/BoltedFace` is the port a part declares through
  `MachineComponent.fasteningFaces()`. A face frame lies in the part's frame
  with its origin on the mating plane and +Z into the part's own material.
  A **clamped** face lists its clearance holes, the clearance diameter, the
  grip (material from bearing seat to mating plane), an optional named size
  and an optional head recess. A **receiving** face either lists the holes
  already tapped (size, thread depth, thickness, through or blind, material)
  or is **drilled to suit**: a footprint plus the deepest thread the material
  takes, optionally a `thicknessAt(x, y)` where a bore thins the wall.
- `assembly/JointFasteners.derive` (also `MachineAssembly.deriveFasteners()`,
  which adds the BOM lines once) pairs every clamped face with the one
  receiving face it meets (planes coincide, normals opposed, hole extents
  overlap, members held together by fixed mates) and validates the pair:
  pattern correspondence in the receiving frame, size agreement, clearance
  against the catalogue screw, head against the recess, the shortest ISO 4762
  length (`SocketHeadCapScrew.standardLengths`) whose engagement meets
  1.0/1.5/2.0 diameters (steel/cast iron/aluminium) beyond the grip, thread
  depth with a one-pitch tip gap in a blind hole, unthreaded shank, edge
  distance and hole spacing in a footprint, and back wall under the tap-drill
  depth. Findings carry codes `fasteners.*`; duplicate counting is refused by
  claiming each tapped hole once and by hole spacing in drilled-to-suit seats.
- The result is a `FastenedJoint` list: fastener (standard, size, length,
  property class, count), grip and engagement, and for a drilled-to-suit host
  its hole positions, tap drill, thread depth and drilling depth.

Adopters: `LinearRail` (counterbored bottom face, catalogue screw),
`ServoMotor` (drilled flange), `MillCasting` (rail pads drilled to suit, the
saddle with its nut-bore wall), bench-mill `MillMotorPlate` (tapped through).
Acceptance: the bench mill derives 48 M4x16 (six rails), 8 M5x12 and 4 M6x12,
the same lines B6 declared, with the cast-iron engagement and the saddle's thin
wall over the nut bore now checked rather than assumed. The saddle check is
real: with a uniform 10 mm wall the M4x16 rail screws fail `fasteners.back-wall`.

Second slice (2026-11-04, branch `joint-fasteners-2`):

- **Derived holes are geometry.** `deriveFasteners()` hands the derivation's `machining` (tap-drill holes of every
  host drilled to suit; clearance holes and counterbores of a clamped face whose part does not draw them,
  `BoltedFace.drawn` false; clearance holes of an undrawn `through` plate) to the parts with
  `MachineComponent.machine(cuts)`. A part that cuts them opts in with `machinable()` and ends `geometry()` with
  `machined(...)`; deriving a joint that needs a hole in a part that cannot cut it is `fasteners.machining`. The
  derivation is computed once from the solved joint (no second hole list); the cuts are in the part's frame, so no
  cycle with poses: poses come from connectors, geometry follows the derivation. The geometry key and the BOM revision
  include the cuts (a drilled casting is another revision), so caches cannot serve the undrilled solid. `MillCasting`
  is machinable: the bench mill's 48 rail holes are in the casting solids and `JointFastenerTests` checks the volume
  removed. Preview speed: one extra boolean per casting build, kept by `PosedParts` per geometry key.
- **Head, washer, nut and driver clearance.** Each `FastenedJoint` carries `volumes` (receiving face frame, +Z into the
  receiving part): the hole through the clamped part and the thread/plate hole, the washer, the head, the nut and
  washer of a bolt, and a driver access cylinder of `FastenerOptions.accessLength` (default 25 mm, a hex key's short
  arm). `AssemblyIntegrity.fasteners(assembly)` (also option `fasteners`, on in `strict`) poses them against the
  members: `fasteners.hole` (material in a hole the screw passes), `fasteners.head-clearance` (head, washer or nut meets
  a member, including the clamped part's own body: a recess the geometry does not draw), `fasteners.access`
  (only members rigidly fixed with the clamped part count: a moving member can be moved away).
- **Washers and nuts.** `BoltedFace.clamped(..., washer)` asks for an ISO 7089 washer under the head (it lifts the head:
  the recess must still hold it); `BoltedFace.through` is a plate with clearance holes, and the derivation selects the
  ISO 7089 washer and ISO 4032 nut, takes the length from the stack (grip + washers + plate + nut + two pitches) and adds
  the washer and nut to the BOM lines.
- **Adopters added.** `LeadScrewNut` flange (two alternative clamped faces, `BoltedFace.alternative`: barrel side and
  back; an unmatched face is only an error when none of its alternatives is bolted on), `NutBlock` (tapped) and the bench
  mill's `MillNutBracket` (a `through` plate): the mill now derives 18 M5x30 flange bolts with 18 washers and 18 nuts (six
  per axis) besides the 48 + 12 + 8 screws.

Third slice (2026-11-04, branch `joint-fasteners-3`):

- **Motor corners fixed in the model, findings unpinned.** The servo body has corner reliefs behind the flange (`MotorBody`
  `relief`, a cylinder per bolt axis over the whole body, the screw head the flange's hole takes plus 1 mm for the driver,
  `ServoMotor.reliefDiameter`; the maker data gives no relief dimension, so the 1 mm is assumed). The X motor bridge ends
  against the plate's inner face instead of standing 12 mm behind it, where the lowest flange screws' heads are. The mill's
  fastener fit is 0 findings and `JointFastenerTests` expects 0.
- **Clamped faces drilled to suit.** `BoltedFace.clampedToSuit` is a clamped footprint with no listed holes: it pairs with every
  tapped receiving face whose holes are centred inside it and takes that face's pattern (`paired`), so one seat serves several
  rail blocks. Holes keep an edge distance from the footprint (`fasteners.edge`); two faces drilled to suit are
  `fasteners.suit`. Its holes and recess are never drawn: the derivation cuts them (the part is `machinable()`).
- **Recess as the driver's way in.** A declared recess is a pocket: the driver access volume runs the whole pocket to its
  mouth and then `accessLength` beyond, so a rail, workpiece or casting wall across the mouth is `fasteners.access`.
  `IntegrityOptions.removable` lists members the operator takes away to reach a screw (the mill's workpiece, clamps and vise,
  `BenchMill.workholdingMembers()`).
- **Rail blocks.** `LinearRailBlock` declares its top as a tapped receiving face (the catalogue's thread and depth, none where
  the table gives only the thread). The bench mill's saddle, table and head castings (`MillCasting.blockSeats`) are clamped
  footprints drilled to suit, one per guide rail: clearance holes and a counterbored pocket from the casting's far face
  leaving `BLOCK_SOLE` = 12 mm under the heads (one nominal diameter of engagement in the 5 mm thread plus the standard
  length), pocket diameter head + 2 mm (assumed). Result: 12 blocks, 48 M4x16, all engage. The checks forced one model
  change: the saddle's four Y blocks stand under its X rails (half span 45 mm, not 32.7) so their pockets fall either side of
  the rails; with the old span the outer pockets opened under the X rails (`fasteners.access`).
- **Screw support feet.** `ScrewSupportUnit.fasteningFaces` declares the foot: the maker's hole diameter, grip = the bolt
  seat (`boltSeatY`, measured on the maker's CAD where reviewed, else the row's assumed value), the row's counterbore, its
  depth assumed as deep as the head (flush; not published), the screw the hole passes (`SocketHeadCapScrew.takenByHole`).
  The bench mill's Y stands and X bridges are `MillDriveBox` bodies with a drilled-to-suit top face (tapped through when 12 mm
  or thinner); the Y stands grew to hold 8 mm of edge round the hole pattern. The mill derives 12 more M5 screws (BK12 4 each
  x 2, BF12 2 each x 2; M5x16 and M5x40 from the BF12's reviewed bolt seat).

Acceptance now: the bench mill derives 138 screws (48 rail, 48 block, 12 support, 12 motor, 18 nut flange) plus 18 washers
and 18 nuts, fits the mill's solids with 0 findings, and every number above is checked by `JointFastenerTests`.

Not done yet (open, in order):

1. **One path (old item 3).** Fold `MillRailFasteners` and `MountServiceScrews` into the derivation. They instance real screw
   members with `head` mates and `Coaxial` requirements from the mineral frame's `DesignInterfaces` holes, with provenance
   fingerprints, and select the length by `DesignChecks.blindFastener` (usable thread depth, tip gap, back wall, relief) rather
   than the derivation's engagement rule. The mineral insert holes must become receiving faces carrying that thread policy
   (`ThreadSpec` usable/cylindrical depth, point, stock, minimum engagement, tip gap, back wall) and the derivation must
   instance `SocketHeadCapScrew` members from a `FastenedJoint` (member per hole: head connector mate, coaxial requirement on
   the rail connector and the insert hole, provenance), then `RailSeatTests`, `LinearMountContactTests`,
   `LinearMountAssemblyTests`, `LinearMountTests` and `LinearRackMount` (sheet router) move to it and both old classes go.
   Member count: instancing is only needed where an insert carries provenance (the mineral frames, 36 screws in the small
   frame); the bench mill's 138 screws stay BOM lines only, because as members they would add 138 solids to every preview
   with nothing to mate them to.
2. **Z screw supports (done).** They stand on steel bosses cast into the column's side wall (`MillCasting.ColumnBoss`, `BenchMill.zBosses`):
   a 16 mm tapped block in a pocket, its face 1 mm proud at the supports' foot plane, so the screw axis did not move; the six foot
   screws are derived (the mill: 144 screws) and the bosses need none of their own. The support's foot is 14 mm in front of the column
   face, so the front row of holes is over the boss's forward part, beside the head.
3. **Motor mounts and stands to castings.** `MotorMount` (HZMotion/BK/BF-style motor seats) still declares only `tappedFoot`; as a
   `BoltedFace` it needs the foot grip and bolt recess from its drawing (`boltSeatY` exists only where measured) and its
   receiving face (the y-servo variant). The Y stands and X bridges themselves are not bolted to the base and saddle yet
   (their lower faces have no clamped face; the base casting's `screw-support-pad` is a 4 mm pad with no holes).
4. Hobber adoption (another worktree owns those files): Y nut seat becomes a receiving face drilled to suit on the housing
   (tapped to the nut flange's clamped pattern, its hole positions then come out of the derivation); X carriage and Z saddle
   fixings to the rail blocks and column are clamped faces on the carriage/saddle meeting the block's top, which now declares
   its receiving face (use `BoltedFace.clampedToSuit` on the carriage and `recess` for a counterbored pocket, or `clamped`
   with listed holes) and the column (drilled to suit); scale heads get a clamped face from the head recess and a receiving
   face on the bracket; the swivel plate's M5 heads are the recess case, a clamped face with `recess` from the counterbore so a
   head standing proud of the face fails `fasteners.head-clearance` rather than escaping the checks. A recess is also the
   driver's way in: it is checked clear to its mouth. The hobber's parts that are drilled to suit must be `machinable()` and end
   `geometry()` with `machined(...)`. Replace each hand-placed screw by the derived list and delete the hand placement in the
   same change.

Fourth slice (2026-11-05, branch `hobber-bolts`): hobber adoption, item 4 above, mostly done.

- The hobber calls `deriveFasteners()` at the end of its constructor: 36 joints, 150 screws (M4x10 32, M4x16 24, M5x12 12, M5x16 18, M5x40 2, M6x16 20,
  M6x20 16, M6x30 22, M6x40 4), before: none. Strict integrity sweep 0 findings, `tests/hobber` 2138, `tests/hobbing` 1044, joint-fasteners and the
  machined package pass (its hole tables now carry the derived holes: `HobberSaddle.derivedHoles`).
- Library: `CastInsert.fasteningFaces` (tapped receiving faces from the insert's own threaded holes, one per entry plane and thread size: the
  rails' and the pads' screws check against the pattern `InsertMountPattern` already cut; it does not cut a second set), `MotorMount` (`motor` tapped,
  `foot` clamped where the CAD measured the bolt seat), `NutBlock.base` (tapped M6), `ScrewSupportUnit.screwSize` (a machine may bolt a foot with
  a smaller screw than the hole passes), `MachinedCut.kind/thread` (hole tables). `JointFasteners`: among several receiving faces in a plane the one the
  pattern lands on pairs; spacing is checked between joints only (a catalogue pattern, the BK12 foot's M6 holes 13 mm apart, is the part's own).
- Hobber: carriage, Z saddle and Y saddle are `clampedToSuit` on the guide blocks (counterbored from the far face); nut blocks bolt to the carriage and Z
  saddle (`withBlockBolts` deleted); pads and Y brackets are drilled to suit under the BK12/BF12 feet (only the dowels stay hand-cut); the swivel plate
  takes the Y rails; the spindle seat and the Y saddle's plate carry the four M8 (seat holes moved to z -38/46 for the 1.5 d edge rule). Findings that changed
  the design: the Y floating bracket is 59.2 mm long (-114.6..-55.4) and the BF12 there takes M5; the coolant port moved to y 59..69.5 off the Y block's driver access.
  The X carriage's rear block screws and its nut-block bolts are under the column: the integrity run lists the column as `removable` (screws fitted before the column is lowered).
- Still hand-cut or not derived (library or design gaps): dowels (no dowel concept in `BoltedFace`), the Y nut seat (its block bolts would enter from under the
  bar where the Y saddle edge and the swivel plate stand: needs a redesign, e.g. bolts from the block side), pads to inserts (no bolt positions on the pads),
  Y brackets to the swivel plate, swivel plate / adapter ring / eRob output / Z saddle to eRob (the eRob declares no faces), scale housings and heads,
  the cradle (other session: the motor face is declared; the cradle floor's holes at y 215 are 7 mm from the seat's end, short of the 12 mm M8 edge rule).

### B7 — Rollout and hard gate
- Done in `fa1f1534f`. The full gate passed 11/24 shards with no new
  failure or exception compared with B6. The only differences from the
  untouched baseline remain the accepted tending input fingerprints in shards
  16 and 23.
- Measurement: the 22 exportable machines have 1,058 procurement lines,
  including 380 Assumed lines. All procurement lines and 3,798 engineering
  occurrences have fingerprints. Part-number lint reports no invalid
  engineering or procurement lines.
  Three recipes cannot be constructed from default values. Machine tending
  remains blocked by the existing stale reach certificate.
- Made, code-only parts receive an authored drawing number at construction.
  Assembly builders pass their design namespace and part role; they do not
  assign numbers by inspecting component types. Export rejects an undrawn
  made occurrence and reports its complete assembly path.
- Gantry-generated made plates use a short, author-declared machine design
  namespace and their authored member role as a drawing number. That identity
  does not change when dimensions move and does not collide across different
  reference machines. Their fingerprints include exact plate, bore, motor
  interface and section inputs as applicable. Reusing a drawing across roles
  requires an explicitly shared drawing rather than a geometry-derived number.
- Cobot cover, gasket, tool-housing and wrist-rotor drawings similarly use
  the authored reach-class namespace and member role. Their fingerprints use
  exact dimensions and the relevant joint or flange fingerprints.
- Pinion-carrier purchase numbers identify the machine design and axis. X and
  Y carriers can have different mounting inputs even when their motor faces
  and visible axial dimensions match; the two equivalent Y carriers share
  a number. Their fingerprints retain the complete recipe and motor inputs.
- The sheet-router weldment receives its exact design inputs and generator
  version from its builder. Individual frame members fingerprint their section,
  length and end cuts without treating installed position as a part revision.
- Welding service leads are individual assumed purchases with typed lengths,
  conductor area or core count where modeled. Their fingerprints use those
  inputs; pressure ratings, terminations and other unspecified details remain
  selection work for sourcing.
- Bring every example machine through B2–B6.
- The CNC-router belt and folded-Z configurations also run through the BOM
  checks. Their axle and plates have explicit drawings and manufacturing
  declarations, so choosing either modeled drive does not leave bare parts.
- Lint, fingerprint presence and procurement checks become hard failures in
  the machinekit tests. Example BOM exports are added to the test suite.

### B8 — Sourcing contract
- Done in `6479b4072`. The full gate passed 11/24 shards, with no new
  failing shard or exception message compared with B7. Focused schema,
  sourcing, supplier-axis and BOM example checks passed.
- The existing Haxe offer helper moves to the separate `materia-sourcing`
  package under `tools/suppliers`. Sourcing reads the exported v1 document
  only, using procurement lines keyed by part number and fingerprint or
  released revision. RFQ and quotation workflows remain out of scope.
  Shared wire types live in the independent `machinekit.bomschema` namespace,
  alongside their owning exporter, so the schema has one canonical version.
  The sourcing package imports only those wire types; its project still
  depends on the MachineKit package that publishes them.
- `BomJson.parse` rejects non-v1 BOM envelopes. Contract tests pin the v1
  document, line, item, quantity, use, procurement and requirement fields.
  `machinekit/scripts/BOM_SCHEMA.md` defines the export and release records.
- The public bench-mill export retains the B7 engineering and procurement
  views, with 55 procurement lines and no invalid part numbers. Re-exporting
  all 22 machines measured 1,058 procurement lines, 380
  Assumed lines and 3,798 engineering occurrences; no line or occurrence
  had an invalid part number or lacked a fingerprint. Supplier-example
  recipes also gain missing purchase or manufacturing intent and maturity
  so their JSON exports can be sourced.
- Decision: move `SupplierOffer` and `SourcingPlan` fully into the sourcing
  layer. Their supplier selection, prices, availability and review status are
  sourcing data; a supplier SKU does not define the engineering part. Keep
  catalogue manufacturer/MPN evidence in the engineering BOM when known.
- Freeze the version 1 JSON and document it next to the exporter.
- The sourcing layer (separate plan, Python under `tools/suppliers/` next to
  `lcsccable.py`) reads `build/bom/*.json` and reports lines with no quote and
  quotes that are stale or orphaned.


## Declared extrusion-section identity follow-up

Done in this follow-up commit. The full gate passed 11/24 shards with no new
failing shard or exception message compared with B8.

Drawn sections require a BOM-lint-valid author-declared `section_id`, saved
with the validated section geometry. IDs are limited to 12 characters to
leave space in the 40-character purchase number for cut length and material.
Different designs need distinct IDs; revisions of one design retain its ID
and change its fingerprint. Drawing
and measured T-slot purchase numbers use the ID and cut length. Canonical
section geometry remains in the recipe and fingerprint key, and supplier
SKUs and URLs remain outside it. The extracted reference sections have
explicit IDs in `tools/suppliers/profile_sections.py`.

The full gate builds examples and runs the smoke suite in shards. Its smoke
suite does not run the supplier-fasteners example or its profile, sourcing,
slot-hardware, motor-mount and supplier-axis entrypoints, nor the hobber tray
BOM entrypoint. Run these BOM-relevant entrypoints directly in addition to
the gate; compilation by `build-examples` is not runtime coverage. The
profile checks retain the distinct-variant assertion and also verify stable
numbers with changed fingerprints, required IDs and section serialization.

Runtime verification: the supplier-fasteners example passed with 36 generator
configurations and 72 occurrences; `ProfileRecipeChecks`, `SourcingChecks`,
`SlotHardwareChecks` (all 7 recipes and 22 measured slots),
`MotorMountChecks --supplier-cad` (4 variants), `SupportedSfu20AxisChecks`
(155 checks), `ConfigurableAxisChecks` (656 checks),
`VallderCatalogAxisChecks` (247 checks), and the hobber tray BOM passed.
The standalone BOM document, example and part-number key projects passed.
These checks use their individual `haxeon*.json` projects and separate output
paths; the configurable and Vallder projects share an output path and must be
built and run sequentially. The reference section extractor's existing Python
unit tests also passed.

## Recorded decisions

1. Made parts use author-declared drawing numbers. Their fingerprints change
   with the design inputs without changing the drawing number.
2. The bench mill declared fasteners from modeled patterns in B6. F1 replaced
   that with derivation from the parts' bolted faces (`JointFasteners`).
3. B8 moves `SourcingPlan` and `SupplierOffer` to the sourcing layer.
