# MachineKit BOM JSON, version 1

`machinekit/scripts/bom-export <machine>` writes `build/bom/<machine>.json`.
The supported names and constructors are in `BomExport.hx`. The command
requires an existing MachineKit native build, selects it with
`MATERIA_BOM_NATIVE_ROOT` when necessary, and uses the worktree's Haxe code.
It records `git rev-parse HEAD` and the current UTC time. The commit does not
capture uncommitted changes; export a committed worktree when the commit must
identify all source inputs.

The only supported document schema is `"machinekit.bom"`, version `1`.
`machinekit.component.BomJson.parse` rejects other schema names and versions
and requires the two views. It checks the envelope, not every field: consumers
must validate the fields they use. Do not accept another shape as version 1.
The JSON is an engineering snapshot, not a purchase order, supplier quote,
stock-availability statement, or compatibility approval.

## Document and views

The root has `schema`, integer `version`, `machineId`, `sourceCommit`,
`generatedAt`, `assembly`, and `procurement`. `assembly` is a node with
`path`, `lines`, and `children`; each child has the same shape. Each line in
either view is `{ "item": ..., "uses": [...] }`. The engineering tree
retains all modeled occurrences, including user-supplied items and pieces
that are part of a purchased unit.

`procurement` is derived from that tree. It excludes `NotSupplied` items and
items whose `includedIn` names their purchase owner, then rolls up the
remaining lines by `partNumber`. The owner of an included piece must be a
non-included, supplied line in the same assembly node. The exporter rejects an
orphan or conflicting records for a rolled-up number. It does not infer kits
or vendor bundles; those are later sourcing decisions.

Each use has `kind` (`member` or `inventory`), `path`, and `quantity`.
`member` paths name assembly occurrences; `inventory` paths identify explicit
BOM extras. Kind distinguishes an inventory extra from a member even when
both use the same path text. Quantities for the uses of a procurement line sum
to its quantity, in the same unit.

## Item fields

- `partNumber` is the engineering identity. Made, non-recipe parts have
  author-declared drawing numbers; bought parts use their declared or recipe
  identities. `description` is explanatory text, not a key.
- `quantity` is `{ "value": <positive number>, "unit": <unit> }`. The
  supported units are `piece`, `m`, `m2`, `kg`, and `l`. `cuts` is an array
  of `{ "length": <metres>, "count": <positive integer> }`; it is empty
  when there are no cuts and is only valid for a line ordered in metres.
  Cut counts include occurrence multiplicity. Cut lengths must not exceed
  the ordered length; extra length can represent allowance.
- `procurement` always has `kind`, `process`, `manufacturer`, `mpn`, and
  `standard`. `kind` is `Make`, `Buy`, or `NotSupplied`. `Make` has a
  nonempty `process`; `Buy` may name a manufacturer, manufacturer part number,
  or standard; fields that do not apply are null. `material` is supplied for
  Make when known and is null for Buy and NotSupplied.
- `maturity` is `Assumed`, `Catalog`, `Stated`, or `Derived`, independent of
  the procurement kind. `requirements` is an array of `{name, min, max,
  nominal, unit}` records. Bounds and nominal values are strings or null:
  numeric terms use formatted decimal strings in the stated unit; categorical
  choices use `unit: "1"` and a string nominal. An Assumed line has at least
  one requirement. Missing requirements such as an unmodeled pressure rating
  do not imply that a part is qualified.
- `fingerprint` is the lowercase SHA-256 fingerprint of the exact modeled
  inputs and author-controlled generator version, not an exported B-rep hash.
  It can be null when a model cannot yet provide complete fingerprint inputs.
  `releasedRevision` and `releaseCurrent` are both null without a release;
  otherwise the revision is a letter sequence and `releaseCurrent` says
  whether its pinned fingerprint matches the current line.
- `typeId` and `valuesKey` identify a recipe and its exact configuration when
  one exists; both can be null for code-only components. `familyId`,
  `manufacturingVariant`, and `manufacturingProcess` identify a selected
  manufacturing family variant when present. These are separate from the
  procurement process.
- `includedIn` is null or the part number of the local purchase owner.
  `properties` is null except for panel lines, where it contains `widthMm`,
  `heightMm`, `areaM2`, `massKg`, and `construction`. Construction records
  the layer materials and thicknesses, fabrication, joints, coating, and
  stated or unknown qualifications. `properties` replaces the old special
  enclosure-panel field.

For example, a purchase line can contain:

```json
{
  "item": {
    "partNumber": "ISO4762-M4x16",
    "description": "Socket head cap screw",
    "quantity": {"value": 48, "unit": "piece"},
    "material": null,
    "typeId": "machinekit.standard.socket-head-cap-screw",
    "valuesKey": "<exact recipe key>",
    "familyId": null,
    "manufacturingVariant": null,
    "manufacturingProcess": null,
    "properties": null,
    "cuts": [],
    "includedIn": null,
    "procurement": {"kind": "Buy", "process": null, "manufacturer": null, "mpn": null, "standard": "ISO 4762"},
    "maturity": "Catalog",
    "requirements": [],
    "fingerprint": "<64 lowercase hexadecimal characters>",
    "releasedRevision": null,
    "releaseCurrent": null
  },
  "uses": [{"kind": "inventory", "path": "@inventory/0", "quantity": {"value": 48, "unit": "piece"}}]
}
```

The fingerprint and recipe-key strings above are placeholders, not a release
record or an orderable specification for a screw.

## Release records

An optional `machinekit/examples/<machine>/releases.json` has exactly the
version-one envelope `{ "version": 1, "releases": [...] }`. Each record has
`partNumber`, uppercase-letter `revision`, a 64-character lowercase hexadecimal
`fingerprint`, and `date` in `YYYY-MM-DD` format. Records for each part are
chronological, and a part/revision pair cannot repeat. The last record for a
part is the current release. A changed input fingerprint keeps the drawing
number and released revision but makes `releaseCurrent` false. No release is
created automatically.

## Sourcing boundary

Manufacturer/MPN and standards are engineering evidence in the BOM. Supplier
SKUs, prices, quotes, selection, and review are separate from it. The existing
Haxe offer-selection helper is in the `materia-sourcing` package under
`tools/suppliers`; it reads the parsed v1 procurement view and keys offers
and selections by part number, fingerprint and released revision. It does
not implement RFQs. A later sourcing layer can join quotes to a part number and an explicit released
revision, and identify absent, stale, or orphaned quotes. It must check the
schema version before interpreting the document and must not treat an assumed
line or a matching name as supplier qualification.
