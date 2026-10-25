# LCSC cable reference harness

This example captures the public LCSC Cable selectors and carries one terminal
harness through a specification, compatibility checks, BOM, connection schedule,
nominal CAD geometry, and STEP roundtrip. It makes no order or account changes.

From the repository root:

```sh
python3 tools/suppliers/lcsccable.py
python3 tools/suppliers/cable_assembly.py --catalog build/suppliers/lcsccable/catalog.json
MATERIA_ROOT="$PWD" ./haxeon/scripts/haxeon run --project machinekit/examples/cable-harness/haxeon.json
python3 -m unittest discover -s tools/suppliers/tests -p test_lcsccable.py
```

The capture defaults to `build/suppliers/lcsccable/`. `--refresh` replaces cached
responses; `--offline` replays the existing cache without network requests.
Failed API responses remain in the raw evidence and produce incomplete coverage,
not an empty catalogue claimed as complete. A cached failure requires `--refresh`
to retry. Frontend extraction never evaluates supplier JavaScript and fails when
its expected data layout changes. Source response hashes and fetch times are kept.

The initial capture on 2026-10-24 contains 90 wire-style records, 25 gauge options,
467 housing identities and 52 contact identities. These are public selector lists;
the site's connector search can expose additional parts. No product of the wire,
gauge, housing and contact lists is asserted to be manufacturable. Family/RFQ
entries are captured separately from configurable terminal harnesses.

Wire-style records preserve the complete original object. In particular, UL3239,
H05VV-F and H05V-K occur more than once with different display names or voltages.
Supplier voltage strings, including unexpected values, remain unverified source
data. They are not cable ratings. Style IDs hash the complete raw record, so
reordering a selector does not change an ID.

## Generated files

- `catalog.json`: source-bearing capabilities, frontend options and exact identities.
- `coverage.json` and `coverage.csv`: one row per style/part, evidence status and gaps.
- `raw/`: replayable response bodies, headers, timestamps, status codes and hashes.
- `example/report.json`: validated reference design and outstanding requirements.
- `example/bom.json`: two XHP-2 housings, four SXH-001T-P0.6 contacts and wire cut lengths by color.
- `example/connections.json`: explicit cavity map, cut lengths, route lengths and termination allowances.
- `example/drawing.svg`: dimensioned wire schedule, schematic and not to scale.
- `example/geometry.json`: nominal geometry input for MachineKit's `CableHarness`.
- `example/harness.step` and `example/cad-checks.json`: native CAD output and checks.

`tools/suppliers/lcsccable-reference.json` pins reviewed JST catalogue dimensions
and wire compatibility for 17 standard XH housings and three contacts that occur
in the captured selectors. The document is identified by URL, SHA-256 and page.
The housing envelope is drawing dimension C across the cavities, 7.75 mm along
the mating axis and 5.7 mm high. It is not a detailed shell or manufacturer STEP.
The official JST CAD links require a registration/license form, which was not
submitted. Public catalogue PDFs can be archived and checked separately:

```sh
python3 tools/suppliers/cable_assembly.py --fetch-documents --catalog build/suppliers/lcsccable/catalog.json
```

This downloads only the public pinned PDF. A changed PDF checksum stops the
command for a new review. The normal recipe and geometry commands make no network
requests. Keep the generated `documents/` and `raw/` with a snapshot for review.

## Recipe contract and limits

`xh-two-wire.json` follows `machinekit/schema/cable-assembly-v1.schema.json`.
The Python domain validator checks exact supplier style membership when supplied
a catalogue, housing/contact compatibility, AWG and insulation OD ranges,
complete cavity assignments, finite positive dimensions, connector frame axes,
route-to-cavity endpoints, exit tangents, cut-length budgets, strip/tinning
length consistency and wire envelope overlap. Unknown fields in engineering
properties are represented by `null`; nominal values carry `design_assumption`
and a reason. The JSON Schema is available for independent structural validation.

All recipe lengths use mm. `cut_length_mm` is a wire blank length. Route length
is the exposed centreline between nominal rear-face cavity datums. Each end has
an explicit termination allowance; unrepresented slack is reported. The LCSC
wire-only length field is kept separate and is unset in this example pending
supplier confirmation of the measurement basis. No connector length is silently
added to a supplier order length.

The two 200 mm exposed wire routes use 210 mm cut lengths with assumed 5 mm
termination allowances at each end. AWG24 and 1.4 mm insulation OD form a reference
design; an exact wire MPN, strand construction and OD tolerance are not known.
Crimp strip lengths, length tolerances and bend limits remain unknown. The JST
family's 3 A statement applies to AWG22 and is not assigned to this example.

Geometry v1 supports straight wires and drawing-sized housing envelopes, with
explicit connector exit directions and orientations. It rejects sharp corners
and nonparallel multiwire routes rather than claiming bend or collision safety.
It does not model contacts, hidden wire, crimp deformation, insulation stripping,
manufacturer cavity datums or connector mating geometry. Connector-to-connector
and wire-to-housing collision qualification, smooth bends, branch junctions,
shielding, jackets, twisted pairs and RF/USB/IDC assemblies remain future work.
The broad supplier catalogue does not imply geometry support for those families.

`manufacturing_qualified` remains false. Qualification requires an exact wire
selection, approved end processing and tolerances, verified connector datums,
application ratings and an approved supplier drawing. Neither the source capture
nor valid nominal solids supplies that approval.
