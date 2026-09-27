# ManufacturingKit

ManufacturingKit contains Materia's sheet-stock planning and local cut-execution
records in the `materia.sheet` namespace. It depends on ProjectKit for shared
materials and unit conversion. The `.sheet.json` format and companion-file
convention remain unchanged.

Run the sheet-domain tests with:

```sh
./haxeon/scripts/haxeon run --project=manufacturingkit/tests/haxeon.json
```

The picking-station command-line demo exercises stock registration, planning,
exports, completed cuts, reopening, and remnant reuse:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/picking-station/haxeon.json -- \
  demo /tmp/picking-station.sheet.json /tmp/picking-station-exports
```

Use working copies of the project and its `.sheet.json` companion for real
operations. The checked-in picking-station files are shareable examples; the
demo creates a separate destination and refuses to overwrite it.
