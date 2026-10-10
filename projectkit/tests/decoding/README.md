# Artifact decoding benchmarks

These isolate binary decoding and validation from compilation, disk I/O, geometry
preparation and rendering. Use an actual cached `.mtrg` file from
`${XDG_CACHE_HOME:-$HOME/.cache}/materia/generated-artifacts`.

From the repository root, build and run the native benchmark:

```sh
./haxeon/scripts/haxeon build --project=projectkit/tests/decoding/haxeon.json --compiler-only
LD_LIBRARY_PATH=haxeon/out:haxeon/.tools/hashlink \
  haxeon/.tools/hashlink/hl projectkit/tests/decoding/build/host/main.hl /path/to/artifact.mtrg
```

Each JSON line reports decoding time, GC-tracked allocation bytes, and a separate
mesh-validation pass. Native mesh allocations use external storage, so the GC
counter does not include their payload bytes. Both copied and borrowed decoding
are measured. The first iteration includes initialization; compare medians and
retain the same input and runtime when comparing changes.

Run both browser backends in headless Chrome:

```sh
python3 projectkit/tests/decoding/benchmark-browser.py /path/to/artifact.mtrg
```

This builds temporary guest modules, transfers the input outside the timed region,
warms up once, and records five decodes on each backend. It checks stable part
counts and writes `/tmp/materia-artifact-browser-timings.json`. `--chrome` and
`--output` select the executable and result path. These are decoder measurements;
the browser editor cannot yet open source projects through its child-process path.

On 2026-10-07, a 17,782,918-byte Gantry artifact with 62 shared geometry definitions
measured as follows on the development machine:

| Measurement | Before | After |
| --- | ---: | ---: |
| Native decode median | 104 ms | 18 ms (borrowed), 25 ms (copied) |
| Native GC-tracked allocations per decode | 93 MB | 7.2 MB |
| Chrome wasm32 decode median | — | 45 ms |
| Chrome wasm-gc decode median | — | 17 ms |

The largest native saving came from replacing read-only assembly flattening's
JSON round-trip copies with a borrowed flat view. Independent `flatten` copies
remain available. The reader also reads scalars and text directly and offers
borrowed mesh streams through `SceneArtifact.decodeView`; ordinary `decode`
continues to own independent streams.

The browser parser previously scanned UTF-8 strings using repeated `charCodeAt`
walks, making large JSON metadata quadratic. A 171 KB assembly JSON probe exceeded
10 seconds before the fix and parsed in about 16 ms afterwards. Wasm now scans
UTF-8 bytes, slices strings at byte boundaries, and encodes Unicode escapes as
UTF-8. Native scanning retains its UTF-16 positions.

Correctness checks live in `ProjectKitTests` and
`haxeon/tests/integration/test-json-parser.sh`: mesh ownership, source preservation,
truncated and malformed artifacts, nested assemblies, Unicode, surrogate pairs,
malformed JSON, large documents and decoded object arrays.
