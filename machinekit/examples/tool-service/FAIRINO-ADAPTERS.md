# Fairino purchased dual-tool adapters

`machinekit.robotics.FairinoDualToolAdapter("Standard" | "Expert")` is a registered
purchased-component recipe. Add the selected adapter to a `MachineAssembly` with
`addComponent`; `billOfMaterials()` counts the entire purchased unit once. Molded
bodies, connectors and included leads are geometry of that unit, not extra purchases.
Both products are selectable in the robotics component registry and persist through
recipe values. They are alternative products, not manufacturing variants of a fabricated part.

| Variant | Manufacturer PN (listing HAN) | Welectron SKU | Power |
| --- | --- | --- | --- |
| Standard | ICACX1500GSCSTA | FA2-606082 | Robot tool supply |
| Expert | ICACX1500GSCEXP | FA2-860952 | External 24 V DC |

Sources inspected 2026-10-18:
- https://www.welectron.com/Fairino-End-Effector-2-to-1-Adapter-Standard
- https://www.welectron.com/Fairino-End-Effector-2-to-1-Adapter-Expert

The parent listing's FA2-616083-P is a variant-group reference; do not order it as
an identified Standard or Expert unit. The individual listings identify the HANs above.
Photo references:
- https://www.welectron.com/media/image/product/34282/lg/fairino-endeffektor-2-auf-1-adapter-standard.jpg
- https://www.welectron.com/media/image/product/34283/lg/fairino-endeffektor-2-auf-1-adapter-expert.jpg

The Standard photo shows a rigid splitter; Expert shows a molded cable split,
right-angle robot plug, two tool plugs and a long power lead. Expert's listing
states black = GND and blue = +24 V; this is product-specific, not generic wiring advice.
DH PGC compatibility is not established by these listings (their compatibility lists
name Fairino IR grippers). No public STEP or dimensioned drawing was found.

## Geometry and electrical limits

All dimensions are estimated planning inputs in millimetres, fixed in the recipe;
none are measured supplier dimensions. Standard body is 42 x 24 x 20, with nominal
connector envelopes R7/R8. Expert body is 36 x 44 x 20, with R3 illustrative cables,
a right-angle wrist connector and two socket envelopes. The displayed Expert leads
are straightened short layout segments, not actual stock cable lengths. These are
solid layout models, not manufacturing drawings. Pins, keys and threads are omitted.
The generic rubber material only supplies preview appearance and estimated mass;
the real purchased unit contains multiple materials and has no verified mass here.

The `layout` frame is an estimated splitter centre, NOT a mounting attachment.
`robot`, `tool1`, `tool2`, and Expert-only `externalPower` frames mark approximate
terminal mouths, with +Y pointing outward. No inferred hole is added to either body.
Connector gender/coding, seating, current ratings, cable lengths/OD/bend limits and
actual wiring/backfeed handling remain unverified. `currentRatingA` is null.

Ports expose the intended services and the distinct power entry. No pass-through
bridges are asserted, so neither component can certify a wired circuit. Expert
has no robot-power input: this expresses the intended external-power arrangement,
not proof of electrical isolation in the actual product. Do not use these envelopes
to certify mating, installation clearances, cable dynamics or robot payload.

## Generate and verify

From the repository root:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/tool-service/haxeon.fairino-adapters.json
```

Outputs in `build/fairino-adapters/`: `standard.step`, `expert.step`, matching
`.mtrg` scenes, product metadata and `bom-check.json`. The latter deliberately
checks two Standard instances and one Expert: two BOM lines with quantities 2 and 1.
The runner also checks recipe reconstruction, valid CAD and the distinct power
endpoints. These examples do not install either adapter into the tending assembly.
