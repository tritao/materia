# Mill-base core strategy comparison

Status: the EPS baseline is measured from the regenerated records kit at
`2641bb56c`. The requested drafted-melamine variant is parked: producing it
requires library changes excluded by this comparison's scope. The production
recipe remains unchanged; there is no qualified alternative to select yet.

Both variants must start from source fingerprint
`d3652114c33eb87032d2df0bd51945ea472b2e9e4fdaf0c183bf76123bc63666`.
No source dimensions or openings were changed for this assessment.

| Item | Current EPS recipe | Requested drafted-melamine recipe |
| --- | --- | --- |
| Release | 14 existing rigid-former draft decisions Pending | Positive draft cannot be proposed/applied through the existing accepted-proposal path |
| Withdrawal/reconstruction | Full native mill checks pass; five frozen-source comparisons have zero difference | Not generated or credited |
| Main shell/support panels | 15 production panels; 2 bounding-blank sheets, excluding trial coupons | Not generated |
| Former work | 14 formers, 38 board pieces; 4 hollow boxes use dry-cavity pocket screws | Tall drafted boxes require leaning walls, currently Unsupported |
| EPS work | 2 source core units, 6 foam pieces, 12 layers; 15.168 L foam stock per casting | Face-open lightening pockets would become reusable formers; enclosed voids must remain EPS |
| Router work | 67 review programs (40 panel, 27 former); cycle time Pending machine/rapid capability and workholding | No programs or time claimed |
| Core uplift/restraint | 538.879 N total factored gross uplift; 67.360 or 134.720 N per foam piece; all 6 restraint qualifications Pending | Not evaluated |
| Peak panel head pressure | 7.908 kPa | Not evaluated |
| Air traps | 0 | Not evaluated |
| Kit status | 625 Pass / 556 Pending / 136 Unsupported / 7 source-corner Fail | No kit or readiness claim |

The sheet count covers the main panel blank layout, not the separate former-board
stock. Uplift and pressure are design estimates from `load-estimates.json`, not
physical restraint evidence. Router time cannot be qualified from program count.

## Missing path, without adding another implementation

1. `CastingFeaturePatterns.underside` declares width, depth and height adjustable,
   but not Draft. Its retained-web fragments must stay linked by `derivedFrom`.
2. `CastabilityAnalysis` emits a release instruction without a mapped draft value.
   Its accepted-proposal application maps insert width only. There is no existing
   draft proposal/application to use for the requested source change.
3. `FormerManufacturing` explicitly reports drafted built-up formers Unsupported
   because leaning wall panels are not implemented. The approved one-board or
   screwed hollow-box construction remains in force; no glued blank or second
   finishing path was introduced.

These gaps prevent the requested comparison under the recipe/fixture-only rule.
No arbitrary draft angle, manual source mutation, substitute zero-draft comparison
or new core type was introduced. The authored opening must retain its dimensions
when a future accepted draft change tapers the feature inward.

The baseline is in `build/tooling/base-fabrication-records/`: `core-units.json`,
`eps-layers.csv`, `panel-blanks.csv`, `blank-layout.json`,
`former-manufacturing.json`, `cam-programs.json`, `load-estimates.json`,
`fill-analysis.json` and `fabrication-validation.json`. Regenerate it with the
existing [tooling entry points](TOOLING_README.md); generated kits are not committed.
