# Compound sourcing (Portugal)

Purchase options for the epoxy-granite compound of the mill and hobber castings, delivered to
Portugal. Prices and availability are web snapshots found on 2026-10-25 and will change. Grain
sizes, purity and dryness are seller claims; no technical sheet has been received yet. None of
this is in `CastingShopRecipe` yet: the compound, its aggregate size and its limits stay
unresolved there until the coupon tests below give measured values.

## Design target

- **Maximum aggregate size 8 mm** (graded 0–8 mm). This is the coarsest size that the insert
  ligaments, ribs and insert-to-mold gaps of about 1 t machines admit. Coarser grain saves little
  resin but doubles every minimum section. Finer grain needs 3–4 points more resin, which means
  more shrinkage, exotherm and expansion, and lower stiffness.
- **Minimum wall or rib** 5 × 8 = 40 mm. **Minimum passage or insert-to-mold gap**
  3 × 8 = 24 mm (`minimumPassageAggregateRatio` = 3). Rampf's datasheet for its own mineral
  casting grades gives the same pairing: 8 mm maximum particle with a 40 mm minimum castable wall
  (EPUMENT 140/8B), 5 mm with 25 mm, and 3 mm with 15 mm.
- **Resin** 9–12 % by mass, low viscosity, slow hardener. Packing decides most of the result:
  less resin beats a better rock.
- **Quartz aggregate first**, because it is the only aggregate sold washed, dried and graded in
  bags. Commercial mineral casting expands at about 17–20 ppm/K (Rampf: 17 for the 8 mm grade,
  19.5 for 5 mm, 20 for 3 mm), well above steel's 11–12. The mismatch with the steel inserts
  falls with lower resin content and lower-expansion rock. Crushed basalt is the later bulk
  alternative for the coarse fractions (see below).

## Mix and quantities

Starting split of the aggregate by mass, from a packing curve up to 8 mm. Tune it with the coupon
tests.

| Fraction | Share of aggregate |
| --- | --- |
| 4–8 mm | 30 % |
| 2–3 mm | 20 % |
| 1–2 mm | 15 % |
| 0.3–0.6 mm | 13 % |
| 0–1 mm (fine) | 11 % |
| Filler below 0.1 mm | 11 % |

Quantities per casting, at 11 % resin, mix density 2,400 kg/m³ and 10 % for waste and leftovers:

| Casting | Wet mix | Aggregate incl. filler | Epoxy |
| --- | --- | --- | --- |
| Mill base | 166 kg (69 L, current kit) | ~163 kg | ~20 kg |
| Mill column | 99 kg (41 L, current kit) | ~97 kg | ~12 kg |
| Hobber plinth | ~1,016 kg (842 kg frozen design + filled bands, slots and sliver) | ~995 kg | ~123 kg |
| Hobber column | ~62 kg (43 kg frozen design + filled back pockets) | ~61 kg | ~7 kg |
| **Total** | ~1.34 t | **~1,315 kg** | **~163 kg** |

The hobber figures predate the roof thickening required by the 40 mm wall rule. The hobber plinth
is a single pour of about 1 t, so the batch schedule (mixer capacity and working time) is a real
planning input.

## Aggregate

### Vetonek (primary)

Spanish bagged silica sand, sold direct and through Leroy Merlin. Grades 0–1 mm (sold for
mortar), 0.3–0.6, 1–2, 2–3, 3–5 and 4–8 mm, in 25 kg bags or 1,000 kg big bags. The page shows
€7.51 per 25 kg bag (incl. VAT) without saying which grades that covers. Leroy Merlin lists the
0.3–0.6 mm bag at about €8.70. An older snapshot shows big bags at about €178–223 per tonne.
Silica content, dryness, washing and grain shape are not stated: ask for the technical sheet.
[Product](https://vetonek.com/pt/products/arena-de-silice),
[EN page with big-bag prices](https://vetonek.com/en/products/arena-de-silice),
[Leroy Merlin 25 kg](https://www.leroymerlin.pt/produtos/areia-de-silica-para-grama-e-caixas-de-areia-saco-de-25-kg-0-3-0-6-mm-92375888.html),
[Leroy Merlin big bag](https://www.leroymerlin.pt/produtos/areia-de-silica-para-grama-e-caixas-de-areia-big-bag-1-000-kg-0-3-0-6-mm-91925898.html).

Vetonek also sells a dry blasting sand (0.3–0.6 mm and 0.7 mm, labelled "seca", about €9). Use it
in place of the 0.3–0.6 mm silica sand if the silica sand turns out not to be dried.
[0.3–0.6 mm dry](https://www.leroymerlin.pt/produtos/areia-explosiva-saco-com-25-kg-granulometria-fina-0-3-0-6-mm-seca-91926042.html).

Shipping to Portugal is a flat rate per order, by total weight, incl. VAT, delivered in 3–9
working days, with no free-shipping threshold
([policy](https://vetonek.com/pt/policies/shipping-policy)):

| Order weight | Shipping |
| --- | --- |
| 1–150 kg | €90 |
| 151–450 kg | €80 |
| 451–900 kg | €85 |
| 901–1,200 kg | €80 |
| 1,201–2,000 kg | €145 |
| 2,001–3,000 kg | €190 |

Planned orders, sand in 25 kg bags and filler in 20 kg bags (no single grade reaches a tonne, so big bags don't pay):

| Grade | Order 1: coupon tests + mill base and column | Order 2: hobber plinth and column |
| --- | --- | --- |
| 4–8 mm | 5 | 13 |
| 2–3 mm | 4 | 9 |
| 1–2 mm | 3 | 7 |
| 0.3–0.6 mm | 3 | 6 |
| 0–1 mm | 3 | 5 |
| Calcium carbonate 63 µm (20 kg bags) | 2 | 6 |
| **Total** | 20 bags, 490 kg, €85 shipping | 46 bags, 1,120 kg, €80 shipping |

- Order 1 ships at €85 because the filler takes it past 450 kg. Drop one sand test bag to get
  back to €80.
- Order 2 waits for the coupon results and the hobber roof thickening. About 3 more sand bags
  still ship at €80; beyond 1,200 kg the rate rises to €145.
- The 3–5 mm grade is left out: a gap between 3 and 4 mm costs little packing.

### Alternatives and top-ups

| Fraction | Option | Notes | Link |
| --- | --- | --- | --- |
| 4–8, 3–5 mm | Pool-filter sílex, Leroy Merlin | Washed, dried, graded; about €44–57 per 25 kg (about €2/kg) | [3–5 mm](https://www.leroymerlin.pt/produtos/arena-de-silex-especial-para-filtros-de-piscinas-sacos-arena-25-kg-para-piscina-granulometria-3-5-mm-86292100.html), [25 kg range](https://www.leroymerlin.pt/produtos/construcao/cimentos-areias-e-britas/areias/sacos-de-areia-de-25-kg-p.html) |
| 3–5 mm | Astralpool sílex gravel | 8 × 25 kg pack, about €150 | [Outlet Piscinas](https://www.outlet-piscinas.pt/grava-silex-astralpool) |
| 2–4 mm | Ecopool silica, AmaCasa | About €9 per 25 kg; also 0.6–1.2 mm | [AmaCasa](https://amacasa.pt/shop/piscina/limpeza/saco-areia-de-silica-para-filtros-piscina-25kg/) |
| 1–2, 0.4–0.8 mm | Pool sílex, Piscinarium | About €8 per 25 kg | [Piscinarium](https://www.piscinarium.com/pt/filtros-de-piscina/1818-areia-de-piscina-de-silex-saco-de-25-kg.html) |
| 0.3–0.8 mm | Sika Cargas‑123 | Dried, graded quartz sold for epoxy floors | [Sika](https://prt.sika.com/pt/sika-produtos/pavimentos/cargas-para-revestimentosindustriais/cargas-123.html), [Ovarmat](https://www.ovarmat.pt/pt/sika/cargas-inertes-para-pavimento-industrial-carga-123-sika-25kg_p16310.html) |
| 0.3–0.8 mm (to confirm) | Areia sílica APB40, Leroy Merlin | Artificial-turf infill, about €6–8 per 25 kg; grading not published | [Leroy Merlin](https://www.leroymerlin.pt/produtos/areia-silica-apb40-25kg-95228158.html) |
| 1–2, 0.2–0.8 mm | Epodex dried quartz sand | 25 kg about €15; a 0.1–0.4 mm 10 kg listing may be stale | [25 kg](https://www.epodex.com/pt/produto/areia-de-quartzo-seca/), [10 kg](https://www.epodex.com/pt/produto/areia-de-quartzo-10kg/) |
| Fine, grading unknown | Mapei Quartzo, Sird | Washed, oven-dried, about €27 per 25 kg ex VAT; ask for the grading | [Sird](https://sird.pt/mapei-quartzo-25kg-areias-de-quartzo-lavadas-e-secas-em-estufa-para-utilizacao-com-resinas-epoxis-e-em-poliuretano/) |
| Bulk sand and flour | Sibelco Portuguesa, Rio Maior | Industrial silica producer; grades and big-bag terms not confirmed | [company](https://infotrustgo.pt/en/companies/sibelco-portuguesa-lda) |

### Later: crushed basalt for the coarse fractions

Basalt from quarries around Lisbon, bought by the tonne, is stiffer, denser and cheaper per kg than
bagged quartz. The rock expands at about 5–8 ppm/K against about 11–12 for quartz, so it brings the
casting's expansion closer to the steel inserts. It must be washed and dried to under 0.1 %
moisture. Switch only after a coupon comparison against the all-quartz mix.

### Avoid

- **Granite** unless the quarry supplies fresh, fine-grained, low-mica stone. Much Portuguese granite
  carries mica, which makes weak planes, and weathered feldspar, which absorbs moisture.
- **Construction or river sand.** Uncontrolled grading, clay and silt, and it arrives damp.
- **Sea sand.** Salt and shell fragments.

## Filler (below 0.1 mm, about 145 kg in total)

| Option | Data | Price | Use |
| --- | --- | --- | --- |
| Vetonek calcium carbonate (agricultural lime), 63 µm grade | Crushed limestone; claims 40 % calcium (pure CaCO₃ content). The "63 µm" grade probably means everything passes a 63 µm sieve. Moisture and purity otherwise not stated. | €8.33 per 20 kg (about €0.42/kg); 1 t big bag also sold | **Primary filler.** It fits below the 0–1 mm sand and ships in the same Vetonek orders. The 800 µm grade overlaps the sands; don't use it. |
| Nazza carbonato de cálcio, ref. 42990 | Marble powder, CaCO₃ > 98.8 %, mean particle about 5 µm, surface area 2.7 m²/g, bulk density 0.75 kg/L | €26.63 per 25 kg incl. VAT; free shipping above €95. Sold to professional buyers only (asks for an IAE registration). | **Optional extra, 2–4 % of the aggregate**, to test for packing. Too fine to be the whole filler share: its surface area would dominate resin demand. |

Links: [Vetonek](https://vetonek.com/pt/products/carbonato-calcio-cal-agricola),
[Nazza](https://www.nazza.pt/produtos-quimicos/209-carbonato-calcio.html)
([datasheet](https://www.nazza.es/img/cms/FT-Carbonato%20de%20calcio-Nazza.pdf)).
Other industrial carbonates: Sur Química, 98.5 %+ purity in 25 and 50 kg bags and 1 t big bags
([datasheet](https://www.gruposur.com/download/hojas_tecnicas/ht-557-53020-000.pdf)), and Geokom
MICARB ([datasheet](https://geokom.com/storage/files/afTaa64eWrFbHUE6wu6e1CCL7MNOPE1ijTK1hba9.pdf)).

Silica flour is slightly stiffer but is respirable crystalline silica; use it only with a P3
respirator and wet cleanup. The fines in the Vetonek 0–1 mm sand may reduce the filler needed;
the coupon tests decide.

## Epoxy

Low viscosity (ideally under about 600 mPa·s), slow hardener, solvent-free, in 20–30 kg kits.

| Option | Notes | Link |
| --- | --- | --- |
| Sikafloor‑161 (Sika Portugal) | Low-viscosity two-part epoxy, a binder for quartz mortars, sold at builders' merchants. Confirm it is solvent-free and check its pot life. | [datasheet](https://prt.sika.com/dam/dms/pt01/m/sikafloor_-161.pdf) |
| Resoltech 1050 with 1053S hardener | Infusion grade, 205 mPa·s mixed; via Castro Composites, which ships to Portugal | [Castro Composites](https://www.castrocompositesshop.com/en/resins/1088-epoxy-resin-resoltech-1050-8431252093182.html) |
| Nazza | General epoxies from 0.7 to 25 kg; ask which are low viscosity | [Nazza](https://www.nazza.pt/32-resinas-epoxi) |
| Breddermann E45GB (Germany) | Low-viscosity casting system with 45 min working time, advertised for mineral casting and machine beds; no datasheet found, and shipping to Portugal unconfirmed | search "Breddermann E45GB" |
| SKresin 3530 (Harzspezialisten, Germany) | Very thin resin developed for filled systems, advertised for polymer concrete and machine-bed casting; ask for the datasheet and Portugal shipping | [Harzspezialisten](https://www.harzspezialisten.de/Polymerbeton) |

Not suitable: Acorus Aquastone EPX (12–14 Pa·s, too thick).

## Ready-made products

- **Rampf EPUSELF.** Rampf's do-it-yourself mineral casting: pre-metered containers of graded
  minerals, resin and hardener, mixed in the supplied bucket with a slow stirrer and usually cast
  without compaction. The brochure says 30 kg containers; other listings say 25 kg. It is sold in
  three EPUMENT grades:

  | Grade | Max particle | Min wall | Modulus | Compressive | Flexural | Expansion | Damping (log. decrement) |
  | --- | --- | --- | --- | --- | --- | --- | --- |
  | 140/8B | 8 mm | 40 mm | > 35 GPa | > 130 MPa | > 30 MPa | ~17 ppm/K | 0.03 |
  | 140/5 | 5 mm | 25 mm | 35–40 GPa | 140–160 MPa | 35–45 MPa | ~19.5 ppm/K | 0.035 |
  | 130/3 | 3 mm | 15 mm | > 25 GPa | > 130 MPa | > 35 MPa | ~20 ppm/K | 0.038 |

  Density is about 2.3 for all three. No price or Portugal shipping is published, so ask Rampf for
  a quote. The 140/8B grade matches our design target exactly, so it is both a datasheet-backed
  option for the mill castings (about 9 containers) and the reference mix for the coupon tests.
  [Brochure](https://www.rampf-group.com/fileadmin/Downloads/RMA/Epuself-RAMPF-EN.pdf),
  [product page](https://rampf-group.com/en/products-solutions/details/epuself-1408).
- **Rampf EPUMENT.** The industrial benchmark: density 2.3–2.4, modulus 30–45 GPa. It is sold as a
  German system and casting service, not by the bag.
  [Product](https://www.rampf-group.com/en-us/products-solutions/details/mineral-casting-epument/),
  [brochure](https://freeman-japan.co.jp/wp-content/uploads/2021/09/Mineral-casting-EPUMENT-EN.pdf).
- **Sikadur‑42.** A three-part epoxy grout for machine base plates. Use it for the base–column joint
  grout gap and under feet, not for the castings.
  [Datasheet](https://sgp.sika.com/content/dam/dms/sg01/6/sikadur-42-mp-normal.pdf).

## Interface compounds (not casting binders)

These go on the casting after it cures, for precision interfaces, joints and bushes. They are not
casting binders.

| Product | Use | Data | Link |
| --- | --- | --- | --- |
| Diamant DWH 310 FL (#0795) | Replicating rail seats against a release-coated master (the 4 mm seat allowance), load-bearing gap levelling, casting bushes and centring elements | Steel-filled epoxy, 12–14 Pa·s mixed, 87:13 by weight, 50 min pot life and 18 h to demould at 20 °C, shrinkage < 0.1 %, 165 MPa compressive, 8.7 GPa modulus, −20 to +80 °C. 250 g, 500 g or 1 kg, from €45.03. Surfaces need roughening to Rt about 0.5 mm. | [shop](https://diamant-polymer.de/en/shop/dwh/310-fl/), [datasheet](https://diamant-polymer.de/en/wp-content/uploads/sites/2/DIAMANT_TD_DWH-310-FL_0795_EN_V1-1.pdf) |
| Diamant EP-Flow | Self-levelling support grout; candidate for the base–column joint and under feet, beside Sikadur‑42 | 57 MPa compressive, 8.5 GPa modulus, 9:1 by mass, 60 min pot life, layers up to 100 mm; no minimum layer published (see the sheet-router DIY route) | [product](https://diamant-polymer.de/en/product-overview/ep-flow/) |
| Moglice and other slideway compounds | Rail-seat replication | Compared in [METROLOGY_SOURCING.md](../../METROLOGY_SOURCING.md) | |

Diamant's site says it now ships to "selected EU countries", but its shipping page says Germany
only. Its German rates are €14 up to 5 kg, €17 for 6–15 kg, €20 for 16–29 kg and €40 from 30 kg.
Ask Diamant about delivery to Portugal.
[Shipping](https://diamant-polymer.de/en/shipping-methods/).

## Checks on arrival and coupon tests

Check each grade:
1. **Moisture.** Weigh about 500 g, dry it at about 105 °C for an hour, reweigh. The loss must be
   under 0.1 %; otherwise dry the whole batch before use.
2. **Cleanliness.** Shake some in a jar of water. A muddy cloud means clay or dust, which weakens
   the bond.
3. **Grading.** Sieve against the stated band, and measure the share below 0.1 mm in the 0–1 mm sand.

The moisture check matters most for the agricultural lime, which is often not dried.

Coupon tests compare all quartz with quartz sand plus basalt coarse, each with and without the 2–4 %
Nazza 5 µm addition. If EPUSELF 140/8B is bought, cast it alongside as the datasheet reference. For each mix, measure:
- dry packed density of the blend, tuning the split toward maximum packing;
- the lowest resin content that still flows under vibration;
- the exotherm of a block about 150 mm thick;
- density (target 2.2–2.4) and voids on cut coupons;
- stiffness, and expansion on a bar.

These results replace the unresolved compound inputs in `CastingShopRecipe`: maximum aggregate size,
mix fractions, density, and the section-thickness and lift limits.
