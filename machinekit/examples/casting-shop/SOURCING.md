# Compound sourcing (Portugal)

Purchase options for the epoxy-granite compound of the mill and hobber castings, delivered to
Portugal, with the mix design, mixing equipment, batch plan and coupon tests that go with them. Prices and availability are web snapshots found on 2026-10-25 and will change. Grain
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
- **Design modulus 25 GPa until coupons are measured.** Structural and stiffness checks on the
  mill and hobber use this conservative DIY value, not a professional datasheet value.

## Expected performance against professional castings

The DIY column is an estimate from forum builds and typical behaviour, not a measurement.

| | Schneeberger (standard grades) | Rampf EPUSELF 140/8B (DIY kit) | Our mix (expected) |
| --- | --- | --- | --- |
| Modulus | 30–45 GPa, typically 40–45 | > 35 GPa | 25–35 GPa with good packing and vibration; under 20 GPa with poor packing |
| Density | 2.2–2.45 | ~2.3 | 2.1–2.3 |
| Resin content | very low, optimised many-fraction gradings | fixed by the kit | 10–12 % realistic, 9 % after tuning |
| Expansion | 11.5–14 ppm/K | ~17 ppm/K | probably 17–22 ppm/K |
| Damping (log. decrement) | 0.03–0.10 | 0.03 | similar or higher (more resin adds damping) |
| Compressive strength | 130–140 MPa | > 130 MPa | ~70–130 MPa |
| Consistency | certified per grade | factory-weighed | batch to batch unless weighing and drying are disciplined |

Where professionals are ahead:
- **Material:** optimised packing, aggregates dried and heated, metered mixing, vibration tables and
  controlled cure.
- **Precision:** rigid steel molds and insert fixtures, and mounting faces ground or replicated to
  micrometres.
- **Assurance:** datasheets and test certificates.

How we close most of the gap:
- **Geometry.** Thick, solid sections: 40 mm walls, filled hobber bands, a solid hobber column.
  30 % less modulus is roughly 30 % less structural stiffness, which added section can win back.
- **Replication.** Inserts only need to land within about ±0.2–0.5 mm; the seat allowance and a
  replication compound copy a precision master.
- **Damping** is no worse.

What remains worse:
- **Thermal drift and insert stress.** At about 20 ppm/K against steel's 12, a 1 °C change over a
  600 mm rail span gives roughly 5 µm of differential strain. Keep the shop temperature steady, keep
  resin low, and later use basalt coarse aggregate. Insert anchor and ligament checks use the
  measured expansion.
- **Variability.** Weigh by mass, dry everything, mix to a fixed recipe, and cast a witness coupon
  with every pour.

Sources: [Schneeberger](https://www.schneeberger.com/en/products/mineral-casting/),
[techcon data](https://www.schneeberger.com/fileadmin/documents/downloadcenter/01_product_catalogues_company_brochures/05_Mineral_casting/2023_Flyer_techcon_A4_en.pdf),
[technical brief](https://www.schneeberger.com/fileadmin/documents/White_Paper_and_Technical_Briefs/Technical_brief_Mineral_Casting_Eliminating_Vibration_EN.pdf),
[eb-system data](https://www.eb-system.com/en/schneeberger-mineral-casting/technical-data/),
[brochure](https://promshop.info/cataloguespdf/docmineralcastingen.pdf), and the EPUSELF brochure
below.

## Packing

The aggregate forms a skeleton of touching grains, and the resin fills the gaps. Every gap is resin:
soft, high-expansion and shrinking. Grains of one size leave about 40 % empty space. Each smaller
fraction that fits the remaining gaps lowers it, down to 15–20 % for a well-graded blend.
Professionals blend many closely spaced fractions to a size curve (Fuller, Andreasen or
Funk–Dinger) and measure every blend.

Empty space after vibration, against the resin needed just to fill it (quartz 2.65 g/cm³, resin
1.1 g/cm³):

| Empty space | Resin by mass | Typical of |
| --- | --- | --- |
| 25 % | ~12 % | a simple 3–4 fraction mix |
| 20 % | ~9.5 % | a well-tuned 5–6 fraction mix (our target) |
| 15 % | ~7 % | professional gradings |

The resin also needs a thin film on each grain and a little extra to flow.

Optimising the blend needs only a bucket and scales:
1. Make 2–3 kg dry blends of the Vetonek grades plus filler, starting from the split below. Fill a
   container of known volume, vibrate, top up, level and weigh.
2. Empty space = 1 − bulk density / 2.65. Shift shares between fractions and repeat, keeping the
   blend with the least empty space. Typically that means slightly more fines than the Fuller curve
   (an exponent of about 0.35–0.45 instead of 0.5).
3. Only then add resin, and find the lowest content that still flows and compacts under vibration.

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

Choose the binder for viscosity and working time. Unfilled epoxies all have a modulus of about
2.5–3.5 GPa, so the resin's own stiffness barely changes the casting. Low viscosity allows less
resin, which is what sets stiffness, expansion and stability. Working time decides whether the
hobber plinth can be poured continuously.

Requirements:
- mixed viscosity under about 1,000 mPa·s, ideally 200–600;
- pot life of at least 45–60 minutes at shop temperature, with a slow hardener for the plinth;
- Tg or HDT of at least about 50–60 °C after post-cure;
- low cure shrinkage, solvent-free, and a datasheet that permits fillers;
- 20–30 kg kits.

### Ranking

1. **Best performance: Resoltech 1050** (Castro Composites, Spain).
   - All six hardeners mix at 100:35 by weight and can be blended. Gel times per 70 mL at 23 °C:
     1059S 10 min, 1058S 26 min, 1056S 58 min, 1055S 2 h 38 min, 1054S 5 h, 1053S 14 h. Blending
     1056S with 1055S gives about 60–90 minutes, which suits the plinth.
   - Mixed viscosity 205 mPa·s with 1053S, the lowest of the candidates; expect more with faster
     hardeners.
   - Tg up to 55–78 °C depending on hardener, so post-curing pays.
   - Packs 1 + 0.35, 5 + 1.75 and 25 + 8.75 kg. The small kit is €50.81 excl. VAT (about €38/kg).
     The 25 kg price is not published: ask for a quote, and confirm delivery to Portugal.
   - [Castro Composites](https://www.castrocompositesshop.com/en/resins/1088-epoxy-resin-resoltech-1050-8431252093182.html),
     [catalogue excerpt](https://pdf.nauticexpo.com/pdf/resoltech/resoltech-1050/26759-92324.html),
     [datasheet excerpt](https://pdf.directindustry.es/pdf/resoltech/resoltech-1050/55297-425647.html).
2. **Best value, if its datasheet holds up: Harzspezialisten polymer casting system 3550**
   (Germany).
   - Made for polymer concrete casting, "medium" working time.
   - From €37.95 for about 6.5 kg, about €5.84/kg; all 163 kg would be roughly €950.
   - No published viscosity, Tg or mix ratio, and Portugal shipping is unknown. It becomes the pick
     only if its viscosity is under about 1,000 mPa·s and its working time is at least 45 minutes.
   - [Product](https://www.harzspezialisten.de/polymerverguss-epoxidharz-set).
3. **Budget and local: Sikafloor‑161** (Sika Portugal).
   - 100 % solids, low viscosity, a binder for quartz mortars, 79:21 by weight, 300–400 rpm mixing.
     Dry heat resistance +80 °C for up to 7 days, +100 °C for 12 h.
   - Pot life about 50 min at 10 °C, 25 min at 20 °C and 15 min at 30 °C. That works for the mill
     castings and the hobber column with 30 kg drill batches, mixed and placed within about 12
     minutes on a cool day. It is not suitable for the hobber plinth: a fast-reacting resin in a 1 t
     casting builds heat too quickly, and a warm day leaves no margin.
   - Kits are 10 and 20 kg in France and 4 and 24 kg in Morocco. No Portuguese retail price was
     found; ask a Sika distributor.
   - [Datasheet](https://prt.sika.com/dam/dms/pt01/m/sikafloor_-161.pdf).

Other candidates:

| Option | Notes | Link |
| --- | --- | --- |
| SKresin 3530 (Harzspezialisten) | Very thin, developed for filled systems; about 30 min pot life with Epohard 35, about €20/kg | [Harzspezialisten](https://www.harzspezialisten.de/polymerverguss-epoxidharz) |
| Breddermann E45GB (Germany) | Low-viscosity casting system, 45 min working time, advertised for mineral casting; no datasheet or product link found | search "Breddermann E45GB" |

Not recommended:
- **Nazza art, table and floor resins.** No viscosity, Tg, modulus or creep data.
  - Cristal de Alta Espessura (17 kg about €287, about €17/kg) is the closest: a slow deep-pour resin
    rated for 10 cm, so its exotherm is low. Deep-pour resins usually trade heat resistance and
    stiffness for clarity and creep more under load; test it only if Nazza supplies a datasheet.
  - Estratificados e Reparações (35.2 kg about €443) is described as high-viscosity.
  - Nazza may require a business registration, as its carbonate page does.
  - [Range](https://www.nazza.pt/32-resinas-epoxi),
    [Cristal](https://www.nazza.pt/resinas/248-resina-epoxi-cristal-de-alta-espessura-especial-fundicoes-e-mesas.html).
- **P&T EuroHarz EP 2:** 500–700 mPa·s, but about €32/kg
  ([Baustoffshop](https://www.baustoffshop.de/p-t-epoxidharz-euroharz-ep-2-2k-12-kg-geb-8-4-kg.html)).
- **Acorus Aquastone EPX:** 12–14 Pa·s, too thick ([Acorus](https://acorus.pt/produtos/resina-epoxi)).

Expected choice: Resoltech for the hobber plinth. For the smaller castings, 3550 if its data is
good, otherwise Sikafloor‑161. The coupon tests decide.

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

## Mixing and pouring

### Mixers

Use forced action. A drum concrete mixer tumbles the mix instead of shearing it, so a sticky,
low-slump resin mortar balls up and its fines stay unwetted.

| Use | Equipment | Batch |
| --- | --- | --- |
| Coupons, mill base, mill column, hobber column | Mixing drill of at least 1,400 W with a low gear (about 300–500 rpm) and a 140–160 mm double-spiral paddle, in a 60–90 L mortar tub; about €100–250 | ~25–30 kg |
| Hobber plinth | Forced-action horizontal trough or pan mixer, about 80–120 L, filled to about half for resin mortar; rent one or buy (below) | ~60–80 kg |
| Plinth fallback | Two drill mixers in parallel, crew of three | 2 × 30 kg |

Horizontal trough mortar mixers (a paddle shaft turning in a trough) are forced action, the type used
for epoxy screeds and polymer concrete. Options at Magmag
([range](https://magmag.pt/pt/misturadoras-de-argamassa)):

| Model | Capacity | Power | Price incl. VAT | Fit |
| --- | --- | --- | --- | --- |
| LIS 55 L, removable bucket | 55 L | not stated | €525 | Mixing type not stated. If it is a stand-mounted paddle in a bucket, it is a sturdier drill mixer (about 30–40 kg batches), and the removable bucket eases resin cleanup. [Product](https://magmag.pt/pt/misturadoras-de-argamassa/misturadora-de-argamassa-lis-55-l) |
| LIS MH 80 / MH 90 (the page uses both names) | 80 L | 1,100 W, 110 kg, gravity discharge, wheels | €1,250 | The size for the hobber plinth: about 60–80 kg batches, 13–17 for the plinth, about 2 h. [Product](https://magmag.pt/pt/misturadoras-de-argamassa/misturadora-horizontal-lis-mh-80-1100-w-80-l) |
| LIS MH 100/110 | 100 L | 1,500 W | €1,399 | More capacity than needed |
| LIS MH 120/140 | 120 L | 1,500 W | €1,632 | More capacity than needed |

Ask the seller (suporte@magmag.pt, +351 236 621 161) for:
- shaft speed (about 30–60 rpm is normal for trough mixers);
- the usable batch with a stiff mortar (usually 50–60 % of the trough), and whether the motor
  copes at that load;
- how the shaft ends are sealed, since resin in the bearings or seals ruins a mixer;
- the discharge: bottom gate or tilting.

Buy the MH 80 only if more large castings will follow the hobber. For a single 1 t pour, renting or
two drills is cheaper.

Any mixer used with epoxy:
- dedicated to resin;
- release agent on the trough before the first use;
- loaded to about half capacity with stiff mixes;
- never left with resin gelling inside. Clean it straight after the last batch: run a load of dry
  sand through to scour it, then wipe with acetone.

### Batches

- **Working time.** Each batch is mixed, placed and vibrated within about half the resin's pot life.
- **No cold joints.** Each layer is covered by the next batch before it gels. Every casting is one
  continuous pour.
- **Rate.** With batches weighed in advance, one drill mixer does a 30 kg batch about every
  7 minutes (about 250 kg/h); a pan or trough mixer about 500 kg/h. Mixing takes 3–4 minutes at low
  speed, scraping the tub.
- **Order.** Resin with hardener first, then filler and fine sand into a paste, then the coarse
  fractions.

| Casting | Wet mix | Drill mixer, ~30 kg | Trough or pan mixer, ~80 kg | Pour time |
| --- | --- | --- | --- | --- |
| Mill base | 166 kg | 6 × ~28 kg | — | ~45 min |
| Mill column | 99 kg | 4 × ~25 kg | — | ~30 min |
| Hobber column | ~62 kg | 3 × ~21 kg | — | ~20 min |
| Hobber plinth | ~1,016 kg | 34 × 30 kg | 13 × ~78 kg | 2–2.5 h with a trough mixer or two drills; ~4 h with one drill |

The masses exclude waste. Prepare one extra batch's worth of dry kits per casting, and fill the
coupon moulds from the last batch. On the plinth, each 30 kg batch adds a layer about 18 mm deep,
so the surface stays wet; the crew is two mixing and one placing and vibrating.

### Preparation and equipment

- **Aggregate kits:** dry-blend each batch's aggregate into its own labelled bucket the day before,
  from dried material, and keep them sealed, warm and dry.
- **Resin and hardener:** weighed per batch into separate cups, never premixed.
- **Scales:** a platform scale (about 150 kg, ±50 g) for aggregate, and a bench scale (about 5 kg,
  ±1 g) for resin, hardener and silane.
- **Compaction:** clamp-on form vibrators for the molds, and a small vibrating table for coupons.
- **Shop:** a steady 18–25 °C during the pour and cure; thermometer and timer.
- **Log:** batch number, time, temperature and coupon number.
- **Protection:** nitrile gloves, goggles, and a P3 mask when handling filler.

These are the batch-schedule inputs the kits list as Pending: usable mixer capacity, mixing and
transfer time, and working time. They enter the shop recipe once the mixer and resin are chosen.

## Improving performance

Roughly in order of payoff:
1. **Packing** (see Packing). The biggest lever.
2. **Vibration.** Form vibrators or a table, pouring in layers. An unvibrated mix keeps air and
   needs more resin to flow.
3. **Bone-dry aggregate.** Under 0.1 % moisture; water ruins the bond. Aggregate at about 25–30 °C
   also lowers resin viscosity and improves wetting, but shortens the pot life.
4. **Silane coupling agent.** About 0.5–1 % of the resin weight, of an epoxy- or amino-functional
   silane (GPTMS or APTES). It bonds the resin chemically to the quartz and noticeably raises
   strength and moisture resistance. Professional systems use it.
5. **Mixing method.** Paste first, then coarse fractions; forced action, not a drum.
6. **Controlled cure, then post-cure.** Cure at a steady 20–25 °C, then post-cure as the resin's
   datasheet says (for example a few hours at 40–60 °C in an insulated tent). Post-curing raises
   heat resistance and reduces long-term creep.
7. **Basalt for the coarse fractions** later: stiffer rock, closer to steel's expansion.
8. **A witness coupon from every batch**, so a weak batch shows up.

Items 1–4 together can probably move the mix from about 25 GPa to over 30 GPa, into EPUSELF
territory.

## Checks on arrival and coupon tests

Check each grade:
1. **Moisture.** Weigh about 500 g, dry it at about 105 °C for an hour, reweigh. The loss must be
   under 0.1 %; otherwise dry the whole batch before use.
2. **Cleanliness.** Shake some in a jar of water. A muddy cloud means clay or dust, which weakens
   the bond.
3. **Grading.** Sieve against the stated band, and measure the share below 0.1 mm in the 0–1 mm sand.

The moisture check matters most for the agricultural lime, which is often not dried.

Coupon tests compare:
- **aggregate:** all quartz against quartz sand with basalt coarse, each with and without the 2–4 %
  Nazza 5 µm addition;
- **binder:** Resoltech 1050 with 1056S/1055S, Harzspezialisten 3550 and Sikafloor‑161 on the same
  aggregate blend, with and without silane;
- **reference:** EPUSELF 140/8B, cast alongside if bought.

Coupon purchases, beside Vetonek order 1: the Resoltech 1.35 kg kit with each hardener (about €51
each), the 3550 set of about 6.5 kg (about €38), the smallest Sikafloor‑161 kit, a small pack of
GPTMS or APTES silane, and EPUSELF 140/8B if Rampf quotes it.

For each mix, measure:
- dry packed density of the blend, tuning the split toward maximum packing;
- the lowest resin content that still flows under vibration;
- the exotherm of a block about 150 mm thick;
- density (target 2.2–2.4) and voids on cut coupons;
- stiffness, and expansion on a bar;
- heat resistance after post-cure.

Choose the binder by measured stiffness per euro, as long as its working time covers the plinth
pour.

These results replace the unresolved compound inputs in `CastingShopRecipe`: maximum aggregate size,
mix fractions, density, and the section-thickness and lift limits.
