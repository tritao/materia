# Generic end effector (EOAT) example

`EndEffectorExample.build()` constructs one robot-side adapter plate and changer
master, plus two interchangeable suction-cup tools. Each selected configuration
contains only its tool-side changer, bar, manifold, vacuum generator, and cup.
The flange object sizes the adapter plate but does not enter the EOAT BOM.

`SchmalzEndEffectorExample.build()` is a fixed, source-backed EOAT slice. It
uses the [SAF 40 cup](https://www.schmalz.co.jp/en-jp/products/vacuum-technology-for-automation-301607/vacuum-components-301608/vacuum-suction-cups-301609/flat-suction-cups-round-301610/flat-suction-cups-saf-302308/10.01.01.11401),
[SBP 05 ejector](https://www.schmalz.com/en/vacuum-technology-for-automation/vacuum-components/vacuum-generators/basic-ejectors/basic-ejectors-sbp-307660/10.02.01.00563/),
[G1/4 to 4 mm fitting](https://www.schmalz.com/en-gb/products/vacuum-technology-for-automation-301607/vacuum-components-301608/filters-and-connections-308965/hoses-and-connections-309034/screw-in-push-fittings-309092/10.08.02.00203),
and a 0.2 m cut of [VSL 4-2 PU hose](https://www.schmalz.co.jp/en-jp/products/vacuum-technology-for-automation-301607/vacuum-components-301608/filters-and-connections-308965/hoses-and-connections-309034/vacuum-compressed-air-hoses-vsl-309035/10.07.09.00001).
Catalog rows retain product identifiers and source URLs. The cup's 1,150 mm²
effective area is **derived** from Schmalz's theoretical 69 N at 60 kPa; it is
not a measured sealed area for a particular workpiece. The ejector's 85%
evacuation rating is represented as an 85 kPa upper bound using a conservative
100 kPa ambient reference. A capacity check still needs a guaranteed minimum
vacuum at the cup and measured friction for the workpiece surface. The product
pages give mass but not centre of mass or inertia, so both are modeled from
simple envelopes. These approximations must be verified for final payload
sizing. The BOM material is the dominant material family for each part; the cup
also has an aluminium nipple. The hose is included in the BOM and mass but not
in collision geometry.

The master and tool halves bridge air and signal channels. Air runs through the
tool-side manifold and vacuum generator to the cup. `upstream()` traces both
services back to the robot-side inputs. The components are generic geometry and
ports, with no vendor catalog entry.

Run the example checks with:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/eoat/haxeon.json
```
