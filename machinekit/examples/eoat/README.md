# Generic end effector (EOAT) example

`EndEffectorExample.build()` constructs one robot-side adapter plate and changer
master, plus two interchangeable suction-cup tools. Each selected configuration
contains only its tool-side changer, bar, manifold, vacuum generator, and cup.
The flange object sizes the adapter plate but does not enter the EOAT BOM.

The master and tool halves bridge air and signal channels. Air runs through the
tool-side manifold and vacuum generator to the cup. `upstream()` traces both
services back to the robot-side inputs. The components are generic geometry and
ports, with no vendor catalog entry.

Run the example checks with:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/eoat/haxeon.json
```
