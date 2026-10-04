package processkit.tool;

import robotkit.tool.*;

/** Why a welder stopped, as the `tool_weld` sensor's fault code carries it. `WeldSensor.faultMessage` puts it in words. */
enum abstract WeldFault(Int) from Int to Int {
  var None = 0;
  /** The arc was commanded and did not light within the timeout. */
  var NoArc = 1;
  /** The arc went out while it was commanded on: the torch was pulled away, or the supply dropped out. */
  var ArcLost = 2;
  /** The arc was switched off with the wire still feeding while it touched the work, so it froze in the pool. */
  var WireStuck = 3;
}
