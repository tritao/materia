package machinekit.milling;

import machinekit.assembly.MachineAssembly;

/** A spindle the Z head carries: the datums the layouts size the head and work envelope from, and the
 * assembly it installs. Its gauge line sits `seat()` below the head's lower face, on the layout's spindle axis.
 */
interface MillSpindlePackage {
	/** Gauge line to the head's lower clamping face, mm. */
	function seat():Float;
	/** Spindle body diameter where the head clamps it, mm. */
	function diameter():Float;
	/** Head block height that clamps this spindle and carries its drive, mm. */
	function headHeight():Float;
	function maximumRpm():Float;
	/** Installed spindle for `l`: a `spindle` member whose frame is the gauge line and a `holder` member the tool seats in. */
	function assembly(l:MillZAxisLayout):MachineAssembly;
}
