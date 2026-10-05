package machinekit.motion;

/** Continuous DC output and regenerative absorption capacity. Both power limits
 * apply to the whole source, shared by every outlet, rather than to each wire.
 */
typedef ElectricalSupplyRating = {
	var voltage:machinekit.power.DcVoltage;
	var current:machinekit.power.DcCurrent;
	var regenerativePower:Float;
}
