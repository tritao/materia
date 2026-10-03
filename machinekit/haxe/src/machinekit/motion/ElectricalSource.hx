package machinekit.motion;

/** A part that states the DC voltage available at a supply port. */
interface ElectricalSource {
	function outputVoltage(port:String):Float;
}
