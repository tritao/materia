package machinekit.pneumatic;

/** A part that states regulated gauge pressure in Pa at an air supply port. */
interface PressureSource {
	function outputPressure(port:String):Float;
}
