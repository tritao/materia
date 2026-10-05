package machinekit.fluid;

/** A part that states gauge pressure in Pa at a fluid supply port. */
interface PressureSource {
	function outputPressure(port:String):Float;
}
