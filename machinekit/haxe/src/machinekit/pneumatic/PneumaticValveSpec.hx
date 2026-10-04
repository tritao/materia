package machinekit.pneumatic;

/** Typed spool behavior exposed by a directional valve component. */
interface PneumaticValveSpec {
	function isDoubleSolenoid():Bool;
	function defaultsToA():Bool;
}
