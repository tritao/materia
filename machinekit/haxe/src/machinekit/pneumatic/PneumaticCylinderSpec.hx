package machinekit.pneumatic;

/** Typed mechanical and force data exposed by a pneumatic cylinder component. */
interface PneumaticCylinderSpec {
	function boreMm():Float;
	function rodMm():Float;
	function strokeMm():Float;
	function ratedSpeedMmPerSecond():Float;
	function extendForce(pressurePa:Float):Float;
	function retractForce(pressurePa:Float):Float;
}
