package machinekit.fluid;

/** Typed mechanical and force data shared by fluid-powered cylinder components. */
interface CylinderSpec {
	function boreMm():Float;
	function rodMm():Float;
	function strokeMm():Float;
	function ratedSpeedMmPerSecond():Float;
	function extendForce(pressurePa:Float):Float;
	function retractForce(pressurePa:Float):Float;
}
