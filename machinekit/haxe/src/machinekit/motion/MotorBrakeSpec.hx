package machinekit.motion;

/** Physical spring-applied brake data, independent of the controller release policy. */
interface MotorBrakeSpec {
	function holdingTorque():Float;
	function releaseDelaySeconds():Float;
	function applicationDelaySeconds():Float;
}
