package materia.assembly;

/** A mechanical end stop expressed as a contact response. */
@:wire typedef AssemblyJointStop = {
	/** Positive response time constant in seconds. */
	@:id(1) var timeConstantSec:Float;
	/** Positive dimensionless damping ratio. */
	@:id(2) var dampingRatio:Float;
}
