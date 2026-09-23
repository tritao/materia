package cadkit;

import cadkit.modeling.Vector;

/** Volume-weighted physical properties for a positive-volume CAD shape. */
class PhysicalProperties {
	public final volume:Float;
	public final surfaceArea:Float;
	public final centerOfMass:Vector;

	public function new(volume:Float, surfaceArea:Float, centerOfMass:Vector) {
		this.volume = volume;
		this.surfaceArea = surfaceArea;
		this.centerOfMass = centerOfMass;
	}

	/** Return mass in the matching units for a density per cubic kernel unit. */
	public function mass(density:Float):Float {
		if (!Math.isFinite(density) || density < 0)
			throw "density must be finite and nonnegative";
		return volume * density;
	}
}
