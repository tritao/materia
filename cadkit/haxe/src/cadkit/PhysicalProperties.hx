package cadkit;

import cadkit.modeling.Vector;

/** Volume-weighted physical properties for a positive-volume CAD shape. */
class PhysicalProperties {
	public final volume:Float;
	public final surfaceArea:Float;
	public final centerOfMass:Vector;
	/** Centroidal inertia for unit density, in length^5 units. */
	public final inertia:InertiaTensor;

	public function new(volume:Float, surfaceArea:Float, centerOfMass:Vector, inertia:InertiaTensor) {
		this.volume = volume;
		this.surfaceArea = surfaceArea;
		this.centerOfMass = centerOfMass;
		this.inertia = inertia;
	}

	/** Return mass in the matching units for a density per cubic kernel unit. */
	public function mass(density:Float):Float {
		if (!Math.isFinite(density) || density < 0)
			throw "density must be finite and nonnegative";
		return volume * density;
	}

	/** Return centroidal inertia for density per cubic kernel unit. */
	public function inertiaAtDensity(density:Float):InertiaTensor {
		if (!Math.isFinite(density) || density < 0)
			throw "density must be finite and nonnegative";
		return inertia.scaled(density);
	}
}
