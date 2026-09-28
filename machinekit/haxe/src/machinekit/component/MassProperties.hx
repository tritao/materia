package machinekit.component;

import cadkit.modeling.Vector;
import cadkit.InertiaTensor;

/** Origin of a component's mass and centre of mass estimate. */
enum MassSource {
	Computed(detail:ComponentDetail);
	Declared;
}

/** Mass in kg, centre in mm, and centroidal inertia in kg mm² in the component frame. */
class MassProperties {
	public final mass:Float;
	public final centreOfMass:Vector;
	public final source:MassSource;
	/** Null when a declared mass has no declared inertia tensor. */
	public final inertia:Null<InertiaTensor>;

	public function new(mass:Float, centreOfMass:Vector, source:MassSource, ?inertia:InertiaTensor) {
		this.mass = mass;
		this.centreOfMass = centreOfMass;
		this.source = source;
		this.inertia = inertia;
	}
}
