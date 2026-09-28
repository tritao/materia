package machinekit.component;

import cadkit.modeling.Vector;

/** Origin of a component's mass and centre of mass estimate. */
enum MassSource {
	Computed(detail:ComponentDetail);
	Declared;
}

/** Mass in kg and centre of mass in mm in the component's own frame. */
class MassProperties {
	public final mass:Float;
	public final centreOfMass:Vector;
	public final source:MassSource;

	public function new(mass:Float, centreOfMass:Vector, source:MassSource) {
		this.mass = mass;
		this.centreOfMass = centreOfMass;
		this.source = source;
	}
}
