package machinekit.pneumatic;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** A vacuum source delivering up to `ratedVacuumKpa` at `outputPort`. */
class VacuumSourceFacet implements ComponentFacet {
	public final ratedVacuumKpa:Null<Float>;
	public final outputPort:String;

	public function new(ratedVacuumKpa:Null<Float>, outputPort:String) {
		this.ratedVacuumKpa = ratedVacuumKpa;
		this.outputPort = outputPort;
	}

	public function check(component:MachineComponent):Void {
		component.port(outputPort);
	}

	public function describe():String return 'vacuum source $ratedVacuumKpa kPa at $outputPort';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<VacuumSourceFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, VacuumSourceFacet)) return cast facet;
		return null;
	}
}
