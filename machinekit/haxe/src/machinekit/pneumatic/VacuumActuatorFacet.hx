package machinekit.pneumatic;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** A part that makes vacuum while `inletPort` is supplied, so switching that supply switches the vacuum. */
class VacuumActuatorFacet implements ComponentFacet {
	public final inletPort:String;

	public function new(inletPort:String) {
		this.inletPort = inletPort;
	}

	public function check(component:MachineComponent):Void {
		component.port(inletPort);
	}

	public function describe():String return 'vacuum actuator at $inletPort';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<VacuumActuatorFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, VacuumActuatorFacet)) return cast facet;
		return null;
	}
}
