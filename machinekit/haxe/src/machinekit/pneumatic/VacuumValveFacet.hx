package machinekit.pneumatic;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** A valve switching vacuum on the signal at `controlPort`. */
class VacuumValveFacet implements ComponentFacet {
	public final controlPort:String;

	public function new(controlPort:String) {
		this.controlPort = controlPort;
	}

	public function check(component:MachineComponent):Void {
		component.port(controlPort);
	}

	public function describe():String return 'vacuum valve at $controlPort';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<VacuumValveFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, VacuumValveFacet)) return cast facet;
		return null;
	}
}
