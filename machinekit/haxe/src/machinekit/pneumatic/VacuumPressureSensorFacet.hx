package machinekit.pneumatic;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** A sensor reading the vacuum at `vacuumPort` out on `signalPort`. */
class VacuumPressureSensorFacet implements ComponentFacet {
	public final vacuumPort:String;
	public final signalPort:String;

	public function new(vacuumPort:String, signalPort:String) {
		this.vacuumPort = vacuumPort;
		this.signalPort = signalPort;
	}

	public function check(component:MachineComponent):Void {
		component.port(vacuumPort);
		component.port(signalPort);
	}

	public function describe():String return 'vacuum sensor $vacuumPort to $signalPort';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<VacuumPressureSensorFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, VacuumPressureSensorFacet)) return cast facet;
		return null;
	}
}
