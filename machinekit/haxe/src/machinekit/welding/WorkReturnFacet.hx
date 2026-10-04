package machinekit.welding;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** A work clamp, the return of the weld circuit: `leadPort` is the inlet the work lead from the power source plugs into, and `contactConnector` is where the clamp meets the work (mate it to the workpiece). */
class WorkReturnFacet implements ComponentFacet {
	public final leadPort:String;
	public final contactConnector:String;

	public function new(leadPort:String, contactConnector:String) {
		this.leadPort = leadPort;
		this.contactConnector = contactConnector;
	}

	public function check(component:MachineComponent):Void {
		component.port(leadPort);
		component.connector(contactConnector);
	}

	public function describe():String return 'work return $leadPort to $contactConnector';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<WorkReturnFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, WorkReturnFacet)) return cast facet;
		return null;
	}
}
