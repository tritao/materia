package machinekit.robotics;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** A tool changer lock worked through `inletPort`. */
class ChangerLockFacet implements ComponentFacet {
	public final inletPort:String;

	public function new(inletPort:String) {
		this.inletPort = inletPort;
	}

	public function check(component:MachineComponent):Void {
		component.port(inletPort);
	}

	public function describe():String return 'changer lock at $inletPort';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<ChangerLockFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, ChangerLockFacet)) return cast facet;
		return null;
	}
}
