package machinekit.robotics;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** Gripper jaws with `strokeMm` of travel, opened and closed through two ports. */
class GripFacet implements ComponentFacet {
	public final strokeMm:Float;
	/** Grip force in N, when known. */
	public final forceN:Null<Float>;
	public final openPort:String;
	public final closePort:String;

	public function new(strokeMm:Float, forceN:Null<Float>, openPort:String, closePort:String) {
		this.strokeMm = strokeMm;
		this.forceN = forceN;
		this.openPort = openPort;
		this.closePort = closePort;
	}

	public function check(component:MachineComponent):Void {
		component.port(openPort);
		component.port(closePort);
	}

	public function describe():String return 'grip $strokeMm mm $forceN N, $openPort/$closePort';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<GripFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, GripFacet)) return cast facet;
		return null;
	}
}
