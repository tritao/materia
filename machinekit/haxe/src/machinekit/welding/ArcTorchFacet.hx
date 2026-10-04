package machinekit.welding;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/**
 * An arc torch: `tcpConnector` is the wire tip at nominal stickout with +Z along the wire out of the torch,
 * `controlPort` is the signal inlet that starts and stops the arc, and `stickoutMm` is that stickout: the
 * wire's extension past the contact tip, which the arc model needs.
 */
class ArcTorchFacet implements ComponentFacet {
	public final tcpConnector:String;
	public final controlPort:String;
	public final stickoutMm:Float;

	public function new(tcpConnector:String, controlPort:String, stickoutMm:Float) {
		this.tcpConnector = tcpConnector;
		this.controlPort = controlPort;
		this.stickoutMm = stickoutMm;
	}

	public function check(component:MachineComponent):Void {
		if (!(stickoutMm > 0)) throw 'Arc torch on "${component.designation}" needs a positive stickout';
		component.connector(tcpConnector);
		component.port(controlPort);
	}

	public function describe():String return 'arc torch $tcpConnector, $controlPort, $stickoutMm mm';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<ArcTorchFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, ArcTorchFacet)) return cast facet;
		return null;
	}
}
