package machinekit.screwdriving;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** Spindle capacity in Nm and rad/s, feed travel in mm and spring rate in N/mm. */
class ScrewdriverFacet implements ComponentFacet {
	public final tcpConnector:String;
	public final controlPort:String;
	public final maxTorque:Float;
	public final maxSpeed:Float;
	public final strokeMm:Float;
	public final springRate:Float;
	public final bit:ScrewdriverBit;
	/** Joint and encoder names relative to the carrier's assembly scope. */
	public final feedJoint:String;
	public final spindleJoint:String;
	public final strokeEncoder:String;

	public function new(tcpConnector:String, controlPort:String, maxTorque:Float, maxSpeed:Float, strokeMm:Float, springRate:Float,
		bit:ScrewdriverBit, feedJoint:String, spindleJoint:String, strokeEncoder:String) {
		this.tcpConnector = tcpConnector; this.controlPort = controlPort;
		this.bit = bit; this.feedJoint = feedJoint; this.spindleJoint = spindleJoint; this.strokeEncoder = strokeEncoder;
		this.maxTorque = maxTorque; this.maxSpeed = maxSpeed; this.strokeMm = strokeMm; this.springRate = springRate;
	}
	public function check(component:MachineComponent):Void {
		for (value in [maxTorque, maxSpeed, strokeMm, springRate])
			if (!Math.isFinite(value) || value <= 0) throw 'Screwdriver "${component.designation}" needs finite positive capacity, stroke and spring rate';
		if (bit == null || feedJoint == null || feedJoint.length == 0 || spindleJoint == null || spindleJoint.length == 0 ||
			strokeEncoder == null || strokeEncoder.length == 0) throw 'Screwdriver "${component.designation}" needs its bit, joints and stroke encoder';
		component.connector(tcpConnector);
		var port = component.port(controlPort);
		if (port.kind != Signal || port.role != Consumer) throw 'Screwdriver "${component.designation}" needs a signal control inlet';
	}
	public function describe():String return 'screwdriver $maxTorque Nm, $maxSpeed rad/s, $strokeMm mm, $springRate N/mm';

	public static function of(component:MachineComponent):Null<ScrewdriverFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, ScrewdriverFacet)) return cast facet;
		return null;
	}
}
