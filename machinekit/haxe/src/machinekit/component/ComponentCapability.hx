package machinekit.component;

import machinekit.welding.WeldingControlInterface;
import machinekit.welding.WeldingProcess;

/** Physical and runtime capabilities declared by a component's constructor. */
enum ComponentCapability {
	Coupling(key:String, connector:String);
	Suction(effectiveAreaMm2:Null<Float>, ratedMomentNm:Null<Float>, vacuumPort:String, contactConnector:String);
	Grip(strokeMm:Float, forceN:Null<Float>, openPort:String, closePort:String);
	VacuumSource(ratedVacuumKpa:Null<Float>, outputPort:String);
	VacuumActuator(inletPort:String);
	VacuumValve(controlPort:String);
	VacuumPressureSensor(vacuumPort:String, signalPort:String);
	ChangerLock(inletPort:String);
	/** An arc torch: `tcpConnector` is the wire tip at nominal stickout with +Z along the wire out of
	 * the torch, and `controlPort` is the signal inlet that starts and stops the arc. `stickoutMm` is that
	 * stickout: the wire's extension past the contact tip, which the arc model needs. */
	ArcTorch(tcpConnector:String, controlPort:String, stickoutMm:Float);
	/** A welding power source: the processes it runs, its rated current in amperes, how a
	 * controller drives it, and its efficiency (the fraction of the mains power it delivers to the arc). */
	WeldingSupply(processes:Array<WeldingProcess>, maxCurrentA:Float, controlInterface:WeldingControlInterface,
		efficiency:Float);
	/** A wire feeder: the diameter of the wire it feeds in millimetres, and its top wire speed in
	 * metres per minute. */
	WireFeed(wireDiameterMm:Float, maxSpeedMPerMin:Float, depositionEfficiency:Float);
	/** A work clamp (the return of the weld circuit): where the weld circuit returns through the workpiece. `leadPort` is the
	 * inlet the work lead from the power source plugs into, and `contactConnector` is where the
	 * clamp meets the work (mate it to the workpiece). */
	WorkReturn(leadPort:String, contactConnector:String);
	/**
	 * A planar scanner: it sweeps `rayCount` rays round the horizontal plane of its connector
	 * `scanConnector` (zero bearing along the connector's +X), seeing out to `maxRangeMeters`, and
	 * scans `rateHz` times a second.
	 */
	PlanarScanner(scanConnector:String, rayCount:Int, maxRangeMeters:Float, rateHz:Float);
}
