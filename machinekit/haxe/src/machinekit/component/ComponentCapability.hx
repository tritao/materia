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
	 * the torch, and `controlPort` is the signal inlet that starts and stops the arc. */
	ArcTorch(tcpConnector:String, controlPort:String);
	/** A welding power source: the processes it runs, its rated current in amperes, and how a
	 * controller drives it. */
	WeldingSupply(processes:Array<WeldingProcess>, maxCurrentA:Float, controlInterface:WeldingControlInterface);
}
