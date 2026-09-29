package machinekit.component;

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
}
