package machinekit.component;

/** Runtime capability indicated by named physical service ports. */
enum RuntimePortIntent {
	Gripper(openPort:String, closePort:String);
	VacuumActuator(inletPort:String);
	VacuumValve(controlPort:String);
	VacuumPressureSensor(vacuumPort:String, signalPort:String);
	ChangerLock(inletPort:String);
}
