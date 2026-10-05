package machinekit.pressbrake;

/** A component's declared tooling-lock interface, independent of runtime actuation. */
enum ToolClampControl {
	Manual;
	Powered(lockPort:String, unlockPort:String, lockedPort:String, airPort:String);
}
