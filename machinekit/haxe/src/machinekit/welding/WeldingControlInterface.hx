package machinekit.welding;

/** How a controller drives a welding power source. */
enum WeldingControlInterface {
	/** Trigger and enable on digital I/O, setpoints as 0-10 V. */
	AnalogIo;
	/** Registers over Modbus TCP. */
	ModbusTcp;
}
