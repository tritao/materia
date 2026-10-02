package machinekit.welding;

import machinekit.component.PortInterface;

/** The service interfaces welding equipment shares, so a torch, feeder, power source and cylinder
 * mate only through the connections a real set-up has.
 */
class WeldingInterfaces {
	/** Mains cord: line, neutral and earth. */
	public static function mains():PortInterface return Plug("mains-230v-1ph", 3);

	/** Weld current on a 35 mm² cable, from a power source socket or through a feeder to the torch. */
	public static function weldCable():PortInterface return Plug("weld-cable-35", 1);

	/** Shielding gas hose. */
	public static function gas():PortInterface return Thread("G1/4");

	/** Wire liner for 1.0 to 1.2 mm wire. */
	public static function wireLiner():PortInterface return Plug("wire-liner-1.2", 1);

	/** Robot-to-welder I/O: trigger, enable, setpoints and feedback. */
	public static function control():PortInterface return Plug("welder-io", 12);
}
