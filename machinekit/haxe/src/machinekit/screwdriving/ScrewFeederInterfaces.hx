package machinekit.screwdriving;

import machinekit.component.PortInterface;

/** Separate commissioned feeder control service, independent of tightening program signals. */
class ScrewFeederInterfaces {
	public static function control():PortInterface return Plug("screw-feeder-discrete-io",8);
}
