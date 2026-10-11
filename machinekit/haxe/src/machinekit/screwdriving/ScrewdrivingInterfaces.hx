package machinekit.screwdriving;

import machinekit.component.PortInterface;

/** Service interfaces of the assumed screwdriving equipment. */
class ScrewdrivingInterfaces {
	public static function mains():PortInterface return Plug("mains-230v-1ph", 3);
	public static function tool():PortInterface return Plug("screwdriver-tool", 12);
	public static function control(interfaceKind:TighteningControlInterface = DiscreteIO):PortInterface return switch interfaceKind {
		case DiscreteIO: Plug("screwdriver-io", 12);
		case OpenProtocol: Plug("screwdriver-open-protocol-ethernet", 8);
	};
	public static function vacuum():PortInterface return PushIn(4);
}
