package machinekit.component;

typedef ComponentPort = {
	var name:String;
	var kind:PortKind;
	var role:PortRole;
	var iface:PortInterface;
	var required:Bool;
	?connector:String;
}
