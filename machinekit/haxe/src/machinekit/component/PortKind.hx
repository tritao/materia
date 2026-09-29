package machinekit.component;

@:wire enum PortKind {
	@:id(1) Pneumatic;
	@:id(2) Vacuum;
	@:id(3) ElectricalPower;
	@:id(4) Signal;
}
