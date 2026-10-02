package machinekit.component;

@:wire enum PortKind {
	@:id(1) Pneumatic;
	@:id(2) Vacuum;
	@:id(3) ElectricalPower;
	@:id(4) Signal;
	/** Shielding gas, which does not mix with compressed air. */
	@:id(5) Gas;
	/** Consumable welding wire, fed from a spool through a liner. */
	@:id(6) Wire;
}
