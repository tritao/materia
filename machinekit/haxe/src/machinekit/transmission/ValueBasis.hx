package machinekit.transmission;

/** Where an engineering value comes from; the IDs are part of the wire schema. */
@:wire enum ValueBasis {
	@:id(1) Derived;
	@:id(2) Catalog;
	@:id(3) Stated;
	@:id(4) Assumed;
}
