package machinekit.component;

/** What a component recipe stands for, for catalogues and part pickers. */
enum abstract ComponentKind(String) to String {
	/** A reusable parametric recipe, sized by its inputs or a standard catalogue. */
	var Generic = "generic";

	/** A named supplier's product, with that supplier's data. */
	var Supplier = "supplier";

	/** A drawing, study or reference envelope: geometry to check against, not a part to buy or make. */
	var Reference = "reference";

	/** A part designed for one machine (an example's frame, guard or bracket). */
	var MachinePart = "machine-part";
}
