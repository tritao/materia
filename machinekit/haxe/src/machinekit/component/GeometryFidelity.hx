package machinekit.component;

/** How faithfully a component's preview geometry represents the real part. */
enum abstract GeometryFidelity(String) to String {
	/** The preview is a placeholder of the part's size: one primitive solid such as a box, cylinder or ring. */
	var Envelope = "envelope";

	/** Recognisable features in generic proportions. */
	var Nominal = "nominal";

	/** Key dimensions from a cited standard or datasheet. */
	var Datasheet = "datasheet";

	/** The supplier's own CAD. */
	var Supplier = "supplier";
}
