package machinekit.catalog;

/** How the listed dimensions relate to the cited source. */
enum DimensionKind {
	Nominal;
	Minimum;
	Maximum;
	Mixed;
	Unverified;
}

/** Scope of the generated geometry and interfaces. */
enum Conformance {
	ExactInterface;
	NominalEnvelope;
	GenericApproximation;
}

/** Provenance and limits of one catalog entry. A null edition means it has not been verified. */
typedef CatalogMetadata = {
	var source:String;
	var standard:Null<String>;
	var standardEdition:Null<String>;
	var dimensionKind:DimensionKind;
	var conformance:Conformance;
	/** Additional sources when one row combines independent standard and product tables. */
	@:optional var sources:Array<String>;
	/** Fields checked against `source` and optional `sources` when the row remains partial. */
	@:optional var verifiedFields:Array<String>;
}
