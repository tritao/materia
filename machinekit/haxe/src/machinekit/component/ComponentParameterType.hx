package machinekit.component;

/** Supported editable inputs for a single machine part. */
enum ComponentParameterType {
	Length;
	Angle;
	Count;
	Bool;
	Choice(options:Array<String>);
	CatalogDesignation(catalog:String);
}
