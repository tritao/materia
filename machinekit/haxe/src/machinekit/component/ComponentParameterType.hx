package machinekit.component;

import machinekit.catalog.CatalogIndex;

/** Supported editable inputs for a single machine part. */
enum ComponentParameterType {
	Scalar;
	Length;
	Angle;
	Count;
	Bool;
	Choice(options:Array<String>);
	CatalogDesignation(index:CatalogIndex);
}
