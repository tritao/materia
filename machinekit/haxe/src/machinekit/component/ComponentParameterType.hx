package machinekit.component;

import machinekit.catalog.CatalogIndex;

/** Supported editable inputs for a single machine part. */
enum ComponentParameterType {
	Scalar;
	Length;
	Angle;
	Count;
	Bool;
	Text;
	Choice(options:Array<String>);
	CatalogDesignation(index:CatalogIndex);
	Optional(inner:ComponentParameterType);
}
