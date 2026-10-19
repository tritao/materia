package materia.tooling.panel;

import materia.tooling.ToolingEnums.CoatedFace;
import materia.tooling.ToolingEnums.StockDirection;

/** Measured stock for a tooling recipe; all lengths are mm.
 * materialSpec is a supplier/design reference, not an implicit MaterialLibrary registration.
 * The selected contact face must be oriented explicitly by the later panel planner.
 */
typedef PanelStockSpec = {
	var materialSpec:String;
	var nominalThickness:Float;
	var measuredThickness:Float;
	var thicknessVariation:Float;
	var width:Float;
	var height:Float;
	/** positive-z, negative-z, both or none, in the future panel's local frame. */
	var coatedFace:CoatedFace;
	/** Rectangular blank rotations supported by the sheet bridge: 0 and/or 90 degrees. */
	var allowedRotations:Array<Int>;
	/** free or width-aligned; width-aligned permits only a zero-degree blank rotation. */
	var directionConstraint:StockDirection;
}
