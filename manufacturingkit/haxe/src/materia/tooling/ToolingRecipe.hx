package materia.tooling;

import materia.tooling.panel.PanelStockSpec;
import materia.tooling.casting.CoreStockSpec;

/** Initial manufacturing inputs; panel joints and load models are added with their planners.
 * All lengths are mm. Evidence references record provenance, not automatic qualification.
 */
typedef ToolingRecipe = {
	var id:String;
	var revision:Int;
	var panel:PanelStockSpec;
	var core:CoreStockSpec;
	var jointClearance:Float;
	var linerThickness:Float;
	var routerDiameter:Float;
	var minimumLigament:Float;
	var resinSpec:String;
	var releaseSpec:String;
	var adhesiveSpec:String;
	var processEvidence:Array<String>;
}
