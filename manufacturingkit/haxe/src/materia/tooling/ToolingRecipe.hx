package materia.tooling;

import materia.tooling.panel.PanelStockSpec;
import materia.tooling.casting.CoreStockSpec;

/** Initial manufacturing inputs; panel joints and load models are added with their planners.
 * All lengths are mm. Evidence references record provenance, not automatic qualification.
 */
enum abstract CoreWireAxis(String) to String {
	var X = "x";
	var Y = "y";
	var Z = "z";
}

typedef ToolingHardware = {
	var insertBoreDiameter:Float;
	var insertClearanceDiameter:Float;
	var insertScrewDiameter:Float;
	var insertThreadSpec:String;
	var insertEngagement:Float;
	var plugDepth:Float;
	var plugMaterial:String;
	var sealMargin:Float;
	var sealMaterial:String;
	var minimumBridgeWidth:Float;
	var locatingPinDiameter:Float;
	var supportScrewDiameter:Float;
	var supportClearanceDiameter:Float;
	var secondaryPinTravel:Float;
	var padDepth:Float;
	var labelOffset:Float;
}

typedef ToolingRecipe = {
	var id:String;
	var revision:Int;
	var panel:PanelStockSpec;
	var core:CoreStockSpec;
	var jointClearance:Float;

	/** Total extra opening consumed by coatings and machining uncertainty. */
	var jointLiningAllowance:Float;

	var jointMachiningAllowance:Float;
	var hardware:ToolingHardware;
	var coreWireAxis:CoreWireAxis;
	var linerThickness:Float;
	var routerDiameter:Float;
	var minimumLigament:Float;
	var resinSpec:String;
	var releaseSpec:String;
	var adhesiveSpec:String;
	var processEvidence:Array<String>;
}
