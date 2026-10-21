package materia.tooling.panel;

import materia.casting.CastingProfile.CastingSegment;

import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.ToolingEnums.ContactFace;

enum abstract PanelOperationKind(String) to String {
	var OutsideProfile = "outside-profile";
	var InsideProfile = "inside-profile";
	var Pocket = "pocket";
	var Rebate = "rebate";
	var Drill = "drill";
}

/** One CAM operation, with finished geometry and an explicit setup/tool. No compensation.
 * Rebate is a pocket with an open side. Drill contours are exact circles.
 */
typedef PanelOperation = {
	var id:String;
	var kind:PanelOperationKind;
	var contour:Array<CastingSegment>;
	var side:MachiningSide;
	var setupId:String;
	var depth:Float;
	var through:Bool;
	var toolDiameter:Float;

	/** If present, the operation must use this same tool in CAM. */
	var reliefToolDiameter:Null<Float>;

	/** Explicit deeper cut reached through an existing pocket/rebate. */
	var accessVia:Null<String>;
}

/** Local part only; its one outside-profile operation owns the exact outline.
 * Placement/removal belongs to the assembly. Stock Z spans [0, measuredThickness].
 */
typedef PanelPlan = {
	var id:String;
	var revision:Int;
	var stock:PanelStockSpec;
	var operations:Array<PanelOperation>;
	var minimumLigament:Float;
	var contactFace:ContactFace;

	/** Explicit contact-skin cuts; the consuming fixture/mold must classify each. */
	var contactOpenings:Array<String>;

	var labelSide:MachiningSide;
	var labelX:Float;
	var labelY:Float;
}
