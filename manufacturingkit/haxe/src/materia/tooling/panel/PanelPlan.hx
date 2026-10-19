package materia.tooling.panel;

import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.ToolingEnums.ContactFace;
import materia.tooling.ToolingPrism.ToolingPoint2;

enum abstract PanelCurveKind(String) to String { var Line = "line"; var Arc = "arc"; }
enum abstract PanelOperationKind(String) to String {
    var OutsideProfile = "outside-profile"; var InsideProfile = "inside-profile";
    var Pocket = "pocket"; var Rebate = "rebate"; var Drill = "drill";
}
/** Directed exact curve. Arcs have a center and signed sweep; lines have null/zero.
 * A closed circle uses two half arcs, so each arc has distinct endpoints.
 */
typedef PanelSegment = {
    var kind:PanelCurveKind;
    var start:ToolingPoint2;
    var end:ToolingPoint2;
    var center:Null<ToolingPoint2>;
    var sweep:Float;
}
/** One CAM operation, with finished geometry and an explicit setup/tool. No compensation.
 * Rebate is a pocket with an open side. Drill contours are exact circles.
 */
typedef PanelOperation = {
    var id:String;
    var kind:PanelOperationKind;
    var contour:Array<PanelSegment>;
    var side:MachiningSide;
    var setupId:String;
    var depth:Float;
    var through:Bool;
    var toolDiameter:Float;
    /** If present, the operation must use this same tool in CAM. */
    var reliefToolDiameter:Null<Float>;
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
    var labelSide:MachiningSide;
    var labelX:Float;
    var labelY:Float;
}
