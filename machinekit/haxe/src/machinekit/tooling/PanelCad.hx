package machinekit.tooling;

import cadkit.modeling.Part;
import cadkit.sheetmetal.Segment;
import materia.tooling.panel.PanelContours;
import materia.casting.CastingProfile.CastingSegment;
import materia.tooling.panel.PanelPlan.PanelOperation;

/** Exact neutral-to-CadKit conversion shared by solid realization and drawings. */
class PanelCad {
	public static function segment(s:CastingSegment):Segment
		return machinekit.structural.CastingProfileCad.segment(s);

	public static function curves(op:PanelOperation):Array<Segment>
		return [for (s in PanelContours.operation(op)) segment(s)];

	public static function prism(op:PanelOperation, z0:Float, z1:Float):Part
		return contourPrism(PanelContours.operation(op), z0, z1);

	public static function contourPrism(contour:Array<CastingSegment>,z0:Float,z1:Float):Part
		return machinekit.structural.CastingProfileCad.prism(contour,z0,z1);
}
