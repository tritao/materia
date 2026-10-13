package machinekit.pressbrake.planning;

import cadkit.modeling.Vector;
import cadkit.sheetmetal.OutlineBoolean;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.WallOutline;

/** Explicit Phase-B suction patch and outward gripper envelope, in mm; no inferred holding rating. */
class GripperBox {
	public final width:Float;
	public final length:Float;
	public final height:Float;
	public function new(width:Float, length:Float, height:Float) {
		if (!Math.isFinite(width) || !Math.isFinite(length) || !Math.isFinite(height)
			|| width <= 0 || length <= 0 || height <= 0)
			throw new SheetDiagnosis("press.gripper-box", "Gripper dimensions must be finite positive mm");
		this.width = width; this.length = length; this.height = height;
	}
	public function patch(grip:SheetGrip):WallOutline {
		var x = grip.center.x, y = grip.center.y;
		return WallOutline.polygon([new Point2(x-width/2,y-length/2),new Point2(x+width/2,y-length/2),
			new Point2(x+width/2,y+length/2),new Point2(x-width/2,y+length/2)]);
	}
	/** The complete patch must be supported; checking its center would miss holes and notches. */
	public function supported(sheet:Sheet, grip:SheetGrip):Bool
		return OutlineBoolean.regions(patch(grip), sheet.wall(grip.wall).outline, "grip-patch").length == 0;
	public function at(sheet:Sheet, pose:PressSheetPose, grip:SheetGrip):ConvexPrism {
		var outline = patch(grip);
		var points = [for (segment in outline.loops()[0]) new Vector(segment.start.x-grip.center.x, segment.start.y-grip.center.y, 0)];
		return new ConvexPrism("gripper", points, new Vector(0,0,height)).placed(grip.frame(sheet,pose));
	}
}
