package machinekit.pressbrake.planning;

import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.Sheet;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;

/** A grip belongs to authored wall material, at wall-local coordinates on one stock face. */
class SheetGrip {
	public final wall:Int;
	public final center:Point2;
	public final faceA:Bool;
	public function new(wall:Int, center:Point2, faceA:Bool) {
		if (wall <= 0 || center == null)
			throw new SheetDiagnosis("press.grip", "A grip needs an authored wall and finite local center");
		this.wall = wall; this.center = center; this.faceA = faceA;
	}
	/** The grip frame follows wall FK; its normal always points away from stock material. */
	public function frame(sheet:Sheet, pose:PressSheetPose):Location
		return pose.wall(wall).compose(new Location(new Plane(new Vector(center.x,center.y,faceA ? 0 : -sheet.thickness()),
			Vector.X(),faceA ? Vector.Z() : Vector.Z().scale(-1))));
}
