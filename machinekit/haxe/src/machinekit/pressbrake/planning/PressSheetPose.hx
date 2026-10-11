package machinekit.pressbrake.planning;

import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetEvaluation;
import cadkit.sheetmetal.SheetState;
import cadkit.sheetmetal.ManufacturedSheetState;
import cadkit.sheetmetal.process.BendProcessResult;
import cadkit.sheetmetal.process.ProcessTooling;

/** Native-free sheet placement at one air-bending stroke sample. */
class PressSheetPose {
	public final evaluation:SheetEvaluation;
	public final placement:Location;
	public final depth:Float;
	function new(evaluation:SheetEvaluation, placement:Location, depth:Float) {
		this.evaluation = evaluation; this.placement = placement; this.depth = depth;
	}
	public function wall(id:Int):Location return placement.compose(evaluation.wall(id));

	/** Conserve the developed strip while keeping its concave midpoint under the punch. */
	public static function at(sheet:Sheet, state:SheetState, bend:Int, process:BendProcessResult,
			tooling:ProcessTooling, station:PressStationGeometry, orientation:BendOrientation):PressSheetPose {
		var target = sheet.bend(bend).angle;
		if (!Math.isFinite(process.loadedAngle) || Math.abs(process.loadedAngle) >= Math.PI)
			throw new SheetDiagnosis("press.stroke-angle", "Air-bending placement needs a loaded angle below 180 degrees", ["bend:" + bend]);
		if ((target > 0) != orientation.faceAUp)
			throw new SheetDiagnosis("press.bend-face", "The bend's concave face must face the punch", ["bend:" + bend]);
		return fromEvaluation(sheet, sheet.evaluateStroke(state, bend, process), bend,
			tooling, station, orientation, process.depthOffset, process.loadedAngle);
	}

	/** Actual stock contact geometry is independent of a planner's learned depth correction. */
	public static function manufactured(sheet:Sheet, state:ManufacturedSheetState, bend:Int,
			tooling:ProcessTooling, station:PressStationGeometry, orientation:BendOrientation):PressSheetPose {
		if ((sheet.bend(bend).angle > 0) != orientation.faceAUp)
			throw new SheetDiagnosis("press.bend-face", "The bend's concave face must face the punch", ["bend:" + bend]);
		return fromEvaluation(sheet, sheet.evaluateManufactured(state), bend, tooling, station, orientation, 0, 1);
	}

	static function fromEvaluation(sheet:Sheet, evaluation:SheetEvaluation, bend:Int,
			tooling:ProcessTooling, station:PressStationGeometry, orientation:BendOrientation,
			depthOffset:Float, loadedAngle:Float):PressSheetPose {
		var zone = evaluation.bend(bend), halfAngle = zone.angle / 2;
		if (Math.abs(zone.angle) >= Math.PI)
			throw new SheetDiagnosis("press.stroke-angle", "Air-bending placement needs an angle below 180 degrees", ["bend:" + bend]);
		var upperDepth = orientation.faceAUp ? 0.0 : sheet.thickness();
		var source = new Location(new Plane(zone.point(0.5, zone.width / 2, upperDepth),
			zone.frame.direction(new Vector(Math.cos(halfAngle), 0, Math.sin(halfAngle))),
			zone.frame.direction(new Vector(-Math.sin(halfAngle), 0, Math.cos(halfAngle)))));
		var depth = -sheet.thickness();
		if (zone.angle != 0) {
			var beta = Math.abs(zone.angle) / 2;
			var radius = zone.curvedInnerRadius();
			if ((tooling.vWidth / 2 - (radius + sheet.thickness() + tooling.shoulderRadius) * Math.sin(beta)) / Math.cos(beta) <= 0)
				throw new SheetDiagnosis("press.tool-contact", "The curved strip consumes the straight die-contact legs", ["bend:" + bend]);
			var loss = 2 * Math.sin(beta / 2) * Math.sin(beta / 2);
			// Stable three-circle contact construction, including the large-radius near-flat limit.
			depth = (-sheet.thickness() + tooling.vWidth / 2 * Math.sin(beta)
				- (radius + tooling.shoulderRadius) * loss) / Math.cos(beta);
			// Measured depth correction is interpolated from first contact to the loaded endpoint.
			depth += depthOffset * Math.abs(zone.angle / loadedAngle);
		}
		if (!Math.isFinite(depth))
			throw new SheetDiagnosis("press.stroke-depth", "Stroke contact depth exceeds the finite range", ["bend:" + bend]);
		var destination = new Location(new Plane(new Vector(orientation.bedCenter, station.punchLineY,
			station.dieTopHeight - depth), new Vector(0, orientation.reverse ? -1 : 1, 0),
			new Vector(0, 0, orientation.faceAUp ? 1 : -1)));
		return new PressSheetPose(evaluation, destination.compose(source.inverse()), depth);
	}
}
