package machinekit.pressbrake.planning;

import cadkit.modeling.Vector;
import cadkit.sheetmetal.BendSeam;
import cadkit.sheetmetal.FlangeDepth;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetBounds;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetState;
import cadkit.sheetmetal.process.BendProcess;
import machinekit.pressbrake.ToolRow;
import machinekit.pressbrake.ToolStation;

/** Single-bend manufacturing checks share the evaluated geometry used by sequence search. */
class BendSetup {
	public final press:PressGeometry;
	public function new(press:PressGeometry) { this.press = press; }

	/** Plan against one installed station. The sequence collision query consumes the resulting stroke poses. */
	public function plan(sheet:Sheet, state:SheetState, bend:Int, station:ToolStation,
			orientation:BendOrientation, gaugeEdge:GaugeEdge, process:BendProcess,
			?lot:String, samples:Int = 36):BendSetupResult {
		if (samples < 1 || samples > 3600)
			throw new SheetDiagnosis("press.stroke-samples", "Stroke sampling needs between 1 and 3600 intervals");
		if (station != press.installedStation(station.id))
			throw new SheetDiagnosis("press.station-not-installed", "Setup requires the installed tool station, not a different specification with its identity", [station.id]);
		var before = sheet.evaluate(state), record = sheet.bend(bend);
		if (state.angle(bend) != 0)
			throw new SheetDiagnosis("press.bend-state", "The active bend must be flat before its stroke", ["bend:" + bend]);
		var contact = press.station(station.id), tooling = station.processTooling();
		var prediction = process.predict(sheet.material, tooling, record.angle, record.edgeA.span.length(), lot);
		if (!Math.isFinite(prediction.force) || prediction.force <= 0 || !Math.isFinite(prediction.minimumFlange)
			|| prediction.minimumFlange <= 0 || !Math.isFinite(prediction.depth))
			throw new SheetDiagnosis("press.process-range", "Process force, depth and flange minimum must be finite physical values", ["bend:" + bend]);
		if (prediction.force * 1000 > press.capacityN)
			throw new SheetDiagnosis("press.tonnage", "Bend " + bend + " needs " + prediction.force
				+ " kN, above frame capacity " + press.capacityN / 1000 + " kN", ["bend:" + bend, station.id]);
		for (side in 0...2) {
			var wall = sheet.wall(side == 0 ? record.wallA : record.wallB);
			var attachment = side == 0 ? record.edgeA : record.edgeB;
			var leg = FlangeDepth.minimum(wall.outline, attachment);
			if (leg < prediction.minimumFlange - 1e-8)
				throw new SheetDiagnosis("press.short-flange", "Wall " + wall.id + " at bend " + bend
					+ " has " + leg + " mm straight flange, below " + prediction.minimumFlange + " mm at " + station.id,
					["wall:" + wall.id, "bend:" + bend, station.id]);
		}
		var initial = PressSheetPose.at(sheet, state, bend, prediction, tooling, contact, orientation);
		var zone = initial.evaluation.bend(bend);
		for (interval in BendSeam.intervals(sheet.wall(record.wallA).outline, record.edgeA.span, true)) {
			var first = initial.placement.plane.toWorld(zone.point(0.5, interval.first * zone.width));
			var last = initial.placement.plane.toWorld(zone.point(0.5, interval.last * zone.width));
			var low = Math.min(first.x, last.x), high = Math.max(first.x, last.x);
			if (low < -press.bedLength / 2 - 1e-8 || high > press.bedLength / 2 + 1e-8
				|| !covered(station.punches, station.offset - press.bedLength / 2, low, high)
				|| !covered(station.dies, station.offset - press.bedLength / 2, low, high))
				throw new SheetDiagnosis("press.tool-support", "Installed punch and die do not support the material-bearing bend interval",
					["bend:" + bend, station.id]);
		}
		var registration = BendRegistration.plan(sheet, initial, gaugeEdge, orientation, press);
		var stroke:Array<PressSheetPose> = [], bounds = new SheetBounds();
		var finalTravel = contact.openTipHeight - contact.dieTopHeight + prediction.depth;
		for (i in 0...(samples + 1)) {
			var sampleState = new SheetState();
			for (id in state.ids()) { var copied = sampleState.set(id, id == bend ? prediction.loadedAngle * (i / samples) : state.angle(id)); }
			var pose = PressSheetPose.at(sheet, sampleState, bend, prediction, tooling, contact, orientation);
			var travel = contact.openTipHeight - contact.dieTopHeight + pose.depth;
			if (travel < press.ramTravel.lower - 1e-8 || travel > press.ramTravel.upper + 1e-8)
				throw new SheetDiagnosis("press.ram-travel", "Stroke sample needs ram travel " + travel + " mm outside nominal travel",
					["bend:" + bend, station.id]);
			stroke.push(pose);
			var source = pose.evaluation.bounds();
			for (x in [source.minX, source.maxX]) for (y in [source.minY, source.maxY]) for (z in [source.minZ, source.maxZ])
				bounds.include(pose.placement.plane.toWorld(new Vector(x, y, z)));
		}
		return new BendSetupResult(bend, station.id, orientation, gaugeEdge, registration, prediction, finalTravel,
			new PressEnvelope(bounds.minX, bounds.minY, bounds.minZ, bounds.maxX, bounds.maxY, bounds.maxZ), stroke);
	}
	static function covered(row:ToolRow, offset:Float, low:Float, high:Float):Bool {
		var cursor = low;
		for (piece in row.placements(offset)) {
			if (piece.end < cursor - 1e-8) continue;
			if (piece.start > cursor + 1e-8) return false;
			cursor = Math.max(cursor, piece.end);
			if (cursor >= high - 1e-8) return true;
		}
		return false;
	}
}
