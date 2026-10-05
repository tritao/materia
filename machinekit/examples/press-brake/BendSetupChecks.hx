import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.process.ProcessBendRule;
import cadkit.sheetmetal.process.ProcessTooling;
import machinekit.pressbrake.planning.BendOrientation;
import machinekit.pressbrake.planning.PressSheetPose;
import machinekit.pressbrake.planning.PressStationGeometry;

/** Press placement checks use physical contact construction and independent symmetry assertions. */
class BendSetupChecks {
	public static function main():Void run();
	static function near(actual:Float, expected:Float, label:String):Void {
		if (!Math.isFinite(actual) || Math.abs(actual - expected) > 1e-8)
			throw label + ": " + actual + " != " + expected;
	}
	public static function run():Void {
		var tooling = new ProcessTooling("acute", "v12", 0.5, Math.PI / 6, 12, Math.PI / 6, 0.5);
		var rule = new ProcessBendRule(tooling);
		var stock = new SheetMaterial("placement-steel", 1.5, BendRule.Process(rule), {uts: 400, yield: 250, youngs: 205000});
		var sheet = SheetDemoParts.p3(stock);
		var station = new PressStationGeometry("v12", 800, 7, 880);
		for (bend in sheet.bends()) for (reverse in [false, true]) {
			var process = rule.predict(stock, bend.angle, bend.edgeA.span.length());
			var orientation = new BendOrientation(bend.angle > 0, reverse, -10);
			for (fraction in [0.0, 1e-8, 0.25, 0.5, 0.75, 1.0]) {
				var state = sheet.flat().set(bend.id, process.loadedAngle * fraction);
				var pose = PressSheetPose.at(sheet, state, bend.id, process, tooling, station, orientation);
				var zone = pose.evaluation.bend(bend.id);
				var mid = pose.placement.plane.toWorld(zone.point(0.5, zone.width / 2, orientation.faceAUp ? 0 : stock.thickness));
				near(mid.x, orientation.bedCenter, "Bend centre follows authored lateral placement");
				near(mid.y, station.punchLineY, "Bend midpoint stays below punch line");
				near(mid.z, station.dieTopHeight - pose.depth, "Concave midpoint follows ram contact depth");
				var before = pose.placement.direction(zone.frame.plane.xDirection.scale(-1));
				var after = pose.placement.direction(zone.endFrame().plane.xDirection);
				near(before.z, after.z, "Both flanges rise symmetrically");
				if (before.z < -1e-8) throw "Press placement bends down toward the die";
				if (fraction == 0) near(pose.depth, -stock.thickness, "Flat contact lies one thickness above die");
				if (fraction == 1) near(pose.depth, process.depth, "Loaded placement reaches predicted depth");
			}
			var rejected = false;
			try {
				var wrongFace = PressSheetPose.at(sheet, sheet.flat(), bend.id, process, tooling, station,
					new BendOrientation(!orientation.faceAUp, reverse, -10));
			} catch (failure:SheetDiagnosis) { rejected = failure.code == "press.bend-face"; }
			if (!rejected) throw "Wrong face was accepted for upward bending";
		}
		trace("Press bend placement checks passed");
	}
}
