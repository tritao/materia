import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.WallEdge;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.process.AirBending;
import cadkit.sheetmetal.process.CalibrationEntry;
import cadkit.sheetmetal.process.CalibrationTable;
import cadkit.sheetmetal.process.ProcessBendRule;
import machinekit.pressbrake.planning.BendOrientation;
import machinekit.pressbrake.planning.BendSetup;
import machinekit.pressbrake.planning.GaugeEdge;
import machinekit.pressbrake.planning.PressGeometry;

/** Whole single-bend setup acceptance and named manufacturing refusals. */
class SingleBendSetupChecks {
	public static function main():Void run();
	static function near(actual:Float, expected:Float, label:String):Void {
		if (!Math.isFinite(actual) || Math.abs(actual - expected) > 1e-8) throw label + ": " + actual + " != " + expected;
	}
	static function diagnosis(code:String, action:Void->Void):SheetDiagnosis {
		try { action(); }
		catch (failure:SheetDiagnosis) { if (failure.code == code) return failure; throw failure; }
		throw "Expected " + code;
	}
	static function fixture(stock:SheetMaterial, leg:Float, width:Float):Sheet {
		var sheet = new Sheet(stock), root = sheet.addWall(WallOutline.rectangle(60, width));
		var child = SheetDemoParts.flange(sheet, root, 1, leg, Math.PI / 2, stock.defaultRadius(Math.PI / 2));
		return sheet;
	}
	public static function run():Void {
		var press = new PressBrake(true, null, null, 30), geometry = PressGeometry.of(press);
		var station = press.toolLayout.stations()[0], tooling = station.processTooling();
		var stock = new SheetMaterial("setup-steel", 1.5, BendRule.Process(new ProcessBendRule(tooling)), {uts: 400, yield: 250, youngs: 205000});
		var planner = new BendSetup(geometry), process = new AirBending();
		var sheet = SheetDemoParts.p2(stock), bend = sheet.bends()[0], state = sheet.flat();
		var orientation = new BendOrientation(true, false, -10), edge = new GaugeEdge(bend.wallB, new WallEdge(1));
		var result = planner.plan(sheet, state, bend.id, station, orientation, edge, process);
		var expected = process.predict(stock, tooling, bend.angle, 120);
		near(result.registration.x, 40 - bend.radius - stock.thickness + expected.bendAllowance / 2, "Complete setup mould-line distance");
		near(result.registration.r, 0, "Complete setup flat R datum");
		near(result.process.depth, expected.depth, "Setup uses process depth");
		near(result.process.force, expected.force, "Setup uses process force in kN");
		near(result.ramTravel, 80 + expected.depth, "Ram travel includes physical open gap");
		if (result.stroke().length != 37) throw "Setup must sample both stroke endpoints";
		var poses = result.stroke(); poses.pop();
		if (result.stroke().length != 37) throw "Caller changed the setup's stroke array";
		near(state.angle(bend.id), 0, "Planning does not mutate caller state");
		near(result.stroke()[36].evaluation.bend(bend.id).angle, expected.loadedAngle, "Last sample reaches exact loaded angle");
		var second = sheet.bends()[1];
		var later = planner.plan(sheet, sheet.flat().set(bend.id, bend.angle), second.id, station,
			new BendOrientation(true, true, -10), edge, process);
		near(later.registration.x, result.registration.x + 20, "Complete later setup follows flange difference");
		near(later.registration.r, 40, "Complete later setup lifts to the folded edge");
		for (pose in result.stroke()) {
			var zone = pose.evaluation.bend(bend.id);
			for (fraction in [0.0, 0.5, 1.0]) for (along in [0.0, zone.width]) for (depth in [0.0, stock.thickness]) {
				var point = pose.placement.plane.toWorld(zone.point(fraction, along, depth)), envelope = result.sampledSwing;
				if (point.x < envelope.minX - 1e-8 || point.x > envelope.maxX + 1e-8
					|| point.y < envelope.minY - 1e-8 || point.y > envelope.maxY + 1e-8
					|| point.z < envelope.minZ - 1e-8 || point.z > envelope.maxZ + 1e-8)
					throw "Sampled swing loses a physical bend point";
			}
		}
		var long = fixture(stock, 40, 5000), longBend = long.bends()[0];
		var tonnage = diagnosis("press.tonnage", function() {
			var result = planner.plan(long, long.flat(), longBend.id, station, orientation,
				new GaugeEdge(longBend.wallB, new WallEdge(1)), process);
		});
		var short = fixture(stock, 4, 100), shortBend = short.bends()[0];
		var refusal = diagnosis("press.short-flange", function() {
			var result = planner.plan(short, short.flat(), shortBend.id, station, orientation,
				new GaugeEdge(shortBend.wallB, new WallEdge(1)), process);
		});
		if (refusal.elements.indexOf("wall:" + shortBend.wallB) < 0) throw "Short flange diagnosis must name the failing wall";
		var wide = fixture(stock, 40, 200), wideBend = wide.bends()[0];
		var support = diagnosis("press.tool-support", function() {
			var result = planner.plan(wide, wide.flat(), wideBend.id, station, orientation,
				new GaugeEdge(wideBend.wallB, new WallEdge(1)), process);
		});
		var calibration = new CalibrationTable([new CalibrationEntry("depth-lot", 1.5,
			tooling.punchId, tooling.dieId, 1, 2, 25, 2)]);
		var deepStock = new SheetMaterial("depth-stock", 1.5,
			BendRule.Process(new ProcessBendRule(tooling, calibration, "depth-lot")), stock.mechanical);
		var deep = fixture(deepStock, 40, 100), deepBend = deep.bends()[0];
		var travel = diagnosis("press.ram-travel", function() {
			var result = planner.plan(deep, deep.flat(), deepBend.id, station, orientation,
				new GaugeEdge(deepBend.wallB, new WallEdge(1)), new AirBending(calibration), "depth-lot");
		});
		var folded = diagnosis("press.bend-state", function() {
			var result = planner.plan(sheet, state.set(bend.id, bend.angle), bend.id, station, orientation, edge, process);
		});
		trace("Single bend setup checks passed");
	}
}
