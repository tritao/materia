import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.process.AirBending;
import cadkit.sheetmetal.process.BendProcess;
import cadkit.sheetmetal.process.ProcessTooling;
import cadkit.sheetmetal.process.CalibrationEntry;
import cadkit.sheetmetal.process.CalibrationTable;

/** Independent contact construction and released-angle checks for the assumed process model. */
class SheetProcessSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(actual:Float, expected:Float, tolerance:Float, label:String):Void {
		check(Math.isFinite(actual) && Math.abs(actual - expected) <= tolerance, label + ": " + actual + " != " + expected);
	}
	static function diagnosis(code:String, action:Void->Void):Void {
		checks++;
		try { action(); }
		catch (failure:SheetDiagnosis) { if (failure.code == code) return; throw failure; }
		throw "Expected " + code;
	}

	// Recover the common tangent angle from circle-centre geometry, then solve depth numerically.
	// The inner sheet circle and punch circle touch at their lowest point for a radius bend.
	static function contactAngle(depth:Float, thickness:Float, radius:Float, tooling:ProcessTooling):Float {
		var height = tooling.shoulderRadius + radius - depth;
		var halfWidth = tooling.vWidth / 2;
		var distance = Math.sqrt(halfWidth * halfWidth + height * height);
		return 2 * (Math.asin((radius + thickness + tooling.shoulderRadius) / distance) - Math.atan2(height, halfWidth));
	}
	static function numericDepth(thickness:Float, radius:Float, tooling:ProcessTooling, angle:Float):Float {
		var low = -thickness; var high = tooling.vWidth * 20;
		for (iteration in 0...90) {
			var middle = (low + high) / 2;
			if (contactAngle(middle, thickness, radius, tooling) < angle) low = middle; else high = middle;
		}
		return (low + high) / 2;
	}

	public static function run():Int {
		var steel = new SheetMaterial("material-a", 1.5, null, {uts: 400, yield: 250, youngs: 205000});
		var aluminium = new SheetMaterial("material-b", 1, null, {uts: 228, yield: 193, youngs: 70000});
		var process:BendProcess = new AirBending();
		near(AirBending.defaultOpening(1.5), 12, 1e-12, "thin-stock default die opening");
		diagnosis("sheet.no-default-tooling", function() { var opening = AirBending.defaultOpening(4); });
		diagnosis("sheet.invalid-die-opening", function() { var minimum = AirBending.minimumFlange(0); });
		for (opening in [8.0, 12.0, 16.0]) {
			var tooling = new ProcessTooling("acute-punch", "v" + opening, 0.5, Math.PI / 6, opening, Math.PI / 6, 0.5);
			var result = process.predict(steel, tooling, Math.PI / 2, 120);
			near(result.innerRadius, 0.16 * opening * 400 / 410, 1e-10, "natural final radius");
			near(result.force, 650 * 1.5 * 1.5 * 0.120 / opening * 400 / 450, 1e-10, "kN and length conversion");
			near(result.minimumFlange, 0.7 * opening, 1e-12, "assumed 90-degree minimum flange");
			check(result.recommendedOpening == (opening >= 9 && opening <= 18), "recommended V/T range");
			check(!result.radiusBend && result.overbend > 0, "natural bend has positive springback compensation");
			var mechanical = steel.mechanical; if (mechanical == null) throw "missing steel properties";
			near(AirBending.unloadRadius(result.loadedRadius, steel.thickness, mechanical), result.innerRadius, 1e-10, "radius recovery");
			var released = result.loadedAngle * (result.loadedRadius + steel.thickness / 2) / (result.innerRadius + steel.thickness / 2);
			near(released, Math.PI / 2, 0.01 * Math.PI / 180, "loaded angle releases to target");
			near(result.bendAllowance, result.loadedAngle * (result.loadedRadius + steel.thickness / 2), 1e-10, "neutral length conserved");
			var down = process.predict(steel, tooling, -Math.PI / 2, 120);
			near(down.loadedAngle, -result.loadedAngle, 1e-12, "down loaded angle");
			near(down.depth, result.depth, 1e-12, "bend sign does not alter press depth");
			near(down.bendAllowance, result.bendAllowance, 1e-12, "bend sign does not alter blank allowance");
			var longer = process.predict(steel, tooling, Math.PI / 2, 240);
			near(longer.force, 2 * result.force, 1e-12, "force follows effective bend length");
		}
		var steelMechanical = steel.mechanical; if (steelMechanical == null) throw "missing steel";
		var aluminiumMechanical = aluminium.mechanical; if (aluminiumMechanical == null) throw "missing aluminium";
		var steelLoaded = AirBending.loadRadius(2 * steel.thickness, steel.thickness, steelMechanical);
		var aluminiumLoaded = AirBending.loadRadius(2 * aluminium.thickness, aluminium.thickness, aluminiumMechanical);
		var steelAngle = Math.PI / 2 * 2.5 * steel.thickness / (steelLoaded + steel.thickness / 2);
		var aluminiumAngle = Math.PI / 2 * 2.5 * aluminium.thickness / (aluminiumLoaded + aluminium.thickness / 2);
		check(aluminiumAngle > steelAngle, "material B needs more overbend at the same final R/T");
		var broadRadius = AirBending.loadRadius(1000, 1, aluminiumMechanical);
		near(AirBending.unloadRadius(broadRadius, 1, aluminiumMechanical), 1000, 1e-8, "final radius beyond loaded branch still has a physical inverse");
		var ratio = 2 * steelMechanical.yield / (steelMechanical.youngs * 1.5);
		near(AirBending.unloadRadius(2, 1.5, steelMechanical), 2 / (4 * ratio * ratio * ratio - 3 * ratio + 1), 1e-12, "published cubic radius relation");
		for (thickness in [0.5, 1.5, 2.0]) for (radius in [0.8, 1.5, 2.5]) {
			var tooling = new ProcessTooling("acute", "v16", 0.5, Math.PI / 9, 16, Math.PI / 9, 0.5);
			for (degrees in 20...151) {
				var angle = degrees * Math.PI / 180;
				near(AirBending.depth(thickness, radius, tooling, angle), numericDepth(thickness, radius, tooling, angle), 0.001, "three-circle depth");
			}
			near(AirBending.depth(thickness, radius, tooling, 0), -thickness, 1e-12, "flat sheet contact above die top");
		}
		var largeTip = new ProcessTooling("radius-punch", "v16", 3, Math.PI / 6, 16, Math.PI / 6, 0.5);
		var radiusBend = process.predict(steel, largeTip, Math.PI / 2, 120);
		check(radiusBend.radiusBend, "large punch radius is reported");
		near(radiusBend.loadedRadius, 3, 1e-12, "loaded material respects punch radius");
		check(radiusBend.innerRadius > 3, "radius bend relaxes beyond its punch radius");
		var rightAngleTools = new ProcessTooling("90-punch", "90-die", 0.5, Math.PI / 2, 12, Math.PI / 2, 0.5);
		diagnosis("sheet.tool-angle-limit", function() { var result = process.predict(steel, rightAngleTools, Math.PI / 2, 120); });
		diagnosis("sheet.no-mechanical-data", function() { var result = process.predict(new SheetMaterial("visual", 1), largeTip, 1, 100); });
		diagnosis("sheet.invalid-process-angle", function() { var result = process.predict(steel, largeTip, 0, 100); });
		diagnosis("sheet.invalid-bend-length", function() { var result = process.predict(steel, largeTip, 1, -1); });
		diagnosis("sheet.invalid-process-tooling", function() { var result = new ProcessTooling("p", "d", 0.5, 1, 0, 1, 0.5); });
		diagnosis("sheet.springback-domain", function() { var result = AirBending.unloadRadius(1000, 1, aluminiumMechanical); });
		diagnosis("sheet.tool-contact-limit", function() { var result = AirBending.depth(2, 8, largeTip, 1); });
		var calibratedTools = new ProcessTooling("acute", "v12", 0.5, Math.PI / 6, 12, Math.PI / 6, 0.5);
		var entry = new CalibrationEntry("lot-a", 1.5, "acute", "v12", 1, 2, 0.125, 2);
		var rows = [entry]; var table = new CalibrationTable(rows); rows.pop();
		check(table.entries().length == 1, "calibration owns its input array");
		var exposed = table.entries(); exposed.pop();
		check(table.entries().length == 1, "calibration isolates returned arrays");
		check(table.find("lot-a", 1.5, calibratedTools, 1) == entry, "lower band endpoint included");
		check(table.find("lot-a", 1.5, calibratedTools, 2) == null, "upper band endpoint excluded");
		check(table.find("lot-a", 1.5, calibratedTools, -1.5) == entry, "calibration bands use bend magnitude");
		check(table.find("lot-b", 1.5, calibratedTools, 1.5) == null, "another lot has no correction");
		check(table.find("lot-a", 1.500001, calibratedTools, 1.5) == null, "thickness matching does not guess");
		check(table.find("lot-a", 1.5, largeTip, 1.5) == null, "another tooling pair has no correction");
		var measuredProcess:BendProcess = new AirBending(table);
		var measured = measuredProcess.predict(steel, calibratedTools, Math.PI / 2, 120, "lot-a");
		check(measured.calibrated, "prediction reports measured correction");
		near(measured.innerRadius, 2, 1e-12, "measured radius overrides analytic radius");
		near(measured.depth, AirBending.depth(1.5, measured.loadedRadius, calibratedTools, measured.loadedAngle) + 0.125, 1e-12, "depth correction applies after measured-radius reconstruction");
		near(measured.depthOffset, 0.125, 1e-12, "depth correction remains explicit");
		near(measured.bendAllowance, Math.PI / 2 * 2.75, 1e-12, "measured radius changes development");
		var unmeasured = measuredProcess.predict(steel, calibratedTools, Math.PI / 2, 120);
		check(!unmeasured.calibrated, "unspecified lot does not inherit a calibration");
		near(unmeasured.depth, unmeasured.analyticDepth, 1e-12, "uncalibrated depth is analytic");
		var adjacent = new CalibrationEntry("lot-a", 1.5, "acute", "v12", 2, 2.5, -0.1, 2.1);
		var adjacentTable = new CalibrationTable([entry, adjacent]);
		check(adjacentTable.find("lot-a", 1.5, calibratedTools, 2) == adjacent, "adjacent bands meet without ambiguity");
		diagnosis("sheet.ambiguous-calibration", function() { var result = new CalibrationTable([entry, entry]); });
		diagnosis("sheet.invalid-calibration", function() { var result = new CalibrationEntry("lot-a", 1.5, "p", "d", 2, 1, 0, 2); });
		diagnosis("sheet.invalid-calibration-key", function() { var result = table.find("", 1.5, calibratedTools, 1.5); });
		var impossible = new AirBending(new CalibrationTable([new CalibrationEntry("lot-a", 1.5, "radius-punch", "v16", 1, 2, 0, 2)]));
		diagnosis("sheet.calibration-tool-radius", function() { var result = impossible.predict(steel, largeTip, Math.PI / 2, 120, "lot-a"); });
		return checks;
	}
	static function main():Void { trace("Sheet process smoke passed (" + run() + " assertions)"); }
}
