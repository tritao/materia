import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.SheetState;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.process.ProcessBendRule;
import cadkit.sheetmetal.process.ProcessTooling;

/** Manufacturing overbend preserves the authored blank and leaves other design bends bounded. */
class SheetStrokeSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(actual:Float, expected:Float, label:String):Void {
		check(Math.isFinite(actual) && Math.abs(actual - expected) < 1e-8, label);
	}
	static function diagnosis(code:String, action:Void->Void):Void {
		checks++;
		try { action(); }
		catch (failure:SheetDiagnosis) { if (failure.code == code) return; throw failure; }
		throw "Expected " + code;
	}
	public static function run():Int {
		checks = 0;
		var setup = new ProcessBendRule(new ProcessTooling("acute", "v12", 0.5, Math.PI / 6, 12, Math.PI / 6, 0.5));
		var stock = new SheetMaterial("stroke-steel", 1.5, BendRule.Process(setup), {uts: 400, yield: 250, youngs: 205000});
		for (sign in [1.0, -1.0]) {
			var sheet = new Sheet(stock);
			var base = sheet.addWall(WallOutline.rectangle(60, 100));
			var side = SheetDemoParts.flange(sheet, base, 1, 40, sign * Math.PI / 2, stock.defaultRadius(sign * Math.PI / 2));
			var other = SheetDemoParts.flange(sheet, base, 3, 40, Math.PI / 2, stock.defaultRadius(Math.PI / 2));
			var bend = sheet.bends()[0];
			var second = sheet.bends()[1];
			var process = setup.predict(stock, bend.angle, 100);
			check(Math.abs(process.loadedAngle) > Math.abs(bend.angle), "physical springback needs overbend");
			var flat = new SheetState().set(bend.id, 0).set(second.id, 0);
			var length = sheet.evaluate(flat).bend(bend.id).allowance;
			for (fraction in [0.0, 0.25, 0.5, 0.75, 1.0]) {
				var state = new SheetState().set(bend.id, process.loadedAngle * fraction).set(second.id, second.angle);
				var evaluation = sheet.evaluateStroke(state, bend.id, process);
				var zone = evaluation.bend(bend.id);
				near(zone.allowance, length, "stroke conserves neutral length");
				near(zone.angle, process.loadedAngle * fraction, "stroke evaluates its measured angle");
				near(evaluation.bend(second.id).angle, second.angle, "other bend keeps its design angle");
				if (fraction == 1) near(zone.curvedInnerRadius(), process.loadedRadius, "loaded inner radius agrees with process");
			}
			var loaded = new SheetState().set(bend.id, process.loadedAngle).set(second.id, second.angle);
			diagnosis("sheet.invalid-state", function() { var result = sheet.evaluate(loaded); });
			var tooFar = new SheetState().set(bend.id, process.loadedAngle * 1.01).set(second.id, second.angle);
			diagnosis("sheet.invalid-state", function() { var result = sheet.evaluateStroke(tooFar, bend.id, process); });
			var otherOverbend = new SheetState().set(bend.id, 0).set(second.id, second.angle * 1.01);
			diagnosis("sheet.invalid-state", function() { var result = sheet.evaluateStroke(otherOverbend, bend.id, process); });
			var wrongProcess = setup.predict(stock, sign * Math.PI / 3, 100);
			diagnosis("sheet.stroke-process-mismatch", function() { var result = sheet.evaluateStroke(flat, bend.id, wrongProcess); });
			near(sheet.bend(bend.id).angle, sign * Math.PI / 2, "stroke leaves authored target intact");
			near(sheet.evaluate(flat).bend(bend.id).allowance, length, "stroke leaves developed blank intact");
		}
		return checks;
	}
}
