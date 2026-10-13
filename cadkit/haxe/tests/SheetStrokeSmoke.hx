import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.KFactorTable;
import cadkit.sheetmetal.ManufacturedSheetState;
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
		for (sheet in [SheetDemoParts.p1(stock), SheetDemoParts.p2(stock), SheetDemoParts.p3(stock),
				SheetDemoParts.p4(stock), SheetDemoParts.p5(stock)]) {
			var developed = sheet.evaluate(sheet.flat());
			var folded = sheet.evaluate(sheet.folded());
			check(sheet.material == stock, "manufacturing part owns explicitly selected stock");
			for (bend in sheet.bends()) {
				var process = setup.predict(stock, bend.angle, bend.edgeA.span.length());
				near(bend.radius, process.innerRadius, "manufacturing radius comes from selected process");
				near(developed.bend(bend.id).allowance, process.bendAllowance, "manufacturing blank uses process allowance");
				near(folded.bend(bend.id).curvedInnerRadius(), process.innerRadius, "released manufacturing radius");
			}
		}
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
			var actual = new ManufacturedSheetState(otherOverbend).set(bend.id, process.loadedAngle);
			var manufactured = sheet.evaluateManufactured(actual);
			near(manufactured.bend(second.id).angle, second.angle * 1.01, "actual prior bend can exceed its design target");
			near(manufactured.bend(bend.id).allowance, length, "actual angles preserve the blank");
			near(manufactured.bend(bend.id).curvedInnerRadius(), process.loadedRadius, "actual loaded radius uses fixed neutral length");
			var changedSource = otherOverbend.set(second.id, 0);
			near(actual.angle(second.id), second.angle * 1.01, "manufactured state owns its angle snapshot");
			var fullTurn = new ManufacturedSheetState(flat).set(bend.id, 2 * Math.PI);
			diagnosis("sheet.invalid-manufactured-angle", function() { var result = sheet.evaluateManufactured(fullTurn); });
			var omitted = new ManufacturedSheetState(new SheetState());
			diagnosis("sheet.invalid-state", function() { var result = sheet.evaluateManufactured(omitted); });
			var unknown = new ManufacturedSheetState(flat).set(9999, 0);
			diagnosis("sheet.missing-bend", function() { var result = sheet.evaluateManufactured(unknown); });
			var reversed = new ManufacturedSheetState(flat).set(bend.id, -bend.angle / 2);
			near(sheet.evaluateManufactured(reversed).bend(bend.id).angle, -bend.angle / 2,
				"actual geometry records a bend in the wrong direction");
			var wrongProcess = setup.predict(stock, sign * Math.PI / 3, 100);
			diagnosis("sheet.stroke-process-mismatch", function() { var result = sheet.evaluateStroke(flat, bend.id, wrongProcess); });
			near(sheet.bend(bend.id).angle, sign * Math.PI / 2, "stroke leaves authored target intact");
			near(sheet.evaluate(flat).bend(bend.id).allowance, length, "stroke leaves developed blank intact");
		}
		var sharpStock = new SheetMaterial("sharp-state", 1, BendRule.KFactor(new KFactorTable([0], [0.5])));
		var sharp = new Sheet(sharpStock);
		var sharpBase = sharp.addWall(WallOutline.rectangle(60, 100));
		var sharpSide = SheetDemoParts.flange(sharp, sharpBase, 1, 40, Math.PI / 2, 0);
		var negativeRadius = new ManufacturedSheetState(sharp.folded()).set(sharp.bends()[0].id, Math.PI * 0.6);
		diagnosis("sheet.invalid-manufactured-angle", function() { var result = sharp.evaluateManufactured(negativeRadius); });
		return checks;
	}
}
