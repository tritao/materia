import cadkit.sheetmetal.BendDimensions;
import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.KFactorTable;
import cadkit.sheetmetal.LengthSpec;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;

/** Sheet graph and development checks; core dimension checks do not allocate native geometry. */
class SheetMetalSmoke {
	static var assertions:Int = 0;

	static function near(actual:Float, expected:Float, label:String, tolerance:Float = 1e-9):Void {
		assertions++;
		if (!Math.isFinite(actual) || Math.abs(actual - expected) > tolerance)
			throw label + ": expected " + expected + ", got " + actual;
	}

	static function diagnosis(code:String, action:Void->Void):Void {
		assertions++;
		try {
			action();
		} catch (error:Dynamic) {
			if (Std.isOfType(error, SheetDiagnosis)) {
				var failure = Std.downcast(error, SheetDiagnosis); if (failure == null) throw "unexpected error type";
				if (failure.code == code)
					return;
			}
			throw "Expected " + code + ", got " + Std.string(error);
		}
		throw "Expected " + code;
	}

	static function main():Void {
		var table = new KFactorTable([0, 1, 2], [0.3, 0.38, 0.5]);
		var din = new KFactorTable([0, 1, 2], [0.6, 0.76, 1], true);
		for (ratio in [0.0, 0.5, 1.0, 1.5, 2.0, 5.0])
			near(din.at(ratio), table.at(ratio), "DIN input halves to ANSI");
		near(table.at(0.5), 0.34, "interpolation");
		near(table.at(5), 0.5, "upper clamp");
		near(table.at(0), 0.3, "lower clamp");
		var ratios = [0.0, 2.0];
		var factors = [0.3, 0.5];
		var isolated = new KFactorTable(ratios, factors);
		ratios[1] = 4;
		factors[0] = 0;
		near(isolated.at(1), 0.4, "table owns its input values");
		var rule = BendRule.KFactor(new KFactorTable([0], [0.38]));
		// Hand-computed for T=2, R=1.64, K=.38.
		var angles = [Math.PI / 6, Math.PI / 2, 3 * Math.PI / 4];
		var allowances = [1.2566370614359172, 3.7699111843077517, 5.654866776461628];
		var setbacks = [0.975335060448, 3.64, 8.787737366];
		for (i in 0...angles.length) {
			near(BendDimensions.allowance(2, 1.64, angles[i], rule), allowances[i], "allowance");
			near(BendDimensions.allowance(2, 1.64, -angles[i], rule), allowances[i], "down allowance");
			near(BendDimensions.outsideSetback(2, 1.64, angles[i]), setbacks[i], "setback", 1e-8);
			near(BendDimensions.deduction(2, 1.64, angles[i], rule), 2 * setbacks[i] - allowances[i], "deduction", 2e-8);
		}
		near(BendDimensions.leg(51.76, LengthSpec.OuterSharp, 2, 1.64, Math.PI / 2), 48.12, "published flange");
		near(48.12 + BendDimensions.allowance(2, 1.64, Math.PI / 2, rule) / 2, 50, "published mould-line distance", 0.01);
		near(BendDimensions.leg(12, LengthSpec.Tangential, 2, 1.64, 3 * Math.PI / 4), 8.36, "obtuse tangential length");
		near(BendDimensions.leg(12, LengthSpec.OuterSharp, 2, 1.64, 3 * Math.PI / 4), 3.212262632961934, "obtuse sharp length");
		near(BendDimensions.leg(10, LengthSpec.InnerSharp, 2, 1.64, -Math.PI / 2), 8.36, "down inner sharp length");
		near(BendDimensions.leg(10, LengthSpec.Leg, 2, 1.64, Math.PI), 10, "hem straight length");
		var stock = new SheetMaterial("steel", 1.5, rule, {uts: 400, yield: 250, youngs: 205000});
		near(stock.thickness, 1.5, "sheet stock thickness");
		var mechanical = stock.mechanical; if (mechanical == null) throw "missing test mechanical data";
		near(mechanical.youngs, 205000, "modulus in MPa");
		diagnosis("sheet.no-bend-rule", function() { var value = BendDimensions.allowance(2, 1, 1, null); });
		diagnosis("sheet.sharp-length-angle", function() { var value = BendDimensions.leg(10, LengthSpec.InnerSharp, 2, 1, Math.PI); });
		diagnosis("sheet.invalid-k-table", function() { var value = new KFactorTable([1, 1], [0.3, 0.4]); });
		diagnosis("sheet.invalid-k-table", function() { var value = new KFactorTable([0], [0.6]); });
		diagnosis("sheet.invalid-thickness", function() { var value = BendDimensions.allowance(0, 1, 1, rule); });
		diagnosis("sheet.invalid-radius", function() { var value = BendDimensions.allowance(2, -1, 1, rule); });
		diagnosis("sheet.invalid-angle", function() { var value = BendDimensions.allowance(2, 1, Math.NaN, rule); });
		diagnosis("sheet.invalid-flange-length", function() { var value = BendDimensions.leg(1, LengthSpec.OuterSharp, 2, 1, Math.PI / 2); });
		diagnosis("sheet.invalid-k-table", function() { var value = new KFactorTable([], []); });
		diagnosis("sheet.invalid-radius-ratio", function() { var value = table.at(-1); });
		diagnosis("sheet.invalid-mechanical-data", function() { var value = new SheetMaterial("bad", 1, rule, {uts: 200, yield: 300, youngs: 70000}); });
		assertions += SheetCoreSmoke.run();
		assertions += SheetSolidSmoke.run();
		assertions += SheetFeatureStagingSmoke.run();
		assertions += SheetBaseSmoke.run();
		assertions += SheetEdgeSmoke.run();
		assertions += SheetFlangeGeometrySmoke.run();
		assertions += SheetFlangeSmoke.run();
		trace("Sheet metal smoke passed (" + assertions + " assertions)");
	}
}
