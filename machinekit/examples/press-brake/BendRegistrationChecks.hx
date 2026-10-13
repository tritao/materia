import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.WallEdge;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.process.ProcessBendRule;
import machinekit.pressbrake.planning.BendOrientation;
import machinekit.pressbrake.planning.BendRegistration;
import machinekit.pressbrake.planning.GaugeEdge;
import machinekit.pressbrake.planning.PressGeometry;
import machinekit.pressbrake.planning.PressSheetPose;

/** Gauge settings come from evaluated contacts; mould-line formulas are independent test oracles. */
class BendRegistrationChecks {
	public static function main():Void run();
	static function near(actual:Float, expected:Float, label:String):Void {
		if (!Math.isFinite(actual) || Math.abs(actual - expected) > 1e-8) throw label + ": " + actual + " != " + expected;
	}
	static function diagnosis(code:String, action:Void->Void):Void {
		try { action(); }
		catch (failure:SheetDiagnosis) { if (failure.code == code) return; throw failure; }
		throw "Expected " + code;
	}
	static function shaped(stock:SheetMaterial, outline:WallOutline):Sheet {
		var sheet = new Sheet(stock);
		var base = sheet.addWall(WallOutline.rectangle(60, 120));
		var child = sheet.addWall(outline);
		var bend = sheet.addBend(base.id, new WallEdge(1), child.id, new WallEdge(3), Math.PI / 2, stock.defaultRadius(Math.PI / 2));
		return sheet;
	}
	public static function run():Void {
		var press = new PressBrake(true, null, null, 30);
		var geometry = PressGeometry.of(press), station = press.toolLayout.stations()[0];
		var tooling = station.processTooling(), rule = new ProcessBendRule(tooling);
		var stock = new SheetMaterial("registration-steel", 1.5, BendRule.Process(rule), {uts: 400, yield: 250, youngs: 205000});
		var sheet = SheetDemoParts.p2(stock), bends = sheet.bends();
		var first = bends[0], second = bends[1];
		var forward = new BendOrientation(true, false, 0);
		var firstProcess = rule.predict(stock, first.angle, first.edgeA.span.length());
		var initial = PressSheetPose.at(sheet, sheet.flat(), first.id, firstProcess, tooling, geometry.station(station.id), forward);
		var edge = new GaugeEdge(first.wallB, new WallEdge(1));
		var registered = BendRegistration.plan(sheet, initial, edge, forward, geometry);
		var expected = 40 - first.radius - stock.thickness + firstProcess.bendAllowance / 2;
		near(registered.x, expected, "First bend equals mould-line flange distance");
		near(registered.r, 0, "Flat stock rests at the finger datum");
		var contacts = registered.contacts();
		if (contacts.length != 2) throw "Both physical fingers must register";
		near(contacts[0].x, -30, "Left contact comes from actual finger setup");
		near(contacts[1].x, 30, "Right contact comes from actual finger setup");
		var reverse = new BendOrientation(true, true, 0);
		var secondProcess = rule.predict(stock, second.angle, second.edgeA.span.length());
		var laterState = sheet.flat().set(first.id, first.angle);
		var laterPose = PressSheetPose.at(sheet, laterState, second.id, secondProcess, tooling, geometry.station(station.id), reverse);
		var later = BendRegistration.plan(sheet, laterPose, edge, reverse, geometry);
		near(later.x, expected + 20, "Later bend equals first setting plus outside flange difference");
		near(later.r, 40, "Lift follows the folded flange's outside height");
		diagnosis("press.gauge-edge-miss", function() {
			var result = BendRegistration.plan(sheet, initial, edge, forward, PressGeometry.of(new PressBrake()));
		});
		diagnosis("press.gauge-edge-miss", function() {
			var result = BendRegistration.plan(sheet, initial, new GaugeEdge(first.wallA, new WallEdge(0)), forward, geometry);
		});
		var wrongEnd = PressSheetPose.at(sheet, sheet.flat(), first.id, firstProcess, tooling, geometry.station(station.id), reverse);
		diagnosis("press.gauge-travel", function() { var result = BendRegistration.plan(sheet, wrongEnd, edge, reverse, geometry); });
		var tapered = shaped(stock, WallOutline.polygon([new Point2(0, 0), new Point2(40, 0), new Point2(60, 120), new Point2(0, 120)]));
		var taperedBend = tapered.bends()[0];
		var taperedProcess = rule.predict(stock, taperedBend.angle, 120);
		var taperedPose = PressSheetPose.at(tapered, tapered.flat(), taperedBend.id, taperedProcess, tooling, geometry.station(station.id), forward);
		diagnosis("press.gauge-incompatible", function() {
			var result = BendRegistration.plan(tapered, taperedPose, new GaugeEdge(taperedBend.wallB, new WallEdge(1)), forward, geometry);
		});
		var rounded = shaped(stock, new WallOutline([
			Segment.line(new Point2(0, 0), new Point2(40, 0)),
			Segment.arc(new Point2(40, 60), 60, -Math.PI / 2, Math.PI),
			Segment.line(new Point2(40, 120), new Point2(0, 120)),
			Segment.line(new Point2(0, 120), new Point2(0, 0))]));
		var roundedBend = rounded.bends()[0], roundedProcess = rule.predict(stock, roundedBend.angle, 120);
		var roundedPose = PressSheetPose.at(rounded, rounded.flat(), roundedBend.id, roundedProcess, tooling, geometry.station(station.id), forward);
		var arc = BendRegistration.plan(rounded, roundedPose, new GaugeEdge(roundedBend.wallB, new WallEdge(1)), forward, geometry);
		near(arc.x, roundedProcess.bendAllowance / 2 + 40 + Math.sqrt(60 * 60 - 30 * 30), "Arc contacts use exact circle intersections");
		near(arc.r, 0, "Arc boundary remains in the stock plane");
		trace("Press backgauge registration checks passed");
	}
}
