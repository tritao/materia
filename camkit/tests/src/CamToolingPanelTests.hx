import camkit.CamToolingPanels;
import camkit.CamToolingPanels.PanelCamSettings;
import materia.tooling.panel.PanelPlan;
import materia.tooling.panel.PanelStockSpec;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelContours;
import materia.tooling.ToolingEnums.MachiningSide;
import toolpathkit.path.Point3;
import toolpathkit.path.GeometryTools;
import toolpathkit.tool.Tool;
import toolpathkit.setup.TravelEnvelope;
import toolpathkit.setup.Fixture;
import cnckit.CncWriter;
import cnckit.CncController;

class CamToolingPanelTests {
	static var assertions = 0;

	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok)
			throw message;
	}

	static function rejects(fn:Void->Void, message:String):Void {
		var actual = "";
		try
			fn()
		catch (e:Dynamic)
			actual = Std.string(e);
		check(actual.indexOf(message) >= 0, 'Expected "$message", got "$actual"');
	}

	static function settings():PanelCamSettings
		return {
			chordTolerance: 0.00002,
			safeZ: 0.01,
			spindleRpm: 12000,
			feed: 0.01,
			plungeFeed: 0.002,
			stepDown: 0.004,
			stepOverFraction: 0.4,
			blankMargin: 0.01,
			tools: [new Tool(1, 0, 0.002), new Tool(2, 0, 0.004)],
			fixtures: [],
			travel: new TravelEnvelope(new Point3(-0.1, -0.1, -0.1), new Point3(1, 1, 0.1)),
			workOrigin: new Point3(0, 0, 0),
			workholdingReference: null
		};

	static function panel():PanelPlan {
		var stock:PanelStockSpec = {
			materialSpec: "test-board",
			nominalThickness: 12,
			measuredThickness: 12,
			thicknessVariation: 0.05,
			width: 1200,
			height: 800,
			coatedFace: Both,
			allowedRotations: [0, 90],
			directionConstraint: Free
		};
		var p = PanelJoints.blank("cam-panel", 100, 80, stock, 2, 2);
		p.operations.push(PanelJoints.operation("access", Pocket, PanelContours.circle(25, 35, 8), 3, 2, PositiveZ, false));
		var drill = PanelJoints.operation("deep-drill", Drill, PanelContours.circle(25, 35, 2), 12, 4, PositiveZ, true);
		drill.accessVia = "access";
		p.operations.push(drill);
		p.operations.push(PanelJoints.operation("inside", InsideProfile, PanelContours.circle(50, 60, 5), 12, 2, PositiveZ, true));
		p.operations.push(PanelJoints.operation("back-pocket", Pocket, PanelContours.circle(70, 35, 8), 2, 2, NegativeZ, false));
		p.operations.push(PanelJoints.operation("edge-rebate", Rebate, PanelContours.roundedRectangle(0, 5, 10, 15, 2), 2, 2, PositiveZ, false));
		p.labelX = 80;
		p.labelY = 65;
		return p;
	}

	public static function run():Int {
		var p = panel(), s = settings(), setups = CamToolingPanels.build(p, s);
		check(setups.length == 2 && setups[0].reflectedX && !setups[1].reflectedX, "underside setup precedes releasing outside profile");
		var ids = [for (setup in setups) for (id in setup.operationIds) id];
		check(ids.length == p.operations.length, "one CAM call per operation, no omitted kind");
		check(ids.indexOf("access") < ids.indexOf("deep-drill"), "access pocket precedes deeper drilling");
		check(ids[ids.length - 1] == p.operations[0].id, "outside cut is last");
		var featureIds = new Map<String, Bool>();
		for (setup in setups)
			for (op in setup.program.ops)
				switch op {
					case Move(_, _, _, _, span):
						if (span.featureRef != null)
							featureIds.set(span.featureRef, true);
					case _:
				}
		for (op in p.operations)
			check(featureIds.exists(op.id), "CAM provenance includes " + op.id);
		var circle = p.operations.filter(o -> o.id == "access")[0],
			c = CamToolingPanels.contour(circle, s.chordTolerance);
		for (i in 0...c.vertices.length) {
			var a = c.vertices[i], b = c.vertices[(i + 1) % c.vertices.length];
			var radius = Math.sqrt(Math.pow((a.x + b.x) / 2 - 0.025, 2) + Math.pow((a.y + b.y) / 2 - 0.035, 2));
			check(0.008 - radius <= s.chordTolerance + 1e-12, "arc chord respects sagitta tolerance");
		}
		var mirror = CamToolingPanels.contour(circle, s.chordTolerance, true, 0, 0, 100, 0.01);
		check(Math.abs(mirror.vertices[0].x - (0.1 - c.vertices[0].x + 0.01)) < 1e-12, "negative-face XY reflection and blank margin");
		check(c.signedArea * mirror.signedArea < 0, "reflection preserves orientation explicitly");
		var code = CncWriter.write(setups[1].program, new CncController(), s.travel);
		var relieved = PanelJoints.blank("matched-relief", 100, 80, p.stock, 2, 2);
		relieved.operations.push(PanelJoints.operation("relieved", InsideProfile, PanelContours.relievedRectangle(30, 20, 30, 14, 1), 12, 2, PositiveZ, true,
			2));
		var reliefSetups = CamToolingPanels.build(relieved, s);
		for (segment in relieved.operations[1].contour)
			if (segment.center != null) {
				var center = segment.center, min = 1e30;
				var x = center.x * 0.001 + s.blankMargin,
					y = center.y * 0.001 + s.blankMargin;
				for (op in reliefSetups[0].program.ops)
					switch op {
						case Move(Cut, geometry, _, _, span), Move(Plunge, geometry, _, _, span):
							if (span.featureRef == "relieved") {
								var a = GeometryTools.pointAt(geometry, 0),
									b = GeometryTools.pointAt(geometry, GeometryTools.length(geometry));
								var dx = b.x - a.x,
									dy = b.y - a.y,
									length = dx * dx + dy * dy;
								var t = length == 0 ? 0 : Math.max(0, Math.min(1, ((x - a.x) * dx + (y - a.y) * dy) / length));
								min = Math.min(min, Math.sqrt(Math.pow(x - a.x - t * dx, 2) + Math.pow(y - a.y - t * dy, 2)));
							}
						case _:
					}
				check(min < 1e-7, "matched-radius relief reaches its tool-centre lobe, without dropping the corner");
			}
		var inside = p.operations.filter(o -> o.id == "inside")[0];
		var boundary = camkit.CamExactBoundary.offset(CamToolingPanels.contour(inside, s.chordTolerance), 0.001, -0.012, false);
		for (edge in boundary) {
			var a = GeometryTools.pointAt(edge, 0);
			check(Math.abs(Math.sqrt(Math.pow(a.x - 0.05, 2) + Math.pow(a.y - 0.06, 2)) - 0.004) < 1e-7,
				"inside offset applies actual cutter radius exactly once");
		}
		check(code.indexOf("G21") >= 0 && code.indexOf("G54") >= 0, "LinuxCNC uses mm and explicit setup");
		var repeated = CamToolingPanels.build(p, s);
		check(CncWriter.write(repeated[1].program, new CncController(), s.travel) == code, "repeated CAM generation stable");
		for (kind in [InsideProfile, Pocket]) {
			var point = PanelJoints.blank("point-centreline", 100, 80, p.stock, 2, 2);
			point.operations.push(PanelJoints.operation("point-hole", kind, PanelContours.circle(25, 35, 1), kind == Pocket ? 3 : 12, 2, PositiveZ,
				kind != Pocket));
			var built = CamToolingPanels.build(point, s), found = false;
			for (op in built[0].program.ops)
				switch op {
					case Move(Plunge, geometry, _, _, span):
						if (span.featureRef == "point-hole") {
							var end = GeometryTools.pointAt(geometry, GeometryTools.length(geometry));
							if (Math.abs(end.x - 0.035) + Math.abs(end.y - 0.045) + Math.abs(end.z + (kind == Pocket ? 0.003 : 0.012)) < 1e-10)
								found = true;
						}
					case _:
				}
			check(found, "cutter-sized circle uses its verified point centreline for " + kind);
		}
		var long = panel();
		long.id = StringTools.lpad("panel", "p", 128);
		for (op in long.operations)
			op.setupId = StringTools.lpad(op.side == PositiveZ ? "front" : "back", "s", 128);
		var bounded = CamToolingPanels.build(long, s);
		check(bounded[0].id.length <= 128 && bounded[1].id.length <= 128 && bounded[0].id != bounded[1].id, "derived CAM IDs stay bounded and unique");
		s = settings();
		s.tools = [new Tool(1, 0, 0.002), new Tool(1, 0, 0.004)];
		rejects(() -> CamToolingPanels.build(p, s), "numbers must be unique");
		s = settings();
		s.tools = [new Tool(3, 0, 0.003), new Tool(2, 0, 0.004)];
		rejects(() -> CamToolingPanels.build(p, s), "exactly one tool");
		s = settings();
		s.fixtures = [new Fixture("blocked-pocket", 0.02, 0.05, 0.03, 0.06, -0.012, 0.005)];
		rejects(() -> CamToolingPanels.build(p, s), "fixture");
		s = settings();
		s.travel = new TravelEnvelope(new Point3(-0.1, -0.1, -0.1), new Point3(0.03, 0.03, 0.1));
		rejects(() -> CamToolingPanels.build(p, s), "travel");
		s = settings();
		var bad = panel();
		bad.operations[1].depth = 13;
		rejects(() -> CamToolingPanels.build(bad, s), "depth");
		rejects(() -> CamToolingPanels.contour(circle, 0), "positive chord tolerance");
		Sys.println('CamToolingPanelTests: $assertions assertions passed');
		return assertions;
	}
}
