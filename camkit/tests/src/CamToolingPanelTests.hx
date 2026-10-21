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
			arcTolerance: 0.00002,
			blendTolerance: 0.00002,
			cutDirection: Climb,
			spindleSpinUpSeconds: 2,
			diameterMatchTolerance: 0.00005,
			toolMeasurementReference: "test measured tools",
			singleToolPrograms: false,
			spoilboardPlane: -0.012,
			breakthroughAllowance: 0.001,
			vacuumHold: null,
			onionSkinDepth: null,
			approachClearance: 0.0005,
			drillPeckRatio: 3,
			drillPeckDepth: 0.003,
			travelSpace: MachineCoordinates,
			workOriginReference: "test machine datum",
			nominalDiameters: [
				{
					number: 1,
					diameter: 0.002
				},
				{number: 2, diameter: 0.004}
			],
			safeZ: 0.01,
			spindleRpm: 12000,
			feed: 0.01,
			plungeFeed: 0.002,
			stepDown: 0.004,
			stepOverFraction: 0.4,
			blankMargin: 0.01,
			tools: [
				new Tool(1, 0.05, 0.002, null, 0.025, 0.03),
				new Tool(2, 0.06, 0.004, null, 0.025, 0.03)
			],
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
		p.operations.push(PanelJoints.operation("edge-rebate", Rebate, PanelContours.rectangle(0, 0, 100, 5), 2, 2, PositiveZ, false));
		p.labelX = 80;
		p.labelY = 65;
		return p;
	}

	public static function run():Int {
		var p = panel(), s = settings();
		var setups = CamToolingPanels.build(p, s);
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
		var code = CncWriter.write(setups[1].program, new CncController(cnckit.CncDialect.LinuxCnc, setups[1].program.tools), s.travel);
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
		var lines = code.split("\n");
		var firstRapid = lines.filter(line -> StringTools.startsWith(line, "G0 "))[0];
		check(firstRapid == "G0 Z10", "first physical move is Z-only to safe height");
		var changes = 0;
		for (index in 0...lines.length) {
			if (StringTools.endsWith(lines[index], " M6")) {
				changes++;
				check(StringTools.startsWith(lines[index + 2], "G43 H"), "every measured tool change immediately applies H offset");
			}
			if (StringTools.endsWith(lines[index], " M3"))
				check(lines[index + 1] == "G4 P2", "spindle starts have declared spin-up dwell");
		}
		check(changes == 3, "tool grouping preserves access and outside-last with three necessary tool loads");
		check(code.indexOf("G64 P0.02") >= 0 && code.indexOf("G49\nM2") >= 0, "blended machining and length cancellation are explicit");
		for (index in 0...c.vertices.length) {
			check(Math.abs(mirror.vertices[index].x - (0.1 - c.vertices[index].x + 0.01)) < 1e-12
				&& Math.abs(mirror.vertices[index].y - c.vertices[index].y - 0.01) < 1e-12,
				"full contour reflection includes vertex "
				+ index);
		}
		function area(program:toolpathkit.path.ToolpathProgram, feature:String):Float {
			var sum = 0.0;
			for (op in program.ops)
				switch (op) {
					case Move(Cut, g, _, _, span) if (span.featureRef == feature):
						var a = GeometryTools.pointAt(g, 0);
						for (j in 1...17) {
							var length = GeometryTools.length(g);
							var b = GeometryTools.pointAt(g, j == 16 ? length : length * j / 16);
							sum += a.x * b.y - b.x * a.y;
							a = b;
						}
					case _:
				}
			return sum;
		}
		check(area(setups[1].program, p.operations[0].id) < 0 && area(setups[1].program, "inside") > 0,
			"climb outside is clockwise and climb inside counterclockwise");
		var conventional = settings();
		conventional.cutDirection = Conventional;
		var conventionalPrograms = CamToolingPanels.build(p, conventional);
		check(area(conventionalPrograms[1].program, p.operations[0].id) > 0 && area(conventionalPrograms[1].program, "inside") < 0,
			"conventional cutting reverses both material sides");
		var extended = CamToolingPanels.openRebate(p.operations.filter(op -> op.id == "edge-rebate")[0], PanelContours.outside(p), 1);
		check(materia.casting.CastingProfileGeometry.bounds(extended.contour).x == -1,
			"square rebate's open side extends one cutter radius outside finished outline");
		for (op in setups[1].program.ops)
			switch (op) {
				case Move(Cut, g, _, _, span) if (span.featureRef == p.operations[0].id):
					for (fraction in [0.0, 0.5, 1.0]) {
						var q = GeometryTools.pointAt(g, GeometryTools.length(g) * fraction);
						var dx = Math.max(0, Math.max(0.01 - q.x, q.x - 0.11)),
							dy = Math.max(0, Math.max(0.01 - q.y, q.y - 0.09));
						check(Math.abs(Math.sqrt(dx * dx + dy * dy) - 0.001) < s.arcTolerance * 2, "outside profile compensates actual cutter radius once");
					}
				case _:
			}
		for (corner in [
			new Point3(0.01, 0.01, 0),
			new Point3(0.11, 0.01, 0),
			new Point3(0.01, 0.015, 0),
			new Point3(0.11, 0.015, 0)
		]) {
			var closest = Math.POSITIVE_INFINITY;
			for (op in setups[1].program.ops)
				switch (op) {
					case Move(Cut, g, _, _, span) if (span.featureRef == "edge-rebate"):
						var length = GeometryTools.length(g),
							a = GeometryTools.pointAt(g, 0);
						if (a.z > -0.002 + 1e-9)
							continue;
						for (j in 1...33) {
							var b = GeometryTools.pointAt(g, j == 32 ? length : length * j / 32), dx = b.x - a.x, dy = b.y - a.y;
							var square = dx * dx + dy * dy,
								t = square == 0 ? 0 : Math.max(0, Math.min(1, ((corner.x - a.x) * dx + (corner.y - a.y) * dy) / square));
							closest = Math.min(closest, Math.sqrt(Math.pow(corner.x - a.x - t * dx, 2) + Math.pow(corner.y - a.y - t * dy, 2)));
							a = b;
						}
					case _:
				}
			check(closest <= 0.001 + s.arcTolerance, "square rebate cutter sweep reaches the original outline corner without a lip");
		}
		var measured = settings();
		measured.tools = [
			new Tool(1, 0.05, 0.00196, null, 0.025, 0.03),
			new Tool(2, 0.06, 0.004, null, 0.025, 0.03)
		];
		var measuredBlank = PanelJoints.blank("measured-diameter", 100, 80, p.stock, 2, 2);
		var measuredProgram = CamToolingPanels.build(measuredBlank, measured)[0].program,
			left = Math.POSITIVE_INFINITY;
		for (op in measuredProgram.ops)
			switch (op) {
				case Move(Cut, g, _, _, _):
					left = Math.min(left, GeometryTools.pointAt(g, 0).x);
				case _:
			}
		check(Math.abs(left - (0.01 - 0.00196 / 2)) < 1e-8, "actual measured diameter supplies compensation while nominal diameter selects the tool");
		measured.tools = [
			new Tool(1, 0.05, 0.0022, null, 0.025, 0.03),
			new Tool(2, 0.06, 0.004, null, 0.025, 0.03)
		];
		rejects(() -> CamToolingPanels.build(measuredBlank, measured), "matching tolerance");
		var machineTravel = settings();
		machineTravel.workOrigin = new Point3(1, 0, 0);
		rejects(() -> CamToolingPanels.build(p, machineTravel), "travel");
		var outsideRelief = PanelJoints.blank("outside-relief", 100, 80, p.stock, 2, 2);
		outsideRelief.operations[0].contour = PanelContours.relievedPolygon([
			{x: 0, y: 0},
			{x: 100, y: 0},
			{x: 100, y: 80},
			{x: 60, y: 80},
			{x: 60, y: 60},
			{x: 40, y: 60},
			{x: 40, y: 80},
			{x: 0, y: 80}
		], 1);
		outsideRelief.labelX = 20;
		outsideRelief.labelY = 60;
		var reliefProgram = CamToolingPanels.build(outsideRelief, s)[0].program;
		for (edge in outsideRelief.operations[0].contour)
			if (edge.center != null && edge.sweep < 0) {
				var center = edge.center, found = false;
				for (op in reliefProgram.ops)
					switch (op) {
						case Move(Plunge, g, _, _, span):
							var q = GeometryTools.pointAt(g, GeometryTools.length(g));
							if (Math.abs(q.x - center.x * 0.001 - 0.01) + Math.abs(q.y - center.y * 0.001 - 0.01) < 1e-10)
								found = true;
						case _:
					}
				check(found, "outside concave relief retains its verified cutter-centre entry");
			}
		var negative = setups[1].program.ops.copy();
		for (index in 0...negative.length)
			switch (negative[index]) {
				case Move(Cut, _, feed, tolerance, span):
					negative[index] = Move(Cut, Line(new Point3(0.02, 0.02, -0.014), new Point3(0.03, 0.02, -0.014)), feed, tolerance, span);
					break;
				case _:
			}
		rejects(() -> setups[1].setup.validate(negative, setups[1].program.tools), "below stock bottom/spoilboard allowance");
		var unknown = settings();
		unknown.tools = [
			new Tool(1, 0, 0.002, null, 0.025, 0.03),
			new Tool(2, 0, 0.004, null, 0.025, 0.03)
		];
		rejects(() -> CamToolingPanels.build(p, unknown), "multi-tool programs require measured");
		unknown.singleToolPrograms = true;
		for (batch in CamToolingPanels.build(p, unknown)) {
			var ids = new Map<Int, Bool>();
			for (op in batch.program.ops)
				switch (op) {
					case ToolChange(number, _):
						ids.set(number, true);
					case _:
				}
			check([for (id in ids.keys()) id].length == 1 && batch.readinessPending.length > 0,
				"unmeasured tools are only exported as pending single-tool programs");
		}
		var touchOff = settings();
		touchOff.singleToolPrograms = true;
		touchOff.onionSkinDepth = 0.0005;
		touchOff.tools = [
			new Tool(1, 0, 0.002, null, 0.025, 0.03),
			new Tool(2, 0, 0.004, null, 0.025, 0.03)
		];
		var skin = false, removed = false, pecks = 0;
		for (batch in CamToolingPanels.build(p, touchOff)) {
			var text = CncWriter.write(batch.program, new CncController(cnckit.CncDialect.LinuxCnc, batch.program.tools));
			check(text.indexOf("touch off Z with this tool; no length offset") >= 0 && text.indexOf("G43") < 0 && text.indexOf("G49") >= 0,
				"shipped single-tool touch-off route declares G49 and no length offset");
			check(text.indexOf(" measured ") >= 0 && text.indexOf(", flat)") >= 0, "tool changes describe nominal/measured diameter and cutter shape");
			for (op in batch.program.ops)
				switch op {
					case Move(Cut, g, _, _, span) if (span.featureRef == "outline"):
						var point = GeometryTools.pointAt(g, GeometryTools.length(g));
						if (Math.abs(point.z + 0.0115) < 1e-10)
							skin = true;
						if (Math.abs(point.z + 0.013) < 1e-10)
							removed = true;
					case Move(Plunge, _, _, _, span) if (span.featureRef == "deep-drill"):
						pecks++;
					case _:
				}
		}
		check(skin && removed, "onion skin is left before a separate final removal pass");
		check(pecks >= 5, "deep drilling uses explicit pecks beyond the declared depth/diameter ratio");
		var blindStock:toolpathkit.setup.SetupStock = cast setups[0].setup.stock;
		check(blindStock.breakthroughAllowance == 0, "blind-only setup receives no spoilboard breakthrough allowance");
		var invalidVacuum = settings();
		invalidVacuum.vacuumHold = {
			reference: "bare capacity",
			areaMm2: 100,
			holdingForceN: 10,
			cuttingForceN: 20,
			safetyFactor: 2
		};
		rejects(() -> CamToolingPanels.build(p, invalidVacuum), "holding force is inadequate");
		invalidVacuum.vacuumHold.holdingForceN = 100;
		invalidVacuum.vacuumHold.areaMm2 = 0;
		rejects(() -> CamToolingPanels.build(p, invalidVacuum), "Invalid vacuum capacity");
		var l = new camkit.CamContour([
			      new Point3(0, 0, 0),    new Point3(0.03, 0, 0), new Point3(0.03, 0.01, 0),
			new Point3(0.01, 0.01, 0), new Point3(0.01, 0.03, 0),    new Point3(0, 0.03, 0)
		]);
		for (direction in [camkit.CamCutDirection.Climb, camkit.CamCutDirection.Conventional]) {
			var raster = new camkit.CamJob(0.01, 12000, null, 0, 0, 0, false, direction,
				0.0005).pocket(l, new Tool(1, 0, 0.002), -0.001, 0.01, 0.0008).finish();
			var localLinks = 0, oriented = 0;
			for (op in raster.ops)
				switch op {
					case Move(Cut, Line(a, b), _, _, _):
						if (Math.abs(a.z - 0.0005) < 1e-9 && Math.abs(b.z - 0.0005) < 1e-9)
							localLinks++;
						if (Math.abs(a.z + 0.001) < 1e-9 && Math.abs(b.z + 0.001) < 1e-9 && Math.abs(a.y - b.y) < 1e-9) {
							if (oriented == 0)
								check(direction == camkit.CamCutDirection.Climb ? b.x > a.x : b.x < a.x,
									"first raster clearing pass follows declared direction");
							oriented++;
						}
					case _:
				}
			check(localLinks > 0 && oriented > 0, "adjacent raster passes link over the cleared layer without safe-Z retractions");
		}
		var qualified = settings();
		qualified.fixtures = [new Fixture("declared-vacuum-base", 0.5, 0.51, 0.5, 0.51, -0.02, 0.005)];
		qualified.vacuumHold = {
			reference: "fixture-only vacuum test",
			areaMm2: 10000,
			holdingForceN: 200,
			cuttingForceN: 50,
			safetyFactor: 2
		};
		check(CamToolingPanels.build(p, qualified).filter(batch -> batch.readinessPending.length > 0).length == 0,
			"declared machine coordinates, measured tools, fixtures and vacuum satisfy CAM declaration checks");
		qualified.travelSpace = WorkCoordinates;
		check(CamToolingPanels.build(p, qualified).filter(batch -> batch.readinessPending.length == 0).length == 0,
			"work-coordinate travel never receives a passing CAM qualification");
		var repeated = CamToolingPanels.build(p, s);
		check(CncWriter.write(repeated[1].program, new CncController(cnckit.CncDialect.LinuxCnc, repeated[1].program.tools), s.travel) == code,
			"repeated CAM generation stable");
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
							if (Math.abs(end.x - 0.035) + Math.abs(end.y - 0.045) + Math.abs(end.z + (kind == Pocket ? 0.003 : 0.013)) < 1e-10)
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
		s.tools = [
			new Tool(1, 0, 0.002, null, 0.025, 0.03),
			new Tool(1, 0, 0.004, null, 0.025, 0.03)
		];
		rejects(() -> CamToolingPanels.build(p, s), "numbers must be unique");
		s = settings();
		s.tools = [
			new Tool(3, 0, 0.003, null, 0.025, 0.03),
			new Tool(2, 0, 0.004, null, 0.025, 0.03)
		];
		rejects(() -> CamToolingPanels.build(p, s), "matching tolerance");
		s = settings();
		s.fixtures = [new Fixture("blocked-pocket", 0.02, 0.05, 0.03, 0.06, -0.012, 0.005)];
		rejects(() -> CamToolingPanels.build(p, s), "fixture");
		s = settings();
		s.travel = new TravelEnvelope(new Point3(-0.1, -0.1, -0.1), new Point3(0.03, 0.03, 0.1));
		rejects(() -> CamToolingPanels.build(p, s), "travel");
		s = settings();
		var bad = panel();
		var shortTool = new Tool(1, 0.05, 0.002, null, 0.003, 0.03);
		rejects(() -> new camkit.CamJob(0.01, 12000).profile(CamToolingPanels.contour(bad.operations[0], 0.00002), shortTool, -0.012, 0.01),
			"CAM depth exceeds flute length");
		rejects(() -> CamToolingPanels.contour(circle, 0), "positive chord tolerance");
		Sys.println('CamToolingPanelTests: $assertions assertions passed');
		return assertions;
	}
}
