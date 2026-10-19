package camkit;

import materia.tooling.panel.PanelPlan;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelGeometry;
import materia.tooling.ToolingEnums.MachiningSide;
import toolpathkit.path.Point3;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.tool.Tool;
import toolpathkit.setup.Setup;
import toolpathkit.setup.SetupStock;
import toolpathkit.setup.Fixture;
import toolpathkit.setup.TravelEnvelope;

/** Explicit shop choices, in metres and metres/second. No inferred cutting feeds. */
typedef PanelCamSettings = {
	var chordTolerance:Float;
	var safeZ:Float;
	var spindleRpm:Float;
	var feed:Float;
	var plungeFeed:Float;
	var stepDown:Float;
	var stepOverFraction:Float;
	var blankMargin:Float;
	var tools:Array<Tool>;
	var fixtures:Array<Fixture>;
	var travel:TravelEnvelope;
	var workOrigin:Point3;
	var workholdingReference:Null<String>;
}

typedef PanelCamSetup = {
	var id:String;
	var side:MachiningSide;
	var sourceSetupId:String;
	var operationIds:Array<String>;
	var program:ToolpathProgram;
	var setup:Setup;

	/** XY reflection about the finished blank midline when machining NegativeZ. */
	var reflectedX:Bool;

	var sourceMinX:Float;
	var sourceMinY:Float;
	var widthMm:Float;
	var heightMm:Float;
}

/** Finished panel contours -> one CamJob operation per record. Compensation stays in CamJob. */
class CamToolingPanels {
	public static function contour(op:PanelOperation, tolerance:Float, mirrorX:Bool = false, minX:Float = 0, minY:Float = 0, widthMm:Float = 0,
			margin:Float = 0):CamContour {
		if (!Math.isFinite(tolerance) || tolerance <= 0)
			throw "Panel CAM needs positive chord tolerance";
		var points:Array<Point3> = [];
		var exact:Array<cadkit.sheetmetal.Segment> = [];
		function map(p:materia.tooling.ToolingPrism.ToolingPoint2):cadkit.sheetmetal.Point2 {
			var x = p.x - minX;
			return new cadkit.sheetmetal.Point2((mirrorX ? widthMm - x : x) * 0.001 + margin, (p.y - minY) * 0.001 + margin);
		}
		for (s in PanelContours.operation(op)) {
			var a = map(s.start), b = map(s.end);
			if (s.center == null)
				exact.push(cadkit.sheetmetal.Segment.line(a, b));
			else {
				var center = map(s.center);
				exact.push(cadkit.sheetmetal.Segment.arc(center, PanelContours.distance(s.center, s.start) * 0.001,
					Math.atan2(a.y - center.y, a.x - center.x), s.sweep * (mirrorX ? -1 : 1)));
			}
			var count = 1;
			if (s.center != null) {
				var radius = PanelContours.distance(s.center, s.start) * 0.001;
				var angle = 2 * Math.acos(Math.max(-1, Math.min(1, 1 - tolerance / radius)));
				if (angle <= 0)
					throw "Panel CAM chord tolerance is below numeric resolution";
				count = Std.int(Math.ceil(Math.abs(s.sweep) / Math.min(Math.PI / 2, angle)));
			}
			if (count > 10000 || points.length + count > 50000)
				throw "Panel CAM contour exceeds sampling limit";
			for (i in 0...count) {
				var p = PanelContours.point(s, i / count);
				var x = p.x - minX;
				points.push(new Point3((mirrorX ? widthMm - x : x) * 0.001 + margin, (p.y - minY) * 0.001 + margin, 0));
			}
		}
		var result = new CamContour(points);
		result.exactBoundary = exact;
		result.chordTolerance = tolerance;
		return result;
	}

	public static function build(input:PanelPlan, settings:PanelCamSettings):Array<PanelCamSetup> {
		var p = PanelCodec.snapshot(input);
		var toolNumbers = new Map<Int, Bool>();
		for (tool in settings.tools) {
			if (tool.diameter <= 0 || toolNumbers.exists(tool.number))
				throw "Panel CAM tool numbers must be unique and diameters positive";
			toolNumbers.set(tool.number, true);
		}

		for (v in [
			settings.chordTolerance,
			settings.safeZ,
			settings.spindleRpm,
			settings.feed,
			settings.plungeFeed,
			settings.stepDown,
			settings.stepOverFraction
		])
			if (!Math.isFinite(v) || v <= 0)
				throw "Invalid panel CAM settings";
		if (!Math.isFinite(settings.blankMargin)
			|| settings.blankMargin < 0
			|| settings.stepOverFraction > 1
			|| settings.plungeFeed > settings.feed
			|| settings.travel == null)
			throw "Invalid panel CAM setup";
		var bounds = PanelGeometry.bounds(PanelContours.outside(p));
		var groups = new Map<String, Array<PanelOperation>>();
		for (op in p.operations) {
			var key = op.side + "-" + op.setupId;
			if (!groups.exists(key))
				groups.set(key, []);
			groups.get(key).push(op);
			if (op.accessVia != null) {
				var parent = p.operations.filter(o -> o.id == op.accessVia)[0];
				if (parent.side != op.side || parent.setupId != op.setupId)
					throw "Panel CAM access pocket must be in the same setup";
			}
		}
		var keys = [for (k in groups.keys()) k];
		keys.sort((a, b) -> {
			var ao = groups.get(a).filter(o -> o.kind == OutsideProfile).length;
			var bo = groups.get(b).filter(o -> o.kind == OutsideProfile).length;
			return ao == bo ? Reflect.compare(a, b) : ao - bo;
		});
		var result:Array<PanelCamSetup> = [];
		for (key in keys) {
			var operations = groups.get(key),
				ordered:Array<PanelOperation> = [];
			operations.sort((a, b) -> Reflect.compare(a.id, b.id));
			while (ordered.length < operations.length) {
				var ready = operations.filter(o -> ordered.indexOf(o) < 0
					&& (o.accessVia == null || ordered.filter(parent -> parent.id == o.accessVia).length == 1)
					&& (o.kind != OutsideProfile || ordered.length == operations.length - 1));
				if (ready.length == 0)
					throw "Cyclic panel CAM operation prerequisites";
				ordered.push(ready[0]);
			}
			var stock = new SetupStock(0, bounds.width * 0.001
				+ 2 * settings.blankMargin, 0, bounds.height * 0.001
				+ 2 * settings.blankMargin, 0,
				-p.stock.measuredThickness * 0.001, settings.safeZ, settings.fixtures);
			var setup = new Setup("1", settings.workOrigin, stock);
			var job = new CamJob(settings.safeZ, settings.spindleRpm, new Point3(0, 0, settings.safeZ));
			var mirrored = operations[0].side == NegativeZ;
			for (op in ordered) {
				var tools = settings.tools.filter(t -> Math.abs(t.diameter - op.toolDiameter * 0.001) < 1e-10);
				if (tools.length != 1)
					throw 'Panel CAM needs exactly one tool for ${op.id} (${op.toolDiameter} mm)';
				var tool = tools[0], depth = -op.depth * 0.001;
				var c = contour(op, settings.chordTolerance, mirrored, bounds.x, bounds.y, bounds.width, settings.blankMargin);
				switch op.kind {
					case OutsideProfile, InsideProfile:
						job.profile(c, tool, depth, settings.feed, op.kind == OutsideProfile ? "outside" : "inside", settings.stepDown, settings.plungeFeed,
							op.id);
					case Pocket, Rebate:
						job.pocket(c, tool, depth, settings.feed, tool.diameter * settings.stepOverFraction, settings.stepDown, settings.plungeFeed, op.id);
					case Drill:
						var b = PanelGeometry.bounds(PanelContours.operation(op));
						var x = b.x + b.width / 2 - bounds.x;
						job.drill([
							new Point3((mirrored ? bounds.width - x : x) * 0.001 + settings.blankMargin,
								(b.y + b.height / 2 - bounds.y) * 0.001 + settings.blankMargin, 0)
						], tool, depth, settings.safeZ, settings.plungeFeed, op.id);
					default:
						throw "Unsupported panel CAM operation kind";
				}
			}
			var program = job.finish(setup);
			setup.validate(program.ops, program.tools);
			var violations = settings.travel.check(program);
			if (violations.length > 0)
				throw 'Panel CAM travel: ${violations[0].message()}';
			result.push({
				id: materia.tooling.ToolingIds.derive("", p.id, "-" + key),
				side: operations[0].side,
				sourceSetupId: operations[0].setupId,
				operationIds: [for (o in ordered) o.id],
				program: program,
				setup: setup,
				reflectedX: mirrored,
				sourceMinX: bounds.x,
				sourceMinY: bounds.y,
				widthMm: bounds.width,
				heightMm: bounds.height
			});
		}
		return result;
	}
}
