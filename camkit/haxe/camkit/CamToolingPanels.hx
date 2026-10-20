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

enum abstract PanelTravelSpace(String) to String {
	var MachineCoordinates = "machine";
	var WorkCoordinates = "work";
}

/** Explicit shop choices, in metres and metres/second. No inferred cutting feeds. */
typedef PanelCamSettings = {
	var chordTolerance:Float;
	var arcTolerance:Float;
	var blendTolerance:Float;
	var cutDirection:CamCutDirection;
	var spindleSpinUpSeconds:Float;
	var nominalDiameters:Array<{number:Int, diameter:Float}>;
	var diameterMatchTolerance:Float;
	var toolMeasurementReference:Null<String>;
	var singleToolPrograms:Bool;
	var spoilboardPlane:Null<Float>;
	var breakthroughAllowance:Float;
	var vacuumHoldReference:Null<String>;
	var travelSpace:PanelTravelSpace;
	var workOriginReference:Null<String>;
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
	var readinessPending:Array<String>;
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
	static function declared(reference:Null<String>):Bool
		return reference != null && StringTools.trim(reference).length > 0;

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

	/** Extend only outside the finished panel; neutral operation geometry is untouched. */
	public static function openRebate(op:PanelOperation, outline:Array<PanelSegment>, radiusMm:Float):PanelOperation {
		var segments:Array<PanelSegment> = [];
		for (segment in op.contour) {
			var shared = outline.filter(edge -> edge.center == null
				&& segment.center == null
				&& PanelGeometry.on(edge, segment.start)
				&& PanelGeometry.on(edge, segment.end));
			if (shared.length == 0) {
				segments.push(segment);
				continue;
			}
			var edge = shared[0],
				dx = edge.end.x - edge.start.x,
				dy = edge.end.y - edge.start.y;
			var length = Math.sqrt(dx * dx + dy * dy),
				nx = dy / length * radiusMm,
				ny = -dx / length * radiusMm;
			var a = {x: segment.start.x + nx, y: segment.start.y + ny},
				b = {x: segment.end.x + nx, y: segment.end.y + ny};
			segments.push({
				kind: Line,
				start: segment.start,
				end: a,
				center: null,
				sweep: 0
			});
			segments.push({
				kind: Line,
				start: a,
				end: b,
				center: null,
				sweep: 0
			});
			segments.push({
				kind: Line,
				start: b,
				end: segment.end,
				center: null,
				sweep: 0
			});
		}
		return {
			id: op.id,
			kind: op.kind,
			contour: segments,
			side: op.side,
			setupId: op.setupId,
			depth: op.depth,
			through: op.through,
			toolDiameter: op.toolDiameter,
			reliefToolDiameter: op.reliefToolDiameter,
			accessVia: op.accessVia
		};
	}

	public static function build(input:PanelPlan, settings:PanelCamSettings):Array<PanelCamSetup> {
		var p = PanelCodec.snapshot(input);
		if (settings.cutDirection != CamCutDirection.Climb && settings.cutDirection != CamCutDirection.Conventional)
			throw "Invalid panel CAM cut direction";
		if (settings.travelSpace != MachineCoordinates && settings.travelSpace != WorkCoordinates)
			throw "Invalid panel CAM travel space";
		if (!Math.isFinite(settings.diameterMatchTolerance)
			|| settings.diameterMatchTolerance < 0
			|| !Math.isFinite(settings.breakthroughAllowance)
			|| settings.breakthroughAllowance < 0
			|| !Math.isFinite(settings.arcTolerance)
			|| settings.arcTolerance <= 0
			|| !Math.isFinite(settings.blendTolerance)
			|| settings.blendTolerance <= 0)
			throw "Invalid panel CAM tolerances";
		function matched(op:PanelOperation):Tool {
			var nominal = settings.nominalDiameters.filter(t -> Math.abs(t.diameter - op.toolDiameter * 0.001) < 1e-10);
			if (nominal.length != 1)
				throw 'Panel CAM needs exactly one tool for ${op.id} (${op.toolDiameter} mm)';
			var candidates = settings.tools.filter(t -> t.number == nominal[0].number);
			if (candidates.length != 1 || Math.abs(candidates[0].diameter - nominal[0].diameter) > settings.diameterMatchTolerance + 1e-12)
				throw "Panel CAM measured diameter exceeds matching tolerance: " + op.id;
			return candidates[0];
		}
		var nominalNumbers = new Map<Int, Bool>();
		for (nominal in settings.nominalDiameters) {
			if (nominal.number <= 0
				|| nominalNumbers.exists(nominal.number)
				|| !Math.isFinite(nominal.diameter)
				|| nominal.diameter <= 0)
				throw "Invalid/duplicate nominal panel tool declaration";
			nominalNumbers.set(nominal.number, true);
		}
		var toolNumbers = new Map<Int, Bool>();
		for (tool in settings.tools) {
			if (!Math.isFinite(tool.diameter) || tool.diameter <= 0 || tool.number <= 0 || tool.length < 0 || toolNumbers.exists(tool.number))
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
			settings.stepOverFraction,
			settings.spindleSpinUpSeconds
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
				var previous = ordered.length == 0 ? -1 : matched(ordered[ordered.length - 1]).number;
				ready.sort((a, b) -> {
					var at = matched(a).number, bt = matched(b).number;
					if ((at == previous) != (bt == previous))
						return at == previous ? -1 : 1;
					return at != bt ? at - bt : Reflect.compare(a.id, b.id);
				});
				ordered.push(ready[0]);
			}
			var batches:Array<Array<PanelOperation>> = [];
			for (op in ordered) {
				if (batches.length == 0
					|| settings.singleToolPrograms
					&& matched(op).number != matched(batches[batches.length - 1][0]).number)
					batches.push([]);
				batches[batches.length - 1].push(op);
			}
			for (batchIndex in 0...batches.length) {
				ordered = batches[batchIndex];
				var selected = [
					for (tool in settings.tools) if (ordered.filter(op -> matched(op).number == tool.number).length > 0) tool
				];
				if (selected.length > 1 && selected.filter(t -> t.length <= 0).length > 0)
					throw "Panel CAM multi-tool programs require measured positive tool lengths";
				var stock = new SetupStock(0, bounds.width * 0.001
					+ 2 * settings.blankMargin, 0, bounds.height * 0.001
					+ 2 * settings.blankMargin, 0,
					-p.stock.measuredThickness * 0.001, settings.safeZ, settings.fixtures, settings.spoilboardPlane, settings.breakthroughAllowance);
				var setup = new Setup("1", settings.workOrigin, stock);
				var longest = 0.0;
				for (tool in selected)
					longest = Math.max(longest, tool.length);
				var job = new CamJob(settings.safeZ, settings.spindleRpm, new Point3(0, 0, settings.safeZ + longest), settings.blendTolerance,
					settings.arcTolerance, settings.spindleSpinUpSeconds, true, settings.cutDirection);
				var mirrored = operations[0].side == NegativeZ;
				for (op in ordered) {
					var tool = matched(op);
					var depth = -(op.depth * 0.001 + (op.through ? settings.breakthroughAllowance : 0));
					var source = op.kind == Rebate ? openRebate(op, PanelContours.outside(p), tool.diameter * 500) : op;
					var c = contour(source, settings.chordTolerance, mirrored, bounds.x, bounds.y, bounds.width, settings.blankMargin);
					switch op.kind {
						case OutsideProfile, InsideProfile:
							job.profile(c, tool, depth, settings.feed, op.kind == OutsideProfile ? "outside" : "inside", settings.stepDown,
								settings.plungeFeed, op.id);
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
				var violations = settings.travelSpace == MachineCoordinates ? settings.travel.check(program) : [];
				if (violations.length > 0)
					throw 'Panel CAM travel: ${violations[0].message()}';
				var pending:Array<String> = [];
				if (settings.fixtures.length == 0)
					pending.push("No fixture geometry declared");
				if (selected.filter(t -> t.length <= 0).length > 0 || !declared(settings.toolMeasurementReference))
					pending.push("Tool lengths/measurement evidence are unqualified");
				if (settings.travelSpace != MachineCoordinates || !declared(settings.workOriginReference))
					pending.push("Travel/work origin is not qualified in machine coordinates");
				if (ordered.filter(op -> op.kind == OutsideProfile).length > 0 && !declared(settings.vacuumHoldReference))
					pending.push("Final outside profile requires declared vacuum-hold qualification");
				result.push({
					id: materia.tooling.ToolingIds.derive("", p.id, "-" + key + (batches.length > 1 ? "-part-" + (batchIndex + 1) : "")),
					side: operations[0].side,
					sourceSetupId: operations[0].setupId,
					operationIds: [for (o in ordered) o.id],
					program: program,
					readinessPending: pending,
					setup: setup,
					reflectedX: mirrored,
					sourceMinX: bounds.x,
					sourceMinY: bounds.y,
					widthMm: bounds.width,
					heightMm: bounds.height
				});
			}
		}
		return result;
	}
}
