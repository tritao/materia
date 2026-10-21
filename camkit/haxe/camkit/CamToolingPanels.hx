package camkit;

import materia.tooling.panel.PanelPlan;
import materia.casting.CastingProfile.CastingSegment;
import materia.casting.CastingProfile.CastingCurveKind;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.casting.CastingProfileGeometry;
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
	var onionSkinDepth:Null<Float>;
	var vacuumHold:Null<{
		reference:String,
		areaMm2:Float,
		holdingForceN:Float,
		cuttingForceN:Float,
		safetyFactor:Float
	}>;
	var approachClearance:Float;
	var drillPeckRatio:Float;
	var drillPeckDepth:Float;
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
		function map(p:materia.casting.CastingPrism.CastingPoint2):cadkit.sheetmetal.Point2 {
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
	public static function openRebate(op:PanelOperation, outline:Array<CastingSegment>, radiusMm:Float):PanelOperation {
		var segments:Array<CastingSegment> = [];
		for (segment in op.contour) {
			var shared = outline.filter(edge -> edge.center == null
				&& segment.center == null
				&& CastingProfileGeometry.on(edge, segment.start)
				&& CastingProfileGeometry.on(edge, segment.end));
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
				kind: CastingCurveKind.Line,
				start: segment.start,
				end: a,
				center: null,
				sweep: 0
			});
			segments.push({
				kind: CastingCurveKind.Line,
				start: a,
				end: b,
				center: null,
				sweep: 0
			});
			segments.push({
				kind: CastingCurveKind.Line,
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
		var matchedTools = new Map<Int, Tool>();
		function matched(op:PanelOperation):Tool {
			var nominal = settings.nominalDiameters.filter(t -> Math.abs(t.diameter - op.toolDiameter * 0.001) < 1e-10);
			if (nominal.length != 1)
				throw 'Panel CAM needs exactly one tool for ${op.id} (${op.toolDiameter} mm)';
			var candidates = settings.tools.filter(t -> t.number == nominal[0].number);
			if (candidates.length != 1 || Math.abs(candidates[0].diameter - nominal[0].diameter) > settings.diameterMatchTolerance + 1e-12)
				throw "Panel CAM measured diameter exceeds matching tolerance: " + op.id;
			var tool = candidates[0];
			if (!matchedTools.exists(tool.number))
				matchedTools.set(tool.number,
					new Tool(tool.number, tool.length, tool.diameter, tool.cutter, tool.fluteLength, tool.stickOut, nominal[0].diameter));
			return matchedTools.get(tool.number);
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
			settings.spindleSpinUpSeconds,
			settings.approachClearance,
			settings.drillPeckRatio,
			settings.drillPeckDepth
		])
			if (!Math.isFinite(v) || v <= 0)
				throw "Invalid panel CAM settings";
		if (!Math.isFinite(settings.blankMargin)
			|| settings.blankMargin < 0
			|| settings.stepOverFraction > 1
			|| settings.plungeFeed > settings.feed
			|| settings.travel == null)
			throw "Invalid panel CAM setup";
		if (settings.onionSkinDepth != null
			&& (!Math.isFinite(settings.onionSkinDepth)
				|| settings.onionSkinDepth <= 0
				|| settings.onionSkinDepth >= p.stock.measuredThickness * 0.001))
			throw "Invalid panel onion-skin depth";
		var vacuum = settings.vacuumHold;
		if (vacuum != null) {
			for (value in [vacuum.areaMm2, vacuum.holdingForceN, vacuum.cuttingForceN, vacuum.safetyFactor])
				if (!Math.isFinite(value) || value <= 0)
					throw "Invalid vacuum capacity declaration";
			if (!declared(vacuum.reference)
				|| vacuum.safetyFactor < 1
				|| vacuum.holdingForceN < vacuum.cuttingForceN * vacuum.safetyFactor)
				throw "Vacuum holding force is inadequate for declared cutting force";
		}
		var bounds = CastingProfileGeometry.bounds(PanelContours.outside(p));
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
			ordered = PanelToolOrder.order(operations, op -> matched(op).number);
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
					for (tool in settings.tools)
						if (ordered.filter(op -> matched(op).number == tool.number)
							.length > 0) matched(ordered.filter(op -> matched(op).number == tool.number)[0])
				];
				if (selected.length > 1 && selected.filter(t -> t.length <= 0).length > 0)
					throw "Panel CAM multi-tool programs require measured positive tool lengths";
				var stock = new SetupStock(0, bounds.width * 0.001
					+ 2 * settings.blankMargin, 0, bounds.height * 0.001
					+ 2 * settings.blankMargin, 0,
					-p.stock.measuredThickness * 0.001, settings.safeZ, settings.fixtures, settings.spoilboardPlane,
					ordered.filter(op -> op.through).length > 0 ? settings.breakthroughAllowance : 0);
				var setup = new Setup("1", settings.workOrigin, stock);
				var longest = 0.0;
				for (tool in selected)
					longest = Math.max(longest, tool.length);
				var job = new CamJob(settings.safeZ, settings.spindleRpm, new Point3(0, 0, settings.safeZ + longest), settings.blendTolerance,
					settings.arcTolerance, settings.spindleSpinUpSeconds, true, settings.cutDirection, settings.approachClearance);
				var mirrored = operations[0].side == NegativeZ;
				for (op in ordered) {
					var tool = matched(op);
					if (tool.fluteLength == null || tool.stickOut == null)
						throw "Panel CAM requires declared flute length and stick-out";
					var depth = -(op.depth * 0.001 + (op.through ? settings.breakthroughAllowance : 0));
					if (-depth > tool.fluteLength + 1e-10 || -depth + settings.approachClearance > tool.stickOut + 1e-10)
						throw "Panel CAM depth exceeds flute length/stick-out: " + op.id;
					var source = op.kind == Rebate ? openRebate(op, PanelContours.outside(p), tool.diameter * 500) : op;
					var c = contour(source, settings.chordTolerance, mirrored, bounds.x, bounds.y, bounds.width, settings.blankMargin);
					switch op.kind {
						case OutsideProfile, InsideProfile:
							if (op.kind == OutsideProfile && settings.onionSkinDepth != null) {
								var skinPlane = -p.stock.measuredThickness * 0.001 + settings.onionSkinDepth;
								job.profile(c, tool, skinPlane, settings.feed, "outside", settings.stepDown, settings.plungeFeed, op.id);
								job.profile(c, tool, depth, settings.feed, "outside", skinPlane - depth, settings.plungeFeed, op.id, skinPlane);
							} else
								job.profile(c, tool, depth, settings.feed, op.kind == OutsideProfile ? "outside" : "inside", settings.stepDown,
									settings.plungeFeed, op.id);
						case Pocket, Rebate:
							job.pocket(c, tool, depth, settings.feed, tool.diameter * settings.stepOverFraction, settings.stepDown, settings.plungeFeed, op.id);
						case Drill:
							var b = CastingProfileGeometry.bounds(PanelContours.operation(op));
							var x = b.x + b.width / 2 - bounds.x;
							job.drill([
								new Point3((mirrored ? bounds.width - x : x) * 0.001 + settings.blankMargin,
									(b.y + b.height / 2 - bounds.y) * 0.001 + settings.blankMargin, 0)
							],
								tool, depth, Math.min(settings.safeZ, settings.approachClearance), settings.plungeFeed, op.id,
								-depth > settings.drillPeckRatio * tool.diameter ? settings.drillPeckDepth : 0);
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
				if (ordered.filter(op -> op.kind == OutsideProfile).length > 0
					&& settings.onionSkinDepth == null
					&& settings.vacuumHold == null)
					pending.push("Final outside profile requires onion-skin removal or evidenced vacuum force capacity");
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
