package materia.tooling.panel;

import haxe.Json;
import haxe.crypto.Sha256;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingEnums.CoatedFace;
import materia.tooling.ToolingEnums.StockDirection;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.ToolingEnums.ContactFace;
import materia.tooling.ToolingPrism.ToolingPoint2;
import materia.tooling.panel.PanelPlan.PanelOperation;
import materia.tooling.panel.PanelPlan.PanelOperationKind;
import materia.tooling.panel.PanelPlan.PanelSegment;
import materia.tooling.panel.PanelPlan.PanelCurveKind;

/** Strict exact local-part recipe, separate from occurrence placement and qualification. */
class PanelCodec {
	public static function encode(panel:PanelPlan):String {
		var text = ToolingCodec.canonical({format: "materia.tooling-panel", version: 3, panel: snapshot(panel)}) + "\n";
		if (haxe.io.Bytes.ofString(text).length > 262144)
			throw "Panel record exceeds size limit";
		return text;
	}

	public static function fingerprint(panel:PanelPlan):String
		return Sha256.encode(encode(panel));

	public static function decode(text:String):PanelPlan {
		if (text == null || haxe.io.Bytes.ofString(text).length > 262144)
			throw "Panel record exceeds size limit";
		var r:Dynamic = Json.parse(text);
		obj(r, ["format", "version", "panel"]);
		if (s(r, "format") != "materia.tooling-panel" || n(r, "version") != 3)
			throw "Unsupported panel format/version";
		var panel = Reflect.field(r, "panel");
		if (!Std.isOfType(Reflect.field(panel, "contactOpenings"), Array))
			throw "Missing/invalid panel contactOpenings";
		return read(panel);
	}

	public static function snapshot(panel:PanelPlan):PanelPlan
		return read(panel);

	public static function side(v:String):MachiningSide
		return switch v {
			case "positive-z":
				MachiningSide.PositiveZ;
			case "negative-z":
				MachiningSide.NegativeZ;
			default:
				throw "Invalid machining side";
		};

	public static function contact(v:String):ContactFace
		return switch v {
			case "none":
				ContactFace.None;
			case "positive-z":
				ContactFace.PositiveZ;
			case "negative-z":
				ContactFace.NegativeZ;
			default:
				throw "Invalid contact face";
		};

	public static function kind(v:String):PanelOperationKind
		return switch v {
			case "outside-profile":
				OutsideProfile;
			case "inside-profile":
				InsideProfile;
			case "pocket":
				Pocket;
			case "rebate":
				Rebate;
			case "drill":
				Drill;
			default:
				throw "Unsupported panel operation kind";
		};

	static function read(r:Dynamic):PanelPlan {
		obj(r, [
			"id",
			"revision",
			"stock",
			"operations",
			"minimumLigament",
			"contactFace",
			"contactOpenings",
			"labelSide",
			"labelX",
			"labelY"
		]);
		var openings:Array<String> = [];
		for (v in arr(r, "contactOpenings", 128)) {
			if (!Std.isOfType(v, String))
				throw "Invalid contact opening";
			var id:String = cast v;
			identifier(id);
			if (openings.indexOf(id) >= 0)
				throw "Duplicate contact opening";
			openings.push(id);
		}
		openings.sort(Reflect.compare);
		var raw = Reflect.field(r, "stock");
		obj(raw, [
			"materialSpec",
			"nominalThickness",
			"measuredThickness",
			"thicknessVariation",
			"width",
			"height",
			"coatedFace",
			"allowedRotations",
			"directionConstraint"
		]);
		var rotations:Array<Int> = [];
		for (v in arr(raw, "allowedRotations", 2)) {
			var a = n({v: v}, "v");
			if (a != 0 && a != 90)
				throw "Invalid panel rotation";
			if (rotations.indexOf(Std.int(a)) >= 0)
				throw "Duplicate panel rotation";
			rotations.push(Std.int(a));
		}
		rotations.sort(Reflect.compare);
		var t = pos(raw, "measuredThickness"),
			variation = nonneg(raw, "thicknessVariation");
		var coating = ToolingCodec.coatedFace(s(raw, "coatedFace")),
			direction = ToolingCodec.stockDirection(s(raw, "directionConstraint"));
		if (variation >= t || rotations.length == 0 || (direction == StockDirection.WidthAligned && rotations.indexOf(90) >= 0))
			throw "Invalid panel stock";
		var revision = pos(r, "revision");
		if (revision != Math.floor(revision) || revision > 1000000000)
			throw "Invalid panel revision";
		var p:PanelPlan = {
			id: s(r, "id"),
			revision: Std.int(revision),
			stock: {
				materialSpec: s(raw, "materialSpec"),
				nominalThickness: pos(raw, "nominalThickness"),
				measuredThickness: t,
				thicknessVariation: variation,
				width: pos(raw, "width"),
				height: pos(raw, "height"),
				coatedFace: coating,
				allowedRotations: rotations,
				directionConstraint: direction
			},
			operations: [],
			contactOpenings: openings,
			minimumLigament: pos(r, "minimumLigament"),
			contactFace: contact(s(r, "contactFace")),
			labelSide: side(s(r, "labelSide")),
			labelX: n(r, "labelX"),
			labelY: n(r, "labelY")
		};
		identifier(p.id);
		if (p.contactFace != ContactFace.None && coating != CoatedFace.Both && (p.contactFace : String) != (coating : String))
			throw "Uncoated contact face";
		var ids = new Map<String, Bool>(), setups = new Map<String, String>();
		for (o in arr(r, "operations", 128)) {
			var op = operationSnapshot(o), contour = op.contour;
			identifier(op.id);
			identifier(op.setupId);
			if (setups.exists(op.setupId) && setups.get(op.setupId) != (op.side : String))
				throw "Setup contains opposite machining faces";
			setups.set(op.setupId, (op.side : String));
			if (ids.exists(op.id))
				throw "Duplicate panel operation";
			ids.set(op.id, true);
			PanelContours.operation(op);
			if (op.depth > t || (op.through && op.depth != t) || (!op.through && op.depth >= t))
				throw "Invalid cut depth/through";
			if ((op.kind == OutsideProfile || op.kind == InsideProfile) && !op.through)
				throw "Profile operation must be through";
			if ((op.kind == Pocket || op.kind == Rebate) && (op.through || t - op.depth < p.minimumLigament))
				throw "Pocket leaves insufficient shelf";
			if (op.kind != OutsideProfile
				&& p.contactFace != ContactFace.None
				&& (op.through || (op.side : String) == (p.contactFace : String))
				&& openings.indexOf(op.id) < 0)
				throw "Cut pierces contact skin";

			p.operations.push(op);
		}
		for (op in p.operations)
			if (op.accessVia != null) {
				var parents = p.operations.filter(v -> v.id == op.accessVia);
				if (parents.length != 1)
					throw "Unknown operation access pocket";
				var parent = parents[0],
					bounds = PanelGeometry.bounds(op.contour);
				if ((parent.kind != Pocket && parent.kind != Rebate)
					|| parent.through
					|| parent.side != op.side
					|| op.depth <= parent.depth
					|| !PanelGeometry.containsRect(parent.contour, bounds.x, bounds.y, bounds.width, bounds.height))
					throw "Invalid operation access pocket";
			}
		for (id in openings)
			if (!ids.exists(id) || p.operations.filter(o -> o.id == id && o.kind != OutsideProfile).length != 1)
				throw "Unknown contact opening";
		p.operations.sort((a, b) -> Reflect.compare(a.id, b.id));
		var outline = PanelContours.outside(p),
			b = PanelGeometry.bounds(outline);
		if (Math.abs(b.x) > 1e-7 || Math.abs(b.y) > 1e-7)
			throw "Panel blank origin must be lower-left";
		if (!((b.width <= p.stock.width && b.height <= p.stock.height && rotations.indexOf(0) >= 0)
			|| (b.width <= p.stock.height && b.height <= p.stock.width && rotations.indexOf(90) >= 0)))
			throw "Panel exceeds stock";
		for (op in p.operations)
			if (op.kind != OutsideProfile) {
				var q = PanelGeometry.bounds(op.contour),
					m = p.minimumLigament;
				if (op.kind == InsideProfile || op.kind == Pocket || op.kind == Rebate)
					cutterFit(op, outline);
				if (op.kind == Rebate) {
					var meets = false;
					for (a in outline)
						for (c in op.contour)
							if (PanelGeometry.intersections(a, c).length > 0)
								meets = true;
					if (!meets)
						throw "Rebate must have an open side";
				} else if (!PanelGeometry.containsRect(outline, q.x - m, q.y - m, q.width + 2 * m, q.height + 2 * m))
					throw "Insufficient panel edge ligament";
				for (other in p.operations)
					if (other.id < op.id && other.kind != OutsideProfile && op.accessVia != other.id && other.accessVia != op.id) {
						var a = PanelGeometry.bounds(other.contour),
							dx = Math.max(0, Math.max(a.x - q.x - q.width, q.x - a.x - a.width));
						var dy = Math.max(0, Math.max(a.y - q.y - q.height, q.y - a.y - a.height));
						if (Math.sqrt(dx * dx + dy * dy) < m && PanelGeometry.distanceBetween(op.contour, other.contour) < m - 1e-7)
							throw "Insufficient ligament between cuts";
					}
			}
		if (p.contactFace != ContactFace.None && (p.contactFace : String) == (p.labelSide : String))
			throw "Label zone on contact face";
		if (!PanelGeometry.containsRect(outline, p.labelX - 15, p.labelY - 4, 30, 8))
			throw "Label zone outside panel";
		for (op in p.operations)
			if (op.kind != OutsideProfile) {
				var q = PanelGeometry.bounds(op.contour);
				if (q.x < p.labelX + 15 && q.x + q.width > p.labelX - 15 && q.y < p.labelY + 4 && q.y + q.height > p.labelY - 4)
					throw "Label zone intersects cut";
			}
		return p;
	}

	/** Positive curvature is an internal stock corner for a CCW removed region.
	 * Rebate corners on/outside the blank are open approach boundaries.
	 */
	static function cutterFit(op:PanelOperation, outline:Array<PanelSegment>):Void {
		function tangent(seg:PanelSegment, atEnd:Bool):ToolingPoint2 {
			var c = seg.center;
			if (c == null)
				return {x: seg.end.x - seg.start.x, y: seg.end.y - seg.start.y};
			var p = atEnd ? seg.end : seg.start,
				sign = seg.sweep > 0 ? 1.0 : -1.0;
			return {x: -(p.y - c.y) * sign, y: (p.x - c.x) * sign};
		}
		var bounds = PanelGeometry.bounds(op.contour);
		if (Math.min(bounds.width, bounds.height) + 1e-7 < op.toolDiameter)
			throw "Cutter does not fit slot: " + op.id;
		for (i in 0...op.contour.length) {
			var seg = op.contour[i], next = op.contour[(i + 1) % op.contour.length], c = seg.center;
			if (c != null && seg.sweep > 0 && 2 * PanelContours.distance(c, seg.start) + 1e-7 < op.toolDiameter)
				throw "Cutter exceeds internal radius: " + op.id;
			var a = tangent(seg, true), b = tangent(next, false);
			var cross = a.x * b.y - a.y * b.x;
			if (cross <= 1e-8 * Math.sqrt((a.x * a.x + a.y * a.y) * (b.x * b.x + b.y * b.y)))
				continue;
			var open = false;
			if (op.kind == Rebate) {
				for (edge in outline)
					if (PanelGeometry.on(edge, seg.end))
						open = true;
				if (!PanelGeometry.inside(outline, seg.end.x, seg.end.y))
					open = true;
			}
			if (!open)
				throw "Cutter cannot cut sharp internal corner: " + op.id;
		}
	}

	public static function operationSnapshot(o:Dynamic):PanelOperation {
		obj(o, [
			"id",
			"kind",
			"contour",
			"side",
			"setupId",
			"depth",
			"through",
			"toolDiameter",
			"reliefToolDiameter",
			"accessVia"
		]);
		var contour = contourSnapshot(Reflect.field(o, "contour"));
		var relief = Reflect.field(o, "reliefToolDiameter");
		var op:PanelOperation = {
			id: s(o, "id"),
			kind: kind(s(o, "kind")),
			contour: contour,
			side: side(s(o, "side")),
			setupId: s(o, "setupId"),
			depth: pos(o, "depth"),
			through: flag(o, "through"),
			toolDiameter: pos(o, "toolDiameter"),
			reliefToolDiameter: relief == null ? null : pos(o, "reliefToolDiameter"),
			accessVia: Reflect.field(o, "accessVia") == null ? null : s(o, "accessVia")
		};
		identifier(op.id);
		identifier(op.setupId);
		validateToolGeometry(op);
		if (op.kind == InsideProfile || op.kind == Pocket)
			cutterFit(op, []);
		return op;
	}

	static function validateToolGeometry(op:PanelOperation):Void {
		var contour = op.contour;
		if (op.reliefToolDiameter != null) {
			if (op.reliefToolDiameter != op.toolDiameter)
				throw "Relief cutter must match operation tool";
			for (seg in contour) {
				var c = seg.center;
				if (c != null && Math.abs(2 * PanelContours.distance(c, seg.start) - op.toolDiameter) > 1e-7)
					throw "Relief radius does not match cutter";
			}
		}
		if (op.kind == Drill) {
			var first = contour[0], c = first.center;
			if (c == null)
				throw "Drill needs an exact circle";
			var diameter = 2 * PanelContours.distance(c, first.start);
			for (seg in contour) {
				var center = seg.center;
				if (center == null
					|| PanelContours.distance(center, c) > 1e-7
					|| Math.abs(2 * PanelContours.distance(center, seg.start) - diameter) > 1e-7)
					throw "Drill needs an exact circle";
			}
			if (Math.abs(op.toolDiameter - diameter) > 1e-7
				|| Math.abs(PanelGeometry.area(contour) - Math.PI * diameter * diameter / 4) > 1e-6)
				throw "Drill tool does not match circle";
		}
	}

	public static function contourSnapshot(raw:Dynamic):Array<PanelSegment> {
		var result:Array<PanelSegment> = [];
		for (seg in arr({contour: raw}, "contour", 512)) {
			obj(seg, ["kind", "start", "end", "center", "sweep"]);
			var curve:PanelCurveKind = switch s(seg, "kind") {
				case "line": Line;
				case "arc": Arc;
				default: throw "Unsupported panel curve kind";
			};
			var c = Reflect.field(seg, "center");
			result.push({
				kind: curve,
				start: point(Reflect.field(seg, "start")),
				end: point(Reflect.field(seg, "end")),
				center: c == null ? null : point(c),
				sweep: n(seg, "sweep")
			});
		}
		PanelGeometry.validate(result);
		return result;
	}

	static function point(v:Dynamic):ToolingPoint2 {
		obj(v, ["x", "y"]);
		return {x: n(v, "x"), y: n(v, "y")};
	}

	public static function identifier(v:String):Void
		materia.tooling.ToolingIds.validate(v);

	static function obj(r:Dynamic, fields:Array<String>):Void {
		if (r == null || !Reflect.isObject(r) || Std.isOfType(r, Array) || Std.isOfType(r, String))
			throw "Expected panel object";
		for (k in Reflect.fields(r))
			if (fields.indexOf(k) < 0)
				throw 'Unknown panel field $k';
		for (k in fields)
			if (!Reflect.hasField(r, k))
				throw 'Missing panel field $k';
	}

	static function s(r:Dynamic, k:String):String {
		var v:Dynamic = Reflect.field(r, k);
		if (!Std.isOfType(v, String))
			throw 'Expected panel text $k';
		var value:String = cast v;
		if (StringTools.trim(value).length == 0 || value.length > 2048 || materia.assembly.AssemblyCodec.containsNul(value))
			throw "Invalid panel text";
		return value;
	}

	static function n(r:Dynamic, k:String):Float {
		var v:Dynamic = Reflect.field(r, k);
		if ((!Std.isOfType(v, Int) && !Std.isOfType(v, Float)) || !Math.isFinite(v))
			throw 'Expected finite panel number $k';
		return v;
	}

	static function pos(r:Dynamic, k:String):Float {
		var v = n(r, k);
		if (v <= 0)
			throw 'Expected positive panel number $k';
		return v;
	}

	static function nonneg(r:Dynamic, k:String):Float {
		var v = n(r, k);
		if (v < 0)
			throw 'Expected nonnegative panel number $k';
		return v;
	}

	static function flag(r:Dynamic, k:String):Bool {
		var v:Dynamic = Reflect.field(r, k);
		if (!Std.isOfType(v, Bool))
			throw 'Expected panel flag $k';
		return v;
	}

	static function arr(r:Dynamic, k:String, limit:Int):Array<Dynamic> {
		var v:Dynamic = Reflect.field(r, k);
		if (!Std.isOfType(v, Array))
			throw 'Expected panel array $k';
		var a:Array<Dynamic> = cast v;
		if (a.length > limit)
			throw "Panel record count exceeded";
		return a;
	}
}
