package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;

/** The design owns the route; tooling chooses only one of its permitted finishes. */
enum abstract InterfaceRoute(String) to String {
	var AsSupplied = "as-supplied";
	var AsCast = "as-cast";
	var Machined = "machined";
	var Ground = "ground";
	var Replicated = "replicated";
	var AdjustedAtAssembly = "adjusted-at-assembly";
}

enum abstract InterfaceFinish(String) to String {
	var AsSupplied = "as-supplied";
	var AsCast = "as-cast";
	var Machined = "machined";
	var Ground = "ground";
	var JigMaster = "jig-master";
	var RailMaster = "rail-master";
	var SelfLevelled = "self-levelled";
	var SurfacePlateMaster = "surface-plate-master";
	var PushScrews = "push-screws";
	var TaperedGib = "tapered-gib";
	var ReplicatedEdge = "replicated-edge";
}

/** A plane uses frame +Z as normal; a straight reference edge uses frame +X.
 * Frames describe the finished design interface, in the owning part's coordinates.
 */
enum PrecisionGeometry {
	Plane(frame:AssemblyFrame);
	Line(frame:AssemblyFrame);
}

typedef InterfaceRequirement = {
	var id:String;
	var geometry:PrecisionGeometry;
	var route:InterfaceRoute;
	/** Stock above the finished face for machining/grinding, or the layer below
	 * the finished plane for replication. Zero records an explicitly absent allowance;
	 * planning must report the unsatisfied stock/layer requirement, never invent it.
	 */
	var allowanceMm:Float;
	/** Declared finishing error share, independent of geometric stock. Null means
	 * the source accuracy budget is unresolved; it never supplies a default. */
	var finishingProcessErrorMm:Null<Float>;
	var permittedFinishes:Array<InterfaceFinish>;
	var positionToleranceMm:Float;
	var datumReference:String;
	var relationshipIds:Array<String>;
}

/** Defensive source records, with no CAD handles or process evidence. The owning
 * casting document versions these embedded records together with its source geometry.
 */
class InterfaceRequirements {
	public static function snapshot(raw:Dynamic):Array<InterfaceRequirement> {
		if (!Std.isOfType(raw, Array)) throw "Invalid interface requirement list";
		var rows:Array<Dynamic> = cast raw;
		if (rows.length > 512) throw "Too many interface requirements";
		var result:Array<InterfaceRequirement> = [];
		var seen:Map<String, Bool> = [];
		for (row in rows) {
			CastingFeatureCodec.fields(row, ["id", "geometry", "route", "allowanceMm", "permittedFinishes",
				"finishingProcessErrorMm", "positionToleranceMm", "datumReference", "relationshipIds"]);
			var id = CastingRecordCodec.string(row, "id");
			CastingFeatureCodec.identifier(id);
			if (seen.exists(id)) throw "Duplicate interface requirement: " + id;
			seen.set(id, true);
			var g = Reflect.field(row, "geometry");
			if (Std.isOfType(g, PrecisionGeometry)) g = geometryRecord(cast g);
			CastingFeatureCodec.fields(g, ["kind", "frame"]);
			var frame = CastingRecordCodec.frame(Reflect.field(g, "frame"));
			var geometry:PrecisionGeometry = switch CastingRecordCodec.string(g, "kind") {
				case "plane": Plane(frame);
				case "line": Line(frame);
				default: throw "Unknown precision geometry: " + id;
			};
			var route = readRoute(CastingRecordCodec.string(row, "route"));
			var finishes:Array<InterfaceFinish> = [];
			for (value in CastingRecordCodec.array(row, "permittedFinishes")) {
				if (!Std.isOfType(value, String)) throw "Invalid permitted interface finish: " + id;
				var finish = readFinish(cast value);
				if (finishes.indexOf(finish) >= 0) throw "Duplicate permitted interface finish: " + id;
				if (!permits(route, finish)) throw "Interface finish conflicts with its route: " + id;
				finishes.push(finish);
			}
			if (finishes.length == 0) throw "Interface requires permitted finishes: " + id;
			finishes.sort((a, b) -> Reflect.compare((a:String), (b:String)));
			var allowance = CastingRecordCodec.number(row, "allowanceMm");
			var tolerance = CastingRecordCodec.number(row, "positionToleranceMm");
			if (!Reflect.hasField(row,"finishingProcessErrorMm")) throw "Missing finishing process error allocation: " + id;
			var error:Null<Float> = Reflect.field(row,"finishingProcessErrorMm") == null
				? null : CastingRecordCodec.number(row,"finishingProcessErrorMm");
			if (error != null && error < 0) throw "Invalid finishing process error allocation: " + id;
			if (allowance < 0 || tolerance <= 0) throw "Invalid interface allowance or tolerance: " + id;
			if ((route == InterfaceRoute.AsCast || route == InterfaceRoute.AsSupplied) && allowance != 0)
				throw "Interface route cannot carry finishing stock: " + id;
			switch geometry {
				case Line(_):
					if (route != InterfaceRoute.AdjustedAtAssembly) throw "Reference edge requires an assembly adjustment route: " + id;
				case Plane(_):
					if (route == InterfaceRoute.AdjustedAtAssembly) throw "Assembly-adjusted reference requires line geometry: " + id;
			}
			var datum = CastingRecordCodec.string(row, "datumReference");
			CastingFeatureCodec.identifier(datum);
			var relationships = CastingRecordCodec.identifiers(row, "relationshipIds");
			result.push({id: id, geometry: geometry, route: route, allowanceMm: allowance, finishingProcessErrorMm: error,
				permittedFinishes: finishes, positionToleranceMm: tolerance, datumReference: datum,
				relationshipIds: relationships});
		}
		result.sort((a, b) -> Reflect.compare(a.id, b.id));
		return result;
	}

	public static function records(requirements:Array<InterfaceRequirement>):Array<Dynamic> {
		var result:Array<Dynamic> = [];
		for (r in snapshot(requirements)) result.push({id: r.id, geometry: geometryRecord(r.geometry), route: (r.route:String),
			allowanceMm: r.allowanceMm, finishingProcessErrorMm: r.finishingProcessErrorMm, permittedFinishes: [for (f in r.permittedFinishes) (f:String)],
			positionToleranceMm: r.positionToleranceMm, datumReference: r.datumReference, relationshipIds: r.relationshipIds});
		return result;
	}

	/** Remaining combined fixture/casting/metrology budget; no geometric use. */
	public static function fixtureBudgetMm(r:InterfaceRequirement):Null<Float>
		return r.finishingProcessErrorMm == null ? null : r.positionToleranceMm - r.finishingProcessErrorMm;

	public static function lacksDeclaredAllowance(r:InterfaceRequirement):Bool
		return r.allowanceMm == 0 && (r.route == InterfaceRoute.Machined || r.route == InterfaceRoute.Ground || r.route == InterfaceRoute.Replicated);

	public static function geometryFrame(g:PrecisionGeometry):AssemblyFrame return switch g {
		case Plane(frame), Line(frame): CastingRecordCodec.frame(frame);
	};

	public static function geometryRecord(g:PrecisionGeometry):Dynamic return switch g {
		case Plane(frame): {kind: "plane", frame: CastingRecordCodec.frame(frame)};
		case Line(frame): {kind: "line", frame: CastingRecordCodec.frame(frame)};
	};


	public static function readRoute(token:String):InterfaceRoute return switch token {
		case "as-supplied": InterfaceRoute.AsSupplied;
		case "as-cast": InterfaceRoute.AsCast;
		case "machined": InterfaceRoute.Machined;
		case "ground": InterfaceRoute.Ground;
		case "replicated": InterfaceRoute.Replicated;
		case "adjusted-at-assembly": InterfaceRoute.AdjustedAtAssembly;
		default: throw "Unknown interface route: " + token;
	};

	public static function readFinish(token:String):InterfaceFinish return switch token {
		case "as-supplied": InterfaceFinish.AsSupplied;
		case "as-cast": InterfaceFinish.AsCast;
		case "machined": InterfaceFinish.Machined;
		case "ground": InterfaceFinish.Ground;
		case "jig-master": InterfaceFinish.JigMaster;
		case "rail-master": InterfaceFinish.RailMaster;
		case "self-levelled": InterfaceFinish.SelfLevelled;
		case "surface-plate-master": InterfaceFinish.SurfacePlateMaster;
		case "push-screws": InterfaceFinish.PushScrews;
		case "tapered-gib": InterfaceFinish.TaperedGib;
		case "replicated-edge": InterfaceFinish.ReplicatedEdge;
		default: throw "Unknown interface finish: " + token;
	};

	public static function permits(route:InterfaceRoute, finish:InterfaceFinish):Bool return switch route {
		case InterfaceRoute.AsSupplied: finish == InterfaceFinish.AsSupplied;
		case InterfaceRoute.AsCast: finish == InterfaceFinish.AsCast;
		case InterfaceRoute.Machined: finish == InterfaceFinish.Machined;
		case InterfaceRoute.Ground: finish == InterfaceFinish.Ground;
		case InterfaceRoute.Replicated: finish == InterfaceFinish.JigMaster || finish == InterfaceFinish.RailMaster || finish == InterfaceFinish.SelfLevelled || finish == InterfaceFinish.SurfacePlateMaster;
		case InterfaceRoute.AdjustedAtAssembly: finish == InterfaceFinish.PushScrews || finish == InterfaceFinish.TaperedGib || finish == InterfaceFinish.ReplicatedEdge;
		default: false;
	};
}
