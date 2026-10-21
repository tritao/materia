package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.InsertMaterial.InsertMaterialSpec;

enum abstract InsertMakeProcess(String) to String {
	var MachinedSteel = "machined-steel-insert";
}

/** A stock selection is a source design choice, not evidence of delivered stock. */
enum InsertBlank {
	Unselected;
	Stock(reference:String,solid:CastingSolid,partToStock:AssemblyFrame);
}

enum abstract InsertHoleRoute(String) to String {
	var BeforeCasting = "before-casting";
	var TransferAfterCasting = "transfer-after-casting";
}

/** Limits are authored numbers when the inspection is declared. An absent
 * inspection remains a missing requirement; it never becomes a zero tolerance.
 */
enum InsertInspection {
	MaterialIdentity;
	BlankDimensions(toleranceMm:Float);
	HolePosition(holeId:String,toleranceMm:Float);
	ThreadGauge(holeId:String);
	FaceFlatness(interfaceId:String,toleranceMm:Float);
	BondSurface;
}

/** Fabricated-insert manufacturing choices. Operations are derived from these
 * choices and the owning insert's body/holes/interfaces, without storing a second
 * copy of its geometry. The enclosing source document fingerprints all of them.
 */
typedef InsertManufacturing = {
	var material:InsertMaterialSpec;
	var blank:InsertBlank;
	/** Only declared routes are listed; an unlisted source hole has no decided route. */
	var holeRoutes:Array<{holeId:String,route:InsertHoleRoute}>;
	var inspections:Array<InsertInspection>;
	var processReference:Null<String>;
}

class InsertManufacturingCodec {
	/** A complete typed draft with no invented stock, hole route or inspection limit. */
	public static function draft(material:InsertMaterialSpec):InsertManufacturing
		return {material:InsertMaterial.InsertMaterials.snapshot(material),blank:Unselected,
			holeRoutes:[],inspections:[],processReference:null};

	public static function snapshot(raw:Dynamic):InsertManufacturing {
		CastingFeatureCodec.fields(raw,["material","blank","holeRoutes","inspections","processReference"]);
		var blankRaw:Dynamic=Reflect.field(raw,"blank");
		if (Std.isOfType(blankRaw,InsertBlank)) blankRaw=blankRecord(cast blankRaw);
		var blank:InsertBlank=switch CastingRecordCodec.string(blankRaw,"kind") {
			case "unselected":
				CastingFeatureCodec.fields(blankRaw,["kind"]);
				Unselected;
			case "stock":
				CastingFeatureCodec.fields(blankRaw,["kind","reference","solid","partToStock"]);
				Stock(text(blankRaw,"reference"),CastingSolid.CastingSolidCodec.snapshot(Reflect.field(blankRaw,"solid")),
					CastingRecordCodec.frame(Reflect.field(blankRaw,"partToStock")));
			default:throw "Unknown insert blank kind";
		};
		var routes:Array<{holeId:String,route:InsertHoleRoute}> = [],seen:Map<String,Bool> = [];
		for (row in limited(raw,"holeRoutes")) {
			CastingFeatureCodec.fields(row,["holeId","route"]);
			var id=CastingRecordCodec.string(row,"holeId");
			CastingFeatureCodec.identifier(id);
			if (seen.exists(id)) throw "Duplicate insert manufacturing hole route: "+id;
			seen.set(id,true);
			var route:InsertHoleRoute=switch CastingRecordCodec.string(row,"route") {
				case "before-casting":InsertHoleRoute.BeforeCasting;
				case "transfer-after-casting":TransferAfterCasting;
				default:throw "Unknown insert manufacturing hole route";
			};
			routes.push({holeId:id,route:route});
		}
		routes.sort((a,b) -> Reflect.compare(a.holeId,b.holeId));
		var inspections:Array<InsertInspection> = [],keys:Map<String,Bool> = [];
		for (row in limited(raw,"inspections")) {
			if (Std.isOfType(row,InsertInspection)) row=inspectionRecord(cast row);
			var kind=CastingRecordCodec.string(row,"kind");
			var inspection:InsertInspection=switch kind {
				case "material-identity":
					CastingFeatureCodec.fields(row,["kind"]);MaterialIdentity;
				case "bond-surface":
					CastingFeatureCodec.fields(row,["kind"]);BondSurface;
				case "blank-dimensions":
					CastingFeatureCodec.fields(row,["kind","toleranceMm"]);BlankDimensions(tolerance(row));
				case "hole-position":
					CastingFeatureCodec.fields(row,["kind","holeId","toleranceMm"]);HolePosition(identifier(row,"holeId"),tolerance(row));
				case "thread-gauge":
					CastingFeatureCodec.fields(row,["kind","holeId"]);ThreadGauge(identifier(row,"holeId"));
				case "face-flatness":
					CastingFeatureCodec.fields(row,["kind","interfaceId","toleranceMm"]);FaceFlatness(identifier(row,"interfaceId"),tolerance(row));
				default:throw "Unknown insert inspection kind";
			};
			var key=inspectionKey(inspection);
			if (keys.exists(key)) throw "Duplicate insert manufacturing inspection: "+key;
			keys.set(key,true);inspections.push(inspection);
		}
		inspections.sort((a,b) -> Reflect.compare(inspectionKey(a),inspectionKey(b)));
		var reference=CastingRecordCodec.optionalString(raw,"processReference");
		if (reference!=null) validText(reference);
		return {material:InsertMaterial.InsertMaterials.snapshot(Reflect.field(raw,"material")),blank:blank,
			holeRoutes:routes,inspections:inspections,processReference:reference};
	}

	public static function record(plan:InsertManufacturing):Dynamic {
		if (plan==null) throw "Missing insert manufacturing record";
		if (plan.inspections==null || plan.holeRoutes==null) throw "Invalid insert manufacturing lists";
		var copy=snapshot({material:plan.material,blank:blankRecord(plan.blank),holeRoutes:plan.holeRoutes,
			inspections:[for (inspection in plan.inspections) inspectionRecord(inspection)],processReference:plan.processReference});
		return {material:copy.material,blank:blankRecord(copy.blank),holeRoutes:copy.holeRoutes,
			inspections:[for (inspection in copy.inspections) inspectionRecord(inspection)],processReference:copy.processReference};
	}

	public static function references(plan:InsertManufacturing,holes:Array<InsertHole>,interfaces:Array<InterfaceRequirement>):Void {
		var holeIds=[for (hole in holes) hole.id],interfaceIds=[for (requirement in interfaces) requirement.id];
		for (choice in plan.holeRoutes)
			if (holeIds.indexOf(choice.holeId)<0) throw "Manufacturing route names unknown insert hole: "+choice.holeId;
		for (inspection in plan.inspections) switch inspection {
			case HolePosition(id,_):
				if (holeIds.indexOf(id)<0) throw "Inspection names unknown insert hole: "+id;
			case ThreadGauge(id):
				if (holes.filter(hole -> hole.id==id && hole.thread!=null).length!=1)
					throw "Thread inspection requires a source threaded hole: "+id;
			case FaceFlatness(id,_):
				if (interfaceIds.indexOf(id)<0) throw "Inspection names unknown source interface: "+id;
				var requirement=interfaces.filter(requirement -> requirement.id==id)[0];
				switch requirement.geometry {case Plane(_):case Line(_):throw "Flatness inspection requires a plane: "+id;}
			case MaterialIdentity,BlankDimensions(_),BondSurface:
		}
	}

	public static function blankRecord(blank:InsertBlank):Dynamic {
		if (blank==null) throw "Missing insert blank selection";
		return switch blank {
			case Unselected:{kind:"unselected"};
			case Stock(reference,solid,frame):
				validText(reference);
				{kind:"stock",reference:reference,solid:CastingSolid.CastingSolidCodec.record(solid),partToStock:CastingRecordCodec.frame(frame)};
		};
	}
	public static function inspectionRecord(inspection:InsertInspection):Dynamic return switch inspection {
		case MaterialIdentity:{kind:"material-identity"};
		case BondSurface:{kind:"bond-surface"};
		case BlankDimensions(t):{kind:"blank-dimensions",toleranceMm:t};
		case HolePosition(id,t):{kind:"hole-position",holeId:id,toleranceMm:t};
		case ThreadGauge(id):{kind:"thread-gauge",holeId:id};
		case FaceFlatness(id,t):{kind:"face-flatness",interfaceId:id,toleranceMm:t};
	};
	public static function inspectionKey(inspection:InsertInspection):String return switch inspection {
		case MaterialIdentity:"material-identity";
		case BondSurface:"bond-surface";
		case BlankDimensions(_):"blank-dimensions";
		case HolePosition(id,_):"hole-position/"+id;
		case ThreadGauge(id):"thread-gauge/"+id;
		case FaceFlatness(id,_):"face-flatness/"+id;
	};
	static function identifier(raw:Dynamic,field:String):String {
		var id=CastingRecordCodec.string(raw,field);CastingFeatureCodec.identifier(id);return id;
	}
	static function tolerance(raw:Dynamic):Float {
		var value=CastingRecordCodec.number(raw,"toleranceMm");
		if (value<=0) throw "Insert inspection tolerance must be positive";
		return value;
	}
	static function limited(raw:Dynamic,field:String):Array<Dynamic> {
		var rows=CastingRecordCodec.array(raw,field);
		if (rows.length>512) throw "Too many insert manufacturing records: "+field;
		return rows;
	}
	static function validText(value:String):Void {
		if (value==null || StringTools.trim(value).length==0 || materia.assembly.AssemblyCodec.containsNul(value))
			throw "Invalid insert manufacturing text";
	}
	static function text(raw:Dynamic,field:String):String {
		var value=CastingRecordCodec.string(raw,field);validText(value);return value;
	}
}
