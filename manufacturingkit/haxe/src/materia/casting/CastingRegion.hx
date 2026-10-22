package materia.casting;

import materia.casting.CastingFeature.CastingGeometry;
import materia.casting.CastingProfile.CastingSegment;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Exact finished-region expression. Bounds are stock/layout data only; these
 * operations are evaluated by the geometry backend, without box substitution.
 */
enum CastingRegion {
	Placed(geometry:CastingGeometry);
	Drafted(featureId:String,opening:Array<CastingSegment>,pose:AssemblyFrame,depth:Float,degrees:Float);
	Union(regions:Array<CastingRegion>);
	Intersection(regions:Array<CastingRegion>);
	Difference(body:CastingRegion,cuts:Array<CastingRegion>);
}

/** Strict expression records; the containing document owns its schema version.
 * Commutative operands have canonical order. Size/depth budgets apply across
 * the whole tree, including typed enum inputs, before recursive serialization.
 */
class CastingRegionCodec {
	public static inline var MAX_NODES=2048;
	public static inline var MAX_DEPTH=32;
	public static function snapshot(raw:Dynamic):CastingRegion {
		var budget={nodes:0};
		return read(raw,0,budget);
	}
	public static function record(region:CastingRegion):Dynamic {
		var budget={nodes:0};
		return write(read(region,0,budget));
	}
	static function typed(raw:Dynamic):Dynamic return switch (cast raw:CastingRegion) {
		case Placed(geometry):{kind:"placed",geometry:geometry};
		case Drafted(id,opening,pose,depth,degrees):{kind:"drafted",featureId:id,opening:opening,pose:pose,depth:depth,degrees:degrees};
		case Union(regions):{kind:"union",regions:regions};
		case Intersection(regions):{kind:"intersection",regions:regions};
		case Difference(body,cuts):{kind:"difference",body:body,cuts:cuts};
	};
	static function read(raw:Dynamic,depth:Int,budget:{nodes:Int}):CastingRegion {
		if (depth>MAX_DEPTH || ++budget.nodes>MAX_NODES) throw "Casting region expression exceeds limit";
		if (Std.isOfType(raw,CastingRegion)) raw=typed(raw);
		return switch CastingRecordCodec.string(raw,"kind") {
			case "placed":
				CastingFeatureCodec.fields(raw,["kind","geometry"]);
				var geometry:Dynamic=Reflect.field(raw,"geometry");
				CastingFeatureCodec.fields(geometry,["solid","pose"]);
				CastingRegion.Placed({solid:CastingFeature.CastingGeometrySolids.snapshot(Reflect.field(geometry,"solid")),
					pose:CastingRecordCodec.frame(Reflect.field(geometry,"pose"))});
			case "drafted":
				CastingFeatureCodec.fields(raw,["kind","featureId","opening","pose","depth","degrees"]);
				var id=CastingRecordCodec.string(raw,"featureId"),opening=CastingProfile.CastingProfileCodec.snapshot(Reflect.field(raw,"opening"));
				var pose=CastingRecordCodec.frame(Reflect.field(raw,"pose"));
				var height=CastingRecordCodec.number(raw,"depth"),degrees=CastingRecordCodec.number(raw,"degrees");
				CastingDraftSections.sections(id,opening,pose,height,degrees);
				CastingRegion.Drafted(id,opening,pose,height,degrees);
			case "union":
				CastingFeatureCodec.fields(raw,["kind","regions"]);
				CastingRegion.Union(children(raw,"regions",depth,budget,true));
			case "intersection":
				CastingFeatureCodec.fields(raw,["kind","regions"]);
				CastingRegion.Intersection(children(raw,"regions",depth,budget,true));
			case "difference":
				CastingFeatureCodec.fields(raw,["kind","body","cuts"]);
				var body=read(Reflect.field(raw,"body"),depth+1,budget);
				CastingRegion.Difference(body,children(raw,"cuts",depth,budget,false));
			default: throw "Unknown casting region kind";
		};
	}
	static function children(raw:Dynamic,field:String,depth:Int,budget:{nodes:Int},required:Bool):Array<CastingRegion> {
		var rows=CastingRecordCodec.array(raw,field);
		if (rows.length>MAX_NODES || (required && rows.length==0)) throw "Invalid casting region operand count";
		var result=[for (row in rows) read(row,depth+1,budget)];
		result.sort((a,b) -> Reflect.compare(key(write(a)),key(write(b))));
		return result;
	}
	static function write(region:CastingRegion):Dynamic return switch region {
		case Placed(geometry):{kind:"placed",geometry:{solid:CastingFeature.CastingGeometrySolids.record(geometry.solid),pose:geometry.pose}};
		case Drafted(id,opening,pose,depth,degrees):{kind:"drafted",featureId:id,opening:opening,pose:pose,depth:depth,degrees:degrees};
		case Union(regions):{kind:"union",regions:[for (region in regions) write(region)]};
		case Intersection(regions):{kind:"intersection",regions:[for (region in regions) write(region)]};
		case Difference(body,cuts):{kind:"difference",body:write(body),cuts:[for (region in cuts) write(region)]};
	};
	/** Records contain only plain objects, arrays and scalar values at this point. */
	static function key(raw:Dynamic):String {
		if (Std.isOfType(raw,Array)) {
			var rows:Array<Dynamic> =cast raw;
			return "["+[for (row in rows) key(row)].join(",")+"]";
		}
		if (raw!=null && Reflect.isObject(raw) && !Std.isOfType(raw,String)) {
			var fields=Reflect.fields(raw);fields.sort(Reflect.compare);
			return "{"+[for (field in fields) haxe.Json.stringify(field)+":"+key(Reflect.field(raw,field))].join(",")+"}";
		}
		return haxe.Json.stringify(raw);
	}
}
