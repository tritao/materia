package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef EmbeddedInsertAnchors = {
	var owner:String;
	var pose:AssemblyFrame;
	var spec:InsertSpec;
}

/** Derived source occupancy, copied from the owning inserts when a foundation
 * is built. It is never a second independent anchor layout or a box substitute. */
class EmbeddedAnchors {
	public static function snapshot(raw:Dynamic):Array<EmbeddedInsertAnchors> {
		if (!Std.isOfType(raw,Array)) throw "Invalid embedded anchor list";
		var rows:Array<Dynamic> = cast raw,result:Array<EmbeddedInsertAnchors> = [],seen:Map<String,Bool> = [];
		if (rows.length>512) throw "Too many embedded anchor owners";
		for (row in rows) {
			CastingFeatureCodec.fields(row,["owner","pose","spec"]);
			var owner=CastingRecordCodec.string(row,"owner");CastingFeatureCodec.identifier(owner);
			if (seen.exists(owner)) throw "Duplicate embedded anchor owner: "+owner;seen.set(owner,true);
			var spec=InsertSpec.InsertSpecCodec.snapshot(Reflect.field(row,"spec"));
			switch spec.anchorage {case Bolted(_,_):default:throw "Embedded hardware requires bolted source anchorage: "+owner;}
			result.push({owner:owner,pose:CastingRecordCodec.frame(Reflect.field(row,"pose")),spec:spec});
		}
		result.sort((a,b) -> Reflect.compare(a.owner,b.owner));return result;
	}
	public static function records(rows:Array<EmbeddedInsertAnchors>):Array<Dynamic>
		return [for (row in snapshot(rows)) {owner:row.owner,pose:row.pose,spec:InsertSpec.InsertSpecCodec.record(row.spec)}];
}
