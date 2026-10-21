package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingRecordCodec as C;
import materia.casting.CastingFeatureCodec as Fields;
import materia.casting.InsertSpec.InsertSpecCodec;
import materia.casting.InterfaceRequirement.InterfaceRequirements;

/** Facts copied from source components, in the foundation snapshot's frame.
 * Each component occurs once even when several joints reference it.
 */
typedef FoundationJointBody = {
	var member:String;
	var pose:AssemblyFrame;
	var interfaces:Array<InterfaceRequirement>;
	/** Members owned by explicitly declared InsertRecess owners. */
	var insertMembers:Array<String>;
}
typedef FoundationJointInsert = {var member:String; var pose:AssemblyFrame; var spec:InsertSpec;}
enum abstract FoundationJointScrewKind(String) to String {
	var Iso4762Steel88 = "iso4762-8.8";
}
typedef FoundationJointScrew = {
	var member:String;
	var kind:FoundationJointScrewKind;
	var partNumber:String;
	var thread:InsertHole.InsertThread;
	var shankLength:Float;
	var threadedLength:Float;
	/** Bearing plane with +Z pointing along the actual shank. */
	var bearingFrame:AssemblyFrame;
}
typedef FoundationJointSources = {
	var bodies:Array<FoundationJointBody>;
	var inserts:Array<FoundationJointInsert>;
	var screws:Array<FoundationJointScrew>;
}
typedef FoundationJointOccurrence = {var id:String; var joint:FoundationJoint;}

class FoundationJointSourceCodec {
	public static function empty():FoundationJointSources return {bodies:[],inserts:[],screws:[]};
	public static function snapshot(raw:Dynamic):FoundationJointSources {
		Fields.fields(raw,["bodies","inserts","screws"]);
		var result=empty(),seen:Map<String,Bool> = [];
		for (row in bounded(raw,"bodies")) {
			Fields.fields(row,["member","pose","interfaces","insertMembers"]);
			var body:FoundationJointBody={member:member(row,seen),pose:C.frame(Reflect.field(row,"pose")),
				interfaces:InterfaceRequirements.snapshot(Reflect.field(row,"interfaces")),insertMembers:[]};
			var owners:Map<String,Bool> = [];
			for (value in bounded(row,"insertMembers")) {
				if (!Std.isOfType(value,String)) throw "Invalid foundation joint insert owner";
				var id:String=cast value;C.referencePath(id);
				if (owners.exists(id)) throw "Duplicate foundation joint insert owner: "+id;
				owners.set(id,true);body.insertMembers.push(id);
			}
			body.insertMembers.sort(Reflect.compare);result.bodies.push(body);
		}
		for (row in bounded(raw,"inserts")) {
			Fields.fields(row,["member","pose","spec"]);
			result.inserts.push({member:member(row,seen),pose:C.frame(Reflect.field(row,"pose")),
				spec:InsertSpecCodec.snapshot(Reflect.field(row,"spec"))});
		}
		for (row in bounded(raw,"screws")) {
			Fields.fields(row,["member","kind","partNumber","thread","shankLength","threadedLength","bearingFrame"]);
			if (C.string(row,"kind")!="iso4762-8.8") throw "Unsupported foundation joint screw kind";
			var partNumber=C.string(row,"partNumber");
			if (StringTools.trim(partNumber).length==0 || partNumber.length>2048 || materia.assembly.AssemblyCodec.containsNul(partNumber))
				throw "Invalid foundation joint screw part number";
			var screw:FoundationJointScrew={member:member(row,seen),kind:FoundationJointScrewKind.Iso4762Steel88,partNumber:partNumber,
				thread:InsertThreadCodec.snapshot(Reflect.field(row,"thread")),shankLength:C.number(row,"shankLength"),
				threadedLength:C.number(row,"threadedLength"),bearingFrame:C.frame(Reflect.field(row,"bearingFrame"))};
			if (screw.thread.internal || screw.shankLength<=0 || screw.threadedLength<=0 || screw.threadedLength>screw.shankLength)
				throw "Invalid foundation joint external screw geometry: "+screw.member;
			result.screws.push(screw);
		}
		result.bodies.sort((a,b) -> Reflect.compare(a.member,b.member));
		result.inserts.sort((a,b) -> Reflect.compare(a.member,b.member));
		result.screws.sort((a,b) -> Reflect.compare(a.member,b.member));return result;
	}
	public static function record(source:FoundationJointSources):Dynamic {
		var s=snapshot(source);
		return {bodies:[for (body in s.bodies) {
			member:body.member,pose:body.pose,interfaces:InterfaceRequirements.records(body.interfaces),insertMembers:body.insertMembers
		}],inserts:[for (insert in s.inserts) {
			member:insert.member,pose:insert.pose,spec:InsertSpecCodec.record(insert.spec)
		}],screws:s.screws};
	}
	public static function occurrences(raw:Dynamic):Array<FoundationJointOccurrence> {
		if (!Std.isOfType(raw,Array)) throw "Invalid foundation joint occurrence list";
		var rows:Array<Dynamic> = cast raw;
		if (rows.length>256) throw "Too many foundation joint occurrences";
		var result:Array<FoundationJointOccurrence> = [],seen:Map<String,Bool> = [];
		for (row in rows) {
			Fields.fields(row,["id","joint"]);
			var id=C.string(row,"id");C.referencePath(id);
			if (seen.exists(id)) throw "Duplicate foundation joint occurrence: "+id;
			seen.set(id,true);result.push({id:id,joint:FoundationJoint.FoundationJoints.snapshot(Reflect.field(row,"joint"))});
		}
		result.sort((a,b) -> Reflect.compare(a.id,b.id));return result;
	}
	static function member(row:Dynamic,seen:Map<String,Bool>):String {
		var value=C.string(row,"member");C.referencePath(value);
		if (seen.exists(value)) throw "Duplicate foundation joint source member: "+value;
		seen.set(value,true);return value;
	}
	static function bounded(raw:Dynamic,name:String):Array<Dynamic> {
		var rows=C.array(raw,name);if (rows.length>512) throw "Too many foundation joint source records";return rows;
	}
}
