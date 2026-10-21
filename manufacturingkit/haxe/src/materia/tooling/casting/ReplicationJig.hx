package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingSolid;
import materia.casting.CastingSolid.CastingSolidCodec;
import materia.casting.CastingRecordCodec as C;
import materia.casting.CastingFeatureCodec as Fields;
import materia.casting.InterfaceRelationship;
import materia.casting.InterfaceRelationship.InterfaceRelationships as Relationships;
import materia.casting.InterfaceRequirement;
import materia.casting.InterfaceRequirement.InterfaceRequirements;

enum abstract ReplicationMasterKind(String) to String {
	var GroundBar = "ground-bar";
	var GroundPlate = "ground-plate";
	var GraniteSquare = "granite-square";
	var AxisMaster = "axis-master";
	var Rail = "rail";
	var SurfacePlate = "surface-plate";
}

typedef ReplicationJack = {
	var id:String;
	/** Contact position in the master's blank coordinates. */
	var position:{x:Float,y:Float,z:Float};
	var lockReference:Null<String>;
}

typedef ReplicationMaster = {
	var id:String;
	var kind:ReplicationMasterKind;
	var body:CastingSolid;
	/** Blank placement on the common jig frame. */
	var pose:AssemblyFrame;
	/** Contact plane (+Z normal), reference edge (+X), or axis (+Z) in the blank. */
	var reference:AssemblyFrame;
	var members:Array<InterfaceMember>;
	var finish:InterfaceFinish;
	var jacks:Array<ReplicationJack>;
	var releaseSpec:Null<String>;
	var flatnessMm:Null<Float>;
	var inspectionReference:Null<String>;
}

typedef ReplicationJig = {
	var id:String;
	var datumSchemeId:String;
	var frameReference:String;
	/** Common rigid frame in the foundation's source coordinates. */
	var pose:AssemblyFrame;
	var masters:Array<ReplicationMaster>;
}

/** Authored process geometry, never a second design interface or tolerance.
 * Records alone do not qualify rigidity, support hardware or measured alignment.
 */
class ReplicationJigCodec {
	public static function snapshot(raw:Dynamic):Array<ReplicationJig> {
		if (!Std.isOfType(raw,Array)) throw "Invalid replication jig list";
		var rows:Array<Dynamic> = cast raw;
		if (rows.length>128) throw "Too many replication jigs";
		var result:Array<ReplicationJig> = [],ids:Map<String,Bool> = [];
		for (row in rows) {
			Fields.fields(row,["id","datumSchemeId","frameReference","pose","masters"]);
			var jig:ReplicationJig={id:id(row,"id"),datumSchemeId:id(row,"datumSchemeId"),
				frameReference:reference(row,"frameReference"),pose:C.frame(Reflect.field(row,"pose")),masters:[]};
			if (ids.exists(jig.id)) throw "Duplicate replication jig: "+jig.id;
			ids.set(jig.id,true);
			var masterIds:Map<String,Bool> = [],members:Map<String,Bool> = [];
			for (m in bounded(row,"masters",128)) {
				Fields.fields(m,["id","kind","body","pose","reference","members","finish","jacks","releaseSpec","flatnessMm","inspectionReference"]);
				var master:ReplicationMaster={id:id(m,"id"),kind:kind(C.string(m,"kind")),
					body:CastingSolidCodec.snapshot(Reflect.field(m,"body")),pose:C.frame(Reflect.field(m,"pose")),
					reference:C.frame(Reflect.field(m,"reference")),members:[],finish:InterfaceRequirements.readFinish(C.string(m,"finish")),jacks:[],
					releaseSpec:optionalReference(m,"releaseSpec"),flatnessMm:null,inspectionReference:optionalReference(m,"inspectionReference")};
				if (masterIds.exists(master.id)) throw "Duplicate replication master: "+master.id;
				masterIds.set(master.id,true);
				if (master.finish!=InterfaceFinish.JigMaster && master.finish!=InterfaceFinish.RailMaster
					&& master.finish!=InterfaceFinish.SurfacePlateMaster) throw "Replication master requires a physical master finish: "+master.id;
				if (master.kind==ReplicationMasterKind.Rail && master.finish!=InterfaceFinish.RailMaster
					|| master.kind==ReplicationMasterKind.SurfacePlate && master.finish!=InterfaceFinish.SurfacePlateMaster
					|| master.kind!=ReplicationMasterKind.Rail && master.kind!=ReplicationMasterKind.SurfacePlate && master.finish!=InterfaceFinish.JigMaster)
					throw "Replication master kind conflicts with its finish: "+master.id;
				if (Reflect.field(m,"flatnessMm")!=null) {
					master.flatnessMm=C.number(m,"flatnessMm");
					if (master.flatnessMm<0) throw "Negative master flatness: "+master.id;
				}
				for (value in bounded(m,"members",512)) {
					var member=Relationships.readMember(value),key=Relationships.memberKey(member);
					if (members.exists(key)) throw "Ambiguous replication jig member: "+key;
					members.set(key,true);master.members.push(member);
				}
				if (master.members.length==0) throw "Replication master requires source members: "+master.id;
				master.members.sort((a,b) -> Reflect.compare(Relationships.memberKey(a),Relationships.memberKey(b)));
				var jackIds:Map<String,Bool> = [];
				for (value in bounded(m,"jacks",16)) {
					Fields.fields(value,["id","position","lockReference"]);
					var position=Reflect.field(value,"position");Fields.fields(position,["x","y","z"]);
					var jack:ReplicationJack={id:id(value,"id"),position:{x:C.number(position,"x"),y:C.number(position,"y"),z:C.number(position,"z")},
						lockReference:optionalReference(value,"lockReference")};
					if (jackIds.exists(jack.id)) throw "Duplicate replication jack: "+jack.id;
					jackIds.set(jack.id,true);master.jacks.push(jack);
				}
				master.jacks.sort((a,b) -> Reflect.compare(a.id,b.id));jig.masters.push(master);
			}
			if (jig.masters.length==0) throw "Replication jig requires masters: "+jig.id;
			jig.masters.sort((a,b) -> Reflect.compare(a.id,b.id));result.push(jig);
		}
		result.sort((a,b) -> Reflect.compare(a.id,b.id));return result;
	}

	public static function records(jigs:Array<ReplicationJig>):Array<Dynamic> return [for (jig in snapshot(jigs)) {
		id:jig.id,datumSchemeId:jig.datumSchemeId,frameReference:jig.frameReference,pose:jig.pose,
		masters:[for (m in jig.masters) {id:m.id,kind:(m.kind:String),body:CastingSolidCodec.record(m.body),pose:m.pose,
			reference:m.reference,members:[for (member in m.members) Relationships.memberRecord(member)],finish:(m.finish:String),
			jacks:m.jacks,releaseSpec:m.releaseSpec,flatnessMm:m.flatnessMm,inspectionReference:m.inspectionReference}]
	}];

	static function bounded(row:Dynamic,name:String,maximum:Int):Array<Dynamic> {
		var values=C.array(row,name);if (values.length>maximum) throw "Too many replication jig records: "+name;return values;
	}
	static function id(row:Dynamic,name:String):String {
		var result=C.string(row,name);Fields.identifier(result);return result;
	}
	static function reference(row:Dynamic,name:String):String {
		var result=C.string(row,name);
		if (StringTools.trim(result).length==0 || result.length>2048 || materia.assembly.AssemblyCodec.containsNul(result))
			throw "Invalid replication reference: "+name;
		return result;
	}
	static function optionalReference(row:Dynamic,name:String):Null<String>
		return Reflect.field(row,name)==null ? null : reference(row,name);
	static function kind(token:String):ReplicationMasterKind return switch token {
		case "ground-bar":ReplicationMasterKind.GroundBar;
		case "ground-plate":ReplicationMasterKind.GroundPlate;
		case "granite-square":ReplicationMasterKind.GraniteSquare;
		case "axis-master":ReplicationMasterKind.AxisMaster;
		case "rail":ReplicationMasterKind.Rail;
		case "surface-plate":ReplicationMasterKind.SurfacePlate;
		default:throw "Unknown replication master kind";
	};
}
