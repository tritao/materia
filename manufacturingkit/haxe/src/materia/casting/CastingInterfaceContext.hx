package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition.AssemblyVector;
import materia.casting.InterfaceRequirement.InterfaceRequirements;
import materia.casting.InterfaceRelationship;
import materia.casting.InterfaceRelationship.InterfaceRelationships;

/** Facts copied from the machine assembly, expressed in the casting body's
 * source coordinates. External references remain external; no target geometry,
 * tolerance, relationship or machine axis is invented by the tooling planner.
 */
typedef CastingInterfaceContext = {
	var bodyMember:String;
	var inserts:Array<{id:String,member:String}>;
	var relationships:Array<{id:String,relationship:InterfaceRelationship}>;
	var sources:Array<InterfaceContextSource>;
	var foundationJoints:Array<FoundationJointSources.FoundationJointOccurrence>;
	var jointSources:FoundationJointSources;
}

enum InterfaceContextSource {
	Precision(member:String,requirement:InterfaceRequirement,sourceToFoundation:AssemblyFrame);
	Axis(joint:String,frame:AssemblyFrame,direction:AssemblyVector);
}

class CastingInterfaceContexts {
	public static function snapshot(raw:Dynamic):CastingInterfaceContext {
		CastingFeatureCodec.fields(raw,["bodyMember","inserts","relationships","sources","foundationJoints","jointSources"]);
		var body=CastingRecordCodec.string(raw,"bodyMember");
		CastingRecordCodec.referencePath(body);
		var result:CastingInterfaceContext={bodyMember:body,inserts:[],relationships:[],sources:[],
			foundationJoints:FoundationJointSources.FoundationJointSourceCodec.occurrences(Reflect.field(raw,"foundationJoints")),
			jointSources:FoundationJointSources.FoundationJointSourceCodec.snapshot(Reflect.field(raw,"jointSources"))};
		FoundationJointAnalysis.validate(result.jointSources,result.foundationJoints);
		for (instance in result.foundationJoints) if (instance.joint.lowerBody!=body && instance.joint.upperBody!=body)
			throw "Foundation joint does not concern this casting: "+instance.id;
		var ids:Map<String,Bool> = [],members:Map<String,Bool> = [body=>true];
		for (row in bounded(raw,"inserts")) {
			CastingFeatureCodec.fields(row,["id","member"]);
			var id=CastingRecordCodec.string(row,"id"),member=CastingRecordCodec.string(row,"member");
			CastingFeatureCodec.identifier(id);CastingRecordCodec.referencePath(member);
			if (ids.exists(id) || members.exists(member)) throw "Duplicate interface context insert binding: "+id;
			ids.set(id,true);members.set(member,true);result.inserts.push({id:id,member:member});
		}
		result.inserts.sort((a,b) -> Reflect.compare(a.id,b.id));
		var sourceKeys:Map<String,InterfaceContextSource> = [];
		for (row in bounded(raw,"sources")) {
			var source=readSource(row),key=sourceKey(source);
			if (sourceKeys.exists(key)) throw "Duplicate interface context source: "+key;
			sourceKeys.set(key,source);result.sources.push(source);
		}
		result.sources.sort((a,b) -> Reflect.compare(sourceKey(a),sourceKey(b)));
		ids=[];
		var used:Map<String,Bool> = [];
		for (row in bounded(raw,"relationships")) {
			CastingFeatureCodec.fields(row,["id","relationship"]);
			var id=CastingRecordCodec.string(row,"id");
			CastingRecordCodec.referencePath(id);
			if (ids.exists(id)) throw "Duplicate interface relationship occurrence: "+id;
			ids.set(id,true);
			var relationship=InterfaceRelationships.snapshot([Reflect.field(row,"relationship")])[0],local=false;
			for (reference in relationship.members) {
				var key=InterfaceRelationships.memberKey(reference),source=sourceKeys.get(key);
				if (source==null) throw "Missing interface context source: "+id+": "+key;
				used.set(key,true);
				switch source {
					case Precision(member,requirement,_):
						if (requirement.relationshipIds.indexOf(relationship.id)<0)
							throw "Context source omits participating relationship: "+id+": "+key;
						if (members.exists(member)) local=true;
					case Axis(_,_,_):
				}
			}
			if (!local) throw "Interface relationship does not concern the casting: "+id;
			result.relationships.push({id:id,relationship:relationship});
		}
		result.relationships.sort((a,b) -> Reflect.compare(a.id,b.id));
		for (source in result.sources) switch source {
			case Precision(member,requirement,_):
				if (members.exists(member)) {
					for (id in requirement.relationshipIds) {
						var found=false;
						for (instance in result.relationships) if (instance.relationship.id==id)
							for (reference in instance.relationship.members)
								if (InterfaceRelationships.memberKey(reference)==sourceKey(source)) found=true;
						if (!found) throw "Missing casting relationship occurrence: "+member+"/"+requirement.id+": "+id;
					}
				} else if (!used.exists(sourceKey(source))) throw "Unused external interface context source";
			case Axis(_,_,_): if (!used.exists(sourceKey(source))) throw "Unused interface context machine axis";
		}
		return result;
	}

	public static function record(context:CastingInterfaceContext):Dynamic {
		var c=snapshot(context);
		return {bodyMember:c.bodyMember,inserts:c.inserts,foundationJoints:c.foundationJoints,
			jointSources:FoundationJointSources.FoundationJointSourceCodec.record(c.jointSources),
			relationships:[for (instance in c.relationships) {id:instance.id,relationship:InterfaceRelationships.records([instance.relationship])[0]}],
			sources:[for (source in c.sources) sourceRecord(source)]};
	}

	public static function sourceKey(source:InterfaceContextSource):String return switch source {
		case Precision(member,requirement,_): InterfaceRelationships.memberKey(ComponentInterface(member,requirement.id));
		case Axis(joint,_,_): InterfaceRelationships.memberKey(MachineAxis(joint));
	};

	public static function sourceRecord(source:InterfaceContextSource):Dynamic return switch source {
		case Precision(member,requirement,pose):
			{kind:"precision",member:member,requirement:InterfaceRequirements.records([requirement])[0],pose:CastingRecordCodec.frame(pose)};
		case Axis(joint,frame,direction):
			{kind:"axis",joint:joint,frame:CastingRecordCodec.frame(frame),direction:{x:direction.x,y:direction.y,z:direction.z}};
	};

	static function readSource(raw:Dynamic):InterfaceContextSource {
		if (Std.isOfType(raw,InterfaceContextSource)) raw=sourceRecord(cast raw);
		return switch CastingRecordCodec.string(raw,"kind") {
			case "precision":
				CastingFeatureCodec.fields(raw,["kind","member","requirement","pose"]);
				var member=CastingRecordCodec.string(raw,"member");CastingRecordCodec.referencePath(member);
				Precision(member,InterfaceRequirements.snapshot([Reflect.field(raw,"requirement")])[0],CastingRecordCodec.frame(Reflect.field(raw,"pose")));
			case "axis":
				CastingFeatureCodec.fields(raw,["kind","joint","frame","direction"]);
				var joint=CastingRecordCodec.string(raw,"joint");CastingRecordCodec.referencePath(joint);
				var d=Reflect.field(raw,"direction");CastingFeatureCodec.fields(d,["x","y","z"]);
				var direction={x:CastingRecordCodec.number(d,"x"),y:CastingRecordCodec.number(d,"y"),z:CastingRecordCodec.number(d,"z")};
				if (Math.abs(direction.x*direction.x+direction.y*direction.y+direction.z*direction.z-1)>1e-8)
					throw "Interface context machine axis must be unit length";
				Axis(joint,CastingRecordCodec.frame(Reflect.field(raw,"frame")),direction);
			default: throw "Unknown interface context source kind";
		};
	}

	static function bounded(raw:Dynamic,name:String):Array<Dynamic> {
		var rows=CastingRecordCodec.array(raw,name);
		if (rows.length>512) throw "Too many interface context records: "+name;
		return rows;
	}
}
