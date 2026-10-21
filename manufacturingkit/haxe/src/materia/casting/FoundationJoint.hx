package materia.casting;

enum abstract FoundationJointRoute(String) to String {
	var Grouted = "grouted";
	var Replicated = "replicated";
}

/** Assembly-scoped source interface or hole, never a display label. The field
 * using the reference determines whether it must resolve to a plane or a bore.
 */
typedef FoundationJointReference = materia.assembly.AssemblyFeatureReference;

@:wire typedef FoundationJointSeat = {
	@:id(1) var id:String;
	@:id(2) var lower:FoundationJointReference;
	@:id(3) var upper:FoundationJointReference;
}

@:wire typedef FoundationJointJack = {
	@:id(1) var id:String;
	@:id(2) var screw:String;
	@:id(3) var bearing:FoundationJointReference;
	@:id(4) var receiver:FoundationJointReference;
}

@:wire typedef FoundationJointAnchor = {
	@:id(1) var id:String;
	@:id(2) var screw:String;
	/** A real source tapped HoleInterface. */
	@:id(3) var target:FoundationJointReference;
	/** The real source clearance holes crossed by this screw. */
	@:id(4) var through:Array<FoundationJointReference>;
}

/** Finished assembly design, not a mold process option. Missing declarations
 * remain missing; gaps and tolerances are always resolved design numbers.
 * Physical cure evidence and joint strength belong to process qualification.
 */
@:wire typedef FoundationJoint = {
	@:id(1) var id:String;
	@:id(2) var lowerBody:String;
	@:id(3) var upperBody:String;
	@:id(4) var route:FoundationJointRoute;
	@:id(5) var gapMm:Float;
	@:id(6) var gapToleranceMm:Float;
	@:id(7) var seats:Array<FoundationJointSeat>;
	@:id(8) var jacks:Array<FoundationJointJack>;
	@:id(9) var anchors:Array<FoundationJointAnchor>;
	@:id(10) var cureBeforeTorque:Bool;
}

class FoundationJoints {
	public static function snapshot(raw:Dynamic):FoundationJoint {
		CastingFeatureCodec.fields(raw,["id","lowerBody","upperBody","route","gapMm","gapToleranceMm",
			"seats","jacks","anchors","cureBeforeTorque"]);
		var route:FoundationJointRoute=switch CastingRecordCodec.string(raw,"route") {
			case "grouted":FoundationJointRoute.Grouted;
			case "replicated":FoundationJointRoute.Replicated;
			default:throw "Unknown foundation joint route";
		};
		var result:FoundationJoint={id:id(raw,"id"),lowerBody:path(raw,"lowerBody"),upperBody:path(raw,"upperBody"),
			route:route,gapMm:CastingRecordCodec.number(raw,"gapMm"),gapToleranceMm:CastingRecordCodec.number(raw,"gapToleranceMm"),
			seats:[],jacks:[],anchors:[],cureBeforeTorque:true};
		if (result.lowerBody==result.upperBody) throw "Foundation joint needs two distinct casting bodies";
		if (result.gapMm<=0 || result.gapToleranceMm<0 || result.gapToleranceMm>=result.gapMm)
			throw "Foundation joint requires a positive gap throughout its declared tolerance";
		if (!Std.isOfType(Reflect.field(raw,"cureBeforeTorque"),Bool) || Reflect.field(raw,"cureBeforeTorque")!=true)
			throw "Foundation joint requires cure before final torque";
		var ids:Map<String,Bool> = [],pairs:Map<String,Bool> = [];
		for (row in bounded(raw,"seats")) {
			CastingFeatureCodec.fields(row,["id","lower","upper"]);
			var seat:FoundationJointSeat={id:id(row,"id"),lower:reference(Reflect.field(row,"lower")),upper:reference(Reflect.field(row,"upper"))};
			unique(ids,seat.id,"seat");
			var lower=key(seat.lower),upper=key(seat.upper),pair=lower+"|"+upper;
			if (lower==upper || pairs.exists(pair)) throw "Duplicate or self-referenced foundation joint seat";
			pairs.set(pair,true);result.seats.push(seat);
		}
		ids=[];
		var screws:Map<String,Bool> = [];
		for (row in bounded(raw,"jacks")) {
			CastingFeatureCodec.fields(row,["id","screw","bearing","receiver"]);
			var jack:FoundationJointJack={id:id(row,"id"),screw:path(row,"screw"),bearing:reference(Reflect.field(row,"bearing")),receiver:reference(Reflect.field(row,"receiver"))};
			unique(ids,jack.id,"jack");unique(screws,jack.screw,"jack screw");result.jacks.push(jack);
		}
		ids=[];screws=[];
		for (row in bounded(raw,"anchors")) {
			CastingFeatureCodec.fields(row,["id","screw","target","through"]);
			var anchor:FoundationJointAnchor={id:id(row,"id"),screw:path(row,"screw"),target:reference(Reflect.field(row,"target")),through:[]};
			unique(ids,anchor.id,"anchor");unique(screws,anchor.screw,"anchor screw");
			var holes:Map<String,Bool> = [key(anchor.target)=>true];
			for (value in bounded(row,"through")) {
				var hole=reference(value);unique(holes,key(hole),"anchor hole");anchor.through.push(hole);
			}
			if (anchor.through.length==0) throw "Foundation joint anchor requires source clearance holes";
			anchor.through.sort((a,b) -> Reflect.compare(key(a),key(b)));result.anchors.push(anchor);
		}
		if (result.seats.length==0 || result.jacks.length==0 || result.anchors.length==0)
			throw "Foundation joint requires seats, jacking screws and anchors";
		result.seats.sort((a,b) -> Reflect.compare(a.id,b.id));
		result.jacks.sort((a,b) -> Reflect.compare(a.id,b.id));
		result.anchors.sort((a,b) -> Reflect.compare(a.id,b.id));return result;
	}

	/** Definition IDs remain local. Every assembly-member reference is remapped. */
	public static function remap(authored:FoundationJoint,map:String->String):FoundationJoint {
		var joint=snapshot(authored);
		joint.lowerBody=map(joint.lowerBody);joint.upperBody=map(joint.upperBody);
		for (seat in joint.seats) {seat.lower.member=map(seat.lower.member);seat.upper.member=map(seat.upper.member);}
		for (jack in joint.jacks) {
			jack.screw=map(jack.screw);jack.bearing.member=map(jack.bearing.member);jack.receiver.member=map(jack.receiver.member);
		}
		for (anchor in joint.anchors) {
			anchor.screw=map(anchor.screw);anchor.target.member=map(anchor.target.member);
			for (hole in anchor.through) hole.member=map(hole.member);
		}
		return snapshot(joint);
	}

	static function key(r:FoundationJointReference):String return r.member+"/"+r.feature;
	static function reference(raw:Dynamic):FoundationJointReference {
		CastingFeatureCodec.fields(raw,["member","feature"]);
		return {member:path(raw,"member"),feature:id(raw,"feature")};
	}
	static function unique(ids:Map<String,Bool>,id:String,kind:String):Void {
		if (ids.exists(id)) throw "Duplicate foundation joint "+kind+": "+id;
		ids.set(id,true);
	}
	static function id(raw:Dynamic,name:String):String {
		var value=CastingRecordCodec.string(raw,name);CastingFeatureCodec.identifier(value);return value;
	}
	static function path(raw:Dynamic,name:String):String {
		var value=CastingRecordCodec.string(raw,name);CastingRecordCodec.referencePath(value);return value;
	}
	static function bounded(raw:Dynamic,name:String):Array<Dynamic> {
		var rows=CastingRecordCodec.array(raw,name);
		if (rows.length>256) throw "Too many foundation joint records: "+name;
		return rows;
	}
}
