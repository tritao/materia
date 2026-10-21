package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertHole;

/** Nominal hardware facts copied from the selected standard components. These
 * dimensions identify geometry, never pull-out/shear capacity in the compound. */
typedef InsertAnchorScrew = {
	var partNumber:String;
	var thread:InsertThread;
	var lengthMm:Float;
	var threadedLengthMm:Float;
	var headDiameterMm:Float;
	var headHeightMm:Float;
	var socketSizeMm:Float;
	var socketDepthMm:Float;
}

typedef InsertAnchorWasher = {
	var partNumber:String;
	var innerDiameterMm:Float;
	var outerDiameterMm:Float;
	var thicknessMm:Float;
}

enum abstract InsertAnchorLock(String) to String {
	var Shoulder = "shoulder";
	var RetainingCompound = "retaining-compound";
}

/** Limits are source design rules, not unverified evidence or implicit defaults.
 * Embedment and cover refer to the surrounding casting's exposed boundary and
 * must be qualified with that geometry, not estimated from the insert thickness.
 */
typedef InsertAnchorRules = {
	var minimumEmbedmentMm:Float;
	var minimumBearingAreaMm2:Float;
	var minimumSpacingMm:Float;
	var minimumCoverMm:Float;
}

typedef InsertBoltAnchor = {
	var id:String;
	var holeId:String;
	/** Tip insertion measured from the source finished-steel hole entry.
	 * Finishing stock never counts toward permanent thread engagement. */
	var insertionMm:Float;
	var screw:InsertAnchorScrew;
	var washer:InsertAnchorWasher;
	var locking:InsertAnchorLock;
	/** Process reference; a name alone does not establish cure/torque evidence. */
	var lockingReference:Null<String>;
}

enum InsertAnchorage {
	Undeclared;
	Bolted(rules:InsertAnchorRules,anchors:Array<InsertBoltAnchor>);
	CatalogueRetention;
}

class InsertAnchorageCodec {
	public static function snapshot(raw:Dynamic):InsertAnchorage {
		if (Std.isOfType(raw,InsertAnchorage)) raw=record(cast raw);
		return switch CastingRecordCodec.string(raw,"kind") {
			case "undeclared":CastingFeatureCodec.fields(raw,["kind"]);Undeclared;
			case "catalogue-retention":CastingFeatureCodec.fields(raw,["kind"]);CatalogueRetention;
			case "bolted":
				CastingFeatureCodec.fields(raw,["kind","rules","anchors"]);
				var r=Reflect.field(raw,"rules");
				CastingFeatureCodec.fields(r,["minimumEmbedmentMm","minimumBearingAreaMm2","minimumSpacingMm","minimumCoverMm"]);
				var rules:InsertAnchorRules={minimumEmbedmentMm:positive(r,"minimumEmbedmentMm"),
					minimumBearingAreaMm2:positive(r,"minimumBearingAreaMm2"),minimumSpacingMm:positive(r,"minimumSpacingMm"),
					minimumCoverMm:positive(r,"minimumCoverMm")};
				var anchors:Array<InsertBoltAnchor> = [],ids:Map<String,Bool> = [],holes:Map<String,Bool> = [];
				for (a in CastingRecordCodec.array(raw,"anchors")) {
					CastingFeatureCodec.fields(a,["id","holeId","insertionMm","screw","washer","locking","lockingReference"]);
					var id=CastingRecordCodec.string(a,"id"),holeId=CastingRecordCodec.string(a,"holeId");
					CastingFeatureCodec.identifier(id);CastingFeatureCodec.identifier(holeId);
					if (ids.exists(id) || holes.exists(holeId)) throw "Duplicate anchor ID or source hole: "+id;
					ids.set(id,true);holes.set(holeId,true);
					var s=Reflect.field(a,"screw"),w=Reflect.field(a,"washer");
					CastingFeatureCodec.fields(s,["partNumber","thread","lengthMm","threadedLengthMm","headDiameterMm","headHeightMm","socketSizeMm","socketDepthMm"]);
					CastingFeatureCodec.fields(w,["partNumber","innerDiameterMm","outerDiameterMm","thicknessMm"]);
					var thread=InsertThreadCodec.snapshot(Reflect.field(s,"thread"));
					if (thread.internal || thread.profile!=InsertThreadProfile.Metric) throw "Anchor requires an external metric ISO 4762 screw: "+id;
					var screw:InsertAnchorScrew={partNumber:text(s,"partNumber"),thread:thread,lengthMm:positive(s,"lengthMm"),
						threadedLengthMm:positive(s,"threadedLengthMm"),headDiameterMm:positive(s,"headDiameterMm"),headHeightMm:positive(s,"headHeightMm"),
						socketSizeMm:positive(s,"socketSizeMm"),socketDepthMm:positive(s,"socketDepthMm")};
					var washer:InsertAnchorWasher={partNumber:text(w,"partNumber"),innerDiameterMm:positive(w,"innerDiameterMm"),
						outerDiameterMm:positive(w,"outerDiameterMm"),thicknessMm:positive(w,"thicknessMm")};
					if (screw.socketDepthMm>=screw.headHeightMm || screw.socketSizeMm>=screw.headDiameterMm
						|| screw.threadedLengthMm>screw.lengthMm || screw.headDiameterMm<=thread.majorDiameter
						|| washer.innerDiameterMm<=thread.majorDiameter || washer.outerDiameterMm<=washer.innerDiameterMm
						|| screw.headDiameterMm<=washer.innerDiameterMm) throw "Inconsistent anchor hardware: "+id;
					var locking:InsertAnchorLock=switch CastingRecordCodec.string(a,"locking") {
						case "shoulder":InsertAnchorLock.Shoulder;
						case "retaining-compound":InsertAnchorLock.RetainingCompound;
						default:throw "Unknown anchor locking method: "+id;
					};
					var reference=Reflect.field(a,"lockingReference")==null ? null : text(a,"lockingReference");
					var insertion=positive(a,"insertionMm");
					if (insertion+washer.thicknessMm>screw.lengthMm+1e-8) throw "Anchor washer intersects the insert entry: "+id;
					anchors.push({id:id,holeId:holeId,insertionMm:insertion,screw:screw,washer:washer,locking:locking,lockingReference:reference});
				}
				if (anchors.length==0 || anchors.length>256) throw "Bolted anchorage requires 1..256 anchors";
				anchors.sort((a,b) -> Reflect.compare(a.id,b.id));Bolted(rules,anchors);
			default:throw "Unknown insert anchorage kind";
		};
	}

	public static function record(value:InsertAnchorage):Dynamic return switch value {
		case Undeclared:{kind:"undeclared"};
		case CatalogueRetention:{kind:"catalogue-retention"};
		case Bolted(rules,anchors):{kind:"bolted",rules:rules,anchors:anchors};
	};

	public static function references(value:InsertAnchorage,body:InsertSpec.InsertBody,holes:Array<InsertHole>,origin:InsertOrigin,interfaces:Array<InterfaceRequirement>):Void {
		switch value {
			case Undeclared:
			case CatalogueRetention:
				switch origin {case Catalogue(_):default:throw "Catalogue retention requires a catalogue insert";}
				switch body {case CrimpedSocket(_,_):default:throw "Catalogue retention requires declared product retention geometry";}
			case Bolted(_,anchors):
				var back:Null<Float> = null,stock:Null<Box3> = null;
				switch InsertFinishing.assess(body,interfaces) {
					case Resolved(steel):back=steel.pose.z;stock=InsertFinishing.box(steel);
					case Invalid(_),Unsupported(_):
				}
				for (anchor in anchors) {
					var matches=holes.filter(hole -> hole.id==anchor.holeId);
					if (matches.length!=1) throw "Unknown anchor source hole: "+anchor.holeId;
					var hole=matches[0],normal=F.transformVector(hole.frame,0,0,1);
					if (hole.extent!=HoleExtent.Blind || hole.thread==null) throw "Anchor requires a blind source tap: "+anchor.id;
					if ((back!=null && Math.abs(hole.frame.z-back)>1e-8)
						|| Math.abs(normal.x)>1e-10 || Math.abs(normal.y)>1e-10 || normal.z<1-1e-10)
						throw "Anchor source hole must enter the insert back face: "+anchor.id;
					if (stock!=null) try InsertStages.validateServiceHole(hole,stock)
					catch (error:Dynamic) throw "Anchor "+anchor.id+": "+Std.string(error);
					var internal=hole.thread;
					if (internal==null) throw "Missing anchor thread";
					var external=anchor.screw.thread;
					if (internal.majorDiameter!=external.majorDiameter || internal.pitch!=external.pitch
						|| internal.starts!=external.starts || internal.hand!=external.hand || internal.profile!=external.profile)
						throw "Anchor screw differs from its source thread: "+anchor.id;
					switch origin {
						case Fabricated(manufacturing):
							if (manufacturing.holeRoutes.filter(route -> route.holeId==hole.id
								&& route.route==InsertManufacturing.InsertHoleRoute.BeforeCasting).length!=1)
								throw "Anchor source hole must be drilled before casting: "+anchor.id;
						case Catalogue(_):
					}
				}
		}
	}

	public static function occupiedHoleIds(value:InsertAnchorage):Array<String> return switch value {
		case Bolted(_,anchors):[for (anchor in anchors) anchor.holeId];
		case Undeclared,CatalogueRetention:[];
	};

	public static function bearingFrame(hole:InsertHole,anchor:InsertBoltAnchor):AssemblyFrame
		return F.compose(hole.frame,F.translation(0,0,anchor.insertionMm-anchor.screw.lengthMm));

	static function positive(raw:Dynamic,name:String):Float {
		var n=CastingRecordCodec.number(raw,name);if (n<=0) throw "Invalid anchor dimension: "+name;return n;
	}
	static function text(raw:Dynamic,name:String):String {
		var s=CastingRecordCodec.string(raw,name);
		if (StringTools.trim(s).length==0 || materia.assembly.AssemblyCodec.containsNul(s)) throw "Invalid anchor reference: "+name;
		return s;
	}
}
