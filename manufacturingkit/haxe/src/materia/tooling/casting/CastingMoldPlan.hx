package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingDocument;
import materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;
import materia.tooling.panel.PanelAssemblyPlan.PanelPlacement;
import materia.tooling.casting.CoreSupport.SupportHardware;

typedef MoldBox = materia.casting.Box3;

enum abstract PenetrationClass(String) to String {
	var OutsideCasting = "outside-casting";
	var InsertSealed = "insert-sealed";
	var CoreSealed = "core-sealed";
}

typedef PenetrationPlan = {
	var panelId:String;
	var operationId:String;
	var kind:PenetrationClass;
	var insertId:Null<String>;
	@:optional var sourceHoleId:String;
	var coreId:Null<String>;
	var sealMargin:Float;
	var sealMaterial:String;
	var sealingFace:Null<AssemblyFrame>;
	var plug:Null<{
		diameter:Float,
		length:Float,
		materialSpec:String
	}>;
}

enum abstract CoreRemoval(String) to String {
	var BreakoutThroughFill = "breakout-through-fill";
	var BreakoutThroughBed = "breakout-through-bed";
}

enum MoldConstruction {
	SharedBridges;
	SinglePocket;
}

/** Special sealing details belong to the same opening as its coverage record. */
typedef OpeningProtection = {
	var kind:PenetrationClass;
	var insertId:Null<String>;
	var sourceHoleId:Null<String>;
	var coreId:Null<String>;
	var sealMargin:Float;
	var sealingFace:Null<AssemblyFrame>;
	var plug:Null<{diameter:Float,length:Float,materialSpec:String}>;
}

enum abstract MoldPartMaterial(String) to String {
	var EPSStock = "eps";
	var SteelStock = "steel";
	var FormerStock = "former";
}

typedef MoldVolumeProfile = {
	var panelId:String;
	var operationId:String;
	var contour:Array<materia.casting.CastingProfile.CastingSegment>;
	var pose:materia.assembly.AssemblyRecord.AssemblyFrame;
	var depth:Float;
	var subtract:Array<materia.casting.CastingRegion>;
}

typedef MoldVolumeCut = {
	var operation:materia.tooling.panel.PanelPlan.PanelOperation;
	var pose:AssemblyFrame;
}

typedef MoldVolume = {
	var id:String;
	var material:MoldPartMaterial;
	var pieces:Array<MoldBox>;
	var regions:Array<materia.casting.CastingRegion>;
	var profiles:Array<MoldVolumeProfile>;
	var cuts:Array<MoldVolumeCut>;
}

enum abstract FeatureToolingKind(String) to String {
	var EPSFeature = "eps";
	var FormerFeature = "former";
	var InsertFeature = "insert";
	var UnsupportedFeature = "unsupported";
}

typedef FeatureTooling = {
	var featureId:String;
	var kind:FeatureToolingKind;
	var memberIds:Array<String>;
}

typedef CastingMoldCore = {
	var id:String;
	var featureId:String;
	var cavity:materia.casting.CastingRegion;
	/** Lined foam envelope, including the declared inter-piece adhesive gaps. */
	var foamEnvelope:materia.casting.CastingRegion;
	var layers:Array<materia.tooling.casting.CoreSupport.CoreCutting>;
	var coveringThickness:Float;
	var support:materia.tooling.casting.CoreSupport;
}

/** One source core intent, with all foam pieces created around retained supports. */
typedef CastingCoreUnit = {
	var id:String;
	var parentFeatureId:String;
	var featureIds:Array<String>;
	var coreIds:Array<String>;
}

typedef MoldFormer = {
	var volume:MoldVolume;
	var featureIds:Array<String>;
	var withdrawal:{x:Float, y:Float, z:Float};
	var draftDegrees:Float;
}

enum abstract MoldOpeningKind(String) to String {
	var ExteriorOpening = "outside-casting";
	var ToolingOpening = "tooling-sealed";
}

typedef MoldOpening = {
	var panelId:String;
	var operationId:String;
	var kind:MoldOpeningKind;
	var sealingIds:Array<String>;
	var sealSpec:String;
	var protection:Null<OpeningProtection>;
}

typedef PrecisionInterface = {
	var insertId:String;
	var jigReference:Null<String>;
	var masterReference:Null<String>;
	var masterFlatnessMm:Null<Float>;
	var releaseSpec:Null<String>;
	var holeProtectionReference:Null<String>;
	var evidence:Array<String>;
}

enum abstract CompactionMethod(String) to String {
	var Undeclared = "undeclared";
	var VibrationTable = "vibration-table";
	var ExternalVibrators = "external-vibrators";
	var InternalPoker = "internal-poker";
	var NoCompaction = "none";
}

typedef CompactionSpec = {
	var method:CompactionMethod;
	var bedInterfaceReference:Null<String>;
	var frequencyHz:Null<Float>;
	var amplitudeMm:Null<Float>;
	var durationSeconds:Null<Float>;
	var evidence:Array<String>;
}

enum abstract DesignChangeKind(String) to String {
	var LiftingPoints = "lifting-points";
	var ReleaseAlternative = "release-alternative";
	var InterfaceAllowance = "interface-allowance";
	var InsertFixtureHole = "insert-fixture-hole";
	var FoundationJointDefinition = "foundation-joint";
	var MountingHoleRoute = "mounting-hole-route";
}

typedef DesignChangeRequest = {
	var id:String;
	var kind:DesignChangeKind;
	var sourceIds:Array<String>;
	var reason:String;
	var blockingChecks:Array<String>;
}

/** Process choices do not override the source assembly joint geometry. */
typedef FoundationJointProcess = {
	var jointId:String;
	var materialReference:Null<String>;
	var cureEvidence:Array<String>;
}

enum abstract OverlapRule(String) to String {
	var ReservedThenPriority = "inserts-eps-datums-then-priority-id";
}

enum abstract LiftingMethod(String) to String {
	var CastInPoints = "cast-in-points";
	var ExternalCradle = "external-cradle";
}

typedef LiftingQualification = {
	var method:LiftingMethod;
	var equipmentReference:String;
	var capacityKg:Float;
	var verifiedMassKg:Float;
	var centerOfMass:{x:Float, y:Float, z:Float};
	var positionToleranceMm:Float;
	var attachmentIds:Array<String>;
	var verificationReference:String;
}

typedef HandlingPlan = {
	var requiresFlip:Bool;
	var moldToService:AssemblyFrame;
	var castingMassKg:Float;
	var centerOfMassMoldMm:{x:Float, y:Float, z:Float};
}

typedef CastingMoldOptions = {
	var foundationJointProcesses:Array<FoundationJointProcess>;
	var replicationJigs:Array<ReplicationJig>;
	var precision:Array<PrecisionInterface>;
	var compaction:CompactionSpec;
	var serviceOrientation:AssemblyFrame;
	var overlapRule:OverlapRule;
	var liftingEvidence:Array<LiftingQualification>;
}

typedef BoreMaskProtection = {
	var sourceHoleId:String;
	var hardwareId:String;
	var plugLength:Null<Float>;
	var farEndId:Null<String>;
	var withdrawal:{x:Float, y:Float, z:Float};
}

typedef InsertFixtureFixing = {
	var sourceHoleId:String;
	var hardwareId:String;
	var panelId:String;
	var operationId:String;
	var entry:AssemblyFrame;
	var engagement:Float;
	var threadSpec:String;
}


enum MoldFixtureSupport {
	OnPanel(panelId:String);
	OnDatum(volume:MoldVolume,withdrawal:{x:Float,y:Float,z:Float},
		lateralLocation:{primaryId:String,secondaryId:String,baselineMm:Float});
}

typedef MoldInsertFixture = {
	var insertId:String;
	var location:DatumFixtureLocation.DatumLocationPlan;
	var support:MoldFixtureSupport;
	var mountingFace:AssemblyFrame;
	var attachmentEvidence:Array<String>;
	var fixings:Array<InsertFixtureFixing>;
	var boreMasks:Array<BoreMaskProtection>;
	var assemblyClearance:{x:Float, y:Float, z:Float};
}

typedef CastingMoldPlan = {
	var construction:MoldConstruction;
	var id:String;
	var document:ToolingDocument;
	var sourceFingerprint:String;
	var options:CastingMoldOptions;
	var handling:HandlingPlan;
	var envelope:MoldBox;
	var cavity:MoldBox;
	var panels:PanelAssemblyRecord;
	var supportPanels:Array<PanelPlacement>;
	var cores:Array<CastingMoldCore>;
	var coreUnits:Array<CastingCoreUnit>;
	var formers:Array<MoldFormer>;
	var fixtures:Array<MoldInsertFixture>;
	var features:Array<FeatureTooling>;
	var openings:Array<MoldOpening>;
	var hardware:Array<SupportHardware>;
	var changeRequests:Array<DesignChangeRequest>;
	var validation:Array<materia.tooling.ToolingValidation>;
	var fabrication:materia.tooling.ToolingValidation.FabricationReadiness;
}

/** Access explicit support variants; a direct panel fixture has no independent volume. */
class MoldFixtures {
	public static function volumes(fixture:MoldInsertFixture):Array<MoldVolume> return switch fixture.support {
		case OnPanel(_):[]; case OnDatum(volume,_,_):[volume];
	};
	public static function datum(fixture:MoldInsertFixture):MoldVolume return switch fixture.support {
		case OnDatum(volume,_,_):volume; case OnPanel(_):throw "Panel fixture has no separate datum volume: "+fixture.insertId;
	};
	public static function withdrawal(fixture:MoldInsertFixture):{x:Float,y:Float,z:Float} return switch fixture.support {
		case OnDatum(_,direction,_):direction;
		case OnPanel(_):materia.assembly.AssemblyFrames.transformVector(fixture.mountingFace,0,0,1);
	};
	public static function lateral(fixture:MoldInsertFixture):{primaryId:String,secondaryId:String,baselineMm:Float}
		return switch fixture.support {
			case OnDatum(_,_,location):location;
			case OnPanel(_):throw "Panel fixture has no separate datum locating pins: "+fixture.insertId;
		};
	public static function panelId(fixture:MoldInsertFixture):String return switch fixture.support {
		case OnPanel(panelId):panelId; case OnDatum(_,_,_):throw "Fixture is supported by a datum: "+fixture.insertId;
	};
	public static function allVolumes(plan:CastingMoldPlan):Array<MoldVolume> {
		var result:Array<MoldVolume>=[];
		for (fixture in plan.fixtures) for (volume in volumes(fixture)) result.push(volume);
		return result;
	}
}

/** One canonical opening table. Physical penetration records are read-only
 * compositions for sealing checks, not a second stored plan or encoding. */
class MoldOpenings {
	public static function fromRecord(record:PenetrationPlan):MoldOpening return {
		panelId:record.panelId,operationId:record.operationId,
		kind:record.kind==PenetrationClass.OutsideCasting ? MoldOpeningKind.ExteriorOpening : MoldOpeningKind.ToolingOpening,
		sealingIds:record.insertId!=null ? [record.insertId] : record.coreId!=null ? [record.coreId] : [],
		sealSpec:record.sealMaterial,
		protection:record.kind==PenetrationClass.OutsideCasting ? null : {
			kind:record.kind,insertId:record.insertId,sourceHoleId:record.sourceHoleId,coreId:record.coreId,
			sealMargin:record.sealMargin,sealingFace:record.sealingFace,plug:record.plug}
	};
	public static function attach(plan:CastingMoldPlan,record:PenetrationPlan):Void {
		var matches=plan.openings.filter(open -> open.panelId==record.panelId && open.operationId==record.operationId);
		if (matches.length!=1 || matches[0].protection!=null) throw "Opening protection must have one unassigned operation";
		matches[0].protection=fromRecord(record).protection;
	}
	public static function records(plan:CastingMoldPlan,?insertId:String):Array<PenetrationPlan> {
		var result:Array<PenetrationPlan>=[];
		for (open in plan.openings) {
			var protection=open.protection;
			if (protection==null) {
				if (insertId==null && open.kind==MoldOpeningKind.ExteriorOpening)
					result.push({panelId:open.panelId,operationId:open.operationId,kind:PenetrationClass.OutsideCasting,
						insertId:null,coreId:null,sealMargin:0,sealMaterial:open.sealSpec,sealingFace:null,plug:null});
			} else if (insertId==null || protection.insertId==insertId)
				result.push({panelId:open.panelId,operationId:open.operationId,kind:protection.kind,
					insertId:protection.insertId,sourceHoleId:protection.sourceHoleId,coreId:protection.coreId,
					sealMargin:protection.sealMargin,sealMaterial:open.sealSpec,sealingFace:protection.sealingFace,plug:protection.plug});
		}
		return result;
	}
	public static function sourceProtection(plan:CastingMoldPlan,operationId:String):OpeningProtection {
		var matches=plan.openings.filter(open -> open.operationId==operationId && open.protection!=null);
		if (matches.length!=1) throw "Expected one protected source opening: "+operationId;
		return cast matches[0].protection;
	}
}

/** Strict authored process inputs. Missing physical evidence stays explicit, never inferred. */
class MoldOptions {
	public static function copy(o:CastingMoldOptions):CastingMoldOptions
		return read(record(o));

	public static function record(authored:CastingMoldOptions):Dynamic {
		var options=authored,result:Dynamic={};
		for (key in Reflect.fields(options)) Reflect.setField(result,key,Reflect.field(options,key));
		Reflect.setField(result,"replicationJigs",ReplicationJig.ReplicationJigCodec.records(options.replicationJigs));
		return result;
	}

	public static function read(o:Dynamic):CastingMoldOptions {
		obj(o, [
			"foundationJointProcesses",
			"replicationJigs",
			"precision",
			"compaction",
			"serviceOrientation",
			"overlapRule",
			"liftingEvidence"
		]);
		var joints:Array<FoundationJointProcess> = [];
		for (j in arr(o,"foundationJointProcesses")) {
			obj(j,["jointId","materialReference","cureEvidence"]);
			joints.push({jointId:text(j,"jointId"),materialReference:nullableText(j,"materialReference"),cureEvidence:strings(j,"cureEvidence")});
		}
		var precision:Array<PrecisionInterface> = [];
		for (p in arr(o, "precision")) {
			obj(p, [
				"insertId",
				"masterReference",
				"jigReference",
				"masterFlatnessMm",
				"releaseSpec",
				"holeProtectionReference",
				"evidence"
			]);
			precision.push({
				insertId: text(p, "insertId"),
				masterReference: nullableText(p, "masterReference"),
				jigReference: nullableText(p, "jigReference"),
				masterFlatnessMm: nullableNumber(p, "masterFlatnessMm"),
				releaseSpec: nullableText(p, "releaseSpec"),
				holeProtectionReference: nullableText(p, "holeProtectionReference"),
				evidence: strings(p, "evidence")
			});
		}
		var c = Reflect.field(o, "compaction");
		obj(c, [
			"method",
			"bedInterfaceReference",
			"frequencyHz",
			"amplitudeMm",
			"durationSeconds",
			"evidence"
		]);
		var method:CompactionMethod = switch text(c, "method") {
			case "undeclared": Undeclared;
			case "vibration-table": VibrationTable;
			case "external-vibrators": ExternalVibrators;
			case "internal-poker": InternalPoker;
			case "none": NoCompaction;
			default: throw "Invalid compaction method";
		};
		var jointIds=new Map<String,Bool>();
		for (j in joints) {
			materia.casting.CastingRecordCodec.referencePath(j.jointId);
			if (jointIds.exists(j.jointId)) throw "Duplicate foundation joint process ID";
			jointIds.set(j.jointId,true);
		}
		joints.sort((a,b) -> Reflect.compare(a.jointId,b.jointId));
		if (text(o, "overlapRule") != "inserts-eps-datums-then-priority-id")
			throw "Unsupported overlap ownership rule";
		var pose = Reflect.field(o, "serviceOrientation");
		obj(pose, ["x", "y", "z", "qx", "qy", "qz", "qw"]);
		var orientation:materia.assembly.AssemblyRecord.AssemblyFrame = {
			x: number(pose, "x"),
			y: number(pose, "y"),
			z: number(pose, "z"),
			qx: number(pose, "qx"),
			qy: number(pose, "qy"),
			qz: number(pose, "qz"),
			qw: number(pose, "qw")
		};
		materia.assembly.AssemblyCodec.validateFrame(orientation);
		if (Math.abs(orientation.qx * orientation.qx + orientation.qy * orientation.qy + orientation.qz * orientation.qz +
			orientation.qw * orientation.qw - 1) > 1e-8)
			throw "Service orientation needs unit rotation";
		var lifting:Array<LiftingQualification> = [];
		for (item in arr(o, "liftingEvidence")) {
			obj(item, [
				"method",
				"equipmentReference",
				"capacityKg",
				"verifiedMassKg",
				"centerOfMass",
				"positionToleranceMm",
				"attachmentIds",
				"verificationReference"
			]);
			var method:LiftingMethod = switch text(item, "method") {
				case "cast-in-points": CastInPoints;
				case "external-cradle": ExternalCradle;
				default: throw "Invalid lifting method";
			};
			var com = Reflect.field(item, "centerOfMass");
			obj(com, ["x", "y", "z"]);
			var capacity = number(item, "capacityKg"),
				mass = number(item, "verifiedMassKg"),
				tolerance = number(item, "positionToleranceMm");
			if (capacity <= 0 || mass <= 0 || tolerance < 0)
				throw "Invalid lifting qualification capacity/mass/tolerance";
			lifting.push({
				method: method,
				equipmentReference: text(item, "equipmentReference"),
				capacityKg: capacity,
				verifiedMassKg: mass,
				centerOfMass: {x: number(com, "x"), y: number(com, "y"), z: number(com, "z")},
				positionToleranceMm: tolerance,
				attachmentIds: strings(item, "attachmentIds"),
				verificationReference: text(item, "verificationReference")
			});
		}
		precision.sort((a, b) -> Reflect.compare(a.insertId, b.insertId));
		return {
			foundationJointProcesses:joints,
			replicationJigs:ReplicationJig.ReplicationJigCodec.snapshot(Reflect.field(o,"replicationJigs")),
			precision: precision,
			compaction: {
				method: method,
				bedInterfaceReference: nullableText(c, "bedInterfaceReference"),
				frequencyHz: nullableNumber(c, "frequencyHz"),
				amplitudeMm: nullableNumber(c, "amplitudeMm"),
				durationSeconds: nullableNumber(c, "durationSeconds"),
				evidence: strings(c, "evidence")
			},
			serviceOrientation: orientation,
			overlapRule: ReservedThenPriority,
			liftingEvidence: lifting
		};
	}

	static function boxes(o:Dynamic, k:String):Array<MoldBox> {
		var out:Array<MoldBox> = [];
		for (b in arr(o, k)) {
			obj(b, ["x", "y", "z", "width", "depth", "height"]);
			var n:MoldBox = {
				x: number(b, "x"),
				y: number(b, "y"),
				z: number(b, "z"),
				width: number(b, "width"),
				depth: number(b, "depth"),
				height: number(b, "height")
			};
			if (n.width <= 0 || n.depth <= 0 || n.height <= 0)
				throw "Invalid process geometry";
			out.push(n);
		}
		return out;
	}

	static function obj(o:Dynamic, keys:Array<String>):Void {
		if (o == null || !Reflect.isObject(o) || Std.isOfType(o, Array) || Reflect.fields(o).length != keys.length)
			throw "Invalid mold process record";
		for (k in keys)
			if (!Reflect.hasField(o, k))
				throw "Missing mold process field " + k;
	}

	static function arr(o:Dynamic, k:String):Array<Dynamic> {
		var v:Dynamic = Reflect.field(o, k);
		if (!Std.isOfType(v, Array))
			throw "Expected mold process array";
		var a:Array<Dynamic> = cast v;
		if (a.length > 512)
			throw "Too many mold process records";
		return a;
	}

	static function strings(o:Dynamic, k:String):Array<String> {
		var out:Array<String> = [];
		for (v in arr(o, k)) {
			if (!Std.isOfType(v, String))
				throw "Expected evidence reference";
			var s:String = cast v;
			if (StringTools.trim(s).length == 0 || s.length > 2048 || materia.assembly.AssemblyCodec.containsNul(s))
				throw "Invalid evidence reference";
			out.push(s);
		}
		return out;
	}

	static function text(o:Dynamic, k:String):String {
		var v:Dynamic = Reflect.field(o, k);
		if (!Std.isOfType(v, String))
			throw "Expected mold process text";
		var s:String = cast v;
		if (StringTools.trim(s).length == 0 || s.length > 2048 || materia.assembly.AssemblyCodec.containsNul(s))
			throw "Invalid mold process text";
		return s;
	}

	static function nullableText(o:Dynamic, k:String):Null<String>
		return Reflect.field(o, k) == null ? null : text(o, k);

	static function number(o:Dynamic, k:String):Float {
		var v:Dynamic = Reflect.field(o, k);
		if ((!Std.isOfType(v, Float) && !Std.isOfType(v, Int)) || !Math.isFinite(v))
			throw "Expected finite process number";
		return v;
	}

	static function nullableNumber(o:Dynamic, k:String):Null<Float> {
		if (Reflect.field(o, k) == null)
			return null;
		var n = number(o, k);
		if (n < 0)
			throw "Negative process dimension/capacity";
		return n;
	}
}
