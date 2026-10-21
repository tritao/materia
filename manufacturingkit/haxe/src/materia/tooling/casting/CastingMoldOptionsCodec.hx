package materia.tooling.casting;

import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.MoldPlan.MoldBox;

/** Strict authored process inputs. Missing physical evidence stays explicit, never inferred. */
class CastingMoldOptionsCodec {
	public static function snapshot(o:CastingMoldOptions):CastingMoldOptions
		return read(o);

	public static function record(authored:CastingMoldOptions):Dynamic {
		var options=snapshot(authored),result:Dynamic={};
		for (key in Reflect.fields(options)) Reflect.setField(result,key,Reflect.field(options,key));
		Reflect.setField(result,"replicationJigs",ReplicationJig.ReplicationJigCodec.records(options.replicationJigs));
		return result;
	}

	public static function read(o:Dynamic):CastingMoldOptions {
		obj(o, [
			"groutedJoints",
			"replicationJigs",
			"precision",
			"compaction",
			"anchorage",
			"serviceOrientation",
			"overlapRule",
			"liftingEvidence"
		]);
		var joints:Array<GroutedJoint> = [];
		for (j in arr(o, "groutedJoints")) {
			obj(j, [
				"id",
				"insertIds",
				"gapMm",
				"jackingReference",
				"boltSleeveReference",
				"cureBeforeTorqueEvidence"
			]);
			joints.push({
				id: text(j, "id"),
				insertIds: strings(j, "insertIds"),
				gapMm: nullableNumber(j, "gapMm"),
				jackingReference: nullableText(j, "jackingReference"),
				boltSleeveReference: nullableText(j, "boltSleeveReference"),
				cureBeforeTorqueEvidence: strings(j, "cureBeforeTorqueEvidence")
			});
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
		var anchorage:Array<InsertAnchorage> = [];
		for (a in arr(o, "anchorage")) {
			obj(a, ["insertId", "kind", "geometry", "pulloutN", "shearN", "evidence"]);
			var kind:AnchorageKind = switch text(a, "kind") {
				case "pending": Pending;
				case "studs": Studs;
				case "undercut": Undercut;
				case "bonded-keys": BondedKeys;
				default: throw "Invalid anchorage kind";
			};
			var geometry = boxes(a, "geometry");
			if (geometry.length > 0)
				throw "Anchorage geometry needs a corresponding source adapter revision before this template can consume it";
			anchorage.push({
				insertId: text(a, "insertId"),
				kind: kind,
				geometry: geometry,
				pulloutN: nullableNumber(a, "pulloutN"),
				shearN: nullableNumber(a, "shearN"),
				evidence: strings(a, "evidence")
			});
		}
		var jointIds = new Map<String, Bool>();
		for (j in joints) {
			materia.tooling.ToolingIds.validate(j.id);
			if (jointIds.exists(j.id))
				throw "Duplicate grouted joint ID";
			jointIds.set(j.id, true);
		}
		joints.sort((a, b) -> Reflect.compare(a.id, b.id));
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
		anchorage.sort((a, b) -> Reflect.compare(a.insertId, b.insertId));
		return {
			groutedJoints: joints,
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
			anchorage: anchorage,
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
