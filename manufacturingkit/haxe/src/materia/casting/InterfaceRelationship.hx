package materia.casting;

/** Component paths name the part owning the interface; machine axes name joints.
 * Both can be qualified when an assembly is nested, without interpreting labels.
 */
@:wire enum InterfaceMember {
	@:id(1) ComponentInterface(memberId:String, interfaceId:String);
	@:id(2) MachineAxis(axisId:String);
}

/** Geometric relations use the member's actual kind: plane-line parallelism
 * means a perpendicular plane normal and line direction; plane-line squareness
 * means parallel normal and direction. Normals are not substituted for planes.
 */
@:wire enum InterfaceRelationshipKind {
	@:id(1) Coplanar;
	@:id(2) Parallel;
	@:id(3) Perpendicular;
	@:id(4) Distance(separationMm:Float);
	@:id(5) Straightness;
	@:id(6) InPlaneParallelism;
	@:id(7) InPlaneSquareness;
}

/** Angular budgets retain their declared measurement length rather than silently
 * interpreting micrometres per 300 mm as an absolute position tolerance.
 */
@:wire enum RelationshipTolerance {
	@:id(1) Linear(maxDeviationMm:Float);
	/** Lateral rise over a gage span measured along the ideal reference direction;
	 * its angular slope is maxDeviationMm/spanMm (tan of the allowed angle).
	 */
	@:id(2) PerLength(maxDeviationMm:Float, spanMm:Float);
}

enum abstract MetrologyMethod(String) to String {
	IndicatorReversal = "indicator-reversal";
	ElectronicLevel = "electronic-level";
	AxisMasterIndicator = "axis-master-indicator";
	GraniteSquareIndicator = "granite-square-indicator";
	AutocollimatorReversal = "autocollimator-reversal";
	LaserStraightness = "laser-straightness";
}

@:wire typedef InterfaceMetrology = {
	@:id(1) var method:MetrologyMethod;
	@:id(2) var instrumentReference:Null<String>;
	/** Physical measurement uncertainty, in mm at the tolerance's declared span.
	 * Null means missing evidence, never an unbounded design tolerance.
	 */
	@:id(3) var uncertaintyMm:Null<Float>;
}

@:wire typedef InterfaceRelationship = {
	@:id(1) var id:String;
	@:id(2) var kind:InterfaceRelationshipKind;
	@:id(3) var members:Array<InterfaceMember>;
	@:id(4) var tolerance:RelationshipTolerance;
	@:id(5) var metrology:InterfaceMetrology;
}

/** The assembly supplies ownership; interface names are local to their source. */
typedef ScopedInterfaceRequirement = {
	var member:InterfaceMember;
	var requirement:InterfaceRequirement;
}

class InterfaceRelationships {
	/** Resolve both directions of the source/assembly contract. A relationship must
	 * name existing interfaces and each participating interface must reference it.
	 * This checks declarations only, never measured alignment or uncertainty.
	 */
	public static function validateReferences(relationships:Array<InterfaceRelationship>,
			interfaces:Array<ScopedInterfaceRequirement>, axisIds:Array<String>):Void {
		var sources:Map<String, InterfaceRequirement> = [];
		for (source in interfaces) {
			var requirement = InterfaceRequirement.InterfaceRequirements.snapshot([source.requirement])[0];
			var interfaceId = switch source.member {
				case ComponentInterface(_, id): id;
				case MachineAxis(_): throw "Machine axis cannot own an interface requirement";
			};
			if (interfaceId != requirement.id) throw "Scoped interface ID differs from requirement: " + requirement.id;
			var key = memberKey(source.member);
			if (sources.exists(key)) throw "Duplicate scoped interface: " + key;
			sources.set(key, requirement);
		}
		var axes:Map<String, Bool> = [];
		for (axis in axisIds) {
			CastingRecordCodec.referencePath(axis);
			if (axes.exists(axis)) throw "Duplicate machine axis: " + axis;
			axes.set(axis, true);
		}
		var resolved:Map<String, InterfaceRelationship> = [];
		for (relationship in snapshot(relationships)) {
			resolved.set(relationship.id, relationship);
			for (member in relationship.members) switch member {
				case MachineAxis(axis):
					if (!axes.exists(axis)) throw "Unknown relationship machine axis: " + relationship.id + ": " + axis;
				default:
					var key = memberKey(member);
					var source = sources.get(key);
					if (source == null) throw "Unknown relationship interface: " + relationship.id + ": " + key;
					if (source.relationshipIds.indexOf(relationship.id) < 0)
						throw "Interface omits participating relationship: " + relationship.id + ": " + key;
			}
		}
		for (key in sources.keys()) {
			var source = sources.get(key);
			for (id in source.relationshipIds) {
				var relationship = resolved.get(id);
				if (relationship == null) throw "Unknown interface relationship: " + key + ": " + id;
				var found = false;
				for (member in relationship.members) if (memberKey(member) == key) found = true;
				if (!found) throw "Interface references unrelated relationship: " + key + ": " + id;
			}
		}
	}

	public static function snapshot(raw:Dynamic):Array<InterfaceRelationship> {
		if (!Std.isOfType(raw, Array)) throw "Invalid interface relationship list";
		var rows:Array<Dynamic> = cast raw;
		if (rows.length > 512) throw "Too many interface relationships";
		var result:Array<InterfaceRelationship> = [];
		var seen:Map<String, Bool> = [];
		for (row in rows) {
			CastingFeatureCodec.fields(row, ["id", "kind", "members", "tolerance", "metrology"]);
			var id = CastingRecordCodec.string(row, "id");
			CastingFeatureCodec.identifier(id);
			if (seen.exists(id)) throw "Duplicate interface relationship: " + id;
			seen.set(id, true);
			var kind = readKind(Reflect.field(row, "kind"));
			var members:Array<InterfaceMember> = [];
			var keys:Array<String> = [];
			var rawMembers = CastingRecordCodec.array(row, "members");
			if (rawMembers.length > 128) throw "Too many relationship members: " + id;
			for (value in rawMembers) {
				var member = readMember(value);
				var key = memberKey(member);
				if (keys.indexOf(key) >= 0) throw "Duplicate relationship member: " + id;
				keys.push(key);
				members.push(member);
			}
			var validCount = switch kind {
				case Straightness: members.length == 1;
				case Distance(_), Perpendicular, InPlaneSquareness: members.length == 2;
				case Coplanar, Parallel, InPlaneParallelism: members.length >= 2;
			};
			if (!validCount) throw "Invalid relationship member count: " + id;
			members.sort((a, b) -> Reflect.compare(memberKey(a), memberKey(b)));
			var tolerance = readTolerance(Reflect.field(row, "tolerance"));
			var angular = switch kind {
				case Parallel, Perpendicular, InPlaneParallelism, InPlaneSquareness: true;
				default: false;
			};
			var perLength = switch tolerance {
				case PerLength(_, _): true;
				case Linear(_): false;
			};
			if (angular != perLength) throw "Relationship tolerance needs the correct linear/angular basis: " + id;
			var m = Reflect.field(row, "metrology");
			CastingFeatureCodec.fields(m, ["method", "instrumentReference", "uncertaintyMm"]);
			var instrument = CastingRecordCodec.optionalString(m, "instrumentReference");
			if (instrument != null && (StringTools.trim(instrument).length == 0 || materia.assembly.AssemblyCodec.containsNul(instrument)))
				throw "Invalid metrology instrument reference: " + id;
			var uncertainty:Null<Float> = Reflect.field(m, "uncertaintyMm") == null ? null : CastingRecordCodec.number(m, "uncertaintyMm");
			if (uncertainty != null && uncertainty <= 0) throw "Measurement uncertainty must be positive: " + id;
			result.push({id: id, kind: kind, members: members, tolerance: tolerance,
				metrology: {method: readMethod(CastingRecordCodec.string(m, "method")), instrumentReference: instrument,
					uncertaintyMm: uncertainty}});
		}
		result.sort((a, b) -> Reflect.compare(a.id, b.id));
		return result;
	}

	public static function records(relationships:Array<InterfaceRelationship>):Array<Dynamic> {
		var result:Array<Dynamic> = [];
		for (r in snapshot(relationships)) result.push({id: r.id, kind: kindRecord(r.kind),
			members: [for (m in r.members) memberRecord(m)], tolerance: toleranceRecord(r.tolerance),
			metrology: {method: (r.metrology.method:String), instrumentReference: r.metrology.instrumentReference,
				uncertaintyMm: r.metrology.uncertaintyMm}});
		return result;
	}

	/** Definition ID remains stable in source requirements; only physical member paths move. */
	public static function remapMembers(relationship:InterfaceRelationship,map:String->String):InterfaceRelationship {
		var result=snapshot([relationship])[0];
		result.members=[for (member in result.members) switch member {
			case ComponentInterface(part,face): ComponentInterface(map(part),face);
			case MachineAxis(axis): MachineAxis(map(axis));
		}];
		return snapshot([result])[0];
	}

	public static function maxDeviation(t:RelationshipTolerance):Float return switch t {
		case Linear(mm), PerLength(mm, _): mm;
	};

	/** Null is missing physical evidence; a negative value cannot qualify the relationship. */
	public static function uncertaintyMargin(r:InterfaceRelationship):Null<Float> {
		var uncertainty = r.metrology.uncertaintyMm;
		return uncertainty == null ? null : maxDeviation(r.tolerance) - uncertainty;
	}

	public static function memberKey(member:InterfaceMember):String
		return haxe.Json.stringify(memberRecord(member));

	public static function readMember(raw:Dynamic):InterfaceMember {
		if (Std.isOfType(raw, InterfaceMember)) raw = memberRecord(cast raw);
		var kind = CastingRecordCodec.string(raw, "kind");
		var result = switch kind {
			case "component-interface":
				CastingFeatureCodec.fields(raw, ["kind", "memberId", "interfaceId"]);
				ComponentInterface(CastingRecordCodec.string(raw,"memberId"),CastingRecordCodec.string(raw,"interfaceId"));
			case "machine-axis":
				CastingFeatureCodec.fields(raw, ["kind", "axisId"]);
				MachineAxis(CastingRecordCodec.string(raw, "axisId"));
			default: throw "Unknown interface relationship member";
		};
		memberRecord(result);
		return result;
	}

	public static function memberRecord(m:InterfaceMember):Dynamic return switch m {
		case ComponentInterface(member,face):
			CastingRecordCodec.referencePath(member);
			CastingFeatureCodec.identifier(face);
			{kind: "component-interface", memberId: member, interfaceId: face};
		case MachineAxis(axis):
			CastingRecordCodec.referencePath(axis);
			{kind: "machine-axis", axisId: axis};
	};

	static function readKind(raw:Dynamic):InterfaceRelationshipKind {
		if (Std.isOfType(raw, InterfaceRelationshipKind)) raw = kindRecord(cast raw);
		var kind = CastingRecordCodec.string(raw, "kind");
		if (kind == "distance") {
			CastingFeatureCodec.fields(raw, ["kind", "separationMm"]);
			var distance = CastingRecordCodec.number(raw, "separationMm");
			if (distance < 0) throw "Relationship separation cannot be negative";
			return Distance(distance);
		}
		CastingFeatureCodec.fields(raw, ["kind"]);
		return switch kind {
			case "coplanar": Coplanar;
			case "parallel": Parallel;
			case "perpendicular": Perpendicular;
			case "straightness": Straightness;
			case "in-plane-parallelism": InPlaneParallelism;
			case "in-plane-squareness": InPlaneSquareness;
			default: throw "Unknown interface relationship kind";
		};
	}

	public static function kindRecord(kind:InterfaceRelationshipKind):Dynamic return switch kind {
		case Coplanar: {kind: "coplanar"};
		case Parallel: {kind: "parallel"};
		case Perpendicular: {kind: "perpendicular"};
		case Distance(mm): {kind: "distance", separationMm: mm};
		case Straightness: {kind: "straightness"};
		case InPlaneParallelism: {kind: "in-plane-parallelism"};
		case InPlaneSquareness: {kind: "in-plane-squareness"};
	};

	static function readTolerance(raw:Dynamic):RelationshipTolerance {
		if (Std.isOfType(raw, RelationshipTolerance)) raw = toleranceRecord(cast raw);
		var kind = CastingRecordCodec.string(raw, "kind");
		var mm = CastingRecordCodec.number(raw, "maxDeviationMm");
		if (mm <= 0) throw "Relationship tolerance must be positive";
		return switch kind {
			case "linear":
				CastingFeatureCodec.fields(raw, ["kind", "maxDeviationMm"]);
				Linear(mm);
			case "per-length":
				CastingFeatureCodec.fields(raw, ["kind", "maxDeviationMm", "spanMm"]);
				var span = CastingRecordCodec.number(raw, "spanMm");
				if (span <= 0) throw "Relationship evaluation span must be positive";
				PerLength(mm, span);
			default: throw "Unknown relationship tolerance basis";
		};
	}

	public static function toleranceRecord(t:RelationshipTolerance):Dynamic return switch t {
		case Linear(mm): {kind: "linear", maxDeviationMm: mm};
		case PerLength(mm, span): {kind: "per-length", maxDeviationMm: mm, spanMm: span};
	};

	public static function readMethod(token:String):MetrologyMethod return switch token {
		case "indicator-reversal": IndicatorReversal;
		case "electronic-level": ElectronicLevel;
		case "axis-master-indicator": AxisMasterIndicator;
		case "granite-square-indicator": GraniteSquareIndicator;
		case "autocollimator-reversal": AutocollimatorReversal;
		case "laser-straightness": LaserStraightness;
		default: throw "Unknown interface metrology method";
	};
}
