package materia.casting;

/** Exterior and source hole records share one contract for rectangular and round inserts. */
typedef InsertSpec = {
	var body:InsertBody;
	var holes:Array<InsertHole>;
	var origin:InsertOrigin;
	var interfaces:Array<InterfaceRequirement>;
}

/** Source and tooling use the same body and holes; document codecs own versions. */
class InsertSpecCodec {
	public static function snapshot(raw:Dynamic):InsertSpec {
		CastingFeatureCodec.fields(raw, ["body", "holes", "origin", "interfaces"]);
		var body = InsertBodyCodec.snapshot(Reflect.field(raw, "body"));
		var holes = InsertHole.InsertHoleCodec.snapshot(Reflect.field(raw, "holes"));
		var origin=InsertOrigin.InsertOriginCodec.snapshot(Reflect.field(raw,"origin"),body,holes);
		var interfaces=InterfaceRequirement.InterfaceRequirements.snapshot(Reflect.field(raw,"interfaces"));
		switch origin {
			case Fabricated(manufacturing):InsertManufacturing.InsertManufacturingCodec.references(manufacturing,holes,interfaces);
			case Catalogue(_):
		}
		return {
			body: body,
			interfaces: interfaces,
			holes: holes,
			origin: origin
		};
	}

	public static function record(spec:InsertSpec):Dynamic {
		if (spec == null)
			throw "Missing insert specification";
		return {
			body: InsertBodyCodec.record(spec.body),
			interfaces: InterfaceRequirement.InterfaceRequirements.records(spec.interfaces),
			holes: InsertHole.InsertHoleCodec.snapshot(spec.holes),
			origin: InsertOrigin.InsertOriginCodec.record(InsertOrigin.InsertOriginCodec.snapshot(spec.origin, spec.body, spec.holes))
		};
	}
}

/** Product retention geometry belongs to an insert, never to a casting cut primitive. */
typedef InsertCrimpedTail = {
	var length:Float;
	var width:Float;
	var thickness:Float;
	var wave:Float;
	var blend:Float;
}

enum InsertBody {
	Primitive(solid:CastingSolid);
	CrimpedSocket(mouth:CastingSolid, tail:InsertCrimpedTail);
}

class InsertBodyCodec {
	public static function encode(body:InsertBody):String return haxe.Json.stringify(record(body));
	public static function decode(text:String):InsertBody return snapshot(haxe.Json.parse(text));
	public static function snapshot(raw:Dynamic):InsertBody {
		if (Std.isOfType(raw, InsertBody)) raw = record(cast raw);
		var result = switch CastingRecordCodec.string(raw, "kind") {
			case "primitive":
				CastingFeatureCodec.fields(raw, ["kind", "solid"]);
				Primitive(CastingSolid.CastingSolidCodec.snapshot(Reflect.field(raw, "solid")));
			case "crimped-socket":
				CastingFeatureCodec.fields(raw, ["kind", "mouth", "tail"]);
				var tail = Reflect.field(raw, "tail");
				CastingFeatureCodec.fields(tail, ["length", "width", "thickness", "wave", "blend"]);
				CrimpedSocket(CastingSolid.CastingSolidCodec.snapshot(Reflect.field(raw, "mouth")), {
					length: CastingRecordCodec.number(tail, "length"), width: CastingRecordCodec.number(tail, "width"),
					thickness: CastingRecordCodec.number(tail, "thickness"), wave: CastingRecordCodec.number(tail, "wave"),
					blend: CastingRecordCodec.number(tail, "blend")});
			default: throw "Unknown insert body kind";
		};
		validate(result);
		return result;
	}
	public static function record(body:InsertBody):Dynamic {
		validate(body);
		return switch body {
			case Primitive(solid): {kind: "primitive", solid: CastingSolid.CastingSolidCodec.record(solid)};
			case CrimpedSocket(mouth, tail): {kind: "crimped-socket", mouth: CastingSolid.CastingSolidCodec.record(mouth),
				tail: {length: tail.length, width: tail.width, thickness: tail.thickness, wave: tail.wave, blend: tail.blend}};
		};
	}
	public static function validate(body:InsertBody):Void {
		if (body == null) throw "Missing insert body";
		switch body {
			case Primitive(solid): CastingSolid.CastingSolidCodec.validate(solid);
			case CrimpedSocket(mouth, tail):
				CastingSolid.CastingSolidCodec.validate(mouth);
				var dimensions = switch mouth {
					case Cylinder(diameter, height): {diameter: diameter, height: height};
					default: throw "Crimped socket mouth must be cylindrical";
				};
				if (tail == null) throw "Missing insert crimped tail";
				for (value in [tail.length, tail.width, tail.thickness, tail.wave, tail.blend])
					if (!Math.isFinite(value) || value <= 0) throw "Insert tail dimensions must be finite and positive";
				if (tail.width < dimensions.diameter || tail.thickness >= dimensions.diameter
					|| tail.blend >= Math.min(dimensions.height, tail.length)) throw "Inconsistent insert crimped tail";
		}
	}
	public static function box(body:InsertBody, id:String):{width:Float, depth:Float, height:Float} {
		validate(body);
		return switch body {
			case Primitive(solid): CastingSolid.CastingSolidCodec.box(solid, id);
			case CrimpedSocket(_, _): throw "Unsupported non-box solid: " + id;
		};
	}
	public static function height(body:InsertBody):Float {
		validate(body);
		return switch body {
			case Primitive(solid): CastingSolid.CastingSolidCodec.height(solid);
			case CrimpedSocket(mouth, tail): CastingSolid.CastingSolidCodec.height(mouth) + tail.length;
		};
	}
	/** Mouth span only; this is not a bounding-box approximation of the insert. */
	public static function mouth(body:InsertBody):CastingSolid return switch body {
		case Primitive(solid): solid;
		case CrimpedSocket(mouth, _): mouth;
	};
}

enum abstract InsertRole(String) to String {
	var TableMount = "table-mount";
	var LiftingMount = "lifting-mount";
	var ThreadedAnchor = "threaded-anchor";
	var GenericMount = "generic-mount";
	var GuideSeat = "guide-seat";
	var BearingSeat = "bearing-seat";
	var ColumnMount = "column-mount";
	var LevelingSeat = "leveling-seat";
	var HatchMount = "hatch-mount";
	var DriveMount = "drive-mount";
	var EnclosureMount = "enclosure-mount";
}

enum abstract InsertFinish(String) to String {
	var AsSupplied = "as-supplied";
	var GroundAfterCasting = "ground-after-casting";
	var MachinedAfterCasting = "machined-after-casting";
	var ReplicatedAfterCasting = "replicated-after-casting";
	var SourceAsCast = "as-cast";
}

class InsertFinishes {
	public static function values():Array<InsertFinish>
		return [InsertFinish.AsSupplied, InsertFinish.GroundAfterCasting, InsertFinish.MachinedAfterCasting, InsertFinish.ReplicatedAfterCasting, InsertFinish.SourceAsCast];

	public static function tokens():Array<String>
		return [for (finish in values()) (finish : String)];
}

/** Typed role vocabulary shared by source schemas and neutral validation. */
class InsertRoles {
	public static function values():Array<InsertRole>
		return [
			InsertRole.TableMount,
			InsertRole.LiftingMount,
			InsertRole.ThreadedAnchor,
			InsertRole.GenericMount,
			InsertRole.GuideSeat,
			InsertRole.BearingSeat,
			InsertRole.ColumnMount,
			InsertRole.LevelingSeat,
			InsertRole.HatchMount,
			InsertRole.DriveMount,
			InsertRole.EnclosureMount
		];

	public static function tokens():Array<String>
		return [for (role in values()) (role : String)];
}
