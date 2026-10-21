package materia.casting;

enum abstract InsertGeometryBasis(String) to String {
	var NominalStudy = "nominal-study";
	var SupplierDrawing = "supplier-drawing";
	var Measured = "measured";
}

/** Product identity is separate from its display label and geometry evidence. */
enum InsertOrigin {
	Fabricated(manufacturing:InsertManufacturing);
	Catalogue(product:CatalogueInsert);
}

typedef CatalogueInsert = {
	var manufacturer:String;
	var reference:String;
	var revision:String;
	var holePolicyId:String;
	var document:String;
	var geometryBasis:InsertGeometryBasis;
	var geometryFingerprint:Null<String>;
	var evidence:Array<{reference:String, sha256:String}>;
}

class InsertOriginCodec {
	public static function geometryFingerprint(product:CatalogueInsert, body:InsertSpec.InsertBody, holes:Array<InsertHole>):String
		return haxe.crypto.Sha256.encode(haxe.Json.stringify({
			product:{manufacturer:product.manufacturer,reference:product.reference,revision:product.revision,
				holePolicyId:product.holePolicyId,document:product.document},
			body:InsertSpec.InsertBodyCodec.record(body),holes:InsertHole.InsertHoleCodec.snapshot(holes)
		}));

	public static function snapshot(raw:Dynamic, body:InsertSpec.InsertBody, holes:Array<InsertHole>):InsertOrigin {
		if (raw == null) throw "Missing insert origin";
		if (Std.isOfType(raw, InsertOrigin)) raw = record(cast raw);
		var kind = CastingRecordCodec.string(raw, "kind");
		if (kind == "fabricated") {
			CastingFeatureCodec.fields(raw, ["kind","manufacturing"]);
			return Fabricated(InsertManufacturing.InsertManufacturingCodec.snapshot(Reflect.field(raw,"manufacturing")));
		}
		if (kind != "catalogue") throw "Unknown insert origin";
		CastingFeatureCodec.fields(raw, ["kind", "product"]);
		raw = Reflect.field(raw, "product");
		CastingFeatureCodec.fields(raw, ["manufacturer", "reference", "revision", "holePolicyId", "document", "geometryBasis", "geometryFingerprint", "evidence"]);
		var basis:InsertGeometryBasis = switch CastingRecordCodec.string(raw, "geometryBasis") {
			case "nominal-study": NominalStudy;
			case "supplier-drawing": SupplierDrawing;
			case "measured": Measured;
			default: throw "Invalid insert geometry basis";
		};
		var result:CatalogueInsert = {
			manufacturer: text(raw, "manufacturer"), reference: text(raw, "reference"), revision: text(raw, "revision"), holePolicyId: text(raw, "holePolicyId"), document: text(raw, "document"),
			geometryBasis: basis, geometryFingerprint: CastingRecordCodec.optionalString(raw, "geometryFingerprint"), evidence: []
		};
		for (row in CastingRecordCodec.array(raw, "evidence")) {
			CastingFeatureCodec.fields(row, ["reference", "sha256"]);
			var hash = text(row, "sha256");
			if (!~/^[a-f0-9]{64}$/.match(hash)) throw "Invalid insert evidence SHA-256";
			result.evidence.push({reference: text(row, "reference"), sha256: hash});
		}
		result.evidence.sort((a, b) -> Reflect.compare(a.reference, b.reference));
		if (basis == NominalStudy) {
			if (result.geometryFingerprint != null || result.evidence.length != 0)
				throw "Nominal insert geometry cannot claim verified geometry evidence";
		} else if (result.evidence.length == 0 || result.geometryFingerprint != geometryFingerprint(result, body, holes))
			throw "Insert geometry evidence is absent or bound to different geometry";
		for (hole in holes)
			if (hole.limits.policyId != result.holePolicyId) throw "Catalogue insert hole policy differs from product declaration";
		return Catalogue(result);
	}

	public static function record(origin:InsertOrigin):Dynamic {
		if (origin == null) throw "Missing insert origin";
		return switch origin {
			case Fabricated(manufacturing): {kind: "fabricated",manufacturing:InsertManufacturing.InsertManufacturingCodec.record(manufacturing)};
			case Catalogue(product): {kind: "catalogue", product: {
				manufacturer: product.manufacturer, reference: product.reference, revision: product.revision,
				holePolicyId: product.holePolicyId, document: product.document, geometryBasis: (product.geometryBasis:String),
				geometryFingerprint: product.geometryFingerprint,
				evidence: [for (row in product.evidence) {reference: row.reference, sha256: row.sha256}]
			}};
		};
	}

	static function text(raw:Dynamic, name:String):String {
		var value = CastingRecordCodec.string(raw, name);
		if (StringTools.trim(value).length == 0 || materia.assembly.AssemblyCodec.containsNul(value))
			throw "Invalid insert origin text: " + name;
		return value;
	}
}
