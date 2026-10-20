package materia.casting;

/** Stock condition is part of the design, not inferred by the tooling adapter. */
enum abstract InsertMaterial(String) to String {
	var Steel = "steel";
	var C45Bright = "c45-en-1.0503-bright";
	var C45Ground = "c45-en-1.0503-ground";
}

typedef InsertMaterialSpec = {
	var grade:InsertMaterial;
	var minimumTensileStrengthMPa:Float;
}

class InsertMaterials {
	public static function values():Array<InsertMaterial> return [C45Bright, C45Ground];
	public static function tokens():Array<String> return [for (value in values()) (value:String)];

	/** Procurement minimum for C45 +C flat bar, also required of ground stock.
	 * Trafital, C45 mechanical properties, EN ISO 683-7, Rm 580–820 MPa.
	 * https://www.trafital.it/en/trafilati.html
	 * Not the >800 MPa quenched-and-tempered C45 condition.
	 */
	public static inline var C45_MINIMUM_TENSILE_MPA = 580;

	public static function specification(grade:InsertMaterial):InsertMaterialSpec {
		if (grade != C45Bright && grade != C45Ground)
			throw "Unspecified steel has no qualified tensile strength";
		return {grade: grade, minimumTensileStrengthMPa: C45_MINIMUM_TENSILE_MPA};
	}

	public static function snapshot(raw:Dynamic):InsertMaterialSpec {
		CastingFeatureCodec.fields(raw, ["grade", "minimumTensileStrengthMPa"]);
		var grade:InsertMaterial = cast CastingRecordCodec.string(raw, "grade");
		var expected = specification(grade);
		var strength = CastingRecordCodec.number(raw, "minimumTensileStrengthMPa");
		if (strength != expected.minimumTensileStrengthMPa)
			throw "Insert tensile strength disagrees with its stock specification";
		return {grade: grade, minimumTensileStrengthMPa: strength};
	}
}
