package materia.casting;

/** Stock condition is part of the design, not inferred by the tooling adapter. */
enum abstract InsertMaterial(String) to String {
	var Steel = "steel";
	var C45Bright = "c45-en-1.0503-bright";
	var C45Ground = "c45-en-1.0503-ground";
	var Stainless14571 = "stainless-en-1.4571";
}

typedef InsertMaterialSpec = {
	var grade:InsertMaterial;
	var minimumTensileStrengthMPa:Float;
}

class InsertMaterials {
	public static function values():Array<InsertMaterial> return [C45Bright, C45Ground, Stainless14571];
	public static function tokens():Array<String> return [for (value in values()) (value:String)];

	/** Procurement minimum for C45 +C flat bar, also required of ground stock.
	 * Trafital, C45 mechanical properties, EN ISO 683-7, Rm 580–820 MPa.
	 * https://www.trafital.it/en/trafilati.html
	 * Not the >800 MPa quenched-and-tempered C45 condition.
	 */
	public static inline var C45_MINIMUM_TENSILE_MPA = 580;
	/** Procurement floor, not a supplier certificate or socket capacity.
	 * Outokumpu Supra datasheet, EN 10088-2 table 5: 1.4571 minimum Rm
	 * 540 MPa in coil/sheet, 520 MPa in plate; use the lower published floor.
	 * https://www.outokumpu.com/-/media/files/products/supra/outokumpu-supra-range-datasheet.pdf
	 * The finished product still needs material/geometry evidence.
	 */
	public static inline var STAINLESS_14571_MINIMUM_TENSILE_MPA = 520;

	public static function specification(grade:InsertMaterial):InsertMaterialSpec {
		var strength = switch grade {
			case C45Bright, C45Ground: C45_MINIMUM_TENSILE_MPA;
			case Stainless14571: STAINLESS_14571_MINIMUM_TENSILE_MPA;
			default: throw "Unspecified steel has no qualified tensile strength";
		};
		return {grade: grade, minimumTensileStrengthMPa: strength};
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
