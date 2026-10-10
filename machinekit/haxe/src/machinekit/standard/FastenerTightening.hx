package machinekit.standard;

import cadkit.threads.ThreadSpec;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.standard.PropertyClass.PropertyClasses;

/** Published maximum assembly values, not a joint-specific process program. */
typedef TighteningTableRow = {
	var size:String;
	var propertyClass:PropertyClass;
	var friction:Float;
	var torqueNm:Float;
	var preloadN:Float;
}

/** Guidance tables with their stated friction; absent sizes and fine pitches are never interpolated. */
class FastenerTightening {
	public static inline var STEEL_SOURCE = "https://optimas.com/technical-resources/pre-load-and-tightening-torques-of-metric-thread-steel-fasteners/";
	public static inline var STAINLESS_SOURCE = "https://international.optimas.com/technical-resources/stainless-steel-fasteners-pre-load-and-tightening-torques/";
	static var table:Null<Catalog<TighteningTableRow>>;

	public static function catalog():Catalog<TighteningTableRow> {
		if (table == null) {
			var rows:Array<TighteningTableRow> = [];
			// Optimas coarse table, mu=0.125. M3..M5 torque columns are Ncm; M6 and above are Nm.
			steel(rows, "M3", 2320, 3270, 3920, 1.28, 1.80, 2.15);
			steel(rows, "M4", 4020, 5660, 6790, 2.90, 4.10, 4.95);
			steel(rows, "M5", 6580, 9260, 11100, 5.75, 8.10, 9.70);
			steel(rows, "M6", 9290, 13050, 15650, 9.90, 14.0, 16.5);
			steel(rows, "M8", 17050, 24000, 28800, 24, 34, 40);
			steel(rows, "M10", 27100, 38200, 45800, 48, 67, 81);
			steel(rows, "M12", 39600, 55600, 66800, 83, 117, 140);
			steel(rows, "M14", 54300, 76400, 91700, 132, 185, 220);
			steel(rows, "M16", 74700, 105000, 126000, 200, 285, 340);
			steel(rows, "M20", 117000, 164000, 197000, 390, 550, 660);
			// Optimas stainless table, A2-70, mu=0.1; preload columns are kN.
			stainless(rows, "M3", 1000, 1);
			stainless(rows, "M4", 2970, 1.7);
			stainless(rows, "M5", 4850, 3.4);
			stainless(rows, "M6", 6850, 5.9);
			stainless(rows, "M8", 12600, 14.5);
			stainless(rows, "M10", 20000, 30);
			stainless(rows, "M12", 29100, 50);
			stainless(rows, "M14", 40000, 79);
			stainless(rows, "M16", 55000, 121);
			stainless(rows, "M20", 88600, 224);
			table = new Catalog("fastener tightening", key, rows, row -> ({
				source: row.propertyClass == A270 ? STAINLESS_SOURCE : STEEL_SOURCE,
				standard: row.propertyClass == A270 ? "ISO 3506" : "ISO 898-1", standardEdition: null,
				dimensionKind: Maximum, conformance: GenericApproximation,
				verifiedFields: ["torqueNm", "preloadN", "friction"]
			}));
		}
		return table;
	}

	static function key(row:TighteningTableRow):String return row.size + "/" + PropertyClasses.token(row.propertyClass);

	public static function forThread(thread:ThreadSpec, propertyClass:PropertyClass):TighteningTableRow {
		// The size row is admissible only for the exact catalog coarse pitch.
		var sizes = SocketHeadCapScrew.catalog();
		if (thread.profile != Metric || thread.starts != 1 || thread.hand != RightHand ||
			!sizes.exists(thread.designation)) throw 'No published tightening row for ${thread.designation}';
		var size = sizes.get(thread.designation);
		if (size.diameter != thread.majorDiameter || size.pitch != thread.pitch)
			throw 'No published tightening row for ${thread.designation} at pitch ${thread.pitch}';
		return catalog().get(thread.designation + "/" + PropertyClasses.token(propertyClass));
	}

	static function steel(rows:Array<TighteningTableRow>, size:String, p88:Float, p109:Float, p129:Float,
			t88:Float, t109:Float, t129:Float):Void {
		rows.push({size: size, propertyClass: Steel88, friction: 0.125, torqueNm: t88, preloadN: p88});
		rows.push({size: size, propertyClass: Steel109, friction: 0.125, torqueNm: t109, preloadN: p109});
		rows.push({size: size, propertyClass: Steel129, friction: 0.125, torqueNm: t129, preloadN: p129});
	}

	static function stainless(rows:Array<TighteningTableRow>, size:String, preload:Float, torque:Float):Void
		rows.push({size: size, propertyClass: A270, friction: 0.1, torqueNm: torque, preloadN: preload});
}
