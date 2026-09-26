import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.motion.NemaStepper;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.SocketHeadCapScrew;

/** Independent manufacturer reference values, separate from generator formulas. */
class MachineKitReferenceTests {
	static function equal(actual:Float, expected:Float, label:String):Void {
		if (Math.abs(actual - expected) > 1e-9) throw '$label: $actual != $expected';
	}

	public static function run():Void {
		// NTN 608 and 6000 product tables: d, D, B and minimum rs, in mm.
		for (reference in [{name: "608", bore: 8.0, outside: 22.0, width: 7.0, rs: 0.3},
				{name: "6000", bore: 10.0, outside: 26.0, width: 8.0, rs: 0.3}]) {
			var row = DeepGrooveBearing.catalog().get(reference.name);
			equal(row.bore, reference.bore, reference.name + " bore");
			equal(row.outside, reference.outside, reference.name + " outside");
			equal(row.width, reference.width, reference.name + " width");
			equal(row.chamfer, reference.rs, reference.name + " minimum rs");
			if (DeepGrooveBearing.catalog().metadata(reference.name).dimensionKind != Mixed)
				throw reference.name + " reference metadata";
		}
		// Accu ISO 4762 M5 product sheet: only these fields have been cross-checked.
		var m5 = SocketHeadCapScrew.catalog().get("M5");
		equal(m5.diameter, 5, "M5 diameter");
		equal(m5.pitch, 0.8, "M5 pitch");
		equal(m5.headDiameter, 8.5, "M5 head diameter");
		equal(m5.socketDepth, 2.5, "M5 socket depth");
		var metadata = SocketHeadCapScrew.catalog().metadata("M5");
		if (metadata.dimensionKind != Unverified || metadata.verifiedFields == null || metadata.verifiedFields.length != 4)
			throw "M5 partial verification metadata";
		// StepperOnline model pages: body face/length and shaft diameter/length.
		for (reference in [{name: "17HS19-1684S1", face: 42.0, length: 48.0, shaft: 5.0, shaftLength: 24.0},
				{name: "23HS22-2804S", face: 57.3, length: 56.0, shaft: 6.35, shaftLength: 21.0},
				{name: "34HS31-5504S", face: 86.0, length: 80.0, shaft: 14.0, shaftLength: 35.0}]) {
			var variant = NemaStepper.variantCatalog().get(reference.name);
			equal(variant.bodyFace, reference.face, reference.name + " face");
			equal(variant.bodyLength, reference.length, reference.name + " length");
			equal(variant.shaftDiameter, reference.shaft, reference.name + " shaft diameter");
			equal(variant.shaftLength, reference.shaftLength, reference.name + " shaft length");
		}
	}
}
