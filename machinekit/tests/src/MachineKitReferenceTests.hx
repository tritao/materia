import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.motion.NemaStepper;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.FlatWasher;
import machinekit.standard.HexBolt;
import machinekit.standard.HexNut;
import machinekit.standard.SocketHeadCapScrew;

/** Independent manufacturer reference values, separate from generator formulas. */
class MachineKitReferenceTests {
	static function equal(actual:Float, expected:Float, label:String):Void {
		if (Math.abs(actual - expected) > 1e-9) throw '$label: $actual != $expected';
	}

	static function near(actual:Float, expected:Float, tolerance:Float, label:String):Void {
		if (Math.abs(actual - expected) > tolerance) throw '$label: $actual not within $tolerance of $expected';
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
		// Accu ISO 4762 M5 product sheet: these fields have been cross-checked.
		var m5 = SocketHeadCapScrew.catalog().get("M5");
		equal(m5.diameter, 5, "M5 diameter");
		equal(m5.pitch, 0.8, "M5 pitch");
		equal(m5.headDiameter, 8.5, "M5 head diameter");
		equal(m5.headHeight, 5, "M5 head height");
		equal(m5.socketSize, 4, "M5 socket size");
		equal(m5.socketDepth, 2.5, "M5 socket depth");
		equal(m5.threadLength, 22, "M5 thread length");
		equal(m5.tapDrill, 4.2, "M5 tap drill");
		equal(m5.clearanceFine, 5.3, "M5 fine clearance");
		equal(m5.clearanceMedium, 5.5, "M5 medium clearance");
		equal(m5.counterboreDiameter, 10, "M5 counterbore diameter");
		var metadata = SocketHeadCapScrew.catalog().metadata("M5");
		if (metadata.dimensionKind != Unverified || metadata.verifiedFields == null || metadata.verifiedFields.length != 11)
			throw "M5 partial verification metadata";
		var bolt = HexBolt.catalog().get("M5");
		equal(bolt.acrossFlats, 8, "M5 hex bolt across flats");
		equal(bolt.headHeight, 3.5, "M5 hex bolt head height");
		equal(bolt.tapDrill, 4.2, "M5 hex bolt tap drill");
		equal(bolt.clearanceFine, 5.3, "M5 hex bolt fine clearance");
		equal(bolt.clearanceMedium, 5.5, "M5 hex bolt medium clearance");
		metadata = HexBolt.catalog().metadata("M5");
		if (metadata.verifiedFields == null || metadata.verifiedFields.length != 5)
			throw "M5 hex bolt partial verification metadata";
		var nut = HexNut.catalog().get("M5");
		equal(nut.acrossFlats, 8, "M5 nut across flats");
		equal(nut.height, 4.7, "M5 nut height");
		metadata = HexNut.catalog().metadata("M5");
		if (metadata.verifiedFields == null || metadata.verifiedFields.length != 2)
			throw "M5 nut partial verification metadata";
		var washer = FlatWasher.catalog().get("M5");
		equal(washer.innerDiameter, 5.3, "M5 washer inner diameter");
		equal(washer.outerDiameter, 10, "M5 washer outer diameter");
		equal(washer.thickness, 1, "M5 washer thickness");
		if (FlatWasher.catalog().metadata("M5").dimensionKind != Nominal)
			throw "M5 washer reference metadata";

		// Nanotec manufacturer drawings: frame interface values, with source tolerances.
		for (reference in [{name: "17", face: 42.3, spacing: 31.0, pilot: 22.0, tolerance: 0.1},
				{name: "23", face: 56.4, spacing: 47.14, pilot: 38.1, tolerance: 0.1},
				{name: "34", face: 85.85, spacing: 69.5, pilot: 73.025, tolerance: 0.2}]) {
			var frame = NemaStepper.catalog().get(reference.name);
			near(frame.face, reference.face, reference.tolerance, reference.name + " frame face");
			near(frame.boltSpacing, reference.spacing, reference.tolerance, reference.name + " bolt spacing");
			near(frame.pilotDiameter, reference.pilot, reference.tolerance, reference.name + " pilot");
			if (NemaStepper.catalog().metadata(reference.name).dimensionKind != Mixed)
				throw reference.name + " NEMA reference metadata";
		}
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
