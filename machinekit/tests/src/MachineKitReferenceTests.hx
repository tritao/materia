import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.motion.LinearBearing;
import machinekit.motion.LinearRailSystem;
import machinekit.motion.NemaStepper;
import machinekit.motion.PillowBlock;
import machinekit.robotics.RobotFlange;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.FlatWasher;
import machinekit.standard.HexBolt;
import machinekit.standard.HexNut;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.structural.TSlotExtrusion;

/** Independent manufacturer reference values, separate from generator formulas. */
class MachineKitReferenceTests {
	static function equal(actual:Float, expected:Float, label:String):Void {
		if (Math.abs(actual - expected) > 1e-9) throw '$label: $actual != $expected';
	}

	static function near(actual:Float, expected:Float, tolerance:Float, label:String):Void {
		if (Math.abs(actual - expected) > tolerance) throw '$label: $actual not within $tolerance of $expected';
	}

	public static function run():Void {
		// SKF bearing catalog: ISO 15 boundary dimensions and minimum rs, in mm.
		for (reference in [
				{name: "625", bore: 5.0, outside: 16.0, width: 5.0, rs: 0.3},
				{name: "626", bore: 6.0, outside: 19.0, width: 6.0, rs: 0.3},
				{name: "608", bore: 8.0, outside: 22.0, width: 7.0, rs: 0.3},
				{name: "6000", bore: 10.0, outside: 26.0, width: 8.0, rs: 0.3},
				{name: "6001", bore: 12.0, outside: 28.0, width: 8.0, rs: 0.3},
				{name: "6002", bore: 15.0, outside: 32.0, width: 9.0, rs: 0.3},
				{name: "6003", bore: 17.0, outside: 35.0, width: 10.0, rs: 0.3},
				{name: "6004", bore: 20.0, outside: 42.0, width: 12.0, rs: 0.6},
				{name: "6200", bore: 10.0, outside: 30.0, width: 9.0, rs: 0.6},
				{name: "6201", bore: 12.0, outside: 32.0, width: 10.0, rs: 0.6},
				{name: "6202", bore: 15.0, outside: 35.0, width: 11.0, rs: 0.6},
				{name: "6203", bore: 17.0, outside: 40.0, width: 12.0, rs: 0.6},
				{name: "6204", bore: 20.0, outside: 47.0, width: 14.0, rs: 1.0},
				{name: "6205", bore: 25.0, outside: 52.0, width: 15.0, rs: 1.0},
				{name: "6206", bore: 30.0, outside: 62.0, width: 16.0, rs: 1.0},
				{name: "6207", bore: 35.0, outside: 72.0, width: 17.0, rs: 1.1},
				{name: "6208", bore: 40.0, outside: 80.0, width: 18.0, rs: 1.1},
				{name: "6209", bore: 45.0, outside: 85.0, width: 19.0, rs: 1.1},
				{name: "6210", bore: 50.0, outside: 90.0, width: 20.0, rs: 1.1},
				{name: "6211", bore: 55.0, outside: 100.0, width: 21.0, rs: 1.5},
				{name: "6212", bore: 60.0, outside: 110.0, width: 22.0, rs: 1.5},
				{name: "6213", bore: 65.0, outside: 120.0, width: 23.0, rs: 1.5}]) {
			var row = DeepGrooveBearing.catalog().get(reference.name);
			equal(row.bore, reference.bore, reference.name + " bore");
			equal(row.outside, reference.outside, reference.name + " outside");
			equal(row.width, reference.width, reference.name + " width");
			equal(row.chamfer, reference.rs, reference.name + " minimum rs");
			var bearingMetadata = DeepGrooveBearing.catalog().metadata(reference.name);
			if (bearingMetadata.standard != "ISO 15" || bearingMetadata.standardEdition != "2017" ||
				bearingMetadata.dimensionKind != Nominal || bearingMetadata.verifiedFields == null)
				throw reference.name + " reference metadata";
		}
		// Tuli LM/LME catalog: nominal d, D and L dimensions, in mm.
		for (reference in [{name: "LM8UU", bore: 8.0, outside: 15.0, length: 24.0},
				{name: "LM10UU", bore: 10.0, outside: 19.0, length: 29.0},
				{name: "LM12UU", bore: 12.0, outside: 21.0, length: 30.0},
				{name: "LM16UU", bore: 16.0, outside: 28.0, length: 37.0},
				{name: "LM20UU", bore: 20.0, outside: 32.0, length: 42.0}]) {
			var linear = LinearBearing.catalog().get(reference.name);
			equal(linear.boreDiameter, reference.bore, reference.name + " bore");
			equal(linear.outerDiameter, reference.outside, reference.name + " outside");
			equal(linear.length, reference.length, reference.name + " length");
			var linearMetadata = LinearBearing.catalog().metadata(reference.name);
			if (linearMetadata.dimensionKind != Nominal || linearMetadata.verifiedFields == null)
				throw reference.name + " reference metadata";
		}
		// ISO 9409-1:2004 Table 1 pattern values; geometry remains a raised-pilot preview.
		for (reference in [{name: "31.5", count: 4, screw: "M5", pilot: 20.0, pin: 5.0},
				{name: "40", count: 4, screw: "M6", pilot: 25.0, pin: 6.0},
				{name: "50", count: 4, screw: "M6", pilot: 31.5, pin: 6.0},
				{name: "63", count: 4, screw: "M6", pilot: 40.0, pin: 6.0},
				{name: "80", count: 6, screw: "M8", pilot: 50.0, pin: 8.0},
				{name: "100", count: 6, screw: "M8", pilot: 63.0, pin: 8.0},
				{name: "125", count: 6, screw: "M10", pilot: 80.0, pin: 10.0},
				{name: "160", count: 6, screw: "M10", pilot: 100.0, pin: 10.0}]) {
			var flange = RobotFlange.catalog().get(reference.name);
			if (flange.boltCount != reference.count || flange.screw != reference.screw)
				throw reference.name + " ISO flange pattern";
			equal(flange.pilotDiameter, reference.pilot, reference.name + " pilot");
			equal(flange.pinDiameter, reference.pin, reference.name + " pin");
			var flangeMetadata = RobotFlange.catalog().metadata(reference.name);
			if (flangeMetadata.standard != "ISO 9409-1" || flangeMetadata.standardEdition != "2004" ||
				flangeMetadata.dimensionKind != Nominal || flangeMetadata.conformance != GenericApproximation)
				throw reference.name + " ISO flange metadata";
		}
		// MISUMI HFS5 catalog: cross-section and slot dimensions, in mm.
		for (reference in [{name: "HFS5-2020", width: 20.0, height: 20.0},
				{name: "HFS5-2040", width: 20.0, height: 40.0},
				{name: "HFS5-2060", width: 20.0, height: 60.0},
				{name: "HFS5-4040", width: 40.0, height: 40.0}]) {
			var profile = TSlotExtrusion.catalog().get(reference.name);
			equal(profile.width, reference.width, reference.name + " width");
			equal(profile.height, reference.height, reference.name + " height");
			equal(profile.slotWidth, 6, reference.name + " slot width");
			equal(profile.slotDepth, 6, reference.name + " slot depth");
			equal(profile.tWidth, 12, reference.name + " slot head");
			equal(profile.boreDiameter, 4.2, reference.name + " bore");
			var profileMetadata = TSlotExtrusion.catalog().metadata(reference.name);
			if (profileMetadata.dimensionKind != Nominal || profileMetadata.conformance != NominalEnvelope ||
				profileMetadata.verifiedFields == null)
				throw reference.name + " profile metadata";
		}
		// HIWIN linear guideway catalog: MGN12C rail and block dimensions, in mm.
		var rail = LinearRailSystem.catalog().get("MGN12C");
		equal(rail.railWidth, 12, "MGN12C rail width");
		equal(rail.railHeight, 8, "MGN12C rail height");
		equal(rail.blockWidth, 27, "MGN12C block width");
		equal(rail.blockHeight, 13, "MGN12C block height");
		equal(rail.blockLength, 34.7, "MGN12C block length");
		equal(rail.blockHoleSpacing, 21.7, "MGN12C block hole spacing");
		if (rail.blockMountScrew != "M3x8" || rail.railMountScrew != "M3x8")
			throw "MGN12C mounting screw size";
		equal(rail.railHolePitch, 25, "MGN12C rail hole pitch");
		equal(rail.railEndMargin, 10, "MGN12C rail end margin");
		var railMetadata = LinearRailSystem.catalog().metadata("MGN12C");
		if (railMetadata.dimensionKind != Nominal || railMetadata.conformance != NominalEnvelope ||
			railMetadata.verifiedFields == null)
			throw "MGN12C reference metadata";
		// Koyo/JTEKT UCP204 product page: base-mounted unit dimensions, in mm.
		var pillow = PillowBlock.catalog().get("UCP204");
		equal(pillow.boreDiameter, 20, "UCP204 bore");
		equal(pillow.baseWidth, 38, "UCP204 base width");
		equal(pillow.length, 127, "UCP204 length");
		equal(pillow.shaftHeight, 33.3, "UCP204 shaft height");
		equal(pillow.baseHeight, 16, "UCP204 base height");
		equal(pillow.overallHeight, 64.5, "UCP204 overall height");
		equal(pillow.boltSpacing, 95, "UCP204 bolt spacing");
	if (pillow.mountScrew != "M10" || pillow.bearingDesignation != "6204")
		throw "UCP204 bearing unit interface";
	var pillowMetadata = PillowBlock.catalog().metadata("UCP204");
		if (pillowMetadata.dimensionKind != Nominal || pillowMetadata.conformance != NominalEnvelope ||
			pillowMetadata.verifiedFields == null)
			throw "UCP204 reference metadata";
		for (reference in [
			{name: "UCP205", bore: 25.0, width: 38.0, length: 140.0, height: 36.5, base: 16.0, overall: 70.0, spacing: 105.0, hole: 13.0, screw: "M10"},
			{name: "UCP206", bore: 30.0, width: 48.0, length: 165.0, height: 42.9, base: 17.0, overall: 84.0, spacing: 121.0, hole: 17.0, screw: "M14"},
			{name: "UCP207", bore: 35.0, width: 48.0, length: 167.0, height: 47.6, base: 18.0, overall: 95.0, spacing: 127.0, hole: 17.0, screw: "M14"},
			{name: "UCP208", bore: 40.0, width: 54.0, length: 184.0, height: 49.2, base: 18.0, overall: 98.0, spacing: 137.0, hole: 17.0, screw: "M14"},
			{name: "UCP209", bore: 45.0, width: 54.0, length: 190.0, height: 54.0, base: 20.0, overall: 106.0, spacing: 146.0, hole: 17.0, screw: "M14"},
			{name: "UCP210", bore: 50.0, width: 60.0, length: 206.0, height: 57.2, base: 21.0, overall: 113.0, spacing: 159.0, hole: 20.0, screw: "M16"},
			{name: "UCP211", bore: 55.0, width: 60.0, length: 219.0, height: 63.5, base: 23.0, overall: 125.0, spacing: 171.0, hole: 20.0, screw: "M16"},
			{name: "UCP212", bore: 60.0, width: 70.0, length: 241.0, height: 69.8, base: 25.0, overall: 138.0, spacing: 184.0, hole: 20.0, screw: "M16"},
			{name: "UCP213", bore: 65.0, width: 70.0, length: 265.0, height: 76.2, base: 27.0, overall: 150.0, spacing: 203.0, hole: 25.0, screw: "M20"}]) {
			var row = PillowBlock.catalog().get(reference.name);
			equal(row.boreDiameter, reference.bore, reference.name + " bore");
			equal(row.baseWidth, reference.width, reference.name + " base width");
			equal(row.length, reference.length, reference.name + " length");
			equal(row.shaftHeight, reference.height, reference.name + " shaft height");
			equal(row.baseHeight, reference.base, reference.name + " base height");
			equal(row.overallHeight, reference.overall, reference.name + " overall height");
			equal(row.boltSpacing, reference.spacing, reference.name + " bolt spacing");
			equal(row.mountHoleDiameter, reference.hole, reference.name + " mounting hole");
			if (row.mountScrew != reference.screw)
				throw reference.name + " mounting screw";
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
