import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.assembly.LinearAxis;
import machinekit.assembly.PillowBlock;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.component.Bom;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.motion.LeadScrewNut;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import machinekit.motion.LeadScrewThread.LeadScrewHand;
import machinekit.motion.LinearBearing;
import machinekit.motion.NemaStepper;
import machinekit.motion.PillowBlockHousing;
import machinekit.motion.ShaftCoupling;
import machinekit.motion.SteppedShaft;
import machinekit.standard.Bushing;
import machinekit.standard.BearingFit.BearingHousingFit;
import machinekit.standard.BearingFit.BearingShaftFit;
import machinekit.standard.ClearanceFit;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.FlatWasher;
import machinekit.standard.HexBolt;
import machinekit.standard.HexNut;
import machinekit.standard.ParallelKey;
import machinekit.robotics.EndEffectorPlate;
import machinekit.robotics.Pedestal;
import machinekit.robotics.RobotFlange;
import machinekit.standard.RetainingRing;
import machinekit.standard.ShaftCollar;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.structural.Angle;
import machinekit.structural.Channel;
import machinekit.structural.FlatBar;
import machinekit.structural.FrameAssembly;
import machinekit.structural.RectTube;
import machinekit.structural.RoundTube;
import machinekit.structural.TSlotExtrusion;
import machinekit.transmission.GearPair;
import machinekit.transmission.Rack;
import machinekit.transmission.Sprocket;
import machinekit.transmission.SpurGear;
import machinekit.transmission.TimingPulley;
import machinekit.transmission.TimingBeltProfile;
import materia.project.AssemblyFrames;
import materia.project.AssemblyRecord.AssemblyFrame;

class MachineKitSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value) throw message;
	}

	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void
		check(Math.abs(actual - expected) <= tolerance * Math.max(1, Math.abs(expected)),
			'$message: $actual != $expected');

	static function throws(action:() -> Void, fragment:String):Void {
		try {
			action();
		} catch (error:Dynamic) {
			var message = Std.string(error);
			check(message.indexOf(fragment) >= 0, 'unexpected error "$message"');
			return;
		}
		throw 'expected an error containing "$fragment"';
	}

	static function solid(part:Part, message:String):Void {
		check(part.valid(), '$message is invalid');
		check(part.solidCount() == 1, '$message has ${part.solidCount()} solids');
	}

	static function bounds(part:Part):{minX:Float, minZ:Float, maxX:Float, maxZ:Float} {
		var box = part.shape.bounds();
		return {minX: box.get_min().get_x(), minZ: box.get_min().get_z(),
			maxX: box.get_max().get_x(), maxZ: box.get_max().get_z()};
	}

	static function annulus(outside:Float, inside:Float, length:Float):Float
		return Math.PI * (outside * outside - inside * inside) / 4 * length;

	static function bearings():Void {
		var bearing = DeepGrooveBearing.metric("6204");
		check(bearing.designation == "6204-2Z", "shielded designation");
		check(bearing.bore == 20 && bearing.outside == 47 && bearing.width == 14, "6204 dimensions");
		for (designation in DeepGrooveBearing.catalog().designations())
			DeepGrooveBearing.metric(designation, false);
		throws(() -> DeepGrooveBearing.metric("6299"), 'Unknown deep groove bearing "6299"');

		var envelope = bearing.geometry(Envelope);
		solid(envelope, "bearing envelope");
		near(envelope.volume(), annulus(47, 20, 14), "bearing envelope volume");
		envelope.close();
		var preview = bearing.geometry();
		solid(preview, "bearing preview");
		var box = bounds(preview);
		near(box.maxX, 23.5, "bearing outside radius");
		near(box.minZ, 0, "bearing front face");
		near(box.maxZ, 14, "bearing back face");
		check(preview.volume() < annulus(47, 20, 14), "preview removes shield recesses");
		preview.close();

		var seat = bearing.housingSeat(10, BearingHousingFit.Interference);
		near(seat.volume(), Math.PI * Math.pow((47 - 0.03) / 2, 2) * 10, "housing seat fit volume");
		near(bearing.journalDiameter(BearingShaftFit.Interference), 20.02, "journal interference fit");
		near(bearing.journalDiameter(BearingShaftFit.Slip), 19.98, "journal slip fit");
		near(bearing.journalDiameterAllowance(0.01), 20.01, "explicit journal allowance");
		var slipSeat = bearing.housingSeat(10, BearingHousingFit.Slip);
		check(slipSeat.volume() > seat.volume(), "housing slip fit is larger than press fit");
		var explicitSeat = bearing.housingSeatAllowance(10, -0.02);
		near(explicitSeat.volume(), Math.PI * Math.pow(46.98 / 2, 2) * 10, "explicit housing allowance");
		seat.close();
		slipSeat.close();
		explicitSeat.close();
		near(bearing.connector("axis").frame.z, 7, "bearing axis connector");
		// Connector +Y is the bearing axis (+Z).
		var axis = AssemblyFrames.transformVector(bearing.connector("back").frame, 0, 1, 0);
		near(axis.z, 1, "bearing connector axis");
	}

	static function screws():Void {
		var screw = SocketHeadCapScrew.metric("M6", 25);
		check(screw.designation == "ISO4762-M6x25", "screw part number");
		near(screw.pitch, 1, "M6 pitch");
		near(screw.threadLength, 24, "M6 thread length");
		near(SocketHeadCapScrew.metric("M6", 12.5).threadLength, 12.5, "short screws are fully threaded");
		check(SocketHeadCapScrew.metric("M6", 12.5).designation == "ISO4762-M6x12.5", "fractional length");
		throws(() -> SocketHeadCapScrew.metric("M7", 20), 'Unknown metric screw size "M7"');
		throws(() -> SocketHeadCapScrew.metric("M6", 0), "positive length");

		var envelope = screw.geometry(Envelope);
		solid(envelope, "screw envelope");
		near(envelope.volume(), Math.PI * (25 * 6 + 9 * 25), "screw envelope volume");
		var preview = screw.geometry(Preview);
		solid(preview, "screw preview");
		var socketArea = 3 * Math.sqrt(3) / 2 * Math.pow(5 / Math.sqrt(3), 2);
		near(envelope.volume() - preview.volume(), socketArea * 3, "hex socket volume");
		var box = bounds(preview);
		near(box.minZ, -25, "screw tip");
		near(box.maxZ, 6, "screw head top");
		envelope.close();
		preview.close();

		near(screw.clearanceDiameter(Fine), 6.4, "fine clearance");
		near(screw.clearanceDiameter(), 6.6, "medium clearance");
		near(screw.clearanceDiameter(Coarse), 7, "coarse clearance");
		var clearance = screw.clearanceHole(10);
		near(clearance.volume(), Math.PI * 3.3 * 3.3 * 10, "clearance hole volume");
		near(bounds(clearance).maxZ, 0, "clearance hole seat");
		clearance.close();
		var tap = screw.tapHole(12);
		near(tap.volume(), Math.PI * 2.5 * 2.5 * 12, "tap hole volume");
		tap.close();
		var counterbore = screw.counterboreHole(15);
		solid(counterbore, "counterbore tool");
		near(counterbore.volume(), Math.PI * (3.3 * 3.3 * (15 - 6.4) + 5.5 * 5.5 * 6.4), "counterbore volume");
		counterbore.close();
		throws(() -> screw.counterboreHole(5), "needs depth");
	}

	static function motors():Void {
		for (frame in [17, 23, 34]) {
			var motor = NemaStepper.frame(frame);
			var preview = motor.geometry();
			solid(preview, 'NEMA $frame preview');
			var box = bounds(preview);
			near(box.maxX, motor.variant.bodyFace / 2, 'NEMA $frame face');
			near(box.minZ, -motor.bodyLength, 'NEMA $frame body');
			near(box.maxZ, motor.variant.shaftLength, 'NEMA $frame shaft');
			preview.close();
		}
		var motor = NemaStepper.frame(17, 40);
		check(NemaStepper.model("23HS22-2804S").variant.shaftDiameter == 6.35, "named motor variant shaft");
		check(NemaStepper.frame(23).spec.boltSpacing == 47.14, "NEMA frame bolt spacing");
		check(motor.designation == "GENERIC-NEMA17-L40", "motor designation");
		throws(() -> NemaStepper.frame(11), 'Unknown NEMA frame "11"');
		var envelope = motor.geometry(Envelope);
		var shaftBeyondPilot = Math.PI * 2.5 * 2.5 * (24 - 2);
		near(envelope.volume(), 42 * 42 * 40 + Math.PI * 11 * 11 * 2 + shaftBeyondPilot, "motor envelope volume");
		envelope.close();
		near(motor.connector("bolt1").frame.x, 15.5, "bolt connector x");
		near(motor.connector("bolt3").frame.y, -15.5, "bolt connector y");
		near(motor.connector("shaftTip").frame.z, 24, "shaft tip connector");
		check(motor.mountScrew(8).spec.size == "M3", "NEMA 17 uses M3");
		var cutout = motor.mountingCutout(5);
		near(cutout.volume(), Math.PI * (11.1 * 11.1 + 4 * 1.7 * 1.7) * 5, "motor cutout volume");
		cutout.close();
	}

	static function fasteners():Void {
		var bolt = HexBolt.metric("M6", 25);
		check(bolt.designation == "ISO4017-M6x25", "bolt designation");
		near(bolt.pitch, 1, "M6 bolt pitch");
		near(bolt.threadLength, 25, "ISO 4017 bolts are fully threaded");
		throws(() -> HexBolt.metric("M7", 20), 'Unknown hex bolt size "M7"');
		throws(() -> HexBolt.metric("M6", 0), "positive length");

		var corner = bolt.acrossCorners;
		near(corner, 10 / Math.cos(Math.PI / 6), "bolt across corners");
		var envelope = bolt.geometry(Envelope);
		solid(envelope, "bolt envelope");
		near(envelope.volume(), Math.PI * (corner / 2) * (corner / 2) * 4 + Math.PI * 9 * 25, "bolt envelope volume");
		envelope.close();
		var preview = bolt.geometry();
		solid(preview, "bolt preview");
		near(preview.volume(), Math.sqrt(3) / 2 * 100 * 4 + Math.PI * 9 * 25, "bolt preview volume");
		var box = bounds(preview);
		near(box.maxX, 5, "bolt head across flats");
		near(box.minZ, -25, "bolt tip");
		near(box.maxZ, 4, "bolt head top");
		preview.close();
		near(bolt.connector("head").frame.z, 0, "bolt head connector");
		near(bolt.connector("tip").frame.z, -25, "bolt tip connector");

		near(bolt.clearanceDiameter(Fine), 6.4, "bolt fine clearance");
		var clearance = bolt.clearanceHole(10);
		near(clearance.volume(), Math.PI * 3.3 * 3.3 * 10, "bolt clearance hole volume");
		clearance.close();
		var tap = bolt.tapHole(12);
		near(tap.volume(), Math.PI * 2.5 * 2.5 * 12, "bolt tap hole volume");
		tap.close();
		var seatRadius = (corner + 0.5) / 2;
		var counterbore = bolt.counterboreHole(6);
		solid(counterbore, "bolt counterbore tool");
		near(counterbore.volume(), Math.PI * (seatRadius * seatRadius * 4.5 + 3.3 * 3.3 * 1.5), "bolt counterbore volume");
		counterbore.close();
		throws(() -> bolt.counterboreHole(4), "needs depth over");

		var nut = HexNut.metric("M6");
		check(nut.designation == "ISO4032-M6", "nut designation");
		throws(() -> HexNut.metric("M7"), 'Unknown hex nut size "M7"');
		var nutEnvelope = nut.geometry(Envelope);
		solid(nutEnvelope, "nut envelope");
		near(nutEnvelope.volume(), Math.sqrt(3) / 2 * 100 * 5.2, "nut envelope volume");
		nutEnvelope.close();
		var nutPreview = nut.geometry();
		solid(nutPreview, "nut preview");
		near(nutPreview.volume(), Math.sqrt(3) / 2 * 100 * 5.2 - Math.PI * 9 * 5.2, "nut preview volume");
		nutPreview.close();
		near(nut.connector("axis").frame.z, 2.6, "nut axis connector");
		var pocket = nut.pocket(6);
		solid(pocket, "nut pocket");
		near(pocket.volume(), Math.sqrt(3) / 2 * 10.5 * 10.5 * 6, "nut pocket volume");
		pocket.close();
		throws(() -> nut.pocket(5), "needs depth at least");

		var washer = FlatWasher.metric("M6");
		check(washer.designation == "ISO7089-M6", "washer designation");
		throws(() -> FlatWasher.metric("M7"), 'Unknown flat washer size "M7"');
		var washerPart = washer.geometry();
		solid(washerPart, "washer");
		near(washerPart.volume(), Math.PI * (6 * 6 - 3.2 * 3.2) * 1.6, "washer volume");
		washerPart.close();
		near(washer.connector("axis").frame.z, 0.8, "washer axis connector");
	}

	static function dimensions():Void {
		check(Dimension.format(12.7) == "12.7", "12.7 formats without binary noise");
		check(Dimension.format(0.1 + 0.2) == "0.3", "0.1 + 0.2 formats as 0.3");
		check(Dimension.format(20) == "20", "whole numbers format without a decimal point");
		check(Dimension.format(6.35) == "6.35", "6.35 formats exactly");
		check(Dimension.format(0.0625) == "0.063", "values round to 0.001");
		check(Dimension.format(-2.5) == "-2.5", "negative values keep their sign");
		check(Dimension.format(-0.0001) == "0", "values rounding to zero lose their sign");
	}

	static function shafts():Void {
		var key = ParallelKey.forShaft(6, 6);
		check(key.designation == "DIN6885-B-2x2x6", "key designation");
		check(key.spec.width == 2 && key.spec.height == 2, "key cross-section");
		var keyPreview = key.geometry();
		solid(keyPreview, "key preview");
		near(keyPreview.volume(), 2 * 2 * 6, "key volume");
		keyPreview.close();
		throws(() -> ParallelKey.metric("9x9", 10), 'Unknown parallel key "9x9"');
		throws(() -> ParallelKey.forShaft(100, 10), "No DIN 6885-1 key fits shaft diameter 100");
		throws(() -> ParallelKey.forShaft(0, 10), "positive shaft diameter");

		var shaft = new SteppedShaft(
			[{diameter: 8, length: 51.5}, {diameter: 6, length: 8.5}],
			[{name: "bearingA", z: 10}, {name: "bearingB", z: 43}],
			[{name: "outputKey", z0: 52, key: key}],
			[{name: "ring", z0: 50, width: 1.2, diameter: 7.6}]
		);
		near(shaft.totalLength, 60, "shaft total length");
		near(shaft.diameterAt(0), 8, "shaft start diameter");
		near(shaft.diameterAt(51.5), 6, "shaft boundary belongs to the next section");
		near(shaft.diameterAt(60), 6, "shaft end diameter");
		throws(() -> shaft.diameterAt(-1), "outside 0..60");
		throws(() -> shaft.diameterAt(61), "outside 0..60");

		var envelope = shaft.geometry(Envelope);
		solid(envelope, "shaft envelope");
		near(envelope.volume(), Math.PI * 16 * 51.5 + Math.PI * 9 * 8.5, "shaft envelope volume");
		envelope.close();
		var preview = shaft.geometry();
		solid(preview, "shaft preview");
		// Groove: an outer annulus 7.6..8 wide 1.2. Keyway: the 2 mm slot's circular segment above y=1.8.
		var grooveVolume = Math.PI * (16 - 3.8 * 3.8) * 1.2;
		var keywayVolume = 6 * (Math.sqrt(8) + 9 * Math.atan2(1, Math.sqrt(8)) - 3.6);
		near(preview.volume(), Math.PI * 16 * 51.5 + Math.PI * 9 * 8.5 - grooveVolume - keywayVolume,
			"preview removes the outer groove annulus and keyway", 1e-3);
		var previewBox = bounds(preview);
		near(previewBox.maxX, 4, "groove leaves the shaft's outer surface elsewhere");
		preview.close();
		check(shaft.designation == "SHAFT-8x51.5-6x8.5", "shaft designation");
		check(new SteppedShaft([{diameter: 6.35, length: 20}]).designation == "SHAFT-6.35x20",
			"fractional shaft designation is rounded, not a raw float");

		near(shaft.connector("input").frame.z, 0, "shaft input connector");
		near(shaft.connector("output").frame.z, 60, "shaft output connector");
		near(shaft.connector("bearingA").frame.z, 10, "shaft bearingA connector");
		var seat = shaft.connector("outputKey").frame;
		near(seat.z, 55, "keyway connector z");
		near(seat.y, 3 - 1.2, "keyway connector floor");
		near(shaft.connector("ring").frame.z, 50.6, "groove connector z");

		throws(() -> new SteppedShaft([{diameter: -1, length: 10}]), "positive diameter and length");
		throws(() -> new SteppedShaft([{diameter: 8, length: 10}], [{name: "x", z: 20}]),
			'Face "x" lies outside the shaft');
		throws(() -> new SteppedShaft([{diameter: 8, length: 10}], null,
			[{name: "k", z0: 5, key: ParallelKey.forShaft(6, 8)}]), 'Keyway "k" lies outside the shaft');
		throws(() -> new SteppedShaft(
			[{diameter: 8, length: 10}, {diameter: 6, length: 10}], null,
			[{name: "k", z0: 8, key: ParallelKey.forShaft(6, 4)}]), 'Keyway "k" must lie within one shaft section');
		throws(() -> new SteppedShaft([{diameter: 2, length: 10}], null,
			[{name: "k", z0: 0, key: ParallelKey.forShaft(6, 4)}]), 'Keyway "k" is deeper than the shaft radius');
		throws(() -> new SteppedShaft([{diameter: 8, length: 10}], null, null,
			[{z0: 0, width: 20, diameter: 6}]), "Retaining ring groove lies outside the shaft");
		throws(() -> new SteppedShaft(
			[{diameter: 8, length: 10}, {diameter: 6, length: 10}], null, null,
			[{z0: 8, width: 4, diameter: 5}]), "Retaining ring groove must lie within one shaft section");
		throws(() -> new SteppedShaft([{diameter: 8, length: 10}], null, null,
			[{z0: 0, width: 2, diameter: 9}]), "Retaining ring groove diameter must be smaller than the shaft");
		throws(() -> new SteppedShaft(
			[{diameter: 10, length: 10}, {diameter: 12, length: 2}, {diameter: 10, length: 10}], null,
			[{name: "k", z0: 8, key: ParallelKey.forShaft(10, 6)}]), 'Keyway "k" must lie within one shaft section');
		throws(() -> new SteppedShaft(
			[{diameter: 10, length: 10}, {diameter: 12, length: 2}, {diameter: 10, length: 10}], null, null,
			[{z0: 9, width: 4, diameter: 9}]), "Retaining ring groove must lie within one shaft section");
		throws(() -> new SteppedShaft([{diameter: 12, length: 40}], null,
			[{name: "k", z0: 0, key: ParallelKey.metric("10x8", 20)}]), 'Keyway "k" is too wide for the shaft');
	}

	static function shaftHardware():Void {
		var ring = RetainingRing.forShaft(8);
		check(ring.designation == "DIN471-8", "ring designation");
		check(ring.spec.grooveDiameter == 7.6 && ring.spec.outerDiameter == 12.2, "ring dimensions (DIN 471 d2)");
		check(ring.thickness == 0.8 && ring.grooveSpec().width == 0.9, "ring thickness differs from groove width");
		check(ring.grooveSpec().diameter == 7.6, "ring groove diameter");
		throws(() -> RetainingRing.forShaft(9), 'Unknown retaining ring shaft diameter "9"');
		throws(() -> RetainingRing.forShaft(8.5), 'Unknown retaining ring shaft diameter "8.5"');
		check(RetainingRing.forShaft(12).spec.grooveDiameter == 11.5, "12 mm ring groove diameter");

		var envelope = ring.geometry(Envelope);
		solid(envelope, "ring envelope");
		near(envelope.volume(), annulus(12.2, 7.6, 0.8), "ring envelope volume");
		envelope.close();
		var preview = ring.geometry();
		solid(preview, "ring preview");
		near(preview.volume(), annulus(12.2, 7.6, 0.8) * 8 / 9, "ring preview volume (gapped)");
		preview.close();
		near(ring.connector("seat").frame.z, 0.4, "ring seat connector");

		var collar = ShaftCollar.forShaft(8);
		check(collar.designation == "COLLAR-8", "collar designation");
		check(collar.spec.setScrew == "M4", "collar set screw size");
		throws(() -> ShaftCollar.forShaft(9), 'Unknown shaft collar bore diameter "9"');
		throws(() -> ShaftCollar.forShaft(6.35), 'Unknown shaft collar bore diameter "6.35"');
		var collarPart = collar.geometry();
		solid(collarPart, "collar");
		near(collarPart.volume(), Math.PI * (8 * 8 - 4 * 4) * 11, "collar volume");
		collarPart.close();
		near(collar.connector("axis").frame.z, 5.5, "collar axis connector");
	}

	static function structural():Void {
		var tube = new RectTube(40, 40, 3);
		check(tube.designation == "RECT-40x40x3", "rect tube designation");
		throws(() -> new RectTube(10, 10, 6), "wall is too thick");
		var tubePart = tube.geometry(100);
		solid(tubePart, "rect tube");
		near(tubePart.volume(), (40 * 40 - 34 * 34) * 100, "rect tube volume");
		tubePart.close();

		var round = new RoundTube(20, 2);
		check(round.designation == "ROUND-20x2", "round tube designation");
		throws(() -> new RoundTube(10, 6), "wall is too thick");
		var roundPart = round.geometry(50);
		solid(roundPart, "round tube");
		near(roundPart.volume(), Math.PI * (10 * 10 - 8 * 8) * 50, "round tube volume");
		roundPart.close();

		var angle = new Angle(30, 30, 3);
		check(angle.designation == "ANGLE-30x30x3", "angle designation");
		throws(() -> new Angle(10, 10, 10), "thickness must be less than either leg");
		var anglePart = angle.geometry(40);
		solid(anglePart, "angle");
		near(anglePart.volume(), 171 * 40, "angle volume");
		anglePart.close();

		var channel = new Channel(40, 20, 3);
		check(channel.designation == "CHANNEL-40x20x3", "channel designation");
		throws(() -> new Channel(10, 10, 6), "thickness is too thick for its height");
		throws(() -> new Channel(40, 3, 3), "must be less than the flange width");
		var channelPart = channel.geometry(60);
		solid(channelPart, "channel");
		near(channelPart.volume(), 222 * 60, "channel volume");
		channelPart.close();

		var bar = new FlatBar(50, 5);
		check(bar.designation == "FLAT-50x5", "flat bar designation");
		throws(() -> new FlatBar(-1, 5), "positive width and thickness");
		var barPart = bar.geometry(200);
		solid(barPart, "flat bar");
		near(barPart.volume(), 50 * 5 * 200, "flat bar volume");
		barPart.close();

		var tslot = new TSlotExtrusion(20);
		check(tslot.designation == "TSLOT-20x20", "t-slot designation");
		throws(() -> new TSlotExtrusion(-1), "positive size");
		var tslotPart = tslot.geometry(100);
		solid(tslotPart, "t-slot extrusion");
		var tslotBox = bounds(tslotPart);
		near(tslotBox.maxX, 10, "t-slot outer boundary");
		near(tslotBox.minZ, 0, "t-slot start");
		near(tslotBox.maxZ, 100, "t-slot end");
		var tslotVolume = tslotPart.volume();
		check(tslotVolume < 20 * 20 * 100, "t-slot removes material for slots and bore");
		check(tslotVolume > 10 * 20 * 100, "t-slot keeps most of its cross-section");
		// Four slots (3 mm x 6 mm throat plus 2 mm x 7 mm head) and a 3 mm bore, cut end to end.
		near(tslotVolume, (20 * 20 - 4 * (3 * 6 + 2 * 7) - Math.PI * 1.5 * 1.5) * 100, "t-slot volume");
		tslotPart.close();
		check(new FlatBar(12.7, 3.2).designation == "FLAT-12.7x3.2", "fractional flat bar designation");
		check(new TSlotExtrusion(25.4).designation == "TSLOT-25.4x25.4", "fractional t-slot designation");

		var frame = new FrameAssembly();
		frame.point("A", 0, 0, 0);
		frame.point("B", 0, 0, 500);
		frame.point("C", 300, 0, 500);
		throws(() -> frame.point("A", 1, 1, 1), 'Duplicate frame point "A"');
		frame.point("D", 300, 0, 0);
		frame.member("upright", "A", "B", tube);
		frame.member("beam", "B", "C", tube);
		frame.member("brace", "C", "D", tslot);
		throws(() -> frame.member("upright", "A", "C", tube), 'Duplicate frame member "upright"');
		throws(() -> frame.member("bad", "A", "Z", tube), 'Unknown frame point "Z"');
		throws(() -> frame.member("bad", "A", "A", tube), 'needs distinct endpoints');
		throws(() -> frame.length("missing"), 'Unknown frame member "missing"');

		var zeroFrame = new FrameAssembly();
		zeroFrame.point("A", 0, 0, 0);
		zeroFrame.point("D", 0, 0, 0);
		zeroFrame.member("zero", "A", "D", tube);
		throws(() -> zeroFrame.geometry("zero"), 'has coincident endpoints');

		near(frame.length("upright"), 500, "upright length");
		near(frame.length("beam"), 300, "beam length");
		var upright = frame.geometry("upright");
		solid(upright, "upright member");
		near(upright.volume(), (40 * 40 - 34 * 34) * 500, "upright member volume");
		var uprightBox = bounds(upright);
		near(uprightBox.maxX, 20, "upright cross-section extent");
		near(uprightBox.minZ, 0, "upright start");
		near(uprightBox.maxZ, 500, "upright end");
		upright.close();
		var beam = frame.geometry("beam");
		solid(beam, "beam member");
		var beamBox = bounds(beam);
		near(beamBox.minX, 0, "beam start");
		near(beamBox.maxX, 300, "beam end");
		near(beamBox.minZ, 480, "beam cross-section low");
		near(beamBox.maxZ, 520, "beam cross-section high");
		beam.close();

		// Section orientation: the channel's local +Y (its 40 mm height) follows the reference;
		// local +X (the 20 mm flanges) is reference x member axis.
		var channelFrame = new FrameAssembly();
		channelFrame.point("P", 0, 0, 0);
		channelFrame.point("Q", 100, 0, 0);
		channelFrame.member("up", "P", "Q", channel);
		channelFrame.member("side", "P", "Q", channel, new Vector(0, 1, 0));
		channelFrame.member("parallel", "P", "Q", channel, new Vector(2, 0, 0));
		throws(() -> channelFrame.member("zeroRef", "P", "Q", channel, new Vector(0, 0, 0)), "reference has zero length");
		var upChannel = channelFrame.geometry("up");
		var upBox = bounds(upChannel);
		near(upBox.minX, 0, "channel member start");
		near(upBox.maxX, 100, "channel member end");
		near(upBox.minZ, 0, "channel height starts on the member axis");
		near(upBox.maxZ, 40, "channel height follows the default +Z reference");
		upChannel.close();
		var sideChannel = channelFrame.geometry("side");
		var sideBox = bounds(sideChannel);
		// Reference +Y: local X = Y x X = -Z, so the flanges hang below the member axis.
		near(sideBox.minZ, -20, "channel flanges follow reference x axis");
		near(sideBox.maxZ, 0, "channel web on the member axis");
		sideChannel.close();
		throws(() -> channelFrame.geometry("parallel"), "reference is parallel to its axis");

		var cutList = frame.cutList();
		check(cutList.length == 2, "cut list groups by profile");
		for (line in cutList) {
			if (line.designation == tube.designation) {
				check(line.quantity == 2, "rect tube cut list quantity");
				near(line.totalLength, 800, "rect tube cut list total length");
			} else if (line.designation == tslot.designation) {
				check(line.quantity == 1, "t-slot cut list quantity");
				near(line.totalLength, 500, "t-slot cut list total length");
			} else {
				throw 'unexpected cut list designation "${line.designation}"';
			}
		}
	}

	static function gears():Void {
		var gear = new SpurGear(2, 20, 12);
		check(gear.designation == "SPUR-M2-20T", "spur gear designation");
		near(gear.pitchDiameter, 40, "spur gear pitch diameter");
		near(gear.baseDiameter, 40 * Math.cos(SpurGear.STANDARD_PRESSURE_ANGLE), "spur gear base diameter");
		near(gear.outsideDiameter, 44, "spur gear outside diameter");
		near(gear.rootDiameter, 35, "spur gear root diameter");
		throws(() -> new SpurGear(-1, 20, 12), "positive module");
		throws(() -> new SpurGear(2, 5, 12), "at least 6 teeth");
		check(SpurGear.minimumUnshiftedTeeth(SpurGear.STANDARD_PRESSURE_ANGLE) == 18, "20 degree undercut limit");
		throws(() -> new SpurGear(2, 17, 12), "would require undercut");
		throws(() -> new SpurGear(2, 20, -1), "positive face width");

		var part = gear.geometry();
		solid(part, "spur gear");
		var box = bounds(part);
		near(box.maxX, gear.outsideDiameter / 2, "spur gear outside radius");
		check(box.maxX > gear.pitchDiameter / 2, "spur gear teeth extend past the pitch circle");
		part.close();

		var pinion = new SpurGear(2, 18, 12);
		throws(() -> pinion.centerDistance(new SpurGear(2.5, 20, 12)), "share a module");
		throws(() -> GearPair.mesh(pinion, new SpurGear(2, 20, 12, 25 * Math.PI / 180)), "share a pressure angle");
		var pair = GearPair.mesh(pinion, gear);
		near(pair.centerDistance, (pinion.pitchDiameter + gear.pitchDiameter) / 2, "gear pair centre distance");
		near(pair.ratio(), gear.teeth / pinion.teeth, "gear pair ratio");
		near(pair.pose().x, pair.centerDistance, "gear pair pose offset");
		var aSolid = pinion.geometry(), bSolid = gear.geometry();
		var maxPenetration = 0.0;
		for (sample in 0...17) {
			var aAngle = 2 * Math.PI * sample / (pinion.teeth * 16);
			var bAngle = pair.bRotation - aAngle * pinion.teeth / gear.teeth;
			var aPlaced = aSolid.placed(new Location(new Plane(new Vector(0, 0, 0),
				new Vector(Math.cos(aAngle), Math.sin(aAngle), 0), Vector.Z())));
			var bPlaced = bSolid.placed(new Location(new Plane(new Vector(pair.centerDistance, 0, 0),
				new Vector(Math.cos(bAngle), Math.sin(bAngle), 0), Vector.Z())));
			var overlap = aPlaced.intersect(bPlaced);
			maxPenetration = Math.max(maxPenetration, overlap.volume());
			overlap.close();
			aPlaced.close();
			bPlaced.close();
		}
		aSolid.close();
		bSolid.close();
		// The polyline tooth flanks and CAD booleans can leave tiny numerical overlap.
		check(maxPenetration <= 0.05, 'gear mesh penetrates by $maxPenetration mm^3 over one tooth pitch');
		var badA = pinion.geometry();
		var badBBase = gear.geometry();
		var badB = badBBase.placed(new Location(new Plane(new Vector(pair.centerDistance, 0, 0),
			new Vector(Math.cos(pair.bRotation + Math.PI / gear.teeth),
				Math.sin(pair.bRotation + Math.PI / gear.teeth), 0), Vector.Z())));
		var badOverlap = badA.intersect(badB);
		check(badOverlap.volume() > 1, "wrong tooth phase must produce detectable interference");
		badOverlap.close();
		badA.close();
		badB.close();
		badBBase.close();

		// SpurGear centres a tooth on local angle 0, so `a` points a tooth at the mesh point and
		// `b` must present a tooth space at its local angle pi: (pi - turn) / pitch angle = k + 1/2.
		for (teeth in [20, 21]) {
			var meshed = GearPair.mesh(pinion, new SpurGear(2, teeth, 12));
			var meshedPose = meshed.pose();
			var turn = 2 * Math.atan2(meshedPose.qz, meshedPose.qw);
			near(turn, teeth % 2 == 0 ? Math.PI / teeth : 0.0, 'gear pair turn for $teeth teeth', 1e-9);
			near(meshed.bRotation, turn, 'gear pair bRotation for $teeth teeth', 1e-9);
			var phase = (Math.PI - turn) / (2 * Math.PI / teeth);
			near(phase - Math.ffloor(phase), 0.5, 'gear with $teeth teeth has a tooth space at the mesh point', 1e-9);
			near(meshedPose.x, meshed.centerDistance, 'gear pair offset for $teeth teeth');
		}
		throws(() -> new SpurGear(2, 20, 12, 0.1), "between 14.5 and 25 degrees");
		throws(() -> new SpurGear(2, 20, 12, 0.5), "between 14.5 and 25 degrees");
		check(new SpurGear(2, 33, 12, 14.5 * Math.PI / 180).teeth == 33, "14.5 degree pressure angle accepted");
		check(new SpurGear(2, 20, 12, 25 * Math.PI / 180).teeth == 20, "25 degree pressure angle accepted");
		check(new SpurGear(0.8, 20, 5).designation == "SPUR-M0.8-20T", "fractional module designation");

		var rack = new Rack(2, 10, 12);
		check(rack.designation == "RACK-M2-10T", "rack designation");
		near(rack.length, 10 * Math.PI * 2, "rack length");
		throws(() -> new Rack(2, 0, 12), "at least one tooth");

		var rackPart = rack.geometry();
		solid(rackPart, "rack");
		var rackBox = bounds(rackPart);
		near(rackBox.minX, 0, "rack face start");
		near(rackBox.maxX, 12, "rack face width");
		near(rackBox.minZ, 0, "rack length start");
		near(rackBox.maxZ, rack.length, "rack length end");
		// Square ends: a 0.01 mm slice at each end is the full bar section below the root line
		// (faceWidth x barHeight); the nearest tooth flank starts about 0.66 mm in.
		for (end in [0.0, 1.0]) {
			var slab = Part.box(100, 100, 1.01);
			var placedSlab = slab.translated(new Vector(0, 0, end == 0 ? -1 : rack.length - 0.01));
			slab.close();
			var cap = rackPart.intersect(placedSlab);
			placedSlab.close();
			near(cap.volume(), 12 * rack.barHeight * 0.01, end == 0 ? "rack start face is square" : "rack end face is square", 1e-3);
			cap.close();
		}
		rackPart.close();
		throws(() -> new Rack(2, 10, 12, 0.5), "between 14.5 and 25 degrees");
		check(new Rack(0.8, 10, 5).designation == "RACK-M0.8-10T", "fractional rack designation");
	}

	static function pillowBlock():Void {
		var bearing = DeepGrooveBearing.metric("6204");
		var block = new PillowBlock(bearing);
		check(block.housing.fit == BearingHousingFit.Slip, "pillow block uses a named housing fit");
		check(block.housing.mountScrew == "M6", "pillow block mount screw size");
		near(block.housing.face, 68.4, "pillow block face leaves 1 mm around the M6 heads");
		near(block.housing.depth, 23.4, "pillow block depth");
		near(block.housing.boltSpacing, 56.4, "pillow block bolt spacing");
		for (designation in ["608", "6000", "6001", "6002", "6003", "6204"]) {
			var housing = new PillowBlockHousing(DeepGrooveBearing.metric(designation));
			var screw = housing.mountScrewPart(10).spec, edge = (housing.face - housing.boltSpacing) / 2;
			check(edge >= screw.headDiameter / 2 + 1 - 1e-9, 'housing for $designation keeps screw heads on the face');
			check(edge > screw.clearanceCoarse / 2, 'housing for $designation bolt holes stay inside the edge');
			var part = housing.geometry();
			solid(part, 'housing for $designation');
			part.close();
		}

		var envelope = block.housing.geometry(Envelope);
		solid(envelope, "pillow block envelope");
		var envelopeVolume = envelope.volume();
		near(envelopeVolume, 68.4 * 68.4 * 23.4 - Math.PI * Math.pow((47 + block.housing.allowance) / 2, 2) * 23.4,
			"pillow block envelope volume");
		envelope.close();
		var preview = block.housing.geometry();
		solid(preview, "pillow block preview");
		check(preview.volume() < envelopeVolume, "preview also removes bolt holes");
		preview.close();

		near(block.housing.connector("bore").frame.z, 11.7, "pillow block bore connector");
		near(block.housing.connector("bolt1").frame.x, 28.2, "pillow block bolt connector x");

		var model = new AssemblyModel();
		block.addTo(model, "pb");
		var definition = model.definition("pillow-block");
		check(definition.joints.length == 5, "pillow block joint count");
		var state = model.initialState("pillow-block");
		var depth = block.housing.depth;
		near(state.worldConnector("pb-bearing", "axis").z, depth / 2, "bearing centred in the housing depth");
		near(state.worldConnector("pb-bearing", "front").z, depth / 2 - bearing.width / 2, "bearing front inside the housing");
		// Screws seat on the outer face and reach through the mounting face into the frame.
		near(block.screw.length, 35, "pillow block screw: next standard length over depth + 1.5 d");
		check(block.screw.length >= depth + 1.5 * block.screw.diameter, "pillow block screw engagement");
		for (i in 1...5) {
			var head = state.worldConnector('pb-screw$i', "head");
			var bolt = block.housing.connector('bolt$i').frame;
			near(head.x, bolt.x, 'pillow block screw$i on its bolt x');
			near(head.y, bolt.y, 'pillow block screw$i on its bolt y');
			near(head.z, depth, 'pillow block screw$i head on the outer face');
			check(state.worldConnector('pb-screw$i', "tip").z < 0, 'pillow block screw$i tip below the mounting face');
		}
		near(PillowBlock.standardScrewLength(20), 20, "standard screw length exact");
		near(PillowBlock.standardScrewLength(20.1), 25, "standard screw length rounds up");

		var lines = block.bom().lines();
		check(lines.length == 3, "pillow block BOM line count");
		check(block.bom().quantity(block.screw.designation) == 4, "pillow block screw quantity");
		for (entry in block.components()) {
			var part = entry.component.geometry(Envelope);
			check(part.valid(), '${entry.id} envelope is invalid');
			part.close();
		}
	}

	static function linearAxis():Void {
		var axis = new LinearAxis();
		check(axis.motor.designation == "23HS22-2804S", "linear axis motor designation");
		check(axis.bearing.designation == "6000-2Z", "linear axis default bearing matches the 10 mm screw");
		check(axis.coupling.designation == "COUPLING-6.35x10-18x30", "linear axis coupling joins motor and screw");
		near(axis.carriage.boreDiameter, 10, "linear axis carriage bore");
		check(axis.nut.lead == 2, "linear axis lead nut");
		check(axis.nut.thread.designation == "TR-D10-P2-S1-RH", "axis default thread is explicit");
		check(axis.screw.thread.designation == axis.nut.thread.designation, "axis screw and nut share thread specification");
		check(axis.guideBearingA.designation == "LM8UU", "linear axis round guide bearing");
		check(axis.guideSystem.bearingDesignation == "LM8UU", "linear axis guide catalog row");
		check(axis.guideSystem.rodFit == BearingShaftFit.Slip, "linear axis guide rod fit");
		check(axis.guideSystem.housingFit == BearingHousingFit.Slip, "linear axis guide seat fit");
		near(axis.guideSystem.rodDiameter, 7.99, "linear axis guide rod fit diameter");
		near(axis.guideSystem.seatDiameter, 15.0375, "linear axis guide seat fit diameter");
		near(axis.carriage.guideSeatDiameter, axis.guideSystem.seatDiameter, "carriage uses guide seat fit");
		near(axis.guideSystem.rodLength, axis.length, "guide rods span the axis");
		check(axis.guideRodA == axis.guideSystem.rodA && axis.guideBearingA == axis.guideSystem.bearingA,
			"axis exposes guide system components");
		var carriagePreview = axis.carriage.geometry();
		solid(carriagePreview, "carriage with nut and guide seats");
		check(carriagePreview.volume() < axis.carriage.width * axis.carriage.width * axis.carriage.length,
			"carriage preview cuts mounting and guide holes");
		carriagePreview.close();
		// Layout from the screw input: coupling half (15) + gap (2) + housing (depth), margin (20),
		// carriage (60) + stroke (200), margin (20), housing (depth) flush with the screw end.
		var depth = axis.pillowBlockA.housing.depth;
		near(depth, 14, "6000 housing depth");
		near(axis.bearingAPosition, 15 + 2 + depth / 2, "pillow block A just past the coupling");
		near(axis.travelMin, 15 + 2 + depth + 30 + 30, "carriage travel starts a margin past pillow block A");
		near(axis.travelMax - axis.travelMin, 200, "carriage travel equals the stroke");
		near(axis.bearingBPosition, axis.travelMax + 30 + 30 + depth / 2, "pillow block B a margin past the travel end");
		near(axis.length, 365, "linear axis screw length");
		near(axis.screw.totalLength, axis.length, "linear axis screw total length");
		throws(() -> new LinearAxis(23, 10, -1), "positive stroke");
		throws(() -> new LinearAxis(23, 10, 200, "6001"), "bore does not match the screw diameter");
		throws(() -> new LinearAxis(23, 50), "No catalog deep groove bearing has a 50 mm bore");
		throws(() -> new LinearAxis(23, 10, 200, null, 2), "must clear the pillow block screw heads and lead nut");
		throws(() -> new LinearAxis(23, 8), "explicit thread for a nondefault screw diameter");
		throws(() -> new LinearAxis(23, 8, 200, null, 30, new LeadScrewThread(MetricTrapezoidal, 10, 2)),
			"thread diameter must match the screw diameter");

		for (entry in axis.components()) {
			var part = entry.component.geometry(Envelope);
			check(part.valid(), '${entry.id} envelope is invalid');
			part.close();
		}
		var rail = axis.frame.geometry("rail");
		solid(rail, "linear axis rail");
		var railBox = rail.shape.bounds();
		near(railBox.get_min().get_z(), 21, "rail starts at the screw input");
		near(railBox.get_max().get_z(), 21 + axis.length, "rail ends at the screw end");
		// Beside the screw, clear of the housings (and their screw heads) and the carriage.
		var housing = axis.pillowBlockA.housing;
		check(railBox.get_max().get_y() <= -housing.face / 2 - 1, "rail clears the pillow block housings");
		check(railBox.get_max().get_y() <= -(housing.boltSpacing + axis.pillowBlockA.screw.spec.headDiameter) / 2 - 1,
			"rail clears the pillow block screw heads");
		check(railBox.get_max().get_y() <= -axis.carriage.width / 2 - 1, "rail clears the carriage");
		rail.close();
		var cutList = axis.frame.cutList();
		check(cutList.length == 1, "linear axis frame cut list");
		near(cutList[0].totalLength, axis.length, "linear axis rail length");

		var model = axis.assembly();
		var definition = model.definition("linear-axis");
		check(definition.joints.length == 16, "linear axis joint count");
		for (joint in definition.joints) if (joint.id == "carriage-slide") {
			check(joint.limits.lower != null && joint.limits.lower == axis.travelMin, "carriage lower limit");
			check(joint.limits.upper != null && joint.limits.upper == axis.travelMax, "carriage upper limit");
		}
		var state = model.initialState("linear-axis");
		near(state.worldConnector("coupling", "axis").z, 21, "coupling centred on the motor shaft tip");
		near(state.worldConnector("screw", "input").z, 21, "screw seats on the motor shaft");
		near(state.worldConnector("carriage", "bore").z, 21 + axis.travelMin, "carriage starts at its lower travel limit");
		near(state.worldConnector("guideBearingA", "axis").z, state.worldConnector("carriage", "bore").z,
			"first guide bearing is aligned at the lower stroke end");
		near(state.worldConnector("guideBearingB", "axis").z, state.worldConnector("carriage", "bore").z,
			"second guide bearing is aligned at the lower stroke end");
		near(state.worldConnector("guideRodA", "input").x, -axis.guideSpacing, "first guide rod offset");
		near(state.worldConnector("guideBearingA", "axis").x, -axis.guideSpacing, "first bearing follows guide rod");
		near(state.worldConnector("leadNut", "mountFace").z, state.worldConnector("carriage", "nutMount").z,
			"lead nut mounts to the carriage face");
		near(state.worldConnector("pillowA-bearing", "axis").z, 21 + axis.bearingAPosition, "pillow block A bearing on the screw");
		near(state.worldConnector("pillowB-bearing", "axis").z, 21 + axis.bearingBPosition, "pillow block B bearing on the screw");
		near(state.worldConnector("pillowA-bearing", "axis").z, 45, "pillow block A bearing position");
		near(state.worldConnector("pillowB-bearing", "axis").z, 379, "pillow block B bearing position");
		// Housing A mounts toward the motor, housing B is turned over to mount toward the far end:
		// both screw heads face the carriage and their tips point outboard.
		near(state.worldConnector("pillowA-screw1", "head").z, 21 + axis.bearingAPosition + depth / 2, "pillow A screw heads inboard");
		check(state.worldConnector("pillowA-screw1", "tip").z < 21 + axis.bearingAPosition - depth / 2, "pillow A screw tips outboard");
		near(state.worldConnector("pillowB-screw1", "head").z, 21 + axis.bearingBPosition - depth / 2, "pillow B screw heads inboard");
		check(state.worldConnector("pillowB-screw1", "tip").z > 21 + axis.length, "pillow B screw tips outboard");

		axis.setTravel(state, 100);
		near(state.joint("coupling"), axis.nut.rotationFor(100), "screw rotation follows nut lead");
		near(state.worldConnector("carriage", "bore").z, 21 + axis.travelMin + 100, "carriage travels with screw rotation");
		near(state.worldConnector("guideBearingB", "axis").x, axis.guideSpacing, "bearing stays on second guide");
		axis.setTravel(state, 0);
		var unturned = state.worldPose("carriage");
		axis.setTravel(state, 2);
		near(state.joint("coupling"), 2 * Math.PI, "one screw turn advances by lead");
		near(state.worldPose("carriage").qz, unturned.qz, "carriage does not rotate with screw");
		throws(() -> axis.setTravel(state, axis.stroke + 1), "outside its stroke");
		var multiAxis = new LinearAxis(23, 10, 200, null, 30,
			new LeadScrewThread(MetricTrapezoidal, 10, 2, 4));
		var multiState = multiAxis.assembly().initialState("linear-axis");
		multiAxis.setTravel(multiState, 8);
		near(multiState.joint("coupling"), 2 * Math.PI, "multi-start axis moves 8 mm per turn");
		near(multiState.worldConnector("carriage", "bore").z, 21 + multiAxis.travelMin + 8,
			"multi-start carriage travel");
		var leftAxis = new LinearAxis(23, 10, 200, null, 30,
			new LeadScrewThread(MetricTrapezoidal, 10, 2, 4, LeftHand));
		var leftState = leftAxis.assembly().initialState("linear-axis");
		leftAxis.setTravel(leftState, 8);
		near(leftState.joint("coupling"), -2 * Math.PI, "left-hand axis reverses rotation");
		state.setJoint("carriage-slide", axis.travelMax);
		state.forwardKinematics();
		near(state.worldConnector("guideBearingA", "axis").z, state.worldConnector("carriage", "bore").z,
			"first guide bearing is aligned at the upper stroke end");
		near(state.worldConnector("guideBearingB", "axis").z, state.worldConnector("carriage", "bore").z,
			"second guide bearing is aligned at the upper stroke end");
		var carriageEnd = state.worldConnector("carriage", "bore").z + axis.carriage.length / 2;
		near(state.worldConnector("pillowB-bearing", "axis").z - depth / 2 - carriageEnd, 30,
			"carriage stops a margin short of pillow block B");

		var bom = axis.bom();
		var lines = bom.lines();
		check(lines.length == 11, "linear axis BOM line count");
		check(bom.quantity(axis.bearing.designation) == 2, "linear axis bearing quantity");
		check(bom.quantity(axis.coupling.designation) == 1, "linear axis coupling in the BOM");
		check(bom.quantity(axis.nut.designation) == 1, "linear axis lead nut in the BOM");
		check(bom.quantity(axis.screw.designation) == 1, "thread-specific lead screw in the BOM");
		check(bom.quantity(axis.guideRodA.designation) == 2, "linear axis guide rods in the BOM");
		check(bom.quantity(axis.guideBearingA.designation) == 2, "linear axis guide bearings in the BOM");
		check(bom.quantity("RECT-20x15x2-L365") == 1, "linear axis rail in the BOM");
		check(bom.quantity(axis.pillowBlockA.screw.designation) == 8, "linear axis pillow block screws");
	}

	static function catalogExtras():Void {
		var bushing = new Bushing(8);
		check(bushing.designation == "BUSHING-8x11x12", "bushing designation");
		throws(() -> new Bushing(-1), "positive bore diameter");
		var bushingPart = bushing.geometry();
		solid(bushingPart, "bushing");
		near(bushingPart.volume(), Math.PI * (5.5 * 5.5 - 4 * 4) * 12, "bushing volume");
		bushingPart.close();
		near(bushing.connector("axis").frame.z, 6, "bushing axis connector");

		var coupling = new ShaftCoupling(5, 8);
		check(coupling.designation == "COUPLING-5x8-14.4x24", "shaft coupling designation");
		check(coupling.setScrew == "M3", "shaft coupling set screw size");
		throws(() -> new ShaftCoupling(-1, 8), "positive bore diameters");
		var couplingPart = coupling.geometry();
		solid(couplingPart, "shaft coupling");
		check(couplingPart.volume() < Math.PI * 7.2 * 7.2 * 24, "shaft coupling removes both bores");
		couplingPart.close();
		near(coupling.connector("sideB").frame.z, 24, "shaft coupling sideB connector");

		var linearBearing = LinearBearing.metric("LM8UU");
		check(linearBearing.designation == "LM8UU", "linear bearing designation");
		near(linearBearing.guideRodDiameter(BearingShaftFit.Slip), 7.99, "linear bearing shaft fit diameter");
		near(linearBearing.housingSeatDiameter(BearingHousingFit.Slip), 15.0375, "linear bearing housing fit diameter");
		var linearSeat = linearBearing.housingSeat(10, BearingHousingFit.Interference);
		near(linearSeat.volume(), Math.PI * Math.pow((15 - 0.015) / 2, 2) * 10, "linear bearing housing seat tool");
		linearSeat.close();
		throws(() -> LinearBearing.metric("LM9UU"), 'Unknown linear bearing "LM9UU"');
		var linearBearingPart = linearBearing.geometry();
		solid(linearBearingPart, "linear bearing");
		near(linearBearingPart.volume(), Math.PI * (7.5 * 7.5 - 4 * 4) * 24, "linear bearing volume");
		linearBearingPart.close();

		var sprocket = new Sprocket(12.7, 20, 8, 6);
		check(sprocket.designation == "GENERIC-SPROCKET-P12.7-20T", "sprocket designation");
		near(sprocket.pitchDiameter, 12.7 / Math.sin(Math.PI / 20), "sprocket pitch diameter");
		near(sprocket.rollerDiameter, 0.625 * 12.7, "sprocket default roller diameter");
		near(sprocket.rootDiameter, sprocket.pitchDiameter - 0.625 * 12.7, "sprocket root = pitch - roller diameter");
		near(sprocket.outsideDiameter, 12.7 * (0.6 + Math.cos(Math.PI / 20) / Math.sin(Math.PI / 20)),
			"sprocket outside diameter p(0.6 + cot(pi/z))");
		near(new Sprocket(12.7, 20, 8, 6, 7.92).rootDiameter, sprocket.pitchDiameter - 7.92, "sprocket roller override");
		var ansi40 = Sprocket.forChain("ANSI40", 20, 8, 6);
		check(ansi40.designation == "SPROCKET-ANSI40-20T", "chain family designation");
		near(ansi40.rollerDiameter, 7.92, "ANSI40 roller from catalog");
		near(Sprocket.forChain("ANSI35", 20, 6, 6).pitch, 9.525, "ANSI35 pitch from catalog");
		throws(() -> Sprocket.forChain("ANSI45", 20, 8, 6), "Unknown roller chain");
		throws(() -> new Sprocket(12.7, 20, 8, 6, 7.8, "ANSI40"), "must match its catalog entry");
		throws(() -> new Sprocket(12.7, 20, 8, 6, 13), "roller diameter must be positive and less than the pitch");
		throws(() -> new Sprocket(12.7, 5, 8, 6), "at least 8 teeth");
		throws(() -> new Sprocket(12.7, 8, 40, 6), "must clear the bore");
		var sprocketPart = sprocket.geometry();
		solid(sprocketPart, "sprocket");
		var sprocketBox = bounds(sprocketPart);
		check(sprocketBox.maxX <= sprocket.outsideDiameter / 2 + 1e-6, "sprocket stays within its outside radius");
		check(sprocketBox.maxX > sprocket.pitchDiameter / 2, "sprocket teeth extend past the pitch circle");
		sprocketPart.close();

		var pulley = new TimingPulley(GT2, 20, 5, 6);
		check(pulley.designation == "PULLEY-GT2-20T", "timing pulley designation");
		near(pulley.pitchDiameter, 2 * 20 / Math.PI, "timing pulley pitch diameter");
		near(pulley.pitchLineDifferential, 0.254, "GT2 pitch line differential");
		near(pulley.outsideDiameter, 2 * 20 / Math.PI - 2 * 0.254, "timing pulley OD = PD - 2 PLD");
		near(new TimingPulley(HTD3M, 20, 5, 6).pitchLineDifferential, 0.381, "3 mm pitch line differential");
		near(new TimingPulley(HTD5M, 20, 5, 6).pitchLineDifferential, 0.5715, "5 mm pitch line differential");
		check(new TimingPulley(T5, 20, 5, 6).designation == "PULLEY-T5-20T", "T5 family has its own designation");
		near(new TimingPulley(T5, 20, 5, 6).pitchLineDifferential, 0.5, "T5 PLD differs from HTD5M");
		near(new TimingPulley(XL, 20, 5, 6).outsideDiameter, 5.08 * 20 / Math.PI - 0.508, "explicit PLD");
		check(new TimingPulley(Custom("CUSTOM2032", 2.032, 0.254), 20, 5, 6).designation == "PULLEY-CUSTOM-CUSTOM2032-P2.032-PLD0.254-20T", "fractional pulley designation");
		throws(() -> new TimingPulley(GT2, 5, 5, 6), "at least 8 teeth");
		throws(() -> new TimingPulley(GT2, 8, 11, 6), "must clear the bore");
		var pulleyPart = pulley.geometry();
		solid(pulleyPart, "timing pulley");
		var pulleyBox = bounds(pulleyPart);
		check(pulleyBox.maxX <= pulley.outsideDiameter / 2 + 1e-6, "timing pulley stays within its outside radius");
		check(pulleyBox.maxX > pulley.grooveDiameter / 2, "timing pulley lands extend past the groove circle");
		pulleyPart.close();

		var thread = new LeadScrewThread(MetricTrapezoidal, 8, 2);
		var nut = new LeadScrewNut(thread);
		check(nut.designation == "LEADNUT-TR-D8-P2-S1-RH", "lead screw nut designation");
		check(nut.thread.pitch == 2 && nut.thread.starts == 1, "nut carries pitch and starts");
		check(nut.mountScrew == "M3", "lead screw nut mount screw size");
		near(nut.travelPerRevolution(), 2, "lead screw nut travel per revolution");
		near(nut.rotationFor(10), 10 / 2 * 2 * Math.PI, "lead screw nut rotation for a travel distance");
		throws(() -> new LeadScrewThread(MetricTrapezoidal, -1, 2), "positive screw diameter");
		throws(() -> new LeadScrewThread(MetricTrapezoidal, 8, -1), "positive pitch");
		throws(() -> new LeadScrewThread(MetricTrapezoidal, 8, 2, 0), "at least one start");
		throws(() -> new LeadScrewNut(thread, 2), "at least 3 mounting bolts");
		var multi = new LeadScrewNut(new LeadScrewThread(MetricTrapezoidal, 8, 2, 4));
		check(multi.designation == "LEADNUT-TR-D8-P2-S4-RH", "multi-start nut designation");
		near(multi.lead, 8, "four-start lead is four times pitch");
		near(multi.travelPerRevolution(), 8, "four-start travel per positive revolution");
		var left = new LeadScrewNut(new LeadScrewThread(MetricTrapezoidal, 8, 2, 4, LeftHand));
		near(left.travelPerRevolution(), -8, "left-hand nut travels opposite on positive revolution");

		var nutEnvelope = nut.geometry(Envelope);
		solid(nutEnvelope, "lead screw nut envelope");
		near(nutEnvelope.volume(), Math.PI * 5.2 * 5.2 * 16 + Math.PI * 12 * 12 * 3 - Math.PI * 4 * 4 * 19,
			"lead screw nut envelope volume");
		nutEnvelope.close();
		var nutPreview = nut.geometry();
		solid(nutPreview, "lead screw nut preview");
		nutPreview.close();
		near(nut.connector("bore").frame.z, 8, "lead screw nut bore connector");
		near(nut.connector("mount1").frame.z, 19, "lead screw nut mount connector z");
		near(nut.connector("mount1").frame.x, 7.9, "lead screw nut mount connector x");
		for (size in [6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0]) {
			var sized = new LeadScrewNut(new LeadScrewThread(MetricTrapezoidal, size, 2));
			var screw = sized.mountScrewPart(10).spec, r = sized.boltCircleDiameter / 2;
			check(r - screw.clearanceMedium / 2 >= sized.bodyDiameter / 2 + 1 - 1e-9,
				'lead screw nut D$size mount holes clear the body');
			check(r + screw.headDiameter / 2 <= sized.flangeDiameter / 2 - 1 + 1e-9,
				'lead screw nut D$size mount screw heads stay on the flange');
		}
		check(new LeadScrewNut(new LeadScrewThread(Acme, 6.35, 3.175)).designation == "LEADNUT-ACME-D6.35-P3.175-S1-RH", "fractional nut designation");
	}

	/** `part` (closed) moved to the assembly pose `frame`. */
	static function placedAt(part:Part, frame:AssemblyFrame):Part {
		var x = AssemblyFrames.transformVector(frame, 1, 0, 0), z = AssemblyFrames.transformVector(frame, 0, 0, 1);
		try {
			var result = part.placed(new Location(new Plane(new Vector(frame.x, frame.y, frame.z), new Vector(x.x, x.y, x.z),
				new Vector(z.x, z.y, z.z))));
			part.close();
			return result;
		} catch (error:Dynamic) {
			part.close();
			throw error;
		}
	}

	/** Checks the fused volume of `a` and `b` is their summed volume less `overlap`; closes both. */
	static function checkOverlap(a:Part, b:Part, overlap:Float, message:String):Void {
		var expected = a.volume() + b.volume() - overlap;
		var fused = a.combine(b);
		a.close();
		b.close();
		near(fused.volume(), expected, message, 1e-6);
		fused.close();
	}

	static function robotics():Void {
		var flange = new RobotFlange(50);
		check(flange.designation == "ISO9409-STYLE-50-4-M6", "robot flange designation");
		check(flange.boltCount == 4, "robot flange ISO bolt count");
		check(flange.mountScrew == "M6", "robot flange mount screw size");
		near(flange.boltCircleDiameter, 50, "robot flange pitch circle");
		near(flange.pilotDiameter, 31.5, "robot flange pilot diameter");
		near(flange.pinDiameter, 6, "robot flange pin diameter");
		near(flange.flangeDiameter, 70, "robot flange outer diameter");
		near(flange.thickness, 9, "robot flange thickness");
		near(flange.pilotHeight, 3, "robot flange pilot height");
		var small = new RobotFlange(31.5);
		check(small.designation == "ISO9409-STYLE-31.5-4-M5", "smallest ISO flange designation");
		near(small.pilotDiameter, 20, "31.5 flange pilot");
		near(small.pinDiameter, 5, "31.5 flange pin");
		var large = new RobotFlange(80);
		check(large.boltCount == 6 && large.mountScrew == "M8", "80 flange uses 6 x M8");
		check(new RobotFlange(50, 6).designation == "FLANGE-50-6-M6", "non-standard bolt count drops the ISO prefix");
		throws(() -> new RobotFlange(-1), "positive diameter");
		throws(() -> new RobotFlange(45), 'Unknown ISO 9409-1 flange size "45"');
		throws(() -> new RobotFlange(50, 2), "at least 3 bolts");
		throws(() -> new RobotFlange(50, 12), "too many bolts for its pitch circle");

		var envelope = flange.geometry(Envelope);
		solid(envelope, "robot flange envelope");
		var envelopeVolume = envelope.volume();
		near(envelopeVolume, Math.PI * 35 * 35 * 9 + Math.PI * 15.75 * 15.75 * 3, "robot flange envelope volume");
		envelope.close();
		var preview = flange.geometry();
		solid(preview, "robot flange preview");
		near(envelopeVolume - preview.volume(), Math.PI * (4 * 3.3 * 3.3 + 3 * 3) * 9, "robot flange bolt and pin holes");
		var flangeBox = bounds(preview);
		near(flangeBox.minZ, -9, "robot flange plate behind its face");
		near(flangeBox.maxZ, 3, "robot flange pilot boss in front of its face");
		preview.close();
		near(flange.connector("bolt1").frame.x, 25, "robot flange bolt1 x");
		near(flange.connector("bolt2").frame.y, 25, "robot flange bolt2 y");
		near(flange.connector("face").frame.z, 0, "robot flange face connector");

		var eoat = new EndEffectorPlate(flange);
		check(eoat.designation == "EOAT-50-9-4xM5-PCD70.5", "end effector plate designation");
		near(eoat.thickness, flange.thickness, "end effector plate default thickness");
		// Default tool circle: flange bolt circle + flange head + tool head + 2 mm web.
		near(eoat.toolBoltCircleDiameter, 50 + 10 + 8.5 + 2, "end effector tool bolt circle clears the flange heads");
		near(eoat.diameter, 70.5 + 8.5 + 2, "end effector plate grows to carry the tool bolts");
		var eoatEnvelope = eoat.geometry(Envelope);
		solid(eoatEnvelope, "end effector plate envelope");
		var eoatEnvelopeVolume = eoatEnvelope.volume();
		near(eoatEnvelopeVolume, Math.PI * 40.5 * 40.5 * 9, "end effector plate envelope volume");
		eoatEnvelope.close();
		var eoatPreview = eoat.geometry();
		solid(eoatPreview, "end effector plate preview");
		// Every hole removes its own full volume: the pilot recess (31.7 x 3.2 deep), 4 flange bolt
		// holes (6.6), the pin hole (6.2), and 4 tool bolt holes (5.5), none overlapping.
		near(eoatEnvelopeVolume - eoatPreview.volume(),
			Math.PI * (15.85 * 15.85 * 3.2 + (4 * 3.3 * 3.3 + 3.1 * 3.1 + 4 * 2.75 * 2.75) * 9),
			"end effector plate tool bolt holes remove material");
		eoatPreview.close();
		throws(() -> new EndEffectorPlate(flange, 5), "must be thicker than the flange's pilot boss");
		throws(() -> new EndEffectorPlate(flange, null, 30), "must clear the flange's pilot recess");
		throws(() -> new EndEffectorPlate(flange, null, 50), "must clear the flange's bolt holes");
		throws(() -> new EndEffectorPlate(flange, null, 70.5, 40), "too close together");
		near(eoat.connector("tool").frame.z, eoat.thickness, "end effector plate tool connector");

		// Flange -> plate: the plate stacks on the flange face, the pilot boss sits in its recess.
		var plateModel = new AssemblyModel();
		flange.addTo(plateModel, "flange");
		eoat.addTo(plateModel, "plate");
		plateModel.mate("tool-mount", "fixed", "flange", "face", "plate", "robot");
		var platePose = plateModel.pose("plate");
		near(platePose.z, 0, "plate sits on the flange face");
		checkOverlap(flange.geometry(), placedAt(eoat.geometry(), platePose), 0, "flange and plate stack without overlap");
		// A plate without the recess would overlap exactly the boss: the boss engages the recess.
		checkOverlap(flange.geometry(Envelope), placedAt(eoat.geometry(Envelope), platePose),
			Math.PI * 15.75 * 15.75 * 3, "flange pilot boss reaches into the plate");

		var pedestal = new Pedestal(flange, 300);
		check(pedestal.designation == "PEDESTAL-50-D70x300", "pedestal designation");
		near(pedestal.columnDiameter, 70, "pedestal column defaults to the flange diameter");
		check(pedestal.floorMountScrew == "M10", "pedestal floor screw size");
		near(pedestal.floorBoltCircleDiameter, 70 + 2 * 16, "pedestal floor bolt circle");
		near(pedestal.baseDiameter, 102 + 2 * 16, "pedestal base diameter");
		// Floor screw heads (16 mm) clear the column and stay on the base.
		check(pedestal.floorBoltCircleDiameter / 2 - 8 >= pedestal.columnDiameter / 2 + 1, "floor bolt heads clear the column");
		check(pedestal.floorBoltCircleDiameter / 2 + 8 <= pedestal.baseDiameter / 2, "floor bolt heads stay on the base");
		throws(() -> new Pedestal(flange, 15), "must clear the flange's pilot boss");
		throws(() -> new Pedestal(flange, 300, 55), "wider than the flange bolt circle plus a screw head");
		var pedestalEnvelope = pedestal.geometry(Envelope);
		solid(pedestalEnvelope, "pedestal envelope");
		pedestalEnvelope.close();
		var pedestalPreview = pedestal.geometry();
		solid(pedestalPreview, "pedestal preview");
		pedestalPreview.close();
		near(pedestal.connector("top").frame.z, 300, "pedestal top connector");
		near(AssemblyFrames.transformVector(pedestal.connector("top").frame, 0, 1, 0).z, -1, "pedestal top points into the pedestal");

		// Pedestal -> flange: the flange turns over onto the top face, its boss in the top recess.
		var pedestalModel = new AssemblyModel();
		pedestal.addTo(pedestalModel, "pedestal");
		flange.addTo(pedestalModel, "flange");
		pedestalModel.mate("robot-mount", "fixed", "pedestal", "top", "flange", "face");
		var flangePose = pedestalModel.pose("flange");
		var mountedFlange = placedAt(flange.geometry(), flangePose);
		var mountedBox = bounds(mountedFlange);
		near(mountedBox.minZ, 300 - 3, "flange pilot boss drops into the pedestal");
		near(mountedBox.maxZ, 300 + 9, "flange plate sits on the pedestal top");
		// The flange turns over about X, so its pin (and pedestal's matching hole) lands at -y.
		var pinWorld = AssemblyFrames.transformPoint(flangePose, flange.pinPoint().x, flange.pinPoint().y, 0);
		near(pinWorld.x, flange.pinPoint().x, "flange pin x on the pedestal");
		near(pinWorld.y, -flange.pinPoint().y, "flange pin y mirrored on the pedestal");
		near(pinWorld.z, 300, "flange pin on the pedestal top face");
		checkOverlap(mountedFlange, pedestal.geometry(), 0, "flange and pedestal stack without overlap");
		checkOverlap(placedAt(flange.geometry(Envelope), flangePose), pedestal.geometry(Envelope),
			Math.PI * 15.75 * 15.75 * 3, "flange pilot boss reaches into the pedestal");
	}

	static function assembly():Void {
		var example = new MotorShaftBearings();
		check(example.screw.designation == "ISO4762-M3x10", "selected mount screw");
		var plate = example.plate.geometry();
		solid(plate, "motor plate");
		near(plate.volume(), 62.3 * 62.3 * 6 - Math.PI * (11.1 * 11.1 + 4 * 1.7 * 1.7) * 6, "plate volume");
		plate.close();
		for (entry in example.components()) {
			var part = entry.component.geometry(Envelope);
			check(part.valid(), '${entry.id} envelope is invalid');
			part.close();
		}

		var model = example.assembly();
		var definition = model.definition("motor-shaft-bearings");
		check(definition.joints.length == 10, "assembly joint count");
		var state = model.initialState("motor-shaft-bearings");
		near(state.worldConnector("plate", "bolt2").z, 6, "plate top");
		var head = state.worldConnector("screw2", "head");
		near(head.x, -15.5, "screw head x");
		near(head.y, 15.5, "screw head y");
		near(head.z, 6, "screw head seat");
		near(state.worldConnector("screw2", "tip").z, -4, "screw engagement");
		near(state.worldConnector("bearingA", "front").z, 34, "first bearing");
		near(state.worldConnector("bearingB", "back").z, 74, "second bearing");

		state.setJoint("coupling", Math.PI / 3);
		state.forwardKinematics();
		for (id in ["bearingA", "bearingB"]) {
			var axis = state.worldConnector(id, "axis");
			near(axis.x, 0, '$id stays on the motor axis x', 1e-9);
			near(axis.y, 0, '$id stays on the motor axis y', 1e-9);
		}
		var turned = state.worldPose("bearingA");
		near(2 * Math.atan2(turned.qz, turned.qw), Math.PI / 3, "bearing turns with the shaft");
		near(state.worldConnector("screw2", "head").x, -15.5, "screws do not rotate");

		var lines = example.bom().lines();
		check(lines.length == 7, "BOM line count");
		var bom = example.bom();
		check(bom.quantity("ISO4762-M3x10") == 4, "screw quantity");
		check(bom.quantity("608-2Z") == 2, "bearing quantity");
		check(bom.quantity("17HS19-1684S1") == 1, "motor quantity");
		check(bom.quantity("DIN6885-B-2x2x6") == 1, "key quantity");
		check(bom.quantity("DIN471-8") == 1, "ring quantity");
		var duplicate = new Bom();
		duplicate.add({partNumber: "X", description: "a", quantity: 1, material: null});
		throws(() -> duplicate.add({partNumber: "X", description: "b", quantity: 1, material: null}), "conflicting");
	}

	static function catalogMetadata():Void {
		check(ParallelKey.catalog().metadata("2x2").standard == "DIN 6885-1", "key standard metadata");
		check(RetainingRing.catalog().metadata("8").dimensionKind == Nominal, "ring dimension metadata");
		check(RobotFlange.catalog().metadata("50").conformance == GenericApproximation,
			"raised-pilot ISO-style flange is a generic approximation");
		check(NemaStepper.catalog().metadata("17").dimensionKind == Mixed,
			"NEMA frame dimensions carry manufacturer drawing provenance");
		metadataComplete(DeepGrooveBearing.catalog());
		metadataComplete(FlatWasher.catalog());
		metadataComplete(HexBolt.catalog());
		metadataComplete(HexNut.catalog());
		metadataComplete(ParallelKey.catalog());
		metadataComplete(RetainingRing.catalog());
		metadataComplete(ShaftCollar.catalog());
		metadataComplete(SocketHeadCapScrew.catalog());
		metadataComplete(LinearBearing.catalog());
		metadataComplete(NemaStepper.catalog());
		metadataComplete(NemaStepper.variantCatalog());
		metadataComplete(RobotFlange.catalog());
		metadataComplete(Sprocket.chainCatalog());
	}

	static function metadataComplete<T>(catalog:Catalog<T>):Void {
		for (designation in catalog.designations()) {
			var metadata = catalog.metadata(designation);
			check(metadata.source != null && metadata.source.length > 0,
				'catalog metadata source missing for $designation');
			if (metadata.sources != null)
				for (source in metadata.sources)
					check(source != null && source.length > 0, 'catalog metadata supplementary source missing for $designation');
		}
	}

	static function main():Void {
		MachineKitReferenceTests.run();
		dimensions();
		catalogMetadata();
		bearings();
		screws();
		motors();
		fasteners();
		shafts();
		shaftHardware();
		structural();
		gears();
		pillowBlock();
		linearAxis();
		catalogExtras();
		robotics();
		assembly();
		trace("MachineKit smoke passed");
	}
}
