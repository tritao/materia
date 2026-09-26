import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
import machinekit.assembly.LinearAxis;
import machinekit.assembly.PillowBlock;
import machinekit.component.Bom;
import machinekit.component.ComponentDetail;
import machinekit.motion.LinearBearing;
import machinekit.motion.NemaStepper;
import machinekit.motion.ShaftCoupling;
import machinekit.motion.SteppedShaft;
import machinekit.standard.Bushing;
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
import materia.project.AssemblyFrames;

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

		var seat = bearing.housingSeat(10, -0.02);
		near(seat.volume(), Math.PI * Math.pow(46.98 / 2, 2) * 10, "housing seat volume");
		seat.close();
		near(bearing.journalDiameter(0.01), 20.01, "journal diameter");
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
			near(box.maxX, motor.spec.face / 2, 'NEMA $frame face');
			near(box.minZ, -motor.bodyLength, 'NEMA $frame body');
			near(box.maxZ, motor.spec.shaftLength, 'NEMA $frame shaft');
			preview.close();
		}
		var motor = NemaStepper.frame(17, 40);
		check(motor.designation == "NEMA17-40", "motor designation");
		throws(() -> NemaStepper.frame(11), 'Unknown NEMA frame "11"');
		var envelope = motor.geometry(Envelope);
		var shaftBeyondPilot = Math.PI * 2.5 * 2.5 * (24 - 2);
		near(envelope.volume(), 42.3 * 42.3 * 40 + Math.PI * 11 * 11 * 2 + shaftBeyondPilot, "motor envelope volume");
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
		near(bolt.threadLength, 24, "M6 bolt thread length");
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
		var seatRadius = corner / 2 + 0.5;
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

	static function shafts():Void {
		var key = ParallelKey.forShaft(6, 6);
		check(key.designation == "DIN6885-2x2x6", "key designation");
		check(key.spec.width == 2 && key.spec.height == 2, "key cross-section");
		var keyPreview = key.geometry();
		solid(keyPreview, "key preview");
		near(keyPreview.volume(), 2 * 2 * 6, "key volume");
		keyPreview.close();
		throws(() -> ParallelKey.metric("9x9", 10), 'Unknown parallel key "9x9"');
		throws(() -> ParallelKey.forShaft(100, 10), "No DIN 6885-1 key fits shaft diameter 100");

		var shaft = new SteppedShaft(
			[{diameter: 8, length: 51.5}, {diameter: 6, length: 8.5}],
			[{name: "bearingA", z: 10}, {name: "bearingB", z: 43}],
			[{name: "outputKey", z0: 52, key: key}],
			[{name: "ring", z0: 50, width: 1.2, diameter: 7.4}]
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
		check(preview.volume() < Math.PI * 16 * 51.5 + Math.PI * 9 * 8.5, "preview removes keyway and groove material");
		preview.close();

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
	}

	static function shaftHardware():Void {
		var ring = RetainingRing.forShaft(8);
		check(ring.designation == "DIN471-8", "ring designation");
		check(ring.spec.grooveDiameter == 7.4 && ring.spec.outerDiameter == 12.2, "ring dimensions");
		throws(() -> RetainingRing.forShaft(9), 'Unknown retaining ring shaft diameter "9"');

		var envelope = ring.geometry(Envelope);
		solid(envelope, "ring envelope");
		near(envelope.volume(), annulus(12.2, 7.4, 0.8), "ring envelope volume");
		envelope.close();
		var preview = ring.geometry();
		solid(preview, "ring preview");
		near(preview.volume(), annulus(12.2, 7.4, 0.8) * 8 / 9, "ring preview volume (gapped)");
		preview.close();
		near(ring.connector("seat").frame.z, 0.4, "ring seat connector");

		var collar = ShaftCollar.forShaft(8);
		check(collar.designation == "COLLAR-8", "collar designation");
		check(collar.spec.setScrew == "M4", "collar set screw size");
		throws(() -> ShaftCollar.forShaft(9), 'Unknown shaft collar bore diameter "9"');
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
		tslotPart.close();

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
		throws(() -> new SpurGear(2, 20, -1), "positive face width");

		var part = gear.geometry();
		solid(part, "spur gear");
		var box = bounds(part);
		near(box.maxX, gear.outsideDiameter / 2, "spur gear outside radius");
		check(box.maxX > gear.pitchDiameter / 2, "spur gear teeth extend past the pitch circle");
		part.close();

		var pinion = new SpurGear(2, 12, 12);
		throws(() -> pinion.centerDistance(new SpurGear(2.5, 20, 12)), "share a module");
		var pair = GearPair.mesh(pinion, gear);
		near(pair.centerDistance, (pinion.pitchDiameter + gear.pitchDiameter) / 2, "gear pair centre distance");
		near(pair.ratio(), gear.teeth / pinion.teeth, "gear pair ratio");
		near(pair.pose().x, pair.centerDistance, "gear pair pose offset");

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
		rackPart.close();
	}

	static function pillowBlock():Void {
		var bearing = DeepGrooveBearing.metric("6204");
		var block = new PillowBlock(bearing);
		check(block.housing.mountScrew == "M6", "pillow block mount screw size");
		near(block.housing.face, 65.8, "pillow block face");
		near(block.housing.depth, 23.4, "pillow block depth");
		near(block.housing.boltSpacing, 56.4, "pillow block bolt spacing");

		var envelope = block.housing.geometry(Envelope);
		solid(envelope, "pillow block envelope");
		var envelopeVolume = envelope.volume();
		near(envelopeVolume, 65.8 * 65.8 * 23.4 - Math.PI * 23.525 * 23.525 * 23.4, "pillow block envelope volume");
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
		near(state.worldConnector("pb-bearing", "axis").z, 18.7, "bearing centred in housing, offset by half its width");

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
		check(axis.motor.designation == "NEMA23-56", "linear axis motor designation");
		near(axis.length, 240, "linear axis screw length");
		near(axis.screw.totalLength, 240, "linear axis screw total length");
		near(axis.carriage.boreDiameter, 10, "linear axis carriage bore");
		throws(() -> new LinearAxis(23, 10, -1), "positive stroke");
		throws(() -> new LinearAxis(23, 50, 200, "6001"), "bore is smaller than the screw diameter");

		for (entry in axis.components()) {
			var part = entry.component.geometry(Envelope);
			check(part.valid(), '${entry.id} envelope is invalid');
			part.close();
		}
		var rail = axis.frame.geometry("rail");
		solid(rail, "linear axis rail");
		rail.close();
		var cutList = axis.frame.cutList();
		check(cutList.length == 1, "linear axis frame cut list");
		near(cutList[0].totalLength, 240, "linear axis rail length");

		var model = axis.assembly();
		var definition = model.definition("linear-axis");
		check(definition.joints.length == 12, "linear axis joint count");
		var state = model.initialState("linear-axis");
		near(state.worldConnector("screw", "input").z, 21, "screw seats on the motor shaft");
		near(state.worldConnector("carriage", "bore").z, 71, "carriage starts clear of the screw ends");
		near(state.worldConnector("pillowA-bearing", "axis").z, 14, "pillow block A bearing centred");
		near(state.worldConnector("pillowB-bearing", "axis").z, 234, "pillow block B bearing centred");

		state.setJoint("carriage-slide", 100);
		state.forwardKinematics();
		near(state.worldConnector("carriage", "bore").z, 121, "carriage travels along the screw");

		var lines = axis.bom().lines();
		check(lines.length == 6, "linear axis BOM line count");
		check(axis.bom().quantity(axis.bearing.designation) == 2, "linear axis bearing quantity");
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
		throws(() -> LinearBearing.metric("LM9UU"), 'Unknown linear bearing "LM9UU"');
		var linearBearingPart = linearBearing.geometry();
		solid(linearBearingPart, "linear bearing");
		near(linearBearingPart.volume(), Math.PI * (7.5 * 7.5 - 4 * 4) * 24, "linear bearing volume");
		linearBearingPart.close();

		var sprocket = new Sprocket(12.7, 20, 8, 6);
		check(sprocket.designation == "SPROCKET-P12.7-20T", "sprocket designation");
		near(sprocket.pitchDiameter, 12.7 / Math.sin(Math.PI / 20), "sprocket pitch diameter");
		throws(() -> new Sprocket(12.7, 5, 8, 6), "at least 8 teeth");
		throws(() -> new Sprocket(12.7, 8, 40, 6), "must clear the bore");
		var sprocketPart = sprocket.geometry();
		solid(sprocketPart, "sprocket");
		var sprocketBox = bounds(sprocketPart);
		check(sprocketBox.maxX <= sprocket.outsideDiameter / 2 + 1e-6, "sprocket stays within its outside radius");
		check(sprocketBox.maxX > sprocket.pitchDiameter / 2, "sprocket teeth extend past the pitch circle");
		sprocketPart.close();

		var pulley = new TimingPulley(2, 20, 5, 6);
		check(pulley.designation == "PULLEY-P2-20T", "timing pulley designation");
		near(pulley.pitchDiameter, 2 * 20 / Math.PI, "timing pulley pitch diameter");
		throws(() -> new TimingPulley(2, 5, 5, 6), "at least 8 teeth");
		throws(() -> new TimingPulley(2, 8, 11, 6), "must clear the bore");
		var pulleyPart = pulley.geometry();
		solid(pulleyPart, "timing pulley");
		var pulleyBox = bounds(pulleyPart);
		check(pulleyBox.maxX <= pulley.outsideDiameter / 2 + 1e-6, "timing pulley stays within its outside radius");
		check(pulleyBox.maxX > pulley.grooveDiameter / 2, "timing pulley lands extend past the groove circle");
		pulleyPart.close();
	}

	static function robotics():Void {
		var flange = new RobotFlange(50);
		check(flange.designation == "ISO9409-50", "robot flange designation");
		near(flange.thickness, 8, "robot flange thickness");
		near(flange.pilotDiameter, 25, "robot flange pilot diameter");
		near(flange.boltCircleDiameter, 39, "robot flange bolt circle");
		check(flange.mountScrew == "M5", "robot flange mount screw size");
		throws(() -> new RobotFlange(-1), "positive diameter");
		throws(() -> new RobotFlange(50, 2), "at least 3 bolts");

		var envelope = flange.geometry(Envelope);
		solid(envelope, "robot flange envelope");
		var envelopeVolume = envelope.volume();
		near(envelopeVolume, Math.PI * 25 * 25 * 8 + Math.PI * 12.5 * 12.5 * 3, "robot flange envelope volume");
		envelope.close();
		var preview = flange.geometry();
		solid(preview, "robot flange preview");
		check(preview.volume() < envelopeVolume, "robot flange preview removes bolt and pin holes");
		preview.close();
		near(flange.connector("bolt1").frame.x, 19.5, "robot flange bolt1 x");
		near(flange.connector("bolt2").frame.y, 19.5, "robot flange bolt2 y");

		var eoat = new EndEffectorPlate(flange);
		check(eoat.designation == "EOAT-50-8", "end effector plate designation");
		near(eoat.thickness, flange.thickness, "end effector plate default thickness");
		var eoatEnvelope = eoat.geometry(Envelope);
		solid(eoatEnvelope, "end effector plate envelope");
		near(eoatEnvelope.volume(), Math.PI * 25 * 25 * eoat.thickness, "end effector plate envelope volume");
		eoatEnvelope.close();
		var eoatPreview = eoat.geometry();
		solid(eoatPreview, "end effector plate preview");
		eoatPreview.close();
		throws(() -> new EndEffectorPlate(flange, 1), "must be thicker than the flange's pilot boss");
		near(eoat.connector("tool").frame.z, eoat.thickness, "end effector plate tool connector");

		var pedestal = new Pedestal(flange, 300);
		check(pedestal.designation == "PEDESTAL-50-300", "pedestal designation");
		near(pedestal.baseDiameter, 80, "pedestal base diameter");
		near(pedestal.columnDiameter, 50, "pedestal column diameter");
		throws(() -> new Pedestal(flange, 5), "must clear the flange's pilot boss");
		var pedestalEnvelope = pedestal.geometry(Envelope);
		solid(pedestalEnvelope, "pedestal envelope");
		pedestalEnvelope.close();
		var pedestalPreview = pedestal.geometry();
		solid(pedestalPreview, "pedestal preview");
		pedestalPreview.close();
		near(pedestal.connector("top").frame.z, 300, "pedestal top connector");
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
		check(bom.quantity("NEMA17-48") == 1, "motor quantity");
		check(bom.quantity("DIN6885-2x2x6") == 1, "key quantity");
		check(bom.quantity("DIN471-8") == 1, "ring quantity");
		var duplicate = new Bom();
		duplicate.add({partNumber: "X", description: "a", quantity: 1, material: null});
		throws(() -> duplicate.add({partNumber: "X", description: "b", quantity: 1, material: null}), "conflicting");
	}

	static function main():Void {
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
