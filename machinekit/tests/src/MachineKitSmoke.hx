import cadkit.modeling.Part;
import machinekit.component.Bom;
import machinekit.component.ComponentDetail;
import machinekit.motion.NemaStepper;
import machinekit.motion.SteppedShaft;
import machinekit.standard.ClearanceFit;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.FlatWasher;
import machinekit.standard.HexBolt;
import machinekit.standard.HexNut;
import machinekit.standard.ParallelKey;
import machinekit.standard.SocketHeadCapScrew;
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
			[{z0: 50, width: 1.2, diameter: 7.4}]
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
		check(definition.joints.length == 9, "assembly joint count");
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
		check(lines.length == 6, "BOM line count");
		var bom = example.bom();
		check(bom.quantity("ISO4762-M3x10") == 4, "screw quantity");
		check(bom.quantity("608-2Z") == 2, "bearing quantity");
		check(bom.quantity("NEMA17-48") == 1, "motor quantity");
		check(bom.quantity("DIN6885-2x2x6") == 1, "key quantity");
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
		assembly();
		trace("MachineKit smoke passed");
	}
}
