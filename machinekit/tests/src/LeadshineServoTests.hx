import machinekit.motion.LeadshineServo;
import machinekit.motion.ServoMotor;
import machinekit.component.MachineKitComponents;
import machinekit.assembly.MachineAssembly;

class LeadshineServoTests {
	static function near(a:Float, b:Float, message:String):Void {
		if (!Math.isFinite(a) || Math.abs(a-b) > 1e-7) throw '$message: $a != $b';
	}
	public static function main():Void {
		var names = LeadshineServo.catalog().designations();
		if (names.length != 55) throw "Expected 54 PDF variants and one explicit MS1AE variant";
		MachineKitComponents.standard();
		for (name in names) {
			var motor = LeadshineServo.model(name), spec = motor.spec;
			if (motor.ratingsBasis != Catalog || motor.electricalRating() != null) throw "Catalog mechanics must not invent winding constants";
			near(motor.ratedPhaseCurrent(), spec.ratedCurrent, "Catalog phase current");
			near(motor.massProperties().mass + motor.shaft().massProperties().mass, spec.mass, "Published total mass");
			near(motor.rating.encoderCounts, 8388608, "23-bit encoder");
			for (p in motor.boltPattern()) near(Math.sqrt(p.x*p.x+p.y*p.y), spec.boltCircle/2, "Bolt pitch radius");
			var preview = motor.geometry(), envelope = motor.geometry(Envelope), shaft = motor.shaft().geometry();
			if (preview.solidCount() != 1 || !(preview.volume() > 0) || !(shaft.volume() > 0)) throw 'Invalid motor solid $name';
			if (preview.volume() >= envelope.volume()) throw "Preview must fit inside envelope";
			var bounds = preview.shape.bounds();
			near(bounds.get_min().get_z(), -spec.bodyLength, "Housing rear");
			near(bounds.get_max().get_z(), spec.pilotHeight, "Pilot front; shaft is separate");
			near(bounds.get_max().get_y(), spec.connectorHeight, "Connector LH maximum");
			var assembly = new MachineAssembly(); assembly.addComponent("motor", motor);
			var restored = MachineAssembly.decode(assembly.encode());
			var saved:LeadshineServo = cast restored.components()[0].component;
			if (saved.spec.designation != name || saved.spec.brake != spec.brake) throw "Assembly lost vendor variant";
			// Recipe reconstruction also checks generated solid and stable designation.
			var rebuilt:LeadshineServo = cast motor.componentType().create(motor.values());
			var rebuiltPart = rebuilt.geometry();
			near(rebuiltPart.volume(), preview.volume(), "Reconstructed geometry");
			if (rebuilt.designation != name || ServoMotor.model(name).designation != name) throw "Lost catalog designation";
			var cutout = motor.mountingCutout(10);
			var expected = Math.PI*10*(Math.pow((spec.pilotDiameter+0.2)/2,2)+4*Math.pow(spec.holeDiameter/2,2));
			near(cutout.volume(), expected, "Mounting plate cutout volume");
			preview.close(); envelope.close(); shaft.close(); rebuiltPart.close(); cutout.close();
		}
		var motor = LeadshineServo.model("ELM1H-0400MA60F");
		near(motor.spec.bodyLength, 88.8, "400 W unbraked length");
		near(motor.rating.ratedTorque, 1.27, "400 W rated torque");
		near(motor.rating.rotorInertia, 5.6e-5, "400 W rotor inertia SI");
		var brake = LeadshineServo.model("ELM2H-0400LA60E");
		near(brake.spec.bodyLength, 118.1, "400 W braked length");
		near(brake.rating.rotorInertia, 5.8e-5, "Braked rotor inertia SI");
		var large = LeadshineServo.model("ELM2M-7500LD180ET-H");
		near(large.spec.bodyLength, 345, "7.5 kW braked length");
		near(large.spec.voltage, 380, "7.5 kW voltage");
		near(large.rating.ratedTorque, 48, "7.5 kW rated torque");
		near(large.rating.peakTorque, 119, "7.5 kW peak torque");
		near(large.rating.ratedSpeed, 50*Math.PI, "Large motor 1500 rpm in rad/s");
		near(large.rating.rotorInertia, 0.014, "Large braked rotor inertia SI");
		near(large.spec.shaftDiameter, 42, "7.5 kW shaft diameter");
		near(large.spec.keyHeight, 8, "Large key height differs from width");
		var medium = LeadshineServo.model("ELM2M-2000LB130E-H");
		near(medium.spec.face, 131, "130-class actual face dimension");
		near(medium.rating.maxSpeed, 2800*Math.PI/30, "Medium inertia max speed");
		near(LeadshineServo.model("ELM2H-1800LD130F-HD").spec.maxRpm, 2000, "220 V high inertia max rpm");
		near(LeadshineServo.model("ELM2H-1800LD130FT-H").spec.maxRpm, 3000, "380 V high inertia max rpm");
		near(LeadshineServo.model("ELM2L-1000LA100FT-H").spec.maxRpm, 6000, "Low inertia max rpm");
		var variant = LeadshineServo.model("ELM2L-1500LA100FT-H MS1AE");
		near(variant.spec.voltage,400,"MS1AE page voltage");
		near(variant.ratedPhaseCurrent(),4.5,"MS1AE page current");
		if (LeadshineServo.model("ELM2L-1500LA100FT-H").studyContinuousTorque(3000) != null)
			throw "Curve must not leak from MS1AE into the older PDF row";
		var curve = machinekit.motion.LeadshineTorqueCurves.studyCurve(variant.designation);
		if (curve == null || curve.operatingConditionsVerified) throw "Variant graph match is not qualification";
		var baseTorque:Float = variant.studyContinuousTorque(3000) ?? 0;
		var topTorque:Float = variant.studyContinuousTorque(5000) ?? 0;
		near(baseTorque,4.66871764,"Continuous ordinate includes vertical and horizontal reading allowances");
		near(topTorque,1.95930564,"Digitized continuous endpoint with reading allowances");
		if (baseTorque <= topTorque || topTorque <= 0) throw "Continuous torque must fall above base";
		if (variant.studyContinuousTorque(5001) != null || variant.studyContinuousTorque(-1) != null) throw "Curve must not extrapolate";
		if (machinekit.motion.LeadshineTorqueCurves.qualifiedContinuousTorque(variant.designation,3000) != null)
			throw "Unknown operating conditions must prevent qualification";
		for (name in ["ELM2M-2900LD180FT-H","ELM2M-7500LD180FT-H"])
			if (LeadshineServo.model(name).studyContinuousTorque(1500) != null) throw "Conflicting graphs must remain blocked";
		for (args in [{t:0.0,c:0.2},{t:10.0,c:-1.0},{t:Math.NaN,c:0.2}]) {
			var rejected = false;
			try motor.mountingCutout(args.t,args.c) catch (_:Dynamic) rejected = true;
			if (!rejected) throw "Invalid cutout accepted";
		}
		Sys.println('Leadshine: all ${names.length} variants, geometry, interfaces, ratings and recipes passed');
	}
}
