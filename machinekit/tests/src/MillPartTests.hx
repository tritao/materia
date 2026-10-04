import machinekit.motion.BallScrew;
import machinekit.motion.BallNut;
import machinekit.motion.LeadScrew;
import machinekit.motion.ServoMotor;

/** Mill drive parts use the existing transmission resolver and survive recipe reconstruction. */
class MillPartTests {
	static function near(actual:Float, expected:Float, label:String):Void {
		if (!(Math.abs(actual - expected) < 1e-8)) throw '$label: $actual != $expected';
	}
	public static function run():Void {
		var screw = BallScrew.sfu1605(400), nut = new BallNut();
		var relation = LeadScrew.relation(screw, nut, 1);
		near(relation.ratio, -2 * Math.PI / 5, "SFU1605 radians per millimetre");
		near(relation.efficiency, 0.9, "Ball nut efficiency");
		near(nut.backlash(), 0, "Preloaded reversal clearance");
		near(new BallNut(false).backlash(), 0.01, "Unpreloaded reference clearance");
		near(nut.rotationFor(-5), 2 * Math.PI, "Ball nut signed travel");
		near(screw.axialStiffness(), 200000 * Math.PI * 13 * 13 / 1600, "Shaft axial stiffness");
		near(new BallScrew(200).axialStiffness(), 2 * screw.axialStiffness(), "Shorter shaft stiffness");
		var rebuilt:BallScrew = cast screw.componentType().create(screw.values());
		near(rebuilt.criticalSpeed(Fixed, Simple), screw.criticalSpeed(Fixed, Simple), "Rebuilt support speed");
		var rebuiltNut:BallNut = cast nut.componentType().create(nut.values());
		near(rebuiltNut.connector("mountFace").frame.z, 45, "Catalogue flange face");
		near(rebuiltNut.flangeDiameter, 48, "Rebuilt flange diameter");
		var fixed = machinekit.motion.ScrewSupportUnit.bk12(), floating = machinekit.motion.ScrewSupportUnit.bf12();
		if (fixed.support != Fixed || floating.support != Simple) throw "BK/BF support conditions";
		var rebuiltSupport:machinekit.motion.ScrewSupportUnit = cast floating.componentType().create(floating.values());
		if (rebuiltSupport.support != Simple) throw "Support recipe must retain floating bearing";
		near(fixed.connector("mount").frame.y, -25, "Support foot frame");
		var rail = machinekit.motion.LinearRailSystem.forProfile("HGR15", 400, 2);
		near(rail.stroke, 400 - 40 - 61.4, "Rail room defines travel");
		near(rail.blocks[0].connector("mount1").frame.y, 13, "HGR15 block mounting plane");
		var verified = machinekit.motion.LinearRailSystem.catalog().metadata("HGR15").verifiedFields;
		if (verified == null || verified.length != 0)
			throw "Assumed rail dimensions must not claim vendor verification";
		var spindle = new machinekit.milling.SpindleCartridge();
		near(spindle.connector("gaugeLine").frame.z, 0, "Spindle gauge line");
		near(spindle.connector("pulley").frame.z, machinekit.milling.SpindleCartridge.LENGTH, "Spindle pulley seat");
		var spindleMotor = new machinekit.milling.SpindleMotor();
		near(spindleMotor.rating.ratedTorque * spindleMotor.rating.ratedSpeed, 1100, "Spindle power");
		for (kind in [machinekit.milling.MillCasting.MillCastingKind.Base,
			machinekit.milling.MillCasting.MillCastingKind.Column, machinekit.milling.MillCasting.MillCastingKind.Saddle,
			machinekit.milling.MillCasting.MillCastingKind.Table, machinekit.milling.MillCasting.MillCastingKind.Head]) {
			var casting = new machinekit.milling.MillCasting(kind);
			var mass = casting.massProperties();
			if (!(mass.mass > 0) || mass.inertia == null) throw "Mill castings need computed mass and inertia";
			near(casting.connector("top").frame.z, casting.height, "Casting top frame");
			var rebuiltCasting:machinekit.milling.MillCasting = cast casting.componentType().create(casting.values());
			near(rebuiltCasting.massProperties().mass, mass.mass, "Rebuilt casting mass");
			Sys.println('mill casting ${kind}: ${mass.mass} kg');
		}
		for (watts in [400, 750]) {
			var motor = ServoMotor.model('GENERIC-SERVO-${watts}W');
			near(motor.rating.ratedTorque * motor.rating.ratedSpeed, watts, "Servo rated power");
			var restoredMotor:ServoMotor = cast motor.componentType().create(motor.values());
			near(restoredMotor.rating.ratedTorque, motor.rating.ratedTorque, "Rebuilt servo rated torque");
			if (motor.actuator("motor", "axis", 48, 0.5).assumed == null) throw "Generic servo must declare assumed ratings";
		}
	}
}
