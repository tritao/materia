package machinekit.milling;

import machinekit.motion.ServoMotor;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;

/** Generic 1.1 kW spindle motor rated at 8000 rpm, limited to 10000 rpm. Ratings, inertia and envelope are assumed.
 * Its drive curve is explicit, so the spindle can use the standard motor/driver binding.
 */
class SpindleMotor extends ServoMotor {
	public static inline var POWER:Float = 1100;
	public function new() {
		var speed = SpindleCartridge.MAX_RPM * 2 * Math.PI / 60;
		var torque = POWER / (speed * 0.8);
		super({designation: "GENERIC-SPINDLE-1100W", ratedTorque: torque, peakTorque: 2 * torque,
			ratedSpeed: speed * 0.8, maxSpeed: speed, rotorInertia: 0.001, encoderCounts: 4096,
			bodyDiameter: 90, bodyLength: 160}, Assumed);
	}
	static var spindleMotorRecipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (spindleMotorRecipe == null) spindleMotorRecipe = new ComponentType("machinekit.milling.spindle-motor", [],
			v -> new SpindleMotor());
		return spindleMotorRecipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setToken("material", materialSpec());
}
