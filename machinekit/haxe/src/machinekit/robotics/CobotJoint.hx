package machinekit.robotics;

import machinekit.motion.Gearbox;
import machinekit.motion.ServoMotor;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;

/** An assumed strain-wave cobot module: housing, gearhead and
 * an integrated servo. The geometric pocket removes the installed gearhead's
 * volume from the housing. Output ratings describe the generic size family.
 */
class CobotJoint extends GearedArmJoint implements machinekit.motion.EncoderPart {
	public static inline var RATIO:Float = 100;
	public static inline var EFFICIENCY:Float = 0.85;
	public final size:Int;
	public final inputPilotDepth:Float;
	public final gearbox:Gearbox;
	public final outputRatedTorque:Float;
	public final outputPeakTorque:Float;
	static var recipe:Null<ComponentType>;

	public function new(size:Int, speedDegrees:Float = 180, inputPilotDepth:Float = 0) {
		if (size < 0 || size > 4 || !(speedDegrees > 0) || !Math.isFinite(speedDegrees) ||
			inputPilotDepth < 0 || !Math.isFinite(inputPilotDepth))
			throw "Cobot module needs size 0–4 and a positive joint speed";
		var diameters = [75.0, 96, 110, 140, 150];
		var lengths = [30.0, 40, 50, 70, 65];
		var pockets = [40.0, 60, 70, 100, 110];
		var pocketLengths = [25.0, 30, 40, 50, 55];
		var torques = [12.0, 28, 56, 150, 330];
		var rate = speedDegrees * Math.PI / 180 * RATIO;
		var torque = torques[size] / (RATIO * EFFICIENCY);
		var servo = new ServoMotor({designation: 'COBOT-SERVO-$size-${speedDegrees}-P${inputPilotDepth}',
			ratedTorque: torque, peakTorque: 2 * torque, ratedSpeed: 0.8 * rate, maxSpeed: rate,
			rotorInertia: (size + 1) * 1e-5, encoderCounts: 131072,
			bodyDiameter: pockets[size] - 2, bodyLength: pocketLengths[size] - 2}, Assumed);
		super(diameters[size], lengths[size], pockets[size], pocketLengths[size], null, servo);
		setMaterial(size == 2 || size == 4 ? "steel" : "aluminium 6061");
		this.size = size;
		this.inputPilotDepth = inputPilotDepth;
		addConnector("gearheadSeat", Mount, machinekit.component.Solids.axial(0, 0, 0.5 + inputPilotDepth));
		outputRatedTorque = torques[size];
		outputPeakTorque = 2 * outputRatedTorque;
		gearbox = new Gearbox(RATIO, EFFICIENCY, pocketDiameter - 1, pocketLength - 1 - inputPilotDepth, 8, true);
		gearbox.setMaterial(size == 2 ? "aluminium 6061" : "steel");
	}

	/** Explicit physical drive gains select CSP interpolation, including on an
	 * uncoupled joint. Assumption: peak torque at 0.01 motor radians of error,
	 * with critical damping for the stated rotor inertia. Reflected load inertia
	 * and the drive loop rate determine integration when the arm is compiled.
	 */
	override public function actuator(id:String, joint:String, volts:Float, margin:Float,
			?current:Float):materia.assembly.AssemblyDefinition.AssemblyActuator {
		var result = super.actuator(id, joint, volts, margin, current);
		var motor:ServoMotor = cast servo;
		var stiffness = motor.rating.peakTorque / 0.01;
		result.servoStiffness = stiffness;
		result.servoDamping = 2 * Math.sqrt(stiffness * motor.rating.rotorInertia);
		var assumed:Array<String> = result.assumed == null ? [] : [for (label in result.assumed) label];
		assumed.push("servo position-loop gains");
		result.assumed = assumed;
		return result;
	}

	/** Assumed absolute encoder on the joint output, independent of the motor feedback. */
	public function encoder(id:String, joint:String):materia.assembly.AssemblyDefinition.AssemblyEncoder
		return {id: id, joint: joint, kind: "absolute", counts: 131072};

	public static function moduleRecipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.robotics.cobot-joint", [
			ComponentRecipeSupport.scalar("size", 2), ComponentRecipeSupport.scalar("speedDegrees", 180),
			ComponentRecipeSupport.length("inputPilotDepth", 0)
		], v -> new CobotJoint(Std.int(v.number("size")), v.number("speedDegrees"), v.number("inputPilotDepth")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return moduleRecipeType();
	override public function values():ComponentValues {
		var motor:ServoMotor = cast servo;
		return new ComponentValues().setNumber("size", size)
			.setNumber("speedDegrees", motor.rating.maxSpeed / RATIO * 180 / Math.PI)
			.setNumber("inputPilotDepth", inputPilotDepth)
			.setToken("material", materialSpec());
	}
}
