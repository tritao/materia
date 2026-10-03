package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.MotorDrive;
import machinekit.motion.ServoMotor;
import materia.assembly.AssemblyDefinition.AssemblyActuator;

/** Joint module of a serial arm: a cylindrical housing along local +Z with a fixed `stator` face at
 * z=0 and the rotating output `rotor` face at z=length. The housing belongs to the link before the
 * joint; the next link mates its `start` connector to `rotor` on a revolute joint.
 *
 * A module given a `flange` is the last joint of the arm. Its output carries the `RobotFlange`
 * plate against the housing end, so it has a `tool` connector at the flange's mounting face
 * instead of `rotor`: mate the flange's `face` to it, and a tool mates to the flange's pilot boss.
 * The recipe names the flange by its ISO 9409-1 pitch circle; 0 means no flange.
 *
 * A module may carry a `servo`, the motor inside it: it is then a `MotorDrive`, and
 * `MachineAssembly.addMotor` puts its servo on the joint it turns, usually through a `Gearbox`.
 */
class ArmJoint extends MachineComponent implements MotorDrive {
	public final diameter:Float;
	public final length:Float;
	public final flange:Null<RobotFlange>;
	/** The servo motor inside the module, or null when it is only a housing. */
	public final servo:Null<ServoMotor>;

	public function new(diameter:Float, length:Float, ?flange:RobotFlange, ?servo:ServoMotor) {
		if (!Math.isFinite(diameter) || !Math.isFinite(length) || !(diameter > 0) || !(length > 0))
			throw "Arm joint needs a positive diameter and length";
		var text = '${Dimension.format(diameter)}x${Dimension.format(length)}';
		var base = flange == null ? 'ARM-JOINT-D$text' : 'ARM-JOINT-D$text-${flange.designation}';
		super(servo == null ? base : '$base-${servo.designation}', 'Arm joint module, $text mm' +
			(servo == null ? "" : ' with ${servo.designation}'), "steel 12.9", true);
		this.servo = servo;
		this.diameter = diameter;
		this.length = length;
		this.flange = flange;
		addConnector("stator", Mount, Solids.axial(0, 0, 0));
		if (flange == null) {
			addConnector("rotor", Mount, Solids.axial(0, 0, length));
		} else {
			if (!(diameter >= flange.flangeDiameter + 2)) throw "Arm joint is too narrow for its tool flange";
			addConnector("tool", Mount, flange.pinAlignedFrame(length + flange.thickness));
		}
	}

	public static function recipeType():ComponentType
		return machinekit.component.MachineKitAdditionalRecipes.byId("machinekit.robotics.arm-joint");

	/** Subclasses must declare their own recipe and saved values. */
	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, ArmJoint) ? recipeType() : null;

	override public function values():ComponentValues
		return new ComponentValues().setNumber("diameter", diameter).setNumber("length", length)
			.setNumber("flangePitchCircle", flange == null ? 0 : flange.spec.pitchCircle)
			.setToken("servo", servo == null ? "none" : servo.designation)
			.setToken("material", materialSpec());

	/** The servo inside the module as an actuator on joint `joint`; a module with no servo is not a motor. */
	public function actuator(id:String, joint:String, volts:Float, margin:Float):AssemblyActuator {
		var motor = servo;
		if (motor == null) throw 'Arm joint "$designation" has no servo to drive a joint';
		return motor.actuator(id, joint, volts, margin);
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.cylinderSpan(diameter / 2, 0, length);
}
