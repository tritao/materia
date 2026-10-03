package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyDefinition.AssemblyActuator;

/** What a servo motor can deliver, in SI units: torques in N m, speeds in rad/s, inertia in kg m². */
typedef ServoRating = {
	var designation:String;
	var ratedTorque:Float;
	var peakTorque:Float;
	var ratedSpeed:Float;
	var maxSpeed:Float;
	var rotorInertia:Float;
	/** Encoder counts per revolution. */
	var encoderCounts:Float;
	/** Round body, mm. */
	var bodyDiameter:Float;
	var bodyLength:Float;
}

/**
 * A servo motor: rated and peak torque, rated and maximum speed, rotor inertia and the encoder counts
 * of its own feedback. Its actuator is a servo drive, so a planner relies on its peak torque and
 * maximum speed and its rated torque bounds the average (see `PlanCheck`). Usually it drives a joint
 * through a `Gearbox`.
 *
 * The named ratings are generic: round numbers in the range of the 3000 rpm AC servo families made at 50,
 * 100, 200, 400 and 750 W (rated torque is power over 3000 rpm, peak three times that, 5000 rpm at most, a little
 * under 20 bits' worth of encoder). They are assumptions, not a vendor's datasheet.
 * CAD frame: mounting face at z=0, body toward -Z, shaft along +Z. Connectors: `mountFace`, `shaftAxis`.
 */
class ServoMotor extends MachineComponent implements MotorDrive {
	static var table:Null<Array<ServoRating>>;

	public final rating:ServoRating;
	public final ratingsBasis:machinekit.transmission.ValueBasis;

	public static function ratings():Array<ServoRating> {
		if (table == null) {
			var rpm = 2.0 * Math.PI / 60.0;
			function generic(name:String, watts:Float, inertia:Float, face:Float, length:Float):ServoRating {
				var rated = watts / (3000.0 * rpm);
				return {designation: name, ratedTorque: rated, peakTorque: 3.0 * rated, ratedSpeed: 3000.0 * rpm, maxSpeed: 5000.0 * rpm,
					rotorInertia: inertia, encoderCounts: 131072, bodyDiameter: face, bodyLength: length};
			}
			table = [
				generic("GENERIC-SERVO-50W", 50, 3.0e-6, 40, 70),
				generic("GENERIC-SERVO-100W", 100, 5.0e-6, 40, 85),
				generic("GENERIC-SERVO-200W", 200, 2.5e-5, 60, 100),
				generic("GENERIC-SERVO-400W", 400, 5.0e-5, 60, 130),
				generic("GENERIC-SERVO-750W", 750, 1.2e-4, 80, 145)
			];
		}
		return table;
	}

	static var namedRecipe:Null<machinekit.component.ComponentType>;
	public static function namedRecipeType():machinekit.component.ComponentType {
		if (namedRecipe == null) namedRecipe = new machinekit.component.ComponentType("machinekit.motion.servo-motor",
			[machinekit.component.ComponentRecipeSupport.choice("designation", [for (row in ratings()) row.designation],
				"GENERIC-SERVO-200W")], v -> model(v.token("designation")));
		return namedRecipe;
	}

	/** The generic servo named `designation`. */
	public static function model(designation:String):ServoMotor {
		for (rating in ratings()) if (rating.designation == designation) return new CatalogServoMotor(rating);
		throw 'Unknown servo motor "$designation"; known: ${[for (rating in ratings()) rating.designation].join(", ")}';
	}

	public function new(rating:ServoRating, ratingsBasis:machinekit.transmission.ValueBasis = Stated) {
		if (!(rating.ratedTorque > 0) || !(rating.peakTorque >= rating.ratedTorque) || !(rating.ratedSpeed > 0) ||
			!(rating.maxSpeed >= rating.ratedSpeed) || !(rating.rotorInertia >= 0) || !(rating.encoderCounts >= 0))
			throw "A servo needs 0 < rated torque <= peak torque, 0 < rated speed <= maximum speed and finite ratings";
		super(rating.designation, 'Servo motor, ${Dimension.format(rating.peakTorque)} N m peak', "aluminium 6061", true);
		this.rating = rating;
		this.ratingsBasis = ratingsBasis;
		addConnector("mountFace", Mount, Solids.axial(0, 0, 0));
		addConnector("shaftAxis", Axis, Solids.axial(0, 0, 0));
	}

	/** The actuator: a servo drive with its peak torque and maximum speed as the limits a planner may rely on. */
	public function actuator(id:String, joint:String, volts:Float, margin:Float, ?current:Float):AssemblyActuator
		return {id: id, joint: joint, maxEffort: rating.peakTorque, maxRate: rating.maxSpeed, rotorInertia: rating.rotorInertia, drive: "servo",
			ratedTorque: rating.ratedTorque, peakTorque: rating.peakTorque, ratedSpeed: rating.ratedSpeed, maxSpeed: rating.maxSpeed,
			encoderCounts: rating.encoderCounts,
			torqueSpeed: [0, rating.peakTorque, rating.ratedSpeed, rating.peakTorque, rating.maxSpeed, rating.ratedTorque],
			assumed: ratingsBasis == Assumed ? ["servo ratings"] : null};

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.cylinderSpan(rating.bodyDiameter / 2, -rating.bodyLength, 0), "body");
}

/** A catalogue servo keeps its source designation in saved assemblies. Explicit rating objects
 * remain code-only, so reconstruction never silently replaces their stated or edited ratings.
 */
private class CatalogServoMotor extends ServoMotor {
	public function new(rating:ServoRating) super(rating, Assumed);
	override public function componentType():Null<machinekit.component.ComponentType> return ServoMotor.namedRecipeType();
	override public function values():machinekit.component.ComponentValues return new machinekit.component.ComponentValues()
		.setToken("designation", rating.designation).setToken("material", materialSpec());
}
