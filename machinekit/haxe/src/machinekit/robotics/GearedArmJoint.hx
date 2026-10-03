package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Solids;
import machinekit.motion.ServoMotor;

/** An arm housing machined for a separate gearbox member. The pocket is a physical dimension;
 * its clearance does not contribute mass a second time when the gearhead is installed.
 */
class GearedArmJoint extends ArmJoint {
	static var recipe:Null<ComponentType>;
	public final pocketDiameter:Float;
	public final pocketLength:Float;

	public function new(diameter:Float, length:Float, pocketDiameter:Float, pocketLength:Float,
			?flange:RobotFlange, ?servo:ServoMotor) {
		if (!Math.isFinite(pocketDiameter) || !Math.isFinite(pocketLength) || !(pocketDiameter > 0) ||
			!(pocketDiameter < diameter) || !(pocketLength > 1) || !(pocketLength < length))
			throw "A geared arm joint needs a pocket inside its housing";
		super(diameter, length, flange, servo, "-POCKET-" + machinekit.component.Dimension.format(pocketDiameter) +
			"x" + machinekit.component.Dimension.format(pocketLength));
		this.pocketDiameter = pocketDiameter;
		this.pocketLength = pocketLength;
		addConnector("gearbox", Mount, Solids.axial(0, 0, 0.5));
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.robotics.geared-arm-joint", [
			ComponentRecipeSupport.length("diameter", 100), ComponentRecipeSupport.length("length", 70),
			ComponentRecipeSupport.length("pocketDiameter", 61), ComponentRecipeSupport.length("pocketLength", 36),
			ComponentRecipeSupport.scalar("flangePitchCircle", 0),
			ComponentRecipeSupport.choice("servo", ["none"].concat([for (rating in ServoMotor.ratings()) rating.designation]), "GENERIC-SERVO-200W")
		], v -> new GearedArmJoint(v.number("diameter"), v.number("length"), v.number("pocketDiameter"),
			v.number("pocketLength"), v.number("flangePitchCircle") == 0 ? null : new RobotFlange(v.number("flangePitchCircle")),
			v.token("servo") == "none" ? null : ServoMotor.model(v.token("servo"))));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return Std.isExactType(this, GearedArmJoint) ? recipeType() : null;
	override public function values():ComponentValues return super.values().setNumber("pocketDiameter", pocketDiameter)
		.setNumber("pocketLength", pocketLength);
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cut(Solids.named(super.geometry(detail), "body"),
			[Solids.named(Part.cylinderSpan(pocketDiameter / 2, -1, pocketLength), "pocket")]);
}
