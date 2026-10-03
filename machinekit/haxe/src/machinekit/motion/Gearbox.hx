package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** A drive-level reduction in an assumed cylindrical housing. Ratio and efficiency are stated
 * unless explicitly marked assumed. Teeth, shaft bearings, backlash and compliance are not modelled.
 */
class Gearbox extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public final ratio:Float;
	public final efficiency:Float;
	public final diameter:Float;
	public final length:Float;
	public final bore:Float;
	public final assumed:Bool;
	/** Equivalent rotating inertia at the input shaft, kg m². */
	public final inputInertia:Float;
	public final inertiaAssumed:Bool;
	/** Stated engineering mass; the display envelope does not determine dynamics. */
	public final massKg:Float;

	public function new(ratio:Float, efficiency:Float, diameter:Float = 60, length:Float = 40,
			bore:Float = 8, assumed:Bool = false, inputInertia:Float = 0.00005, inertiaAssumed:Bool = true, ?massKg:Float) {
		for (value in [ratio, efficiency, diameter, length, bore])
			if (!(value > 0) || !Math.isFinite(value)) throw "A gearbox needs finite positive ratings and dimensions";
		if (efficiency > 1 || bore >= diameter) throw "A gearbox needs efficiency at most one and a bore inside its housing";
		if (!Math.isFinite(inputInertia) || inputInertia < 0) throw "Gearbox input inertia must be finite and non-negative";
		super('GEARBOX-${Dimension.format(ratio)}-${Dimension.format(efficiency)}-' +
			'${Dimension.format(diameter)}x${Dimension.format(length)}-B${Dimension.format(bore)}-' + (assumed ? "ASSUMED" : "STATED"),
			'Gearbox, ${Dimension.format(ratio)}:1', "aluminium 6061");
		this.ratio = ratio;
		this.efficiency = efficiency;
		this.diameter = diameter;
		this.length = length;
		this.bore = bore;
		this.assumed = assumed;
		this.inputInertia = inputInertia;
		this.inertiaAssumed = inertiaAssumed;
		// A steel-class annular gearhead is the declared assumption until a catalog entry replaces it.
		this.massKg = massKg == null ? 7.85e-6 * Math.PI * (diameter * diameter - bore * bore) / 4 * length : massKg;
		var radial = (diameter * diameter + bore * bore) / 4;
		var transverse = this.massKg * (3 * radial + length * length) / 12;
		declareMass(this.massKg, new cadkit.modeling.Vector(0, 0, length / 2),
			new cadkit.InertiaTensor(transverse, 0, 0, transverse, 0, this.massKg * radial / 2));
		addConnector("input", Mount, Solids.axial(0, 0, 0));
		addConnector("output", Mount, Solids.axial(0, 0, length));
	}

	public function jointTorque(motorTorque:Float):Float return motorTorque * ratio * efficiency;
	public function jointSpeed(motorSpeed:Float):Float return motorSpeed / ratio;
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.motion.gearbox", [
			ComponentRecipeSupport.scalar("ratio", 10), ComponentRecipeSupport.scalar("efficiency", 0.9),
			ComponentRecipeSupport.length("diameter", 60), ComponentRecipeSupport.length("length", 40),
			ComponentRecipeSupport.length("bore", 8), ComponentRecipeSupport.choice("basis", ["stated", "assumed"], "stated"),
			ComponentRecipeSupport.scalar("inputInertia", 0.00005), ComponentRecipeSupport.scalar("massKg", 0.7),
			ComponentRecipeSupport.choice("inertiaBasis", ["stated", "assumed"], "assumed")
		], v -> new Gearbox(v.number("ratio"), v.number("efficiency"), v.number("diameter"), v.number("length"),
			v.number("bore"), v.token("basis") == "assumed", v.number("inputInertia"), v.token("inertiaBasis") == "assumed", v.number("massKg")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return Std.isExactType(this, Gearbox) ? recipeType() : null;
	override public function values():ComponentValues return new ComponentValues().setNumber("ratio", ratio)
		.setNumber("efficiency", efficiency).setNumber("diameter", diameter).setNumber("length", length)
		.setNumber("bore", bore).setToken("basis", assumed ? "assumed" : "stated").setNumber("inputInertia", inputInertia)
		.setNumber("massKg", massKg).setToken("inertiaBasis", inertiaAssumed ? "assumed" : "stated").setToken("material", materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cut(Solids.named(Part.cylinderSpan(diameter / 2, 0, length), "body"),
			[Solids.named(Part.cylinderSpan(bore / 2, -1, length + 1), "bore")]);
}
