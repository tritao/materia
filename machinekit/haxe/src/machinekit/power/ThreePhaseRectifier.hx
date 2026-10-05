package machinekit.power;

import machinekit.motion.PowerSupply;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;

/** Assumed three-phase rectifier with a capacitor bus and integrated regenerative shunt.
 * Line voltage is phase-to-phase RMS. The nominal unloaded bus is its peak;
 * this reference model does not simulate ripple or mains impedance.
 */
class ThreePhaseRectifier extends PowerSupply {
	public final mains:AcMains;
	public function new(mains:AcMains, current:DcCurrent, outlets:Int = 1,
		width:Float = 240, height:Float = 120, depth:Float = 100, regenerativePower:Float = 0) {
		if (mains == null || mains.system != ThreePhase) throw "This rectifier needs three-phase mains";
		super(mains.capacitorBus(), current, outlets, width, height, depth, regenerativePower);
		this.mains = mains;
		addPort({name: "mains", kind: ElectricalPower, role: Consumer,
			iface: Plug("three-phase-mains", 4), required: true});
		for (index in 1...outlets + 1) addConversion("mains", 'power$index');
	}
	static var rectifierRecipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (rectifierRecipe == null) rectifierRecipe = new ComponentType("machinekit.power.three-phase-rectifier", [
			ComponentRecipeSupport.scalar("lineVoltage", 400, 0.001),
			ComponentRecipeSupport.scalar("frequency", 50, 0.001),
			ComponentRecipeSupport.scalar("current", 20, 0.001), ComponentRecipeSupport.count("outlets", 4),
			ComponentRecipeSupport.length("width", 240), ComponentRecipeSupport.length("height", 120),
			ComponentRecipeSupport.length("depth", 100), ComponentRecipeSupport.scalar("regenerativePower", 8000, 0)
		], v -> new ThreePhaseRectifier(new AcMains(ThreePhase, new AcRmsVoltage(v.number("lineVoltage")), v.number("frequency")), new DcCurrent(v.number("current")),
			v.integer("outlets"), v.number("width"), v.number("height"), v.number("depth"), v.number("regenerativePower")));
		return rectifierRecipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setNumber("lineVoltage", mains.rmsVoltage.volts())
		.setNumber("frequency", mains.frequencyHz).setNumber("current", current.amps()).setInteger("outlets", outlets)
		.setNumber("width", width).setNumber("height", height).setNumber("depth", depth)
		.setNumber("regenerativePower", regenerativePower).setToken("material", materialSpec());
}
