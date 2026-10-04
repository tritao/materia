package machinekit.pneumatic;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Reference 5/2 valve. Exhausts are atmospheric; P supplies A or B, never both.
 * A single coil reverses the spring-return state. Double coils select and latch A/B.
 * Installed 6 mm fittings and the valve envelope are assumed.
 */
class SolenoidValve extends MachineComponent implements PneumaticValveSpec {
	public final doubleSolenoid:Bool;
	public final normallyToA:Bool;
	static var recipe:Null<ComponentType>;
	public function new(doubleSolenoid:Bool = false, normallyToA:Bool = false) {
		super(doubleSolenoid ? "VALVE-5/2-DOUBLE" : "VALVE-5/2-SINGLE", "Reference pneumatic directional valve", "aluminium 6061", true);
		this.doubleSolenoid = doubleSolenoid; this.normallyToA = normallyToA;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "P", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
		for (name in ["A", "B"]) {
			addPort({name: name, kind: Pneumatic, role: Supply, iface: PushIn(6), required: false});
			addBridge("P", name);
		}
		addPort({name: "coilA", kind: Signal, role: Consumer, iface: Plug("24V-coil", 2), required: false});
		if (doubleSolenoid) addPort({name: "coilB", kind: Signal, role: Consumer, iface: Plug("24V-coil", 2), required: false});
	}
	/** Pure spool transition. Retain the previous state when double coils agree. */
	public function routesToA(coilA:Bool, coilB:Bool, previousToA:Bool):Bool
		return doubleSolenoid ? (coilA == coilB ? previousToA : coilA) : (coilA ? !normallyToA : normallyToA);
	public function isDoubleSolenoid():Bool return doubleSolenoid;
	public function defaultsToA():Bool return normallyToA;
	override public function pneumaticValveSpec():Null<PneumaticValveSpec> return this;
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(doubleSolenoid ? 100 : 80, 25, 30), "valve");
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.pneumatic.solenoid-valve", [
			ComponentRecipeSupport.choice("solenoids", ["single", "double"], "single"),
			ComponentRecipeSupport.choice("normalOutlet", ["A", "B"], "B")
		], v -> new SolenoidValve(v.token("solenoids") == "double", v.token("normalOutlet") == "A"));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues()
		.setToken("solenoids", doubleSolenoid ? "double" : "single").setToken("normalOutlet", normallyToA ? "A" : "B")
		.setToken("material", materialSpec());
}
