package machinekit.control;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** A field switch or presence head with one digital output terminal. */
class DigitalSensor extends MachineComponent {
	public final kind:String;
	static var recipe:Null<ComponentType>;

	public function new(kind:String = "reed") {
		if (kind != "reed" && kind != "presence") throw 'Unknown digital sensor kind "$kind"';
		super('DIGITAL-SENSOR-${kind.toUpperCase()}', '$kind digital sensor', "plastic", true);
		this.kind = kind;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "signal", kind: Signal, role: Supply, iface: Unspecified, required: true});
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.control.digital-sensor", [
			ComponentRecipeSupport.choice("kind", ["reed", "presence"], "reed")
		], v -> new DigitalSensor(v.token("kind")));
		return recipe;
	}

	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues
		return new ComponentValues().setToken("kind", kind).setToken("material", materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(12, 8, 20), "sensor-head");
}
