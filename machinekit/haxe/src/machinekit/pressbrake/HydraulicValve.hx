package machinekit.pressbrake;

import cadkit.modeling.Part;
import machinekit.fluid.ValveSpec;
import machinekit.fluid.ValveControlSpec;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed 4/3 hydraulic valve block: agreeing coils close every work port.
 * P is the regulated feed, T the tank return, and A/B the cylinder work outlets.
 */
class HydraulicValve extends MachineComponent implements ValveSpec {
	static var recipe:Null<ComponentType>;
	public function new() {
		super("HYDRAULIC-VALVE-CLOSED-CENTRE", "Assumed closed-centre hydraulic valve block", "steel", true);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "P", kind: Hydraulic, role: Consumer, iface: Thread("G1/2"), required: true});
		addPort({name: "T", kind: Hydraulic, role: Supply, iface: Thread("G1/2"), required: false});
		for (name in ["A", "B"]) {
			addPort({name: name, kind: Hydraulic, role: Supply, iface: Thread("G1/2"), required: false});
			addBridge("P", name);
		}
		for (name in ["coilA", "coilB"])
			addPort({name: name, kind: Signal, role: Consumer, iface: Plug("24V-coil", 2), required: false});
	}
	public function controlSpec():ValveControlSpec return ClosedCentre;
	override public function valveSpec():Null<ValveSpec> return this;
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(120, 60, 60), "hydraulic-valve-block");
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.pressbrake.hydraulic-valve", [], _ -> new HydraulicValve());
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setToken("material", materialSpec());
}
