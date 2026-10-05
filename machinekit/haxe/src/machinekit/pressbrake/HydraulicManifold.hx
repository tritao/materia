package machinekit.pressbrake;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed hydraulic distribution block with one inlet and distinct G1/2 outlets. */
class HydraulicManifold extends MachineComponent {
	public final outlets:Int;
	static var recipe:Null<ComponentType>;
	public function new(outlets:Int) {
		if (outlets < 1 || outlets > 64) throw "Hydraulic manifold needs 1 to 64 outlets";
		super('HYDRAULIC-MANIFOLD-$outlets', 'Assumed $outlets-outlet hydraulic distribution block', "steel", true);
		this.outlets = outlets;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "input", kind: Hydraulic, role: Consumer, iface: Thread("G1/2"), required: true});
		for (i in 1...outlets + 1) {
			addPort({name: 'out$i', kind: Hydraulic, role: Supply, iface: Thread("G1/2"), required: false});
			addBridge("input", 'out$i');
		}
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(24, 24, 16 + 12 * outlets), "hydraulic-manifold");
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.pressbrake.hydraulic-manifold", [
			new machinekit.component.ComponentParameter("outlets", Count, Integer(2), null, 1, 64)
		], v -> new HydraulicManifold(v.integer("outlets")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setInteger("outlets", outlets)
		.setToken("material", materialSpec());
}
