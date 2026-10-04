package machinekit.pneumatic;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

/** Generic air manifold with one required input and N bridged outlets. */
class PneumaticManifold extends MachineComponent {
	public final outlets:Int;

	public function new(outlets:Int) {
		if (outlets < 1) throw "Pneumatic manifold needs at least one outlet";
		super('MANIFOLD-$outlets', 'Generic $outlets-outlet pneumatic manifold', "aluminium 6061", true);
		this.outlets = outlets;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "input", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
		for (i in 1...outlets + 1) {
			addPort({name: 'out$i', kind: Pneumatic, role: Supply, iface: PushIn(6), required: false});
			addBridge("input", 'out$i');
		}
	}

	static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null) recipeTypeCache = new ComponentType("machinekit.pneumatic.manifold", [ComponentRecipeSupport.count("outlets", 2)],
			v -> new PneumaticManifold(v.integer("outlets")), true);
		return recipeTypeCache;
	}

	/** Subclasses must declare their own recipe and saved values. */
	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, PneumaticManifold) ? recipeType() : null;

	override public function values():machinekit.component.ComponentValues return new machinekit.component.ComponentValues().setInteger("outlets", outlets).setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(20, 10, 12 + 8 * outlets);
}
