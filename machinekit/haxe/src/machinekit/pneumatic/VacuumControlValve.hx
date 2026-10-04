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

/** Generic normally closed valve in a vacuum line, commanded by a signal. */
class VacuumControlValve extends MachineComponent {

	public function new(tubeOdMm:Float = 4) {
		if (!Math.isFinite(tubeOdMm) || tubeOdMm <= 0)
			throw "Vacuum control valve needs a positive tube diameter";
		super('VACUUM-CONTROL-VALVE-${tubeOdMm}',
			"Generic normally closed vacuum control valve", "plastic", true);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "vacuumIn", kind: Vacuum, role: Consumer,
			iface: PushIn(tubeOdMm), required: true});
		addPort({name: "vacuumOut", kind: Vacuum, role: Supply,
			iface: PushIn(tubeOdMm), required: false});
		addBridge("vacuumIn", "vacuumOut");
		addPort({name: "control", kind: Signal, role: Consumer,
			iface: Plug("digital-valve", 2), required: true});
		addFacet(new VacuumValveFacet("control"));
	}

	static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null) recipeTypeCache = new ComponentType("machinekit.pneumatic.vacuum-control-valve", [ComponentRecipeSupport.length("tubeOdMm", 4)],
			v -> new VacuumControlValve(v.number("tubeOdMm")), true);
		return recipeTypeCache;
	}

	/** Subclasses must declare their own recipe and saved values. */
	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, VacuumControlValve) ? recipeType() : null;

	override public function values():machinekit.component.ComponentValues return new machinekit.component.ComponentValues().setNumber("tubeOdMm", switch port("vacuumIn").iface { case PushIn(d): d; case _: 0; }).setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(18, 16, 26);
}
