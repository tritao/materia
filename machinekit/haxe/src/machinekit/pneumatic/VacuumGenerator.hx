package machinekit.pneumatic;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Dimension;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;

/** Generic ejector converting compressed air to vacuum. */
class VacuumGenerator extends MachineComponent {
	/** Catalog maximum vacuum below ambient in kPa, not a guaranteed cup pressure. */
	public final ratedVacuumKpa:Null<Float>;

	public function new(?ratedVacuumKpa:Float, ?catalogDesignation:String,
			?airInterface:PortInterface, ?vacuumInterface:PortInterface,
			?catalogDescription:String) {
		if (ratedVacuumKpa != null && (!Math.isFinite(ratedVacuumKpa) ||
				ratedVacuumKpa <= 0 || ratedVacuumKpa > 101.325))
			throw "Vacuum generator rating must be between 0 and 101.325 kPa";
		var ratingId = ratedVacuumKpa == null ? "" : '-V${Dimension.format(ratedVacuumKpa)}';
		super(catalogDesignation == null ? 'VACUUM-GENERATOR$ratingId' : catalogDesignation,
			catalogDesignation == null ? "Generic pneumatic vacuum generator" :
				(catalogDescription == null ? 'Catalog vacuum generator $catalogDesignation' : catalogDescription),
			"aluminium 6061", catalogDesignation == null);
		this.ratedVacuumKpa = ratedVacuumKpa;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "air", kind: Pneumatic, role: Consumer,
			iface: airInterface == null ? PushIn(6) : airInterface, required: true});
		addPort({name: "vacuum", kind: Vacuum, role: Supply,
			iface: vacuumInterface == null ? PushIn(6) : vacuumInterface, required: false});
		addConversion("air", "vacuum");
		addCapability(VacuumActuator("air"));
		addCapability(VacuumSource(ratedVacuumKpa, "vacuum"));
	}

	public static function recipeType():machinekit.component.ComponentType
		return machinekit.component.MachineKitAdditionalRecipes.byId("machinekit.pneumatic.vacuum-generator");

	/** Subclasses must declare their own recipe and saved values. */
	override public function componentType():Null<machinekit.component.ComponentType>
		return Std.isExactType(this, VacuumGenerator) ? recipeType() : null;

	override public function values():machinekit.component.ComponentValues {
		var values = new machinekit.component.ComponentValues()
			.set("ratedVacuumKpa", ratedVacuumKpa == null ? machinekit.component.ComponentValue.Unset : machinekit.component.ComponentValue.Number(ratedVacuumKpa))
			.set("catalogDesignation", codeOnly ? machinekit.component.ComponentValue.Unset : machinekit.component.ComponentValue.Token(designation))
			.set("catalogDescription", codeOnly ? machinekit.component.ComponentValue.Unset : machinekit.component.ComponentValue.Token(description));
		machinekit.component.MachineKitAdditionalRecipes.interfaceValues(values, "airInterface", port("air").iface);
		machinekit.component.MachineKitAdditionalRecipes.interfaceValues(values, "vacuumInterface", port("vacuum").iface);
		return values.setToken("material", materialSpec());
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(18, 14, 30);
}
