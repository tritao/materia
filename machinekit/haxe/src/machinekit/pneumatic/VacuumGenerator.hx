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

	public function new(?ratedVacuumKpa:Float) {
		if (ratedVacuumKpa != null && (!Math.isFinite(ratedVacuumKpa) ||
				ratedVacuumKpa <= 0 || ratedVacuumKpa > 101.325))
			throw "Vacuum generator rating must be between 0 and 101.325 kPa";
		var ratingId = ratedVacuumKpa == null ? "" : '-V${Dimension.format(ratedVacuumKpa)}';
		super('VACUUM-GENERATOR$ratingId', "Generic pneumatic vacuum generator", "aluminium 6061", true);
		this.ratedVacuumKpa = ratedVacuumKpa;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "air", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
		addPort({name: "vacuum", kind: Vacuum, role: Supply, iface: PushIn(6), required: false});
		addConversion("air", "vacuum");
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(18, 14, 30);
}
