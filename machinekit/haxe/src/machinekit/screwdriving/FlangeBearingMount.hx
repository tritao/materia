package machinekit.screwdriving;

import machinekit.assembly.FlangeBearingAssembly;
import machinekit.assembly.FlangeTappedPlate;
import machinekit.component.Solids;
import machinekit.standard.DeepGrooveBearing;

/** Existing flange bearing block seated on a tapped aluminium fixture plate. */
class FlangeBearingMount extends FastenedAssembly {
	public final flange:FlangeBearingAssembly;
	public final plate:FlangeTappedPlate;

	public function new(bearing:DeepGrooveBearing, plateEdgeMargin:Float = 10) {
		super();
		flange = new FlangeBearingAssembly(bearing);
		plate = new FlangeTappedPlate(flange.housing, flange.screw, 20, 16, 0, null, plateEdgeMargin);
		flange.addMemberConnector("housing", "mount", Solids.axial(0, 0, 0));
		flange.exposeConnector("mount", "housing", "mount");
		addComponent("plate", plate);
		include("flange", flange);
		var mount = connector("flange/mount");
		addMate("flange-seat", "fixed", "plate", "mount", mount.instanceId, mount.connectorName);
	}
}
