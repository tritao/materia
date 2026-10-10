package machinekit.screwdriving;

import machinekit.assembly.NemaMountPlate;
import machinekit.motion.NemaStepper;
import machinekit.standard.SocketHeadCapScrew;

/** NEMA 17 workpiece with the same plate and screw sizing as MotorShaftBearings. */
class NemaMotorMount extends FastenedAssembly {
	public final motor:NemaStepper;
	public final plate:NemaMountPlate;
	public final screw:SocketHeadCapScrew;

	public function new() {
		super();
		motor = NemaStepper.frame(17);
		plate = new NemaMountPlate(motor, 6);
		screw = plate.mountScrew();
		addComponent("motor", motor); addComponent("plate", plate);
		addMate("plate-mount", "fixed", "motor", "mountFace", "plate", "motor");
		for (index in 1...5) {
			var id = 'screw$index';
			addComponent(id, screw);
			addMate('$id-seat', "fixed", "plate", 'bolt$index', id, "head");
			var declared = join(id, NemaMountPlate.MINIMUM_ENGAGEMENT_DIAMETERS);
		}
	}
}
