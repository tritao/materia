package machinekit.assembly;

import cadkit.modeling.Part;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.Solids;
import machinekit.motion.NemaStepper;
import machinekit.standard.SocketHeadCapScrew;

/** NEMA pilot and bolt-clearance plate. Its back at z=0 mates to the motor face. */
class NemaMountPlate extends MachineComponent {
	public static inline var MINIMUM_ENGAGEMENT_DIAMETERS:Float = 1.3;
	public static inline var TIP_CLEARANCE:Float = 0.5;
	public final motor:NemaStepper;
	public final thickness:Float;
	public final size:Float;

	public function new(motor:NemaStepper, thickness:Float, margin:Float = 10) {
		if (!Math.isFinite(thickness) || thickness <= 0 || !Math.isFinite(margin) || margin < 0)
			throw "NEMA mount plate needs a positive thickness and nonnegative margin";
		var plateSize = motor.spec.face + 2 * margin;
		super('PLATE-${motor.designation}-${Dimension.format(thickness)}' +
			(margin == 10 ? "" : "-W" + Dimension.format(plateSize)), 'Motor plate for ${motor.designation}', "aluminium 6061");
		this.motor = motor; this.thickness = thickness;
		size = plateSize;
		addConnector("motor", Mount, Solids.axial(0, 0, 0));
		var i = 1;
		for (point in motor.boltPattern()) addConnector('bolt${i++}', Face, Solids.axial(point.x, point.y, thickness));
	}

	/** The existing MotorShaftBearings design: 0.5 mm tip clearance and at least 1.3 diameters engagement. */
	public function mountScrew():SocketHeadCapScrew {
		if (!motor.variant.tappedMount) throw "This NEMA mount needs through bolts and nuts";
		var screw = motor.mountScrew(Math.ffloor(thickness + motor.variant.mountHoleDepth - TIP_CLEARANCE));
		if (screw.length - thickness < MINIMUM_ENGAGEMENT_DIAMETERS * screw.diameter) throw "Mount screw engagement is too short";
		return screw;
	}

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var plate = Part.box(size, size, thickness);
		return detail == Envelope ? plate : Solids.cut(plate, [motor.mountingCutout(thickness)]);
	}
}
