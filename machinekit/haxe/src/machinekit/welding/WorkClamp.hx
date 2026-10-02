package machinekit.welding;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Magnetic work clamp: the return of the weld circuit. The arc closes through the workpiece
 * and back by the work lead, so a workpiece the clamp is not on cannot be welded.
 *
 * It is a magnet block that sits on a steel face of the workpiece, with the lead's lug on its
 * side: a robot cell's work lead goes to the workpiece, not to the table it lies on, because the
 * contact between a loose part and its table (scale, paint, a thin edge) is not a dependable
 * conductor, and a magnet needs no re-clamping for each part.
 *
 * CAD frame: standing on its contact face at z=0, centred over its origin in X and Y. Connector:
 * `contact` (the face, +Y along +Z: mate it to the workpiece). Port: the `lead` inlet, the end
 * of the work lead from the power source.
 */
class WorkClamp extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public final ratedCurrentA:Float;

	public function new(ratedCurrentA:Float = 300, width:Float = 36, depth:Float = 24, height:Float = 20) {
		if (!(ratedCurrentA > 0) || !(width > 0) || !(depth > 0) || !(height > 0))
			throw "Work clamp needs a positive rating and dimensions";
		super('WELD-WORK-CLAMP-MAGNETIC-${Dimension.format(ratedCurrentA)}A',
			'Magnetic work clamp, ${Dimension.format(ratedCurrentA)} A', "steel", true);
		this.ratedCurrentA = ratedCurrentA;
		this.width = width;
		this.depth = depth;
		this.height = height;
		addConnector("contact", Mount, Solids.axial(0, 0, 0));
		addPort({name: "lead", kind: ElectricalPower, role: Consumer, iface: WeldingInterfaces.weldCable(), required: true});
		addCapability(WorkReturn("lead", "contact"));
		declareMass(2.4, new Vector(0, 0, 0.4 * height));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(width, depth, height);
}
