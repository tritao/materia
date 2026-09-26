package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Nominal cylindrical envelope of a lead screw. The thread is semantic and shared with its nut;
 * the flank profile, reliefs and machined ends are not generated.
 */
class LeadScrew extends MachineComponent {
	public final thread:LeadScrewThread;
	public final totalLength:Float;

	public function new(thread:LeadScrewThread, length:Float) {
		if (thread == null) throw "Lead screw needs a thread specification";
		if (!(length > 0) || !Math.isFinite(length)) throw "Lead screw needs a positive length";
		super('LEADSCREW-${thread.designation}-L${Dimension.format(length)}',
			'Lead screw ${thread.designation}, ${Dimension.format(length)} mm long', "steel");
		this.thread = thread;
		totalLength = length;
		addConnector("input", Axis, Solids.axial(0, 0, 0));
		addConnector("output", Axis, Solids.axial(0, 0, length));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cylinder(thread.screwDiameter / 2, 0, totalLength);
}
