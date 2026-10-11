package machinekit.printing;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/**
 * The part being printed, where the simulation lays it: a marker the size of a bead at the machine's zero, the bed's
 * front left corner at its surface, which the print replaces with the material it deposits. Origin: that corner, the
 * surface's normal along +Z. The mass is that of the marker; the deposit adds none to the simulation.
 */
class PrintedPart extends MachineComponent {
	public static inline var MARKER:Float = 1.0;

	public function new() {
		super("PRINTED-PART", "The part being printed", "plastic", true);
		addConnector("origin", Face, Solids.axial(0, 0, 0));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(MARKER, MARKER, MARKER * 0.2), "marker");
}
